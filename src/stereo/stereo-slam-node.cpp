#include "stereo-slam-node.hpp"
#include "utility.hpp"

// -------------------------------------------------------------------------- //
//  Construtor                                                                 //
// -------------------------------------------------------------------------- //

/**
 * @brief Constrói o nó estéreo.
 *
 * Delega a criação da infra-estrutura ROS 2 partilhada ao construtor de
 * SlamNodeBase e acrescenta:
 *   1. Interpretação do argumento strDoRectify.
 *   2. Cálculo dos mapas de rectificação, se necessário.
 *   3. Criação das subscrições e do sincronizador ApproximateTime.
 *
 * @param pSLAM           Pointer para o ORB_SLAM3::System já construído.
 * @param strSettingsFile Caminho absoluto para o ficheiro YAML de calibração.
 * @param strDoRectify    String "true" ou "false" que controla a rectificação.
 *
 * @throws std::runtime_error Se o ficheiro de configuração não puder ser aberto
 *         ou se algum parâmetro de calibração estiver ausente.
 */
StereoSlamNode::StereoSlamNode(ORB_SLAM3::System*  pSLAM,
                               const std::string  &strSettingsFile,
                               const std::string  &strDoRectify)
:   SlamNodeBase("ORB_SLAM3_ROS2", pSLAM)
{
    /// Interpreta o argumento de rectificação.
    /// O bloco delimitado garante que o stringstream é destruído imediatamente.
    {
        std::stringstream ss(strDoRectify);
        ss >> std::boolalpha >> doRectify;
    }

    if (doRectify) {
        InitRectificationMaps(strSettingsFile);
    }

    /**
     * @brief Subscrições e sincronizador de tempo aproximado.
     *
     * message_filters::Subscriber encapsula uma subscrição ROS 2 normal para
     * que possa ser ligada a uma política de sincronização.
     * ApproximateTime emparelha os frames esquerdo e direito cujos timestamps
     * são mais próximos entre si.
     * A dimensão da fila (10) limita o número de mensagens sem correspondência
     * que ficam em espera por tópico.
     */
    left_sub  = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/left");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/right");

    syncApproximate = std::make_shared<
        message_filters::Synchronizer<approximate_sync_policy>>(
            approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);

    RCLCPP_INFO(this->get_logger(), "StereoSlamNode ready");
}

// -------------------------------------------------------------------------- //
//  Inicialização privada                                                      //
// -------------------------------------------------------------------------- //

/**
 * @brief Lê os parâmetros estéreo do ficheiro YAML e pré-calcula os mapas de
 *        rectificação para as câmaras esquerda e direita.
 *
 * A rectificação corrige a distorção de cada imagem e roda ambos os planos
 * de imagem de forma a que as linhas epipolares fiquem horizontais — condição
 * necessária para o algoritmo de correspondência estéreo do ORB-SLAM3.
 *
 * Os mapas são calculados uma única vez aqui para que GrabStereo() possa
 * invocar o barato cv::remap() em cada frame, em vez de recalcular a geometria
 * de rectificação repetidamente.
 *
 * @param strSettingsFile Caminho para o ficheiro YAML de calibração.
 * @throws std::runtime_error Se o ficheiro não puder ser aberto ou se algum
 *         parâmetro de calibração estiver ausente.
 */
void StereoSlamNode::InitRectificationMaps(const std::string &strSettingsFile)
{
    cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        /// FATAL em vez de ERROR porque o nó não consegue funcionar sem calibração.
        RCLCPP_FATAL(this->get_logger(), "Cannot open settings file: %s",
                     strSettingsFile.c_str());
        throw std::runtime_error("Settings file not found");
    }

    /// K  — matriz intrínseca 3×3.
    /// P  — matriz de projecção 3×4 após rectificação.
    /// R  — rotação de rectificação 3×3.
    /// D  — coeficientes de distorção.
    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"]  >> K_l;  fsSettings["RIGHT.K"] >> K_r;
    fsSettings["LEFT.P"]  >> P_l;  fsSettings["RIGHT.P"] >> P_r;
    fsSettings["LEFT.R"]  >> R_l;  fsSettings["RIGHT.R"] >> R_r;
    fsSettings["LEFT.D"]  >> D_l;  fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"],  cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"], cols_r = fsSettings["RIGHT.width"];

    /// cv::Mat::empty() devolve true quando uma chave YAML está ausente ou
    /// não pôde ser interpretada correctamente.
    if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() ||
        R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
        rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
        RCLCPP_FATAL(this->get_logger(),
                     "Stereo rectification parameters missing in settings file");
        throw std::runtime_error("Incomplete calibration data");
    }

    /// P.rowRange(0,3).colRange(0,3) extrai a nova matriz de câmara 3×3 a partir de P.
    cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0,3).colRange(0,3),
                                cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
    cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0,3).colRange(0,3),
                                cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);

    RCLCPP_INFO(this->get_logger(), "Stereo rectification maps initialised");
}

// -------------------------------------------------------------------------- //
//  Callback do sincronizador                                                  //
// -------------------------------------------------------------------------- //

/**
 * @brief Processa um par de imagens estéreo sincronizadas.
 *
 * Passos:
 *   1. Converte ambas as mensagens ROS para cv::Mat via cv_bridge.
 *   2. Aplica os mapas de rectificação, se doRectify == true.
 *   3. Invoca ORB_SLAM3::System::TrackStereo() para obter a pose world-to-camera.
 *   4. Gere transições de estado (LOST / RECOVERED) e dispara reinicialização
 *      automática via OnTrackingLost() / AttemptReinitialization() da base.
 *   5. Delega a publicação de pose, odometria, TF, estado e mapa na base.
 *
 * @param msgLeft  Mensagem de imagem da câmara esquerda.
 * @param msgRight Mensagem de imagem da câmara direita.
 */
void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft,
                                 const ImageMsg::SharedPtr msgRight)
{
    /// Contabiliza todos os pares recebidos, incluindo os que falham mais tarde,
    /// para que DiagnosticsCallback() possa reportar a taxa de entrada real.
    ++total_frames_;

    /**
     * @brief Conversão ROS → OpenCV.
     *
     * toCvShare() evita uma cópia dos dados de pixéis quando a codificação já
     * é compatível com o OpenCV. Blocos try/catch separados identificam qual
     * câmara originou a falha.
     */
    try { cv_ptrLeft  = cv_bridge::toCvShare(msgLeft);  }
    catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge left: %s", e.what());
        return;
    }
    try { cv_ptrRight = cv_bridge::toCvShare(msgRight); }
    catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge right: %s", e.what());
        return;
    }

    /// O timestamp é extraído uma única vez e reutilizado em ambos os ramos.
    Sophus::SE3f Tcw;
    const double stamp = Utility::StampToSec(msgLeft->header.stamp);

    if (doRectify) {
        /// Aplica os mapas pré-calculados antes de enviar ao SLAM.
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,  imLeft,  M1l, M2l, cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
        Tcw = m_SLAM->TrackStereo(imLeft, imRight, stamp);
    } else {
        /// As imagens já foram rectificadas pelo driver da câmara.
        Tcw = m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, stamp);
    }

    /// O estado é sempre publicado, mesmo quando o tracking está perdido,
    /// para que monitores de segurança externos possam reagir.
    const int state = m_SLAM->GetTrackingState();
    PublishTrackingState(state);

    /**
     * @brief Detecção de transições de estado.
     *
     * A comparação com last_tracking_state_ garante que cada mensagem de log
     * dispara exactamente uma vez na transição, em vez de inundar o log a 30 Hz.
     */
    if (state != last_tracking_state_) {
        if (state == ORB_SLAM3::Tracking::LOST) {
            RCLCPP_WARN(this->get_logger(), "Tracking LOST");
        } else if (state == ORB_SLAM3::Tracking::OK &&
                   last_tracking_state_ == ORB_SLAM3::Tracking::LOST) {
            RCLCPP_INFO(this->get_logger(), "Tracking RECOVERED");
            lost_frame_count_ = 0;
        }
        last_tracking_state_ = state;
    }

    /// Retorno antecipado enquanto o tracking está perdido: não há pose válida
    /// para publicar. OnTrackingLost() incrementa o contador e pode chamar Reset().
    if (state == ORB_SLAM3::Tracking::LOST) {
        OnTrackingLost();
        return;
    }

    lost_frame_count_ = 0;  ///< Reinicia o contador a cada frame válido.

    /// Todas as mensagens derivadas da pose partilham este header para que os
    /// seus timestamps sejam idênticos.
    std_msgs::msg::Header header;
    header.stamp    = msgLeft->header.stamp;
    header.frame_id = "map";

    /**
     * @brief Publicação da pose restrita ao estado Tracking::OK.
     *
     * No estado RECENTLY_LOST, o ORB-SLAM3 propaga a pose via modelo de
     * movimento — injectar essa estimativa extrapolada num EKF introduziria
     * ruído correlacionado, pelo que é suprimida.
     */
    if (state == ORB_SLAM3::Tracking::OK) {
        /// Tcw é world-to-camera; inverte-se para obter camera-in-world (Twc).
        const Sophus::SE3f Twc = Tcw.inverse();
        PublishPose(Twc, header);
        PublishOdometry(Twc, header);
        PublishTF(Twc, header);
        ++published_frames_;
    }

    /// Os pontos do mapa são válidos tanto para OK como para RECENTLY_LOST:
    /// a geometria do mapa não é afectada pela incerteza momentânea da pose.
    PublishMapPoints(header);
}