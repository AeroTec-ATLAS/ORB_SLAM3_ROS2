#include "stereo-slam-node.hpp"
#include <opencv2/core/core.hpp>
#include <cstring>  ///< std::memcpy
#include "utility.hpp"

/// Marcadores de posição para std::bind, utilizados no registo do serviço e do temporizador.
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

/**
 * @brief Constrói o nó ROS 2 de SLAM estéreo com ORB-SLAM3.
 *
 * Executa três tarefas pela ordem indicada:
 *   1. Interpreta o argumento @p strDoRectify e converte-o num booleano.
 *   2. Se a rectificação for pedida, lê os parâmetros de calibração estéreo
 *      do ficheiro @p strSettingsFile e pré-calcula os mapas de undistort/rectify
 *      do OpenCV, evitando recalculá-los em cada frame.
 *   3. Cria toda a infra-estrutura ROS 2: subscrições, sincronizador, publishers,
 *      broadcaster de TF, serviço de reset e temporizador de diagnóstico.
 *
 * @param pSLAM           Pointer para o ORB_SLAM3::System já construído.
 *                        A posse do objecto permanece em main().
 * @param strSettingsFile Caminho absoluto para o ficheiro YAML de configuração da câmara.
 * @param strDoRectify    String @c "true" ou @c "false" que controla a rectificação.
 *
 * @throws std::runtime_error Se o ficheiro de configuração não puder ser aberto ou se
 *         algum parâmetro de calibração obrigatório estiver ausente.
 */
StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM,
                               const std::string &strSettingsFile,
                               const std::string &strDoRectify)
:   Node("ORB_SLAM3_ROS2"),  ///< Nome do nó visível em `ros2 node list`.
    m_SLAM(pSLAM)
{
    /// Interpreta o argumento de rectificação. O bloco delimitado garante que o
    /// stringstream é destruído imediatamente após a leitura.
    {
        std::stringstream ss(strDoRectify);
        ss >> std::boolalpha >> doRectify;  ///< boolalpha permite interpretar "true"/"false".
    }

    /**
     * @brief Bloco de inicialização dos mapas de rectificação estéreo.
     *
     * A rectificação corrige a distorção de cada imagem e roda ambos os planos
     * de imagem de forma a que as linhas epipolares fiquem horizontais — condição
     * necessária para o algoritmo de correspondência estéreo do ORB-SLAM3.
     * Os mapas são calculados uma única vez aqui para que GrabStereo() possa
     * invocar o barato cv::remap() em cada frame, em vez de recalcular a
     * geometria de rectificação repetidamente.
     *
     * Este bloco é ignorado quando o driver da câmara já publica imagens
     * rectificadas (doRectify == false).
     */
    if (doRectify) {
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
            /// FATAL em vez de ERROR porque o nó não consegue funcionar sem calibração.
            RCLCPP_FATAL(this->get_logger(), "Cannot open settings file: %s",
                         strSettingsFile.c_str());
            /// throw permite que main() capture a excepção, registe o erro e invoque
            /// rclcpp::shutdown() de forma limpa, em vez de terminar abruptamente.
            throw std::runtime_error("Settings file not found");
        }

        /// K  — matriz intrínseca 3x3.
        /// P  — matriz de projecção 3x4 após rectificação.
        /// R  — rotação de rectificação 3x3.
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

        /// P.rowRange(0,3).colRange(0,3) extrai a nova matriz de câmara 3x3 a partir de P.
        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0,3).colRange(0,3),
                                    cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0,3).colRange(0,3),
                                    cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);

        RCLCPP_INFO(this->get_logger(), "Stereo rectification maps initialised");
    }

    /**
     * @brief Configuração das subscrições e do sincronizador de tempo aproximado.
     *
     * message_filters::Subscriber encapsula uma subscrição ROS 2 normal para que
     * possa ser ligada a uma política de sincronização.
     * ApproximateTime emparelha os frames esquerdo e direito cujos timestamps
     * são mais próximos entre si.
     * A dimensão da fila (10) define o número máximo de mensagens sem correspondência
     * que ficam em espera por tópico enquanto aguardam um par do outro tópico.
     */
    left_sub  = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/left");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/right");
    syncApproximate = std::make_shared<
        message_filters::Synchronizer<approximate_sync_policy>>(
            approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);

    /// Profundidade de fila de 10 é adequada para tópicos de pose e nuvem de pontos a 30 Hz.
    pose_pub_  = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                     "orb_slam3/camera_pose", 10);
    odom_pub_  = this->create_publisher<nav_msgs::msg::Odometry>(
                     "orb_slam3/odometry", 10);
    map_pub_   = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                     "orb_slam3/map_points", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::Int32>(
                     "orb_slam3/tracking_state", 10);

    /// Broadcaster de TF dinâmico: publica map -> camera_link em cada frame com tracking
    /// válido, permitindo que o RViz e os nós de fusão sensorial localizem a câmara
    /// sem precisarem de subscrever um tópico de pose separado.
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    /// Serviço para reset manual pelo operador sem reiniciar o nó:
    /// @code ros2 service call /orb_slam3/reset std_srvs/srv/Trigger @endcode
    reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "orb_slam3/reset",
        std::bind(&StereoSlamNode::ResetCallback, this, _1, _2));

    /// Temporizador de parede (tempo real) para que os diagnósticos continuem a disparar
    /// mesmo durante a reprodução de um rosbag a velocidade reduzida ou nula.
    diagnostics_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&StereoSlamNode::DiagnosticsCallback, this));

    RCLCPP_INFO(this->get_logger(), "StereoSlamNode ready");
}

/**
 * @brief Destrutor — termina o ORB-SLAM3 e guarda a trajectória de keyframes.
 *
 * Shutdown() sinaliza todas as threads internas do ORB-SLAM3 (tracking,
 * mapeamento local, fecho de ciclos) para terminarem antes de o objecto
 * ser destruído.
 * A trajectória é escrita em formato TUM (timestamp tx ty tz qx qy qz qw),
 * compatível com as ferramentas de avaliação do benchmark TUM RGB-D.
 */
StereoSlamNode::~StereoSlamNode()
{
    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    RCLCPP_INFO(this->get_logger(), "Trajectory saved. Node destroyed.");
}

/**
 * @brief Processa um par de imagens estéreo sincronizadas.
 * Os passos são:
 *   1. Converter ambas as imagens ROS para OpenCV via cv_bridge.
 *   2. Aplicar os mapas de rectificação, se necessário.
 *   3. Invocar ORB_SLAM3::System::TrackStereo() para obter a pose world-to-camera.
 *   4. Inspeccionar o estado de tracking, detectar transições perdido/recuperado
 *      e desencadear a reinicialização se o contador de frames perdidos atingir
 *      o limiar definido.
 *   5. Publicar pose, odometria, TF, estado de tracking e pontos do mapa.
 *
 * @param msgLeft  Pointer partilhado para a mensagem de imagem da câmara esquerda.
 * @param msgRight Pointer partilhado para a mensagem de imagem da câmara direita.
 */
void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft,
                                 const ImageMsg::SharedPtr msgRight)
{
    /// Contabiliza todos os pares recebidos, incluindo os que falham mais tarde,
    /// para que DiagnosticsCallback() possa reportar a taxa de entrada real
    /// versus a taxa de publicação.
    ++total_frames_;

    /**
     * @brief Conversão das imagens ROS para OpenCV.
     *
     * toCvShare() evita uma cópia dos dados de pixéis quando a codificação já é
     * compatível com o OpenCV. São usados blocos try/catch separados para que a
     * mensagem de erro identifique qual das câmaras originou a falha.
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

    /// O timestamp é extraído uma única vez e reutilizado em ambos os ramos,
    /// garantindo que os dois caminhos usam exactamente o mesmo valor.
    Sophus::SE3f Tcw;
    const double stamp = Utility::StampToSec(msgLeft->header.stamp);

    if (doRectify) {
        /// Aplica os mapas de undistort/rectify pré-calculados antes de enviar ao SLAM.
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

    /// Retorno antecipado enquanto o tracking está perdido: não há nada válido
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
     * @brief Porta de publicação da pose.
     *
     * A publicação está restrita ao estado Tracking::OK. No estado RECENTLY_LOST,
     * o ORB-SLAM3 propaga a pose usando um modelo de movimento — injectar essa
     * estimativa extrapolada num EKF introduziria ruído correlacionado, pelo que
     * é suprimida.
     */
    if (state == ORB_SLAM3::Tracking::OK) {
        /// Tcw é world-to-camera; inverte-se para obter camera-in-world (Twc).
        Sophus::SE3f Twc = Tcw.inverse();
        PublishPose(Twc, header);
        PublishOdometry(Twc, header);
        PublishTF(Twc, header);
        ++published_frames_;
    }

    /// Os pontos do mapa são válidos tanto para OK como para RECENTLY_LOST:
    /// a geometria do mapa não é afectada pela incerteza momentânea da pose.
    PublishMapPoints(header);
}

/**
 * @brief Publica a pose da câmara como @c geometry_msgs/PoseStamped.
 *
 * Alternativa leve à Odometry — adequada para visualização no RViz, registo
 * e consumidores que não necessitam de informação de covariância.
 *
 * @param Twc    Transformação SE3 camera-in-world (Tcw já invertida pelo chamador).
 * @param header Header ROS partilhado (stamp + frame_id) para este frame.
 */
void StereoSlamNode::PublishPose(const Sophus::SE3f &Twc,
                                  const std_msgs::msg::Header &header)
{
    const Eigen::Vector3f    t = Twc.translation();
    const Eigen::Quaternionf q = Twc.unit_quaternion();  ///< unit_quaternion() é sempre normalizado.

    geometry_msgs::msg::PoseStamped msg;
    msg.header             = header;
    msg.pose.position.x    = t.x();
    msg.pose.position.y    = t.y();
    msg.pose.position.z    = t.z();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    pose_pub_->publish(msg);
}

/**
 * @brief Publica a pose da câmara como @c nav_msgs/Odometry.
 *
 * Mais rico do que PoseStamped: inclui @c child_frame_id e uma matriz de
 * covariância de pose 6x6, ambos exigidos por @c robot_localization (nó EKF)
 * e pelo MAVROS @c vision_pose/pose.
 *
 * @note A diagonal da covariância contém valores de referência. Deverão ser
 *       ajustados com base no erro de reprojecção real do ORB-SLAM3 antes de
 *       utilizar esta saída como medição de um EKF.
 *
 * @param Twc    Transformação SE3 camera-in-world.
 * @param header Header ROS partilhado para este frame.
 */
void StereoSlamNode::PublishOdometry(const Sophus::SE3f &Twc,
                                      const std_msgs::msg::Header &header)
{
    const Eigen::Vector3f    t = Twc.translation();
    const Eigen::Quaternionf q = Twc.unit_quaternion();

    nav_msgs::msg::Odometry msg;
    msg.header         = header;         ///< Frame fixo (pai): "map".
    msg.child_frame_id = "camera_link";  ///< Frame móvel (filho) — convenção ROS.

    msg.pose.pose.position.x    = t.x();
    msg.pose.pose.position.y    = t.y();
    msg.pose.pose.position.z    = t.z();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    /**
     * @brief Matriz de covariância da pose (6x6, row-major, aplanada em 36 elementos).
     *
     * Disposição dos índices: [x, y, z, roll, pitch, yaw].
     * Apenas a diagonal (variâncias) é preenchida; as correlações cruzadas
     * fora da diagonal ficam a zero, o que é uma aproximação razoável para
     * saídas de VIO.
     * Unidades: m^2 para posição, rad^2 para orientação.
     *
     * @warning Estes são valores de referência — substituir por valores obtidos
     *          através da caracterização do sensor antes de integrar num EKF.
     */
    msg.pose.covariance[0]  = 0.01;   ///< Variância em x.
    msg.pose.covariance[7]  = 0.01;   ///< Variância em y.
    msg.pose.covariance[14] = 0.01;   ///< Variância em z.
    msg.pose.covariance[21] = 0.001;  ///< Variância em roll.
    msg.pose.covariance[28] = 0.001;  ///< Variância em pitch.
    msg.pose.covariance[35] = 0.001;  ///< Variância em yaw.

    odom_pub_->publish(msg);
}

/**
 * @brief Difunde a transformação dinâmica de TF @c map → @c camera_link.
 *
 * Sem esta difusão, o RViz não consegue renderizar a nuvem de pontos na posição
 * correcta e qualquer nó que invoque @c tf2::doTransform para projectar pontos
 * no referencial da câmara falhará com um erro de lookup.
 *
 * @param Twc    Transformação SE3 camera-in-world.
 * @param header Header ROS partilhado para este frame (fornece o timestamp).
 */
void StereoSlamNode::PublishTF(const Sophus::SE3f &Twc,
                                const std_msgs::msg::Header &header)
{
    const Eigen::Vector3f    t = Twc.translation();
    const Eigen::Quaternionf q = Twc.unit_quaternion();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header         = header;         ///< Frame pai: "map".
    tf_msg.child_frame_id = "camera_link";  ///< Frame filho: corpo da câmara.

    tf_msg.transform.translation.x = t.x();
    tf_msg.transform.translation.y = t.y();
    tf_msg.transform.translation.z = t.z();
    tf_msg.transform.rotation.x    = q.x();
    tf_msg.transform.rotation.y    = q.y();
    tf_msg.transform.rotation.z    = q.z();
    tf_msg.transform.rotation.w    = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
}

/**
 * @brief Converte os pontos do mapa rastreados para PointCloud2 e publica-os.
 *
 * O mapa é publicado para visualização no RViz. A utilização de
 * GetTrackedMapPoints() em vez de GetAllMapPoints() limita a nuvem ao subconjunto
 * visível no frame actual, mantendo o tamanho da mensagem delimitado.
 *
 * @param header Header ROS partilhado para este frame.
 */
void StereoSlamNode::PublishMapPoints(const std_msgs::msg::Header &header)
{
    const auto map_points = m_SLAM->GetTrackedMapPoints();
    if (!map_points.empty()) {
        map_pub_->publish(MapPointsToPointCloud2(map_points, header));
    }
}

/**
 * @brief Publica o estado actual de tracking do ORB-SLAM3 como @c Int32.
 *
 * Mapeamento dos valores:
 *   - 0 — NO_IMAGES_YET
 *   - 1 — NOT_INITIALIZED
 *   - 2 — OK
 *   - 3 — RECENTLY_LOST
 *   - 4 — LOST
 *
 * Nós externos (p. ex. um monitor de segurança do controlador de voo) podem
 * subscrever este tópico para condicionar decisões à qualidade do tracking.
 *
 * @param state Valor inteiro do estado devolvido por ORB_SLAM3::Tracking.
 */
void StereoSlamNode::PublishTrackingState(int state)
{
    std_msgs::msg::Int32 msg;
    msg.data = state;
    state_pub_->publish(msg);
}

/**
 * @brief Incrementa o contador de frames perdidos e desencadeia a reinicialização
 *        quando este atinge @c kLostFrameThreshold.
 *
 * O aviso é limitado a uma mensagem por cada 2 segundos para evitar a saturação
 * do log à cadência da câmara (30 linhas/segundo sem limitação).
 */
void StereoSlamNode::OnTrackingLost()
{
    ++lost_frame_count_;

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Tracking lost — %d consecutive frames lost",
                         lost_frame_count_);

    if (lost_frame_count_ >= kLostFrameThreshold) {
        AttemptReinitialization();
    }
}

/**
 * @brief Reinicia o sistema ORB-SLAM3 e limpa todas as variáveis de controlo de falhas.
 *
 * Após @c Reset(), o ORB-SLAM3 volta ao estado @c NOT_INITIALIZED e tenta
 * reinicializar-se a partir dos próximos frames recebidos.
 *
 * @c last_tracking_state_ é definido como @c -1 (sentinela inválido) para que
 * a lógica de detecção de transições em GrabStereo() dispare correctamente na
 * primeira mudança de estado após o reset, em vez de a ignorar silenciosamente
 * porque o estado anterior coincide acidentalmente com o novo.
 */
void StereoSlamNode::AttemptReinitialization()
{
    RCLCPP_WARN(this->get_logger(),
                "Tracking lost for %d frames — triggering automatic reset",
                lost_frame_count_);

    m_SLAM->Reset();
    lost_frame_count_    = 0;
    last_tracking_state_ = -1;

    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 reset complete. Waiting for reinitialisation.");
}

/**
 * @brief Callback do serviço ROS 2 para reset manual pelo operador.
 *
 * Reinicia o ORB-SLAM3 sem reiniciar o nó — útil em testes de campo quando
 * o UAV pode reposicionar-se numa área com maior densidade de features.
 *
 * Invocar a partir da linha de comandos:
 * @code
 *   ros2 service call /orb_slam3/reset std_srvs/srv/Trigger
 * @endcode
 *
 * @param req  Não utilizado — @c std_srvs/Trigger não transporta dados no pedido.
 * @param res  Resposta preenchida com o indicador de sucesso e uma mensagem de estado.
 */
void StereoSlamNode::ResetCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>  /*req*/,
          std::shared_ptr<std_srvs::srv::Trigger::Response>   res)
{
    RCLCPP_WARN(this->get_logger(), "Manual reset requested via service");

    m_SLAM->Reset();
    lost_frame_count_    = 0;
    last_tracking_state_ = -1;

    /// A resposta do Trigger é impressa pelo CLI do ros2 no terminal do operador.
    res->success = true;
    res->message = "ORB-SLAM3 reset successfully";
}

/**
 * @brief Frequẽncia de diagnóstico a 1 Hz — regista um resumo do estado do sistema.
 *
 * Imprime o estado de tracking, contagens de frames totais versus publicados,
 * número de pontos do mapa activos e a sequência actual de frames perdidos.
 * Permite ao operador verificar que o nó está activo e a rastrear correctamente
 * sem necessitar de subscrever qualquer tópico.
 *
 * Usa um temporizador de parede para que continue a disparar durante a reprodução
 * de rosbags a velocidade reduzida ou nula.
 */
void StereoSlamNode::DiagnosticsCallback()
{
    const int state    = m_SLAM->GetTrackingState();
    const int map_size = static_cast<int>(m_SLAM->GetTrackedMapPoints().size());

    /// Verificação de limites para evitar comportamento indefinido caso o ORB-SLAM3
    /// devolva um valor de estado inesperado numa versão futura.
    const char* state_names[] = {"NO_IMAGES", "NOT_INITIALIZED", "OK", "RECENTLY_LOST", "LOST"};
    const char* state_str = (state >= 0 && state <= 4) ? state_names[state] : "UNKNOWN";

    RCLCPP_INFO(this->get_logger(),
                "[Diag] State: %s | Frames: %d total / %d published | Map pts: %d | Lost streak: %d",
                state_str, total_frames_, published_frames_, map_size, lost_frame_count_);
}

/**
 * @brief Agrupa os pontos do mapa do ORB-SLAM3 numa mensagem @c sensor_msgs/PointCloud2.
 *
 * A nuvem resultante contém coordenadas XYZ em float32 no referencial do mundo.
 * O buffer de pontos é construído num @c std::vector<float> temporário e copiado
 * em bloco para o array de bytes da mensagem via @c std::memcpy, o que é mais
 * eficiente do que a atribuição byte a byte.
 *
 * @param map_points  Vector de Pointeres brutos para MapPoint do ORB-SLAM3.
 *                    Pontos nulos e pontos marcados como inválidos (culled) são
 *                    ignorados silenciosamente.
 * @param header      Header ROS a anexar à nuvem (fornece stamp e frame_id).
 * @return            Mensagem @c PointCloud2 completamente preenchida, pronta a publicar.
 */
sensor_msgs::msg::PointCloud2 StereoSlamNode::MapPointsToPointCloud2(
    const std::vector<ORB_SLAM3::MapPoint*> &map_points,
    const std_msgs::msg::Header &header)
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header       = header;
    cloud.height       = 1;
    cloud.is_dense     = false;
    cloud.is_bigendian = false;
    cloud.point_step   = 12;

    /**
     * @brief Bloco de descritores de campos.
     *
     * Cada entrada informa os consumidores de PointCloud2 sobre o nome, offset
     * em bytes dentro do registo de ponto, tipo de dados e número de elementos
     * de cada campo. Disposição: x @ byte 0, y @ byte 4, z @ byte 8.
     */
    cloud.fields.resize(3);
    const char* names[] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
        cloud.fields[i].name     = names[i];
        cloud.fields[i].offset   = static_cast<uint32_t>(i * 4);
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[i].count    = 1;
    }

    /// Pré-alocação para evitar realocações repetidas durante o preenchimento do buffer.
    std::vector<float> data;
    data.reserve(map_points.size() * 3);

    for (auto* mp : map_points) {
        /// Ignora Pointers nulos e pontos inválidos/eliminados.
        if (!mp || mp->isBad()) continue;

        const Eigen::Vector3f pos = mp->GetWorldPos();  ///< Posição 3D no referencial do mundo.
        data.push_back(pos.x());
        data.push_back(pos.y());
        data.push_back(pos.z());
    }

    cloud.width    = static_cast<uint32_t>(data.size() / 3);  ///< Número de pontos válidos.
    cloud.row_step = cloud.point_step * cloud.width;           ///< Total de bytes na linha.
    cloud.data.resize(cloud.row_step);

    /// Cópia em bloco do buffer de floats para o array de bytes — mais rápido do que atribuição elemento a elemento.
    std::memcpy(cloud.data.data(), data.data(), cloud.data.size());

    return cloud;
}