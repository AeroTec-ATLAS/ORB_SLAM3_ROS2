#include "stereo-inertial-node.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using std::placeholders::_1;

// -------------------------------------------------------------------------- //
//  Construtor                                                                 //
// -------------------------------------------------------------------------- //

/**
 * @brief Constrói o nó Stereo-Inertial delegando a infra-estrutura ROS 2
 *        ao SlamNodeBase e inicializando as subscrições e a thread de sync.
 *
 * O nome do nó é fixo em "ORB_SLAM3_ROS2" para compatibilidade com launches
 * existentes. A rectificação por software é inicializada a partir do ficheiro
 * de calibração apenas quando strDoRectify == "true".
 *
 * @param pSLAM           Pointer não-possuidor para o ORB_SLAM3::System.
 * @param strSettingsFile Caminho para o ficheiro YAML de calibração da câmara.
 * @param strDoRectify    "true" para activar a rectificação estéreo por software.
 * @param strDoEqual      "true" para activar a equalização CLAHE.
 */
StereoInertialNode::StereoInertialNode(ORB_SLAM3::System* pSLAM,
                                       const std::string &strSettingsFile,
                                       const std::string &strDoRectify,
                                       const std::string &strDoEqual)
:   SlamNodeBase("ORB_SLAM3_ROS2", pSLAM)
{
    /// Análise das flags booleanas passadas como strings da linha de comando.
    {
        std::stringstream ss_rec(strDoRectify);
        ss_rec >> std::boolalpha >> doRectify_;
    }
    {
        std::stringstream ss_eq(strDoEqual);
        ss_eq >> std::boolalpha >> doEqual_;
    }

    bClahe_ = doEqual_;

    RCLCPP_INFO(this->get_logger(), "Rectify: %s | Equalise: %s",
                doRectify_ ? "true" : "false",
                doEqual_   ? "true" : "false");

    /// Inicialização dos mapas de rectificação estéreo a partir dos parâmetros
    /// de calibração no ficheiro YAML. Abortamos com assert() se os parâmetros
    /// estiverem ausentes, pois continuar sem eles produziria resultados silenciosamente
    /// errados.
    if (doRectify_)
    {
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            RCLCPP_FATAL(this->get_logger(),
                         "Cannot open settings file: %s", strSettingsFile.c_str());
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"]  >> K_l;
        fsSettings["RIGHT.K"] >> K_r;
        fsSettings["LEFT.P"]  >> P_l;
        fsSettings["RIGHT.P"] >> P_r;
        fsSettings["LEFT.R"]  >> R_l;
        fsSettings["RIGHT.R"] >> R_r;
        fsSettings["LEFT.D"]  >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        const int rows_l = fsSettings["LEFT.height"];
        const int cols_l = fsSettings["LEFT.width"];
        const int rows_r = fsSettings["RIGHT.height"];
        const int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() ||
            R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || cols_l == 0 || rows_r == 0 || cols_r == 0)
        {
            RCLCPP_FATAL(this->get_logger(),
                         "Stereo rectification parameters missing in settings file.");
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l,
                                    P_l.rowRange(0,3).colRange(0,3),
                                    cv::Size(cols_l, rows_l),
                                    CV_32F, M1l_, M2l_);

        cv::initUndistortRectifyMap(K_r, D_r, R_r,
                                    P_r.rowRange(0,3).colRange(0,3),
                                    cv::Size(cols_r, rows_r),
                                    CV_32F, M1r_, M2r_);
    }

    /// Subscrições — profundidade de fila generosa para o IMU (1000) porque as
    /// medições chegam a 200–400 Hz e devem ser consumidas em bloco pela thread
    /// de sincronização sem perda.
    subImu_      = this->create_subscription<ImuMsg>(
        "imu", 1000, std::bind(&StereoInertialNode::GrabImu, this, _1));
    subImgLeft_  = this->create_subscription<ImageMsg>(
        "camera/left",  100, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    subImgRight_ = this->create_subscription<ImageMsg>(
        "camera/right", 100, std::bind(&StereoInertialNode::GrabImageRight, this, _1));

    syncThread_ = std::thread(&StereoInertialNode::SyncWithImu, this);

    RCLCPP_INFO(this->get_logger(), "StereoInertialNode ready");
}

// -------------------------------------------------------------------------- //
//  Destrutor                                                                  //
// -------------------------------------------------------------------------- //

/**
 * @brief Sinaliza a thread de sincronização e aguarda a sua conclusão.
 *
 * Shutdown() e SaveKeyFrameTrajectoryTUM() são chamados por
 * SlamNodeBase::~SlamNodeBase(), portanto não são repetidos aqui.
 */
StereoInertialNode::~StereoInertialNode()
{
    stopThread_ = true;
    if (syncThread_.joinable()) {
        syncThread_.join();
    }
}

// -------------------------------------------------------------------------- //
//  Callbacks de subscrição                                                    //
// -------------------------------------------------------------------------- //

/**
 * @brief Enfileira uma medição de IMU no buffer partilhado.
 *
 * O mutex protege o acesso concorrente entre o executor ROS 2 (produtor)
 * e a thread de sincronização (consumidor).
 */
void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(bufMutex_);
    imuBuf_.push(msg);
}

/**
 * @brief Substitui a imagem esquerda pendente pela mais recente.
 *
 * Mantém apenas um frame por vez: se já existir uma imagem não consumida,
 * é descartada. Isto evita atrasos acumulados quando o processamento é mais
 * lento do que a cadência da câmara.
 */
void StereoInertialNode::GrabImageLeft(const ImageMsg::SharedPtr msgLeft)
{
    std::lock_guard<std::mutex> lock(bufMutexLeft_);
    if (!imgLeftBuf_.empty()) {
        imgLeftBuf_.pop();
    }
    imgLeftBuf_.push(msgLeft);
}

/**
 * @brief Substitui a imagem direita pendente pela mais recente.
 *
 * Comportamento idêntico a GrabImageLeft().
 */
void StereoInertialNode::GrabImageRight(const ImageMsg::SharedPtr msgRight)
{
    std::lock_guard<std::mutex> lock(bufMutexRight_);
    if (!imgRightBuf_.empty()) {
        imgRightBuf_.pop();
    }
    imgRightBuf_.push(msgRight);
}

// -------------------------------------------------------------------------- //
//  Utilitários internos                                                       //
// -------------------------------------------------------------------------- //

/**
 * @brief Converte uma ImageMsg ROS para cv::Mat em MONO8.
 *
 * O clone() garante que o cv::Mat é independente do buffer da mensagem ROS,
 * permitindo que a mensagem seja libertada pelo executor depois da conversão.
 *
 * @param msg Mensagem de imagem ROS.
 * @return    Imagem em escala de cinzento (clone do buffer interno).
 */
cv::Mat StereoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return {};
    }
    return cv_ptr->image.clone();
}

// -------------------------------------------------------------------------- //
//  Thread de sincronização                                                    //
// -------------------------------------------------------------------------- //

/**
 * @brief Alinha imagens estéreo com medições de IMU e invoca TrackStereo().
 *
 * ### Lógica de alinhamento temporal
 * 1. Descarta imagens direitas antigas até o timestamp direito aproximar-se
 *    do esquerdo (dentro de maxTimeDiff = 10 ms).
 * 2. Faz o mesmo para imagens esquerdas.
 * 3. Rejeita o par se a diferença ainda exceder maxTimeDiff.
 * 4. Aguarda que o buffer de IMU contenha medições até ao instante da imagem.
 * 5. Extrai todas as medições de IMU até tImLeft e passa-as ao tracker.
 *
 * ### Publicação de resultados
 * Após TrackStereo(), publica pose, odometria, TF e nuvem de pontos apenas
 * quando o tracking está em estado OK (2). Transições de e para outros estados
 * são registadas e o contador de frames perdidos é gerido por SlamNodeBase.
 *
 * A thread dorme 1 ms entre iterações para ceder tempo de CPU sem introduzir
 * latência significativa a 30 Hz.
 */
void StereoInertialNode::SyncWithImu()
{
    constexpr double maxTimeDiff = 0.01;  ///< 10 ms de tolerância de sincronização.

    while (!stopThread_)
    {
        // ---------------------------------------------------------------- //
        //  Verificação de disponibilidade de dados                          //
        // ---------------------------------------------------------------- //

        {
            /// Leitura rápida sem lock — se algum buffer estiver vazio não vale
            /// a pena adquirir os mutexes.
            const bool haveLeft  = !imgLeftBuf_.empty();
            const bool haveRight = !imgRightBuf_.empty();
            const bool haveImu   = !imuBuf_.empty();

            if (!haveLeft || !haveRight || !haveImu) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
        }

        // ---------------------------------------------------------------- //
        //  Leitura de timestamps sem consumir os buffers                   //
        // ---------------------------------------------------------------- //

        double tImLeft, tImRight;
        {
            std::lock_guard<std::mutex> lL(bufMutexLeft_);
            tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
        }
        {
            std::lock_guard<std::mutex> lR(bufMutexRight_);
            tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
        }

        // ---------------------------------------------------------------- //
        //  Descarte de imagens direitas atrasadas                          //
        // ---------------------------------------------------------------- //

        {
            std::lock_guard<std::mutex> lR(bufMutexRight_);
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf_.size() > 1)
            {
                imgRightBuf_.pop();
                tImRight = Utility::StampToSec(imgRightBuf_.front()->header.stamp);
            }
        }

        // ---------------------------------------------------------------- //
        //  Descarte de imagens esquerdas atrasadas                         //
        // ---------------------------------------------------------------- //

        {
            std::lock_guard<std::mutex> lL(bufMutexLeft_);
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf_.size() > 1)
            {
                imgLeftBuf_.pop();
                tImLeft = Utility::StampToSec(imgLeftBuf_.front()->header.stamp);
            }
        }

        // ---------------------------------------------------------------- //
        //  Rejeição do par se a diferença temporal ainda for excessiva     //
        // ---------------------------------------------------------------- //

        if (std::abs(tImLeft - tImRight) > maxTimeDiff)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Stereo timestamp difference too large: %.4f s",
                                 std::abs(tImLeft - tImRight));
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // ---------------------------------------------------------------- //
        //  Aguarda medições de IMU até ao instante da imagem               //
        // ---------------------------------------------------------------- //

        {
            std::lock_guard<std::mutex> lI(bufMutex_);
            if (imuBuf_.empty() ||
                tImLeft > Utility::StampToSec(imuBuf_.back()->header.stamp))
            {
                /// O IMU ainda não alcançou o timestamp da imagem — aguarda.
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
        }

        // ---------------------------------------------------------------- //
        //  Extracção das imagens dos buffers                               //
        // ---------------------------------------------------------------- //

        cv::Mat imLeft, imRight;
        {
            std::lock_guard<std::mutex> lL(bufMutexLeft_);
            imLeft = GetImage(imgLeftBuf_.front());
            imgLeftBuf_.pop();
        }
        {
            std::lock_guard<std::mutex> lR(bufMutexRight_);
            imRight = GetImage(imgRightBuf_.front());
            imgRightBuf_.pop();
        }

        if (imLeft.empty() || imRight.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // ---------------------------------------------------------------- //
        //  Extracção das medições de IMU até tImLeft                      //
        // ---------------------------------------------------------------- //

        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        {
            std::lock_guard<std::mutex> lI(bufMutex_);
            while (!imuBuf_.empty() &&
                   Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImLeft)
            {
                const double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                const cv::Point3f acc(
                    static_cast<float>(imuBuf_.front()->linear_acceleration.x),
                    static_cast<float>(imuBuf_.front()->linear_acceleration.y),
                    static_cast<float>(imuBuf_.front()->linear_acceleration.z));
                const cv::Point3f gyr(
                    static_cast<float>(imuBuf_.front()->angular_velocity.x),
                    static_cast<float>(imuBuf_.front()->angular_velocity.y),
                    static_cast<float>(imuBuf_.front()->angular_velocity.z));
                vImuMeas.emplace_back(acc, gyr, t);
                imuBuf_.pop();
            }
        }

        // ---------------------------------------------------------------- //
        //  Pré-processamento de imagem (CLAHE + rectificação)             //
        // ---------------------------------------------------------------- //

        if (bClahe_)
        {
            clahe_->apply(imLeft,  imLeft);
            clahe_->apply(imRight, imRight);
        }

        if (doRectify_)
        {
            cv::remap(imLeft,  imLeft,  M1l_, M2l_, cv::INTER_LINEAR);
            cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
        }

        // ---------------------------------------------------------------- //
        //  Tracking                                                         //
        // ---------------------------------------------------------------- //

        ++total_frames_;

        const Sophus::SE3f Tcw = m_SLAM->TrackStereo(imLeft, imRight,
                                                      tImLeft, vImuMeas);
        const int state = m_SLAM->GetTrackingState();

        // ---------------------------------------------------------------- //
        //  Registo de transições de estado                                 //
        // ---------------------------------------------------------------- //

        if (state != last_tracking_state_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Tracking state changed: %d → %d",
                        last_tracking_state_, state);
            last_tracking_state_ = state;
        }

        // ---------------------------------------------------------------- //
        //  Publicação de resultados (apenas com tracking OK)               //
        // ---------------------------------------------------------------- //

        /// Estado 2 == ORB_SLAM3::Tracking::OK
        if (state == 2)
        {
            lost_frame_count_ = 0;
            ++published_frames_;

            const Sophus::SE3f Twc = Tcw.inverse();

            std_msgs::msg::Header header;
            header.stamp    = this->now();
            header.frame_id = "map";

            PublishPose(Twc, header);
            PublishOdometry(Twc, header);
            PublishTF(Twc, header);
            PublishMapPoints(header);
        }
        else
        {
            OnTrackingLost();
        }

        PublishTrackingState(state);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}