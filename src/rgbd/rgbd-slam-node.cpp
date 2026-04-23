#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

// -------------------------------------------------------------------------- //
//  Construtor                                                                 //
// -------------------------------------------------------------------------- //

/**
 * @brief Constrói o nó RGB-D delegando a infra-estrutura ROS 2 ao SlamNodeBase
 *        e registando as subscrições sincronizadas de RGB e profundidade.
 *
 * A política ApproximateTime com fila de 10 tolera desfasamentos temporais
 * pequenos entre os dois streams, comuns em câmaras RGB-D de consumo (RealSense,
 * Azure Kinect) onde RGB e Depth são publicados em tópicos separados.
 *
 * @param pSLAM Pointer não-possuidor para o ORB_SLAM3::System já construído.
 */
RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   SlamNodeBase("ORB_SLAM3_ROS2", pSLAM)
{
    rgb_sub_   = std::make_shared<message_filters::Subscriber<ImageMsg>>(
                     this, "/camera/color/image_raw");
    depth_sub_ = std::make_shared<message_filters::Subscriber<ImageMsg>>(
                     this, "/camera/depth/image_rect_raw");

    sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(
                ApproximateSyncPolicy(10), *rgb_sub_, *depth_sub_);

    sync_->registerCallback(&RgbdSlamNode::GrabRGBD, this);

    RCLCPP_INFO(this->get_logger(), "RgbdSlamNode ready");
}

// -------------------------------------------------------------------------- //
//  Callback sincronizado                                                      //
// -------------------------------------------------------------------------- //

/**
 * @brief Processa um par de imagens RGB + Depth temporalmente alinhadas.
 *
 * ### Pipeline
 * 1. Converte as mensagens ROS para cv::Mat via cv_bridge (sem cópia desnecessária).
 * 2. Invoca TrackRGBD() com o timestamp da imagem RGB como referência temporal.
 * 3. Publica pose, odometria, TF e nuvem de pontos quando o tracking está em
 *    estado OK (2); caso contrário delega em OnTrackingLost().
 * 4. Publica sempre o estado de tracking para monitorização externa.
 *
 * @param msgRGB   Imagem a cores recebida de /camera/color/image_raw.
 * @param msgDepth Imagem de profundidade recebida de /camera/depth/image_rect_raw.
 */
void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB,
                              const ImageMsg::SharedPtr msgDepth)
{
    // ------------------------------------------------------------------ //
    //  Conversão ROS → OpenCV                                            //
    // ------------------------------------------------------------------ //

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (RGB): %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrDepth;
    try
    {
        cv_ptrDepth = cv_bridge::toCvShare(msgDepth);
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (Depth): %s", e.what());
        return;
    }

    // ------------------------------------------------------------------ //
    //  Tracking                                                           //
    // ------------------------------------------------------------------ //

    ++total_frames_;

    const double timestamp = Utility::StampToSec(msgRGB->header.stamp);

    const Sophus::SE3f Tcw = m_SLAM->TrackRGBD(
        cv_ptrRGB->image, cv_ptrDepth->image, timestamp);

    const int state = m_SLAM->GetTrackingState();

    // ------------------------------------------------------------------ //
    //  Registo de transições de estado                                    //
    // ------------------------------------------------------------------ //

    if (state != last_tracking_state_)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Tracking state changed: %d → %d",
                    last_tracking_state_, state);
        last_tracking_state_ = state;
    }

    // ------------------------------------------------------------------ //
    //  Publicação de resultados (apenas com tracking OK)                  //
    // ------------------------------------------------------------------ //

    /// Estado 2 == ORB_SLAM3::Tracking::OK
    if (state == 2)
    {
        lost_frame_count_ = 0;
        ++published_frames_;

        const Sophus::SE3f Twc = Tcw.inverse();

        std_msgs::msg::Header header;
        header.stamp    = msgRGB->header.stamp;  ///< Reutiliza o stamp original.
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
}