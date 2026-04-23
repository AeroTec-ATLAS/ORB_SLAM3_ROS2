#ifndef __RGBD_SLAM_NODE_HPP__
#define __RGBD_SLAM_NODE_HPP__

#include "../nodes/slam-node-base.hpp"
#include "utility.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.hpp>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

/**
 * @brief Nó ROS 2 para ORB-SLAM3 em modo RGB-D.
 *
 * Herda toda a infra-estrutura ROS 2 partilhada de SlamNodeBase (publishers
 * de pose, odometria, nuvem de pontos, TF, serviço de reset e diagnósticos)
 * e acrescenta apenas a lógica específica deste modo: subscrições sincronizadas
 * de imagem RGB e de profundidade e a invocação de TrackRGBD().
 *
 * A posse do ORB_SLAM3::System e a sua terminação são geridas exclusivamente
 * por SlamNodeBase::~SlamNodeBase() — este destrutor é trivial.
 */
class RgbdSlamNode : public SlamNodeBase
{
public:
    /**
     * @brief Constrói o nó e regista as subscrições sincronizadas RGB + Depth.
     * @param pSLAM Pointer não-possuidor para o ORB_SLAM3::System já construído.
     */
    explicit RgbdSlamNode(ORB_SLAM3::System* pSLAM);

    /**
     * @brief Destrutor trivial.
     *
     * Shutdown() e SaveKeyFrameTrajectoryTUM() são invocados por
     * SlamNodeBase::~SlamNodeBase() — não são repetidos aqui.
     */
    ~RgbdSlamNode() override = default;

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using ApproximateSyncPolicy =
        message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;

    /**
     * @brief Callback sincronizado: recebe um par RGB + Depth alinhado no tempo,
     *        invoca TrackRGBD() e publica pose, odometria, TF e nuvem de pontos.
     *
     * @param msgRGB   Imagem a cores (qualquer encoding — toCvShare trata da conversão).
     * @param msgDepth Imagem de profundidade (tipicamente 16UC1 em mm ou 32FC1 em m).
     */
    void GrabRGBD(const ImageMsg::SharedPtr msgRGB,
                  const ImageMsg::SharedPtr msgDepth);

    // ------------------------------------------------------------------ //
    //  Subscrições sincronizadas                                          //
    // ------------------------------------------------------------------ //

    std::shared_ptr<message_filters::Subscriber<ImageMsg>>              rgb_sub_;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>>              depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> sync_;
};

#endif // __RGBD_SLAM_NODE_HPP__