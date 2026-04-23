#ifndef __STEREO_INERTIAL_NODE_HPP__
#define __STEREO_INERTIAL_NODE_HPP__

#include "../nodes/slam-node-base.hpp"
#include "utility.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <string>
#include <vector>

using ImuMsg   = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

/**
 * @brief Nó ROS 2 para ORB-SLAM3 em modo Stereo-Inertial.
 *
 * Herda toda a infra-estrutura ROS 2 partilhada de SlamNodeBase (publishers,
 * TF, reset service, diagnósticos) e acrescenta apenas a lógica específica
 * deste modo de sensor: subscrições de IMU e imagens estéreo, sincronização
 * temporal e rectificação opcional.
 *
 * A posse do ORB_SLAM3::System e a sua terminação são geridas exclusivamente
 * por SlamNodeBase::~SlamNodeBase() — este destrutor não chama Shutdown() nem
 * SaveKeyFrameTrajectory() para evitar dupla terminação.
 */
class StereoInertialNode : public SlamNodeBase
{
public:
    /**
     * @brief Constrói o nó e inicializa subscrições, rectificação e thread de sync.
     *
     * @param pSLAM           Pointer não-possuidor para o ORB_SLAM3::System.
     * @param strSettingsFile Caminho para o ficheiro YAML de calibração.
     * @param strDoRectify    "true" para activar rectificação estéreo por software.
     * @param strDoEqual      "true" para activar equalização CLAHE.
     */
    StereoInertialNode(ORB_SLAM3::System* pSLAM,
                       const std::string &strSettingsFile,
                       const std::string &strDoRectify,
                       const std::string &strDoEqual);

    /**
     * @brief Sinaliza a thread de sincronização e aguarda a sua conclusão.
     *
     * Shutdown() e SaveKeyFrameTrajectoryTUM() são invocados por
     * SlamNodeBase::~SlamNodeBase() — não são repetidos aqui para evitar
     * dupla terminação.
     */
    ~StereoInertialNode();

private:
    // ------------------------------------------------------------------ //
    //  Callbacks de subscrição                                            //
    // ------------------------------------------------------------------ //

    /** @brief Enfileira uma medição de IMU. */
    void GrabImu(const ImuMsg::SharedPtr msg);

    /** @brief Enfileira a imagem esquerda mais recente (descarta anteriores). */
    void GrabImageLeft(const ImageMsg::SharedPtr msgLeft);

    /** @brief Enfileira a imagem direita mais recente (descarta anteriores). */
    void GrabImageRight(const ImageMsg::SharedPtr msgRight);

    // ------------------------------------------------------------------ //
    //  Utilitários internos                                               //
    // ------------------------------------------------------------------ //

    /**
     * @brief Converte uma ImageMsg ROS para cv::Mat em MONO8.
     * @param msg Mensagem de imagem ROS.
     * @return    Imagem em escala de cinzento (clone do buffer interno).
     */
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);

    /**
     * @brief Thread de sincronização: alinha imagens estéreo com medições IMU,
     *        invoca TrackStereo() e publica pose, odometria, TF e nuvem de pontos.
     *
     * Corre em loop até stopThread_ ser verdadeiro. O alinhamento temporal
     * admite uma diferença máxima de 10 ms entre os timestamps das imagens.
     */
    void SyncWithImu();

    // ------------------------------------------------------------------ //
    //  Subscrições ROS 2                                                  //
    // ------------------------------------------------------------------ //

    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgLeft_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgRight_;

    // ------------------------------------------------------------------ //
    //  Buffers e mutexes                                                  //
    // ------------------------------------------------------------------ //

    std::queue<ImuMsg::SharedPtr>   imuBuf_;
    std::mutex                      bufMutex_;

    std::queue<ImageMsg::SharedPtr> imgLeftBuf_;
    std::queue<ImageMsg::SharedPtr> imgRightBuf_;
    std::mutex                      bufMutexLeft_;
    std::mutex                      bufMutexRight_;

    // ------------------------------------------------------------------ //
    //  Thread de sincronização                                            //
    // ------------------------------------------------------------------ //

    std::thread          syncThread_;
    std::atomic<bool>    stopThread_{false};  ///< Paragem cooperativa da syncThread_.

    // ------------------------------------------------------------------ //
    //  Configuração de rectificação e equalização                         //
    // ------------------------------------------------------------------ //

    bool doRectify_ = false;
    bool doEqual_   = false;
    bool bClahe_    = false;

    cv::Mat              M1l_, M2l_;  ///< Mapas de rectificação — câmara esquerda.
    cv::Mat              M1r_, M2r_;  ///< Mapas de rectificação — câmara direita.
    cv::Ptr<cv::CLAHE>   clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
};

#endif // __STEREO_INERTIAL_NODE_HPP__