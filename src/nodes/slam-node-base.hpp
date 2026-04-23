#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <System.h>   ///< ORB_SLAM3::System

#include <cstring>    ///< std::memcpy
#include <memory>
#include <string>
#include <vector>

/**
 * @brief Nó base ROS 2 para SLAM com ORB-SLAM3.
 *
 * Contém toda a infra-estrutura ROS 2 partilhada independente do modo de sensor:
 * publishers de pose, odometria, nuvem de pontos e estado de tracking,
 * broadcaster de TF, serviço de reset e temporizador de diagnóstico.
 *
 * Classes derivadas (p. ex. StereoSlamNode, MonoSlamNode) herdam esta base e
 * implementam apenas a lógica de aquisição e processamento específica do seu
 * modo de sensor.
 */
class SlamNodeBase : public rclcpp::Node
{
public:
    /**
     * @brief Constrói a infra-estrutura ROS 2 partilhada.
     *
     * @param node_name   Nome do nó visível em `ros2 node list`.
     * @param pSLAM       Pointer para o ORB_SLAM3::System já construído.
     *                    A posse do objecto permanece em main().
     */
    SlamNodeBase(const std::string &node_name,
                 ORB_SLAM3::System* pSLAM);

    /**
     * @brief Destrutor — termina o ORB-SLAM3 e guarda a trajectória de keyframes.
     *
     * Shutdown() sinaliza todas as threads internas do ORB-SLAM3 para terminarem.
     * A trajectória é guardada no formato TUM (timestamp tx ty tz qx qy qz qw).
     */
    virtual ~SlamNodeBase();

protected:
    // ------------------------------------------------------------------ //
    //  Publicadores e broadcaster TF                                      //
    // ------------------------------------------------------------------ //

    /**
     * @brief Publica a pose da câmara como geometry_msgs/PoseStamped.
     * @param Twc    Transformação SE3 camera-in-world (Tcw já invertida pelo chamador).
     * @param header Header ROS partilhado (stamp + frame_id) para este frame.
     */
    void PublishPose(const Sophus::SE3f &Twc,
                     const std_msgs::msg::Header &header);

    /**
     * @brief Publica a pose da câmara como nav_msgs/Odometry.
     *
     * Inclui child_frame_id e covariância de pose 6x6, exigidos por
     * robot_localization (EKF) e MAVROS vision_pose/pose.
     *
     * @note Os valores de covariância são de referência — ajustar antes de
     *       integrar num EKF com base no erro de reprojecção real.
     *
     * @param Twc    Transformação SE3 camera-in-world.
     * @param header Header ROS partilhado para este frame.
     */
    void PublishOdometry(const Sophus::SE3f &Twc,
                         const std_msgs::msg::Header &header);

    /**
     * @brief Difunde a transformação dinâmica de TF map → camera_link.
     * @param Twc    Transformação SE3 camera-in-world.
     * @param header Header ROS partilhado para este frame (fornece o timestamp).
     */
    void PublishTF(const Sophus::SE3f &Twc,
                   const std_msgs::msg::Header &header);

    /**
     * @brief Converte os pontos do mapa rastreados para PointCloud2 e publica-os.
     *
     * Utiliza GetTrackedMapPoints() em vez de GetAllMapPoints() para limitar
     * a nuvem ao subconjunto visível no frame actual.
     *
     * @param header Header ROS partilhado para este frame.
     */
    void PublishMapPoints(const std_msgs::msg::Header &header);

    /**
     * @brief Publica o estado actual de tracking do ORB-SLAM3 como Int32.
     *
     * Mapeamento:
     *   0 — NO_IMAGES_YET | 1 — NOT_INITIALIZED | 2 — OK
     *   3 — RECENTLY_LOST | 4 — LOST
     *
     * @param state Valor inteiro devolvido por ORB_SLAM3::Tracking.
     */
    void PublishTrackingState(int state);

    // ------------------------------------------------------------------ //
    //  Gestão de falhas de tracking                                       //
    // ------------------------------------------------------------------ //

    /**
     * @brief Incrementa o contador de frames perdidos e dispara reinicialização
     *        quando este atinge kLostFrameThreshold.
     */
    void OnTrackingLost();

    /**
     * @brief Reinicia o sistema ORB-SLAM3 e limpa as variáveis de controlo de falhas.
     *
     * last_tracking_state_ é reposto a -1 (sentinela inválido) para que a
     * detecção de transições em classes derivadas dispare correctamente após o reset.
     */
    void AttemptReinitialization();

    // ------------------------------------------------------------------ //
    //  Utilitários                                                        //
    // ------------------------------------------------------------------ //

    /**
     * @brief Agrega pontos do mapa do ORB-SLAM3 numa mensagem PointCloud2.
     *
     * Pontos nulos ou marcados como inválidos (isBad()) são ignorados.
     * O buffer XYZ float32 é copiado em bloco via std::memcpy.
     *
     * @param map_points Vector de raw pointers para MapPoint.
     * @param header     Header ROS a anexar à nuvem.
     * @return           Mensagem PointCloud2 completamente preenchida.
     */
    sensor_msgs::msg::PointCloud2 MapPointsToPointCloud2(
        const std::vector<ORB_SLAM3::MapPoint*> &map_points,
        const std_msgs::msg::Header &header);

    // ------------------------------------------------------------------ //
    //  Membros protegidos — acessíveis pelas classes derivadas            //
    // ------------------------------------------------------------------ //

    ORB_SLAM3::System* m_SLAM;  ///< Pointer não-possuidor para o sistema SLAM.

    /// Número de frames consecutivos em estado LOST antes de disparar o reset.
    static constexpr int kLostFrameThreshold = 30;

    int  lost_frame_count_    = 0;   ///< Contador de frames LOST consecutivos.
    int  last_tracking_state_ = -1;  ///< Sentinela: -1 força a detecção da 1.ª transição.
    int  total_frames_        = 0;   ///< Total de pares de imagens recebidos.
    int  published_frames_    = 0;   ///< Frames com pose publicada (estado OK).

private:
    // ------------------------------------------------------------------ //
    //  Callbacks de serviço e temporizador (privados — não override)      //
    // ------------------------------------------------------------------ //

    /**
     * @brief Callback do serviço de reset manual pelo operador.
     *
     * Invocação:
     * @code
     *   ros2 service call /orb_slam3/reset std_srvs/srv/Trigger
     * @endcode
     */
    void ResetCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
              std::shared_ptr<std_srvs::srv::Trigger::Response>  res);

    /**
     * @brief Callback a 1 Hz — regista estado, contagens e mapa para diagnóstico.
     *
     * Usa temporizador de parede para continuar a disparar em reprodução
     * de rosbag a velocidade reduzida ou nula.
     */
    void DiagnosticsCallback();

    // ------------------------------------------------------------------ //
    //  Handles ROS 2                                                      //
    // ------------------------------------------------------------------ //

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    map_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr             state_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp::TimerBase::SharedPtr                       diagnostics_timer_;
};