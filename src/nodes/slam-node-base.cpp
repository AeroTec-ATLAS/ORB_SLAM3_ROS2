#include "slam-node-base.hpp"
#include "utility.hpp"

#include <cstring>

using std::placeholders::_1;
using std::placeholders::_2;

// -------------------------------------------------------------------------- //
//  Construtor                                                                 //
// -------------------------------------------------------------------------- //

/**
 * @brief Constrói a infra-estrutura ROS 2 partilhada por todos os modos de sensor.
 *
 * Cria publishers de pose, odometria, nuvem de pontos e estado de tracking,
 * o broadcaster de TF dinâmico, o serviço de reset e o temporizador de diagnóstico.
 * Nenhuma subscrição é criada aqui — essa responsabilidade pertence às classes
 * derivadas, que conhecem os tópicos e tipos de mensagem do seu sensor.
 *
 * @param node_name  Nome do nó visível em `ros2 node list`.
 * @param pSLAM      Pointer não-possuidor para o ORB_SLAM3::System já construído.
 */
SlamNodeBase::SlamNodeBase(const std::string &node_name,
                            ORB_SLAM3::System* pSLAM)
:   Node(node_name),
    m_SLAM(pSLAM)
{
    /// Profundidade de fila de 10 é adequada para tópicos de pose e nuvem a 30 Hz.
    pose_pub_  = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                     "orb_slam3/camera_pose", 10);
    odom_pub_  = this->create_publisher<nav_msgs::msg::Odometry>(
                     "orb_slam3/odometry", 10);
    map_pub_   = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                     "orb_slam3/map_points", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::Int32>(
                     "orb_slam3/tracking_state", 10);

    /// Broadcaster de TF dinâmico: publica map -> camera_link em cada frame com
    /// tracking válido, permitindo que o RViz localize a câmara sem subscrever
    /// um tópico de pose separado.
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    /// Serviço para reset manual sem reiniciar o nó:
    /// @code ros2 service call /orb_slam3/reset std_srvs/srv/Trigger @endcode
    reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "orb_slam3/reset",
        std::bind(&SlamNodeBase::ResetCallback, this, _1, _2));

    /// Temporizador de parede para que os diagnósticos continuem a disparar
    /// mesmo durante reprodução de rosbag a velocidade reduzida ou nula.
    diagnostics_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SlamNodeBase::DiagnosticsCallback, this));

    RCLCPP_INFO(this->get_logger(), "SlamNodeBase ready");
}

// -------------------------------------------------------------------------- //
//  Destrutor                                                                  //
// -------------------------------------------------------------------------- //

/**
 * @brief Termina o ORB-SLAM3 e guarda a trajectória de keyframes.
 *
 * Shutdown() sinaliza todas as threads internas (tracking, mapeamento local,
 * fecho de ciclos) para terminarem antes de o objecto ser destruído.
 * O ficheiro é escrito em formato TUM, compatível com as ferramentas de
 * avaliação do benchmark TUM RGB-D.
 */
SlamNodeBase::~SlamNodeBase()
{
    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    RCLCPP_INFO(this->get_logger(), "Trajectory saved. Node destroyed.");
}

// -------------------------------------------------------------------------- //
//  Publicadores                                                               //
// -------------------------------------------------------------------------- //

/**
 * @brief Publica a pose da câmara como geometry_msgs/PoseStamped.
 *
 * Alternativa leve à Odometry — adequada para visualização no RViz e
 * consumidores que não necessitam de covariância.
 *
 * @param Twc    Transformação SE3 camera-in-world (Tcw já invertida pelo chamador).
 * @param header Header ROS partilhado (stamp + frame_id) para este frame.
 */
void SlamNodeBase::PublishPose(const Sophus::SE3f &Twc,
                                const std_msgs::msg::Header &header)
{
    const Eigen::Vector3f    t = Twc.translation();
    const Eigen::Quaternionf q = Twc.unit_quaternion();  ///< Sempre normalizado.

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
 * @brief Publica a pose da câmara como nav_msgs/Odometry.
 *
 * Inclui child_frame_id e covariância de pose 6x6, ambos exigidos por
 * robot_localization (EKF) e MAVROS vision_pose/pose.
 *
 * @note A diagonal da covariância contém valores de referência. Deverão ser
 *       ajustados com base no erro de reprojecção real antes de integrar num EKF.
 *
 * @param Twc    Transformação SE3 camera-in-world.
 * @param header Header ROS partilhado para este frame.
 */
void SlamNodeBase::PublishOdometry(const Sophus::SE3f &Twc,
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
     * @brief Covariância de pose 6x6 (row-major, 36 elementos).
     *
     * Índices: [x, y, z, roll, pitch, yaw].
     * Apenas a diagonal (variâncias) é preenchida; correlações cruzadas ficam
     * a zero — aproximação razoável para saídas de VIO.
     * Unidades: m² para posição, rad² para orientação.
     *
     * @warning Valores de referência — substituir por valores medidos antes de
     *          integrar num EKF de produção.
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
 * @brief Difunde a transformação dinâmica de TF map → camera_link.
 *
 * Sem esta difusão, o RViz não consegue renderizar a nuvem de pontos na posição
 * correcta e tf2::doTransform falhará com um erro de lookup.
 *
 * @param Twc    Transformação SE3 camera-in-world.
 * @param header Header ROS partilhado (fornece o timestamp).
 */
void SlamNodeBase::PublishTF(const Sophus::SE3f &Twc,
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
 * Usa GetTrackedMapPoints() em vez de GetAllMapPoints() para limitar a nuvem
 * ao subconjunto visível no frame actual, mantendo a mensagem delimitada em tamanho.
 *
 * @param header Header ROS partilhado para este frame.
 */
void SlamNodeBase::PublishMapPoints(const std_msgs::msg::Header &header)
{
    const auto map_points = m_SLAM->GetTrackedMapPoints();
    if (!map_points.empty()) {
        map_pub_->publish(MapPointsToPointCloud2(map_points, header));
    }
}

/**
 * @brief Publica o estado actual de tracking do ORB-SLAM3 como Int32.
 *
 * Mapeamento dos valores:
 *   0 — NO_IMAGES_YET | 1 — NOT_INITIALIZED | 2 — OK
 *   3 — RECENTLY_LOST | 4 — LOST
 *
 * @param state Valor inteiro do estado devolvido por ORB_SLAM3::Tracking.
 */
void SlamNodeBase::PublishTrackingState(int state)
{
    std_msgs::msg::Int32 msg;
    msg.data = state;
    state_pub_->publish(msg);
}

// -------------------------------------------------------------------------- //
//  Gestão de falhas de tracking                                               //
// -------------------------------------------------------------------------- //

/**
 * @brief Incrementa o contador de frames perdidos e dispara reinicialização
 *        quando este atinge kLostFrameThreshold.
 *
 * O aviso é limitado a uma mensagem por cada 2 segundos para evitar a saturação
 * do log à cadência da câmara (30 linhas/segundo sem limitação).
 */
void SlamNodeBase::OnTrackingLost()
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
 * @brief Reinicia o sistema ORB-SLAM3 e limpa as variáveis de controlo de falhas.
 *
 * Após Reset(), o ORB-SLAM3 volta a NOT_INITIALIZED e reinicializa-se a partir
 * dos próximos frames recebidos.
 *
 * last_tracking_state_ é reposto a -1 (sentinela) para que a detecção de
 * transições nas classes derivadas dispare correctamente após o reset.
 */
void SlamNodeBase::AttemptReinitialization()
{
    RCLCPP_WARN(this->get_logger(),
                "Tracking lost for %d frames — triggering automatic reset",
                lost_frame_count_);

    m_SLAM->Reset();
    lost_frame_count_    = 0;
    last_tracking_state_ = -1;

    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 reset complete. Waiting for reinitialisation.");
}

// -------------------------------------------------------------------------- //
//  Callbacks privados                                                         //
// -------------------------------------------------------------------------- //

/**
 * @brief Callback do serviço de reset manual pelo operador.
 *
 * Reinicia o ORB-SLAM3 sem reiniciar o nó — útil em testes de campo quando
 * o UAV pode reposicionar-se numa área com maior densidade de features.
 *
 * @param req  Não utilizado — std_srvs/Trigger não transporta dados no pedido.
 * @param res  Resposta preenchida com indicador de sucesso e mensagem de estado.
 */
void SlamNodeBase::ResetCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>  /*req*/,
          std::shared_ptr<std_srvs::srv::Trigger::Response>   res)
{
    RCLCPP_WARN(this->get_logger(), "Manual reset requested via service");

    m_SLAM->Reset();
    lost_frame_count_    = 0;
    last_tracking_state_ = -1;

    res->success = true;
    res->message = "ORB-SLAM3 reset successfully";
}

/**
 * @brief Callback a 1 Hz — regista estado, contagens e mapa para diagnóstico.
 *
 * Permite ao operador verificar que o nó está activo e a rastrear correctamente
 * sem necessitar de subscrever qualquer tópico.
 */
void SlamNodeBase::DiagnosticsCallback()
{
    const int state    = m_SLAM->GetTrackingState();
    const int map_size = static_cast<int>(m_SLAM->GetTrackedMapPoints().size());

    /// Verificação de limites para evitar UB caso o ORB-SLAM3 devolva um valor
    /// de estado inesperado numa versão futura.
    const char* state_names[] = {"NO_IMAGES", "NOT_INITIALIZED", "OK", "RECENTLY_LOST", "LOST"};
    const char* state_str = (state >= 0 && state <= 4) ? state_names[state] : "UNKNOWN";

    RCLCPP_INFO(this->get_logger(),
                "[Diag] State: %s | Frames: %d total / %d published | Map pts: %d | Lost streak: %d",
                state_str, total_frames_, published_frames_, map_size, lost_frame_count_);
}

// -------------------------------------------------------------------------- //
//  Utilitários                                                                //
// -------------------------------------------------------------------------- //

/**
 * @brief Agrega pontos do mapa do ORB-SLAM3 numa mensagem PointCloud2.
 *
 * A nuvem contém coordenadas XYZ em float32 no referencial do mundo.
 * O buffer é construído num std::vector<float> temporário e copiado em bloco
 * para o array de bytes da mensagem via std::memcpy.
 *
 * @param map_points  Vector de raw pointers para MapPoint.
 *                    Pontos nulos e isBad() são ignorados silenciosamente.
 * @param header      Header ROS a anexar à nuvem.
 * @return            Mensagem PointCloud2 completamente preenchida.
 */
sensor_msgs::msg::PointCloud2 SlamNodeBase::MapPointsToPointCloud2(
    const std::vector<ORB_SLAM3::MapPoint*> &map_points,
    const std_msgs::msg::Header &header)
{
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header       = header;
    cloud.height       = 1;
    cloud.is_dense     = false;
    cloud.is_bigendian = false;
    cloud.point_step   = 12;  ///< 3 floats × 4 bytes.

    /**
     * @brief Descritores de campos XYZ.
     *
     * Disposição: x @ byte 0, y @ byte 4, z @ byte 8.
     */
    cloud.fields.resize(3);
    const char* names[] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
        cloud.fields[i].name     = names[i];
        cloud.fields[i].offset   = static_cast<uint32_t>(i * 4);
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[i].count    = 1;
    }

    /// Pré-alocação para evitar realocações durante o preenchimento.
    std::vector<float> data;
    data.reserve(map_points.size() * 3);

    for (auto* mp : map_points) {
        if (!mp || mp->isBad()) continue;
        const Eigen::Vector3f pos = mp->GetWorldPos();
        data.push_back(pos.x());
        data.push_back(pos.y());
        data.push_back(pos.z());
    }

    cloud.width    = static_cast<uint32_t>(data.size() / 3);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step);

    std::memcpy(cloud.data.data(), data.data(), cloud.data.size());

    return cloud;
}