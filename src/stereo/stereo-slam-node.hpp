#pragma once

#include "../nodes/slam-node-base.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/core.hpp>

#include <string>
#include <memory>

/**
 * @brief Nó ROS 2 para SLAM estéreo com ORB-SLAM3.
 *
 * Herda toda a infra-estrutura ROS 2 partilhada de SlamNodeBase e acrescenta
 * apenas a lógica específica do modo estéreo:
 *   - Subscrições sincronizadas das câmaras esquerda e direita.
 *   - Cálculo opcional dos mapas de rectificação via cv::initUndistortRectifyMap.
 *   - Callback GrabStereo() que alimenta o ORB-SLAM3 e delega a publicação
 *     na classe base.
 */
class StereoSlamNode : public SlamNodeBase
{
public:
    /**
     * @brief Constrói o nó estéreo.
     *
     * Executado pela ordem:
     *   1. Interpreta strDoRectify e converte-o em booleano.
     *   2. Se necessário, lê os parâmetros de calibração de strSettingsFile e
     *      pré-calcula os mapas de undistort/rectify (evita recalculá-los por frame).
     *   3. Cria as subscrições e o sincronizador ApproximateTime.
     *
     * A infra-estrutura ROS 2 partilhada (publishers, TF, serviço, temporizador)
     * é inicializada pelo construtor de SlamNodeBase.
     *
     * @param pSLAM           Pointer para o ORB_SLAM3::System já construído.
     * @param strSettingsFile Caminho absoluto para o ficheiro YAML de configuração.
     * @param strDoRectify    String "true" ou "false" que controla a rectificação.
     *
     * @throws std::runtime_error Se o ficheiro de configuração não puder ser aberto
     *         ou se algum parâmetro de calibração obrigatório estiver ausente.
     */
    StereoSlamNode(ORB_SLAM3::System*  pSLAM,
                   const std::string  &strSettingsFile,
                   const std::string  &strDoRectify);

    ~StereoSlamNode() override = default;

private:
    // ------------------------------------------------------------------ //
    //  Tipos ROS 2 internos                                               //
    // ------------------------------------------------------------------ //

    using ImageMsg           = sensor_msgs::msg::Image;
    using approximate_sync_policy = message_filters::sync_policies::ApproximateTime<
                                        ImageMsg, ImageMsg>;

    // ------------------------------------------------------------------ //
    //  Inicialização                                                      //
    // ------------------------------------------------------------------ //

    /**
     * @brief Lê os parâmetros estéreo do ficheiro YAML e pré-calcula os mapas
     *        de rectificação para as câmaras esquerda e direita.
     *
     * Chamado pelo construtor apenas quando doRectify == true.
     * Os mapas M1l/M2l e M1r/M2r ficam guardados como membros para reutilização
     * em cada invocação de GrabStereo().
     *
     * @param strSettingsFile Caminho para o ficheiro YAML de calibração.
     * @throws std::runtime_error Se o ficheiro não puder ser aberto ou se algum
     *         parâmetro de calibração estiver ausente.
     */
    void InitRectificationMaps(const std::string &strSettingsFile);

    // ------------------------------------------------------------------ //
    //  Callback do sincronizador                                          //
    // ------------------------------------------------------------------ //

    /**
     * @brief Processa um par de imagens estéreo sincronizadas.
     *
     * Passos:
     *   1. Converte as mensagens ROS para cv::Mat via cv_bridge.
     *   2. Aplica os mapas de rectificação, se doRectify == true.
     *   3. Invoca ORB_SLAM3::System::TrackStereo() para obter Tcw.
     *   4. Gere transições de estado (OK / RECENTLY_LOST / LOST) e dispara
     *      reinicialização automática quando necessário.
     *   5. Delega a publicação de pose, odometria, TF, mapa e estado
     *      nos métodos de SlamNodeBase.
     *
     * @param msgLeft  Mensagem de imagem da câmara esquerda.
     * @param msgRight Mensagem de imagem da câmara direita.
     */
    void GrabStereo(const ImageMsg::SharedPtr msgLeft,
                    const ImageMsg::SharedPtr msgRight);

    // ------------------------------------------------------------------ //
    //  Estado estéreo                                                     //
    // ------------------------------------------------------------------ //

    bool doRectify = false;  ///< Indica se a rectificação deve ser aplicada por frame.

    /// Mapas de undistort/rectify pré-calculados para a câmara esquerda.
    cv::Mat M1l, M2l;

    /// Mapas de undistort/rectify pré-calculados para a câmara direita.
    cv::Mat M1r, M2r;

    /// Ponteiros de cv_bridge reutilizados entre frames para evitar alocações repetidas.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;

    // ------------------------------------------------------------------ //
    //  Subscrições e sincronizador                                        //
    // ------------------------------------------------------------------ //

    std::shared_ptr<message_filters::Subscriber<ImageMsg>> left_sub;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> right_sub;

    std::shared_ptr<
        message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;
};