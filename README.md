# SLAM para Câmara RealSense

## 1. Instalação do ORB-SLAM3 e Wrapper ROS2 para Ubuntu 24.04

Este documento descreve a sequência de instalação do ORB-SLAM3 e do seu wrapper ROS2 em Ubuntu 24.04.

### 1.1 Passo 1: Clonar e Instalar ORB-SLAM3

Clonar o repositório ORB-SLAM3 e seguir as instruções presentes em:

**Repositório:** https://github.com/AeroTec-ATLAS/ORB-SLAM3-STEREO-FIXED-for-ubuntu-24.04-LTS

```bash
cd ~
git clone https://github.com/AeroTec-ATLAS/ORB-SLAM3-STEREO-FIXED-for-ubuntu-24.04-LTS
cd ORB-SLAM3-STEREO-FIXED-for-ubuntu-24.04-LTS

chmod +x build.sh
./build.sh
```

### 1.2 Passo 2: Instalação Global do Sophus

**Importante:** Este passo é obrigatório para a integração com ROS2 e não está documentado no repositório anterior.

```bash
cd $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Thirdparty/Sophus/build
sudo make install
```

**Para remover posteriormente, caso necessário:**

```bash
sudo rm -rf /usr/local/include/sophus
sudo rm -rf /usr/local/share/sophus
```

### 1.3 Passo 3: Clonar e Instalar Wrapper ROS2

**Repositório:** https://github.com/AeroTec-ATLAS/ORB_SLAM3_ROS2

```bash
cd ~
git clone https://github.com/AeroTec-ATLAS/ORB_SLAM3_ROS2
# Seguir as instruções do README do repositório
```

### 1.4 Passo 4: Configuração Adicional de Variáveis de Ambiente

Adicionar as seguintes linhas ao ficheiro `~/.bashrc`:

```bash
# Carregar ambiente ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Carregar workspace ORB-SLAM3 ROS2
if [ -f "$ORB_SLAM3_ROOT_PATH/ROS2_ORB_SLAM3/install/setup.bash" ]; then
    source "$ORB_SLAM3_ROOT_PATH/ROS2_ORB_SLAM3/install/setup.bash"
fi

# Configurar caminhos das bibliotecas dinâmicas
export LD_LIBRARY_PATH="$ORB_SLAM3_ROOT_PATH/ORB-SLAM3/lib:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="$ORB_SLAM3_ROOT_PATH/Pangolin/build:$LD_LIBRARY_PATH"
```

**Aplicar as alterações:**

```bash
source ~/.bashrc
```

### 1.5 Verificação da Instalação

```bash
# Verificar se o pacote foi instalado
ros2 pkg list | grep orbslam3

# Listar executáveis disponíveis
ros2 pkg executables orbslam3
```

**Deverá ver os seguintes executáveis disponíveis:**

- `orbslam3 mono`
- `orbslam3 rgbd`
- `orbslam3 stereo`
- `orbslam3 stereo-inertial`

---

## 2. Exemplos de Utilização

### 2.1 Modo Monocular

```bash
ros2 run orbslam3 mono \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Vocabulary/ORBvoc.txt \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Examples/Monocular/TUM1.yaml
```

**Tópicos subscritos:**
- `/camera/image_raw` (sensor_msgs/Image)

### 2.2 Modo Estéreo

```bash
ros2 run orbslam3 stereo \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Vocabulary/ORBvoc.txt \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Examples/Stereo/EuRoC.yaml \
    false
```

**Tópicos subscritos:**
- `/camera/left/image_raw` (sensor_msgs/Image)
- `/camera/right/image_raw` (sensor_msgs/Image)

### 2.3 Modo RGB-D

```bash
ros2 run orbslam3 rgbd \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Vocabulary/ORBvoc.txt \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Examples/RGB-D/TUM1.yaml
```

**Tópicos subscritos:**
- `/camera/rgb/image_raw` (sensor_msgs/Image)
- `/camera/depth_registered/image_raw` (sensor_msgs/Image)

### 2.4 Modo Estéreo-Inercial

```bash
ros2 run orbslam3 stereo-inertial \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Vocabulary/ORBvoc.txt \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Examples/Stereo-Inertial/EuRoC.yaml \
    false
```

**Tópicos subscritos:**
- `/camera/left/image_raw` (sensor_msgs/Image)
- `/camera/right/image_raw` (sensor_msgs/Image)
- `/imu` (sensor_msgs/Imu)

---

## 3. Exemplos Práticos

### 3.1 Exemplo com Intel RealSense D435

**Terminal 1: Lançar o nó completo (câmara + SLAM)**

```bash
ros2 launch orbslam3 stereo_realsense.launch.py
```

Este launch file inicia automaticamente:
- O driver da RealSense (`realsense2_camera_node`)
- O nó de SLAM estéreo (`orbslam3_stereo`)
- Os transforms estáticos (`base_link` → `camera_link`)

**Terminal 2 (opcional): Lançar apenas a câmara separadamente**

```bash
ros2 launch realsense2_camera rs_launch.py \
    enable_infra1:=true \
    enable_infra2:=true \
    enable_depth:=false \
    enable_color:=false \
    infra_width:=848 \
    infra_height:=480 \
    infra_fps:=30.0 \
    enable_sync:=true \
    emitter_enabled:=0
```

**Terminal 3 (opcional): Lançar apenas o SLAM separadamente**

```bash
ros2 run orbslam3 stereo \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Vocabulary/ORBvoc.txt \
    /caminho/para/cameraParameters.yml \
    true \
    --ros-args \
    -r camera/left:=/camera/infra1/image_rect_raw \
    -r camera/right:=/camera/infra2/image_rect_raw
```

### 3.2 Teste com ROS2 Bag

**Download de bag de teste:**

```bash
cd $ORB_SLAM3_ROOT_PATH
pip install gdown --break-system-packages
gdown --folder https://drive.google.com/drive/folders/1NY4KEW2qpfKlzR-674E-DFNkUEfDkJsC
```

**Terminal 1: Executar ORB-SLAM3**

```bash
ros2 run orbslam3 mono \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Vocabulary/ORBvoc.txt \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Examples/Monocular/TUM1.yaml
```

**Terminal 2: Reproduzir bag**

```bash
ros2 bag play my_camera_bag
```

### 3.3 Teste com Webcam USB

```bash
sudo apt install ros-${ROS_DISTRO}-v4l2-camera
sudo apt install ros-${ROS_DISTRO}-rqt-image-view
```

**Terminal 1: Lançar câmara**

```bash
ros2 run v4l2_camera v4l2_camera_node \
    --ros-args -r /image_raw:=/camera
```

**Terminal 2 (opcional): Visualizar imagem**

```bash
ros2 run rqt_image_view rqt_image_view
```

**Terminal 3: Executar SLAM**

```bash
ros2 run orbslam3 mono \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Vocabulary/ORBvoc.txt \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Examples/Monocular/TUM1.yaml
```

---

## 4. Teste e Monitorização dos Nós

Esta secção descreve como verificar que os nós estão a funcionar correctamente após o arranque.

### 4.1 Verificar Tópicos Publicados

Após lançar o sistema, confirmar que todos os tópicos esperados estão activos:

```bash
ros2 topic list | grep orb_slam3
```

Deverão aparecer os seguintes tópicos:

```
/orb_slam3/camera_pose
/orb_slam3/odometry
/orb_slam3/map_points
/orb_slam3/tracking_state
```

### 4.2 Monitorizar a Pose em Tempo Real

```bash
# Pose simples (PoseStamped)
ros2 topic echo /orb_slam3/camera_pose

# Odometria com covariância (Odometry)
ros2 topic echo /orb_slam3/odometry

# Apenas posição x, y, z
ros2 topic echo /orb_slam3/camera_pose --field pose.position
```

### 4.3 Monitorizar o Estado de Tracking

```bash
ros2 topic echo /orb_slam3/tracking_state
```

**Mapeamento dos valores:**

| Valor | Estado           | Descrição                                  |
|-------|------------------|--------------------------------------------|
| 0     | NO_IMAGES_YET    | Nenhuma imagem recebida ainda              |
| 1     | NOT_INITIALIZED  | A aguardar inicialização                   |
| 2     | OK               | Tracking a funcionar correctamente         |
| 3     | RECENTLY_LOST    | Perdido recentemente, a usar modelo de movimento |
| 4     | LOST             | Tracking perdido                           |

### 4.4 Verificar a Taxa de Publicação

```bash
# Taxa de publicação da pose (deve ser ~30 Hz)
ros2 topic hz /orb_slam3/camera_pose

# Taxa da odometria
ros2 topic hz /orb_slam3/odometry

# Taxa dos pontos do mapa
ros2 topic hz /orb_slam3/map_points

# Verificar latência (delay entre timestamp da mensagem e hora actual)
ros2 topic delay /orb_slam3/camera_pose
```

### 4.5 Verificar a Árvore de TF

```bash
# Ver todos os transforms activos
ros2 run tf2_tools view_frames

# Verificar transform específico map -> camera_link
ros2 run tf2_ros tf2_echo map camera_link

# Verificar se o transform está a ser publicado
ros2 topic echo /tf --no-arr
```

O comando `view_frames` gera um ficheiro `frames.pdf` com o diagrama completo da árvore de TF. A estrutura esperada é:

```
map
└── odom
    └── base_link
        └── camera_link
```

### 4.6 Visualização no RViz2

```bash
ros2 run rviz2 rviz2
```

**Configuração recomendada no RViz2:**

1. Definir `Fixed Frame` para `map`
2. Adicionar display `PoseStamped` → tópico `/orb_slam3/camera_pose`
3. Adicionar display `Odometry` → tópico `/orb_slam3/odometry`
4. Adicionar display `PointCloud2` → tópico `/orb_slam3/map_points`
5. Adicionar display `TF` para visualizar a árvore de transforms

### 4.7 Reset Manual do SLAM

Caso o tracking seja perdido e seja necessário reiniciar o ORB-SLAM3 sem reiniciar o nó:

```bash
ros2 service call /orb_slam3/reset std_srvs/srv/Trigger
```

**Resposta esperada:**

```
response:
  success: True
  message: ORB-SLAM3 reset successfully
```

### 4.8 Verificar os Diagnósticos do Nó

O nó publica automaticamente um resumo de diagnóstico no log a 1 Hz. Para visualizar:

```bash
ros2 topic echo /rosout | grep Diag
```

Ou directamente nos logs do nó:

```bash
# Ver logs em tempo real (substituir <node_name> pelo nome do nó)
ros2 node info /ORB_SLAM3_ROS2

# Ver logs com nível de detalhe
ros2 run orbslam3 stereo ... --ros-args --log-level debug
```

**Exemplo de saída de diagnóstico esperada:**

```
[INFO] [ORB_SLAM3_ROS2]: [Diag] State: OK | Frames: 450 total / 448 published | Map pts: 1203 | Lost streak: 0
```

### 4.9 Verificar Desempenho do Sistema

```bash
# Verificar carga do CPU e memória
htop

# Verificar estatísticas dos tópicos ROS2
ros2 topic bw /orb_slam3/map_points

# Verificar informação do nó
ros2 node info /ORB_SLAM3_ROS2
```

---

## 5. Resolução de Problemas Comuns

### 5.1 Erro: "Library not found"

```bash
echo $LD_LIBRARY_PATH
source ~/.bashrc
```

### 5.2 ORB-SLAM3 não inicializa

- Verificar se há movimento suficiente da câmara
- Garantir que a cena tem textura adequada
- Verificar se os tópicos estão a publicar dados:

```bash
ros2 topic list
ros2 topic echo /camera/infra1/image_rect_raw --no-arr
ros2 topic hz /camera/infra1/image_rect_raw
```

### 5.3 Tópico `/orb_slam3/camera_pose` não publica

Verificar o estado de tracking:

```bash
ros2 topic echo /orb_slam3/tracking_state
```

Se o valor for `4` (LOST) durante um período prolongado, o nó tenta o reset automático após 30 frames consecutivos perdidos. Pode também fazer reset manual:

```bash
ros2 service call /orb_slam3/reset std_srvs/srv/Trigger
```

### 5.4 TF não disponível

```bash
# Verificar se o transform está a ser publicado
ros2 run tf2_ros tf2_echo map camera_link

# Verificar a árvore completa
ros2 run tf2_tools view_frames
```

### 5.5 Performance baixa

- Reduzir o número de features no ficheiro YAML:
  ```yaml
  ORBextractor.nFeatures: 500
  ```
- Verificar se o Pangolin não está a consumir demasiados recursos
- Considerar desactivar o viewer do Pangolin

---

## 6. Referências

- **ORB-SLAM3 Original:** https://github.com/UZ-SLAMLab/ORB_SLAM3
- **ORB-SLAM3 Ubuntu 24.04:** https://github.com/AeroTec-ATLAS/ORB-SLAM3-STEREO-FIXED-for-ubuntu-24.04-LTS
- **Wrapper ROS2:** https://github.com/AeroTec-ATLAS/ORB_SLAM3_ROS2
- **Documentação ROS2:** https://docs.ros.org/

---

**Última actualização:** 18 de Março de 2026