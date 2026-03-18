# ORB-SLAM3 ROS2 Wrapper

ROS2 wrapper for ORB-SLAM3, supporting Monocular, Stereo, RGB-D, Monocular-Inertial, and Stereo-Inertial SLAM modes.

## Supported ROS2 Distributions

- **ROS2 Humble** (Ubuntu 22.04)
- **ROS2 Jazzy** (Ubuntu 24.04)

## Prerequisites

### 1. ORB-SLAM3 Installation

This wrapper requires ORB-SLAM3 to be installed first. Follow the installation instructions from:

**https://github.com/pedroMVicente/ORB-SLAM3-STEREO-FIXED-for-ubuntu-24.04-LTS**

Make sure you have:
- ORB-SLAM3 compiled successfully
- Pangolin installed
- Sophus installed globally (`sudo make install`)
- `ORB_SLAM3_ROOT_PATH` environment variable set

### 2. ROS2 Installation

Install ROS2 Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04) following the [official ROS2 documentation](https://docs.ros.org/en/jazzy/Installation.html).

Ensure ROS2 is sourced in your `~/.bashrc`:
```bash
# For Humble
source /opt/ros/humble/setup.bash

# For Jazzy  
source /opt/ros/jazzy/setup.bash
```

### 3. Install Dependencies

```bash
# Install required ROS2 packages
sudo apt install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-message-filters

# Install Python package
pip install catkin_pkg --break-system-packages
```

## Installation

### 1. Clone Repository

```bash
cd $ORB_SLAM3_ROOT_PATH
mkdir -p ROS2_ORB_SLAM3/src
cd ROS2_ORB_SLAM3/src

git clone https://github.com/AeroTec-ATLAS/ORB_SLAM3_ROS2.git orbslam3_ros2
```

### 2. Build

```bash
cd $ORB_SLAM3_ROOT_PATH/ROS2_ORB_SLAM3
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Note:** Warnings about deprecated functions are normal and don't affect functionality.

### 3. Setup Environment

Add to `~/.bashrc`:

```bash
# Source ROS2 workspace
source $ORB_SLAM3_ROOT_PATH/ROS2_ORB_SLAM3/install/setup.bash

# Add library paths
export LD_LIBRARY_PATH="$ORB_SLAM3_ROOT_PATH/ORB-SLAM3/lib:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="$ORB_SLAM3_ROOT_PATH/Pangolin/build:$LD_LIBRARY_PATH"
```

Then reload:
```bash
source ~/.bashrc
```

## Verification

```bash
# Check package installation
ros2 pkg list | grep orbslam3

# List available nodes
ros2 pkg executables orbslam3
```

Expected output:
```
orbslam3 mono
orbslam3 rgbd
orbslam3 stereo
orbslam3 stereo-inertial
```

## Usage

### Monocular

```bash
ros2 run orbslam3 mono \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Vocabulary/ORBvoc.txt \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Examples/Monocular/TUM1.yaml
```

**Subscribed Topics:**
- `/camera/image_raw` (sensor_msgs/Image)

### Stereo

```bash
ros2 run orbslam3 stereo \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Vocabulary/ORBvoc.txt \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Examples/Stereo/EuRoC.yaml \
    false
```

**Subscribed Topics:**
- `/camera/left/image_raw` (sensor_msgs/Image)
- `/camera/right/image_raw` (sensor_msgs/Image)

### RGB-D

```bash
ros2 run orbslam3 rgbd \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Vocabulary/ORBvoc.txt \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Examples/RGB-D/TUM1.yaml
```

**Subscribed Topics:**
- `/camera/rgb/image_raw` (sensor_msgs/Image)
- `/camera/depth_registered/image_raw` (sensor_msgs/Image)

### Stereo-Inertial

```bash
ros2 run orbslam3 stereo-inertial \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Vocabulary/ORBvoc.txt \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Examples/Stereo-Inertial/EuRoC.yaml \
    false
```

**Subscribed Topics:**
- `/camera/left/image_raw` (sensor_msgs/Image)
- `/camera/right/image_raw` (sensor_msgs/Image)
- `/imu` (sensor_msgs/Imu)

## Example: Intel RealSense D435i

### Install RealSense ROS2 Wrapper

```bash
sudo apt install ros-${ROS_DISTRO}-realsense2-camera
```

### Launch RealSense Camera

```bash
ros2 launch realsense2_camera rs_launch.py
```

### Run ORB-SLAM3 Stereo

```bash
ros2 run orbslam3 stereo \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Vocabulary/ORBvoc.txt \
    $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Examples/Stereo/RealSense_D435i.yaml \
    false
```

**Note:** You may need to remap topics to match your camera's output. Use:
```bash
ros2 run orbslam3 stereo ... --ros-args \
    -r /camera/left/image_raw:=/camera/infra1/image_rect_raw \
    -r /camera/right/image_raw:=/camera/infra2/image_rect_raw
```

## Troubleshooting

### Build Errors

**Problem:** `Could NOT find ORB_SLAM3`  
**Solution:** Ensure `ORB_SLAM3_ROOT_PATH` is set correctly and ORB-SLAM3 is built.

**Problem:** `cv_bridge.h: No such file or directory`  
**Solution:** Already fixed in this repository. Ensure you're using the latest version.

### Runtime Errors

**Problem:** No image received  
**Solution:** Check topic names with `ros2 topic list` and remap if necessary.

**Problem:** Segmentation fault  
**Solution:** Verify vocabulary file exists and is uncompressed:
```bash
ls -lh $ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Vocabulary/ORBvoc.txt
```

**Problem:** Poor tracking performance  
**Solution:** 
- Ensure adequate lighting
- Avoid pointing at blank walls
- Move camera slowly during initialization
- Check camera calibration parameters in YAML files

## Configuration Files

Configuration YAML files are located in `$ORB_SLAM3_ROOT_PATH/ORB-SLAM3/Examples/`.

Key parameters to adjust for your camera:
- Camera intrinsics (fx, fy, cx, cy)
- Distortion coefficients (k1, k2, p1, p2, k3)
- Camera resolution
- FPS
- Stereo baseline (for stereo cameras)

## Citation

If you use ORB-SLAM3 in your research, please cite:

```bibtex
@article{ORBSLAM3_TRO,
  title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
         and Multi-Map {SLAM}},
  author={Campos, Carlos AND Elvira, Richard AND Rodriguez, Juan J. G\'omez AND 
          Montiel, Jos\'e M. M. AND Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics}, 
  volume={37},
  number={6},
  pages={1874-1890},
  year={2021}
}
```

## License

This wrapper follows the same GPLv3 license as ORB-SLAM3.

## Acknowledgments

- Original ORB-SLAM3: Carlos Campos et al.
- ROS2 wrapper base: zang09
- Ubuntu 24.04 compatibility: pedroMVicente

## Related Repositories

- [ORB-SLAM3 (Ubuntu 24.04 compatible)](https://github.com/pedroMVicente/ORB-SLAM3-STEREO-FIXED-for-ubuntu-24.04-LTS)
- [Original ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [Original ROS2 Wrapper](https://github.com/zang09/ORB_SLAM3_ROS2)