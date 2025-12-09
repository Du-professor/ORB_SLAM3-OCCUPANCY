# 双足机器人 vSLAM 建图环境配置指南

## 目标
建立一个包含以下组件的开发环境：
- ROS 2 (Humble)
- RealSense 相机 + ROS2 wrapper
- CUDA + cuDNN (用于 GPU 加速，可选)
- GPU 加速的 OpenCV + Pangolin + Eigen3
- 使用 ORB-SLAM3 + ROS2 的 vSLAM 系统

---

## 环境配置步骤

### 1. 安装 ROS 2 (Humble)
**推荐方法**：使用小鱼 ROS 一键安装脚本
1. 获取「小鱼一键安装系列」脚本
2. 下载并执行该脚本，安装 ROS2 + 基础环境

**备用方法**：官方二进制安装（适用于 Ubuntu 22.04）
1. 按照官方指南安装：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
2. 安装后在 `~/.bashrc` 中添加环境变量：
```bash
source /opt/ros/humble/setup.bash
```
3. 应用配置：
```bash
source ~/.bashrc
```

### 2. 安装 RealSense ROS2 wrapper
安装支持您的 RealSense 相机（如 D435/D435i/D455等）的驱动，使ROS2能够发布color/depth/pointcloud等topic供SLAM使用。

### 3. 安装 CUDA + cuDNN（GPU加速，可选）
1. 下载适合系统的CUDA Toolkit（推荐CUDA 12.6）
2. 安装后在 `~/.bashrc` 末尾添加：
```bash
export PATH=/usr/local/cuda-12.6/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH
```
3. 应用配置：
```bash
source ~/.bashrc
```
4. 如需深度学习加速，下载对应版本的cuDNN并安装

### 4. 安装 vSLAM 依赖库
安装基础编译工具和依赖库：
```bash
sudo apt update
sudo apt install build-essential cmake git libgtk2.0-dev pkg-config \
    libavcodec-dev libavformat-dev libswscale-dev \
    libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev \
    libdc1394-22-dev libglew-dev libboost-all-dev libssl-dev \
    libeigen3-dev
```
- **OpenCV**：根据需求选择是否启用CUDA加速
- **Pangolin**：安装或编译（SLAM可视化工具依赖）

### 5. 建立工作空间并编译 ORB-SLAM3 + ROS2 Wrapper
1. 创建ROS2工作空间：
```bash
mkdir -p ~/orb_slam3_ros2_ws/src
cd ~/orb_slam3_ros2_ws/src
git clone https://github.com/Mechazo11/ros2_orb_slam3.git
```

2. 安装必要ROS2依赖包（如cv_bridge、message_filters等）

3. 编译工作空间：
```bash
cd ~/orb_slam3_ros2_ws
colcon build
source install/setup.bash
```

### 6. 运行测试：RealSense + ROS2 + ORB-SLAM3
启动RealSense D435i相机驱动：
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  rgb_camera.color_profile:=640x480x30 \
  enable_depth:=true \
  depth_module.depth_profile:=640x480x30 \
  enable_rgbd:=true \
  align_depth.enable:=true \
  enable_sync:=true \
  pointcloud.enable:=true
```

然后启动ORB-SLAM3 ROS2节点，开始vSLAM建图测试。

---

## 注意事项与建议
1. **平台兼容性**：Jetson等嵌入式平台使用ROS2 Humble时，需确保系统为Ubuntu 22.04
2. **OpenCV版本管理**：避免多个OpenCV版本共存，建议删除旧版本后安装兼容版本（如OpenCV 4.5.x/4.6.0）
3. **版本兼容性**：使用GPU加速时，确保CUDA、cuDNN、OpenCV、PyTorch等版本相互兼容
4. **容器化方案**：考虑使用Docker容器避免环境冲突，多个GitHub仓库提供相关Docker镜像

---

## 项目参考资源
- **ORB-SLAM3 ROS2包 (Humble)**：https://github.com/Mechazo11/ros2_orb_slam3
- **ORB-SLAM3 ROS2 Docker包装器**：
  - https://github.com/Gwardii/ORB-SLAM3-ROS2
  - https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker
- **ROS2官方文档**：https://docs.ros.org/en/humble/index.html

---

*注意：本指南基于Ubuntu 22.04和ROS2 Humble版本编写，其他系统版本可能需要相应调整。*
