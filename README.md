````markdown
# 环境搭建 — ROS2 + RealSense + CUDA + cuDNN + ORB-SLAM3 (vSLAM)

## 目标  
建立一个包含以下组件的开发环境：  
- Docker / ROS 2 (Humble)  
- RealSense 相机 + ROS2 wrapper  
- CUDA + cuDNN (用于 GPU 加速)  
- GPU 加速的 OpenCV + Pangolin + Eigen3  
- 使用 ORB-SLAM3 + ROS2 的 vSLAM 系统  

---

## 步骤

### 1. 使用“小鱼 ROS 一键安装脚本”安装 Docker / ROS2  
1. 获取「小鱼一键安装系列」脚本（由相关作者维护）  
2. 下载并执行该脚本，以安装 Docker 和 ROS2 + 基础环境  
3. 若通过官方方式安装 ROS 2 (Humble) — 推荐使用二进制安装包（deb）适用于 Ubuntu 22.04。 :contentReference[oaicite:1]{index=1}  
4. 安装后，在 `~/.bashrc` 中添加 ROS 环境变量，例如：  
    ```bash
    source /opt/ros/humble/setup.bash
    ```  
   然后执行：  
    ```bash
    source ~/.bashrc
    ```

---

### 2. 安装 RealSense ROS2 wrapper (librealsense + ROS2 驱动)  
按需安装支持你的 RealSense 相机（例如 D435 / D435i / D455 等）。  
这样 ROS2 就能发布 color / depth / pointcloud 等 topic，供 SLAM 使用。

---

### 3. 安装 CUDA + cuDNN（用于 GPU 加速 — 可选）  
- 下载适合系统的 CUDA Toolkit（例如 CUDA Toolkit 12.6）与对应驱动／installer。  
- 安装后，编辑 `~/.bashrc`，在末尾加入类似以下内容（根据你的安装路径修改）：  
    ```bash
    export PATH=/usr/local/cuda-12.6/bin:$PATH  
    export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH  
    ```  
  保存后 (Esc → `:wq`)，然后执行：  
    ```bash
    source ~/.bashrc
    ```  
- 若需要 GPU 加速深度学习或视觉处理 (例如 PyTorch, OpenCV CUDA) — 下载与你的 CUDA 对应版本兼容的 cuDNN (可从官方 cuDNN Archive 获取)，然后解压并将 `include` / `lib` 等内容复制到 CUDA 的安装目录 (如 `/usr/local/cuda/`)。  

---

### 4. 安装 vSLAM 依赖：Eigen3、Pangolin、GPU-加速 OpenCV / OpenCV（CPU 可选）  
根据系统与需求安装或编译以下依赖：  

```bash
sudo apt update
sudo apt install build-essential cmake git libgtk2.0-dev pkg-config \
    libavcodec-dev libavformat-dev libswscale-dev \
    libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev \
    libdc1394-22-dev libglew-dev libboost-all-dev libssl-dev \
    libeigen3-dev
````

* 安装 or 编译 OpenCV (若需要 GPU acceleration，可启用 CUDA / cuDNN；否则也可以用系统默认 OpenCV / apt 安装) ([CSDN博客][1])
* 安装 or 编译 Pangolin（许多 SLAM / 可视化工具依赖它） ([ncnynl.com][2])

---

### 5. 建立工作空间并编译 ORB-SLAM3 + ROS2 Wrapper

1. 创建 ROS2 workspace：

   ```bash
   mkdir -p ~/orb_slam3_ros2_ws/src
   cd ~/orb_slam3_ros2_ws/src
   git clone https://github.com/Mechazo11/ros2_orb_slam3.git
   ```

   该包为 ORB-SLAM3 的 ROS2 (Humble) 集成版本。 ([GitHub][3])
2. 安装必要依赖，例如 `cv_bridge`, `message_filters` 等 ROS2 包 (如果需要)；并确保 Eigen3 / OpenCV / Pangolin 等库已安装。 ([HackMD][4])
3. 编译 workspace：

   ```bash
   cd ~/orb_slam3_ros2_ws
   colcon build
   source install/setup.bash
   ```

---

### 6. 运行测试 —— RealSense + ROS2 + ORB-SLAM3

以 RealSense D435i 为例，你可以通过以下命令启动相机驱动 (color + depth + pointcloud)：

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

然后启动 ORB-SLAM3 ROS2 节点 (参照 wrapper 的 launch / run 脚本) — 这样你就可以开始 vSLAM 建图 / 视觉 SLAM 测试。

---

## 注意事项 & 建议

* 如果你使用 Jetson / 嵌入式平台 (如 Jetson AGX / NX)，需要注意系统兼容性 — ROS2 Humble 推荐使用 Ubuntu 22.04。 ([CSDN博客][5])
* 若系统中已有 OpenCV（例如 Jetson 镜像自带） — 必须确保没有多个 OpenCV 版本共存，否则可能导致编译 / 运行错误。很多教程推荐删除旧版本并手动安装兼容版本 (例如 OpenCV 4.5.x / 4.6.0)。 ([CSDN博客][5])
* 如果你使用 GPU + CUDA + cuDNN，一定要确保版本兼容 (CUDA ↔ cuDNN ↔ OpenCV / PyTorch / GPU 库)。
* 使用 Docker / 容器 + ROS2 + ORB-SLAM3 也是一个好方案 — 可以避免环境冲突／版本问题。许多 github 仓库提供 Docker wrapper。 ([GitHub][6])

---

## 项目参考 / 资源

* ORB-SLAM3 ROS2 包 (Humble): [https://github.com/Mechazo11/ros2_orb_slam3](https://github.com/Mechazo11/ros2_orb_slam3) ([GitHub][3])
* ORB-SLAM3 ROS2 Docker Wrapper: [https://github.com/Gwardii/ORB-SLAM3-ROS2](https://github.com/Gwardii/ORB-SLAM3-ROS2) 或 [https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker](https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker) ([GitHub][6])
* Ubuntu 22.04 + ROS2 Humble 官方安装指南：ROS 官方文档 ([docs.ros.org][7])

---


