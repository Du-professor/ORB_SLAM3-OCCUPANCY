# vSLAM 建图环境配置指南
<img width="1496" height="841" alt="image" src="https://github.com/user-attachments/assets/b91b0a14-2924-4420-b6af-9b23d844eea2" />
## 目标
建立一个包含以下组件的开发环境：
-ROS 2 (Humble) &nbsp
- RealSense 相机 + ROS2 wrapper 
- CUDA + cuDNN (用于 GPU 加速，可选)
- GPU 加速的 OpenCV + Pangolin + Eigen3
- 使用 ORB-SLAM3 + ROS2 的 vSLAM 系统
---

## 环境配置步骤
### docker安装
https://www.yuque.com/yuqueyonghuxc8etz/tbgbay/pm4lc8xu4in7ncbt?singleDoc# 《Docker》
### 1. 安装 ROS 2 (Humble)
**推荐方法**：使用小鱼 ROS 一键安装脚本
1. 获取「小鱼一键安装系列」脚本
```bash
wget http://fishros.com/install -O fishros && . fishros
```
2. 下载并执行该脚本，安装 ROS2 + 基础环境

**备用方法**：官方二进制安装（适用于 Ubuntu 22.04）
按照官方指南安装：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html


### 2.安装 ROS2 Humble RealSense 包
（支持您的 RealSense 相机（如 D435/D435i/D455等）的驱动）
#### 确保 ROS2 环境已激活
```bash
source /opt/ros/humble/setup.bash
```
#### 注册public key
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
```
#### 添加apt源
```bash
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
```

#### 方法 A：从二进制包安装（推荐，最简单）
```bash
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description
```

### 3. 安装 CUDA + cuDNN（GPU加速，可选）
下载适合系统的CUDA Toolkit（推荐CUDA 12.6）
#### CUDA Toolkit Installer
ownload Installer for Linux Ubuntu 22.04 arm64-sbsa
```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/sbsa/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-6
```
#### Driver Installer
```bash
sudo apt-get install -y nvidia-open
sudo apt-get install -y cuda-drivers
```
用vim命令打开环境配置文件.bashrc
```bash
vim ~/.bashrc
```
输入i进入编辑模式，在文件的最后输入以下两行内容添加环境变量，esc退出编辑模式，并输入:wq进行保存。此处/usr/local/cuda-12.6/替换成自己的安装路径即可，若不清楚，可以运行whereis cuda查看。
```bash
export PATH=/usr/local/cuda-12.6/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```
重新加载环境配置文件
```bash
source ~/.bashrc
```
所需配置
```bash
JetPack - 6.2 (also compatible with 6.1)
CUDA - 12.6
Python - 3.10
cuSPARSELt - 0.7.0
CUDA installation can be verified with 
```
查看CUDA版本命令
```bash
nvcc --version
$ nvidia-smi
```

### 4. 安装 vSLAM 依赖库
安装基础编译工具和依赖库：
#### Eigen3
```bash
sudo apt install libeigen3-dev
```
#### Pangolin
```bash
cd ~
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin
git checkout v0.8
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

#### OpenCV
本项目使用的是带有CUDA加速的OpenCV-4.10,根据项目需求可以自行更改
先更新apt：
```bash
sudo apt update
sudo apt upgrade
```
安装依赖：
```bash
sudo apt install build-essential cmake pkg-config unzip yasm git checkinstall libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libxvidcore-dev libx264-dev libmp3lame-dev libopus-dev libmp3lame-dev libvorbis-dev ffmpeg libva-dev libdc1394-25 libdc1394-dev libxine2-dev libv4l-dev v4l-utils libgtk-3-dev libtbb-dev libatlas-base-dev gfortran libprotobuf-dev protobuf-compiler libgoogle-glog-dev libgflags-dev libgphoto2-dev libeigen3-dev libhdf5-dev doxygen
```

添加libv4l1-videodev软连接：
```bash
sudo ln -s /usr/include/libv4l1-videodev.h /usr/include/linux/videodev.h
```

安装OpenCV：
```bash
cd ~
mkdir opencv && cd opencv
wget -O opencv.zip https://github.com/opencv/opencv/archive/refs/tags/4.10.0.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/refs/tags/4.10.0.zip
unzip opencv.zip
unzip opencv_contrib.zip
cd opencv-4.10.0
mkdir build
cd build
```

运行下面代码时需要注意：
● OPENCV_PYTHON3_INSTALL_PATH换成实际python3环境
● 如果要安装CUDA版本OpenCV，需要根据GPU更改CUDA_ARCH_BIN的值，可以在显卡计算能力找到显卡的计算能力，Jetson NX算力是8.7，OPENCV_PYTHON3_INSTALL_PATH 注意带有CUDA和CUDNN的选项目，根据你自己的需求选择，OPENCV_PYTHON3_INSTALL_PATH，PYTHON_EXECUTABLE和 OPENCV_EXTRA_MODULES_PATH都需要根据自己的文件目录情况更改
```bash
cmake .. \
-D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D WITH_TBB=ON \
-D ENABLE_FAST_MATH=ON \
-D WITH_V4L=ON \
-D WITH_QT=OFF \
-D WITH_OPENGL=ON \
-D WITH_GSTREAMER=ON \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D OPENCV_PC_FILE_NAME=opencv.pc \
-D OPENCV_ENABLE_NONFREE=ON \
-D OPENCV_PYTHON3_INSTALL_PATH=/usr/local/lib/python3.10/dist-packages \
-D PYTHON_EXECUTABLE=/usr/bin/python3 \
-D OPENCV_EXTRA_MODULES_PATH=/home/test/opencv/opencv_contrib-4.10.0/modules \
-D INSTALL_PYTHON_EXAMPLES=OFF \
-D INSTALL_C_EXAMPLES=OFF \
-D BUILD_EXAMPLES=OFF \
-D WITH_CUDA=ON \
-D WITH_CUDNN=ON \
-D OPENCV_DNN_CUDA=ON \
-D WITH_CUBLAS=ON \
-D CUDA_FAST_MATH=ON \
-D CUDA_ARCH_BIN="8.7" \
-D BUILD_opencv_cudacodec=ON \
-D WITH_NVCUVID=ON \
-D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.6
-D CMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs
```

进行编译和安装：
```bash
make -j$(nproc)
sudo make install
sudo /bin/bash -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig
```

### 5. 创建额外的 Swap 文件（推荐）
为避免构建因显存不足而中断
```bash
# 1. 检查当前 swap
sudo swapon --show
free -h

# 2. 创建一个新的 8GB swap 文件
sudo fallocate -l 8G /swapfile2
sudo chmod 600 /swapfile2
sudo mkswap /swapfile2
sudo swapon /swapfile2

# 3. 验证（现在应该有约 10GB swap）
free -h
swapon --show

# 4. 永久启用新 swap
echo '/swapfile2 none swap sw 0 0' | sudo tee -a /etc/fstab

#关闭现有 swap
sudo swapoff -a

# 删除或重建 swap
# 如果是 swap 文件
sudo rm /swapfile
```

### 6. 安装和使用
直接安装项目
```bash
cd 
git clone https://github.com/Du-professor/ORB_SLAM3-OCCUPANCY.git
cd ~/ORB_SLAM3-OCCUPANCY
colcon build --packages-select orbslam3_dense_ros2 occupancy_grid_map
```
运行项目
```bash
#第一个终端启动相机
cd /ORB_SLAM3-OCCUPANCY/src/orbslam3_dense_ros2
./run_rgbd.sh

#第二个终端使用launch文件启动
cd /ORB_SLAM3-OCCUPANCY
source /home/test/ORB_SLAM3-OCCUPANCY/install/setup.bash
ros2 launch orbslam3_dense_ros2 slam_system.launch.py

#查看GPU利用率
sudo tegrastats
jtop
```

默认 launch 会同时启动：
orb_slam3_main主程序（RGB-D SLAM）
occupancy_grid_map（slam/filtered_cloud，输出 2D 栅格 slam/occupancy_grid）
rviz2(/home/test/ORB_SLAM3-OCCUPANCY/install/orbslam3_dense_ros2/share/orbslam3_dense_ros2/config/rviz_config.rviz）

常用 Launch 参数速览：
filtered_cloud_topic：稠密点云下采样后的输出话题，默认/slam/filtered_cloud
filtered_voxel_leaf_size、filtered_min_range/max_range、filtered_min_z/max_z：发布器的点云裁剪与体素参数
enable_occupancy：是否启用 2D 栅格节点（默认 true）
occupancy_config：占据栅格 YAML（默认为安装目录下的 config/occupancy_grid_map.param.yaml）
start_rviz / rviz_config：是否自动启动 RViz 以及配置文件路径，默认使用安装目录中的 `rviz_config.rviz`

#### 清理编译的产物命令
```bash
cd ~/ORB_SLAM3-OCCUPANCY
rm -rf build install log
```

### 修改代码参数
相关代码根据实际需要更改
#### ORB_SLAM3-OCCUPANCY/src/orbslam3_dense_ros2/orb_slam3/config/RGB-D/RealSense_D435i.yaml
```bash
%YAML:1.0
#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
# 相机参数：若更换相机/分辨率，请务必重新标定并更新此处
#--------------------------------------------------------------------------------------------
File.version: "1.0"   # 配置版本；便于跟踪标定文件更新

Camera.type: "PinHole" # 光学模型类型（针孔、鱼眼等）

# Right Camera calibration and distortion parameters (OpenCV)
# fx/fy 为焦距像素值，cx/cy 为主点坐标，来源于相机标定
Camera1.fx: 379.4932556152344
Camera1.fy: 379.2383728027344
Camera1.cx: 316.54534912109375
Camera1.cy: 245.38729858398438


# distortion parameters
# 径向 (k1-k3) + 切向 (p1-p2) 畸变系数；如已做去畸变可置 0
# Camera1.k1: 0.0
# Camera1.k2: 0.0
# Camera1.p1: 0.0
# Camera1.p2: 0.0
Camera1.k1: -0.05445488914847374
Camera1.k2: 0.065945722162723541
Camera1.p1: -0.00036503968294709921
Camera1.p2: 0.0011364475358277559
Camera1.k3: -0.021424319595098495

# Camera resolution
Camera.width: 640   # 图像宽（像素）
Camera.height: 480  # 图像高（像素）

# Camera frames per second 
Camera.fps: 30      # 期望输入帧率（Hz）

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 1       # 彩色顺序：0=BGR，1=RGB

# Close/Far threshold. Baseline times.
# ThDepth 控制可接受的最大深度（米）；越小越关注近景
Stereo.ThDepth: 4.5
Stereo.b: 0.0745   # 红外相机基线（米）

# Depth map values factor
RGBD.DepthMapFactor: 550.0  # 原始深度值除以该因子得到米

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------
# ORB Extractor: Number of features per image
# 每帧提取的 ORB 特征数量（越大越密集，但耗时增加）
ORBextractor.nFeatures:600

# ORB Extractor: Scale factor between levels in the scale pyramid 	
ORBextractor.scaleFactor: 1.25   # 金字塔层间缩放因子

# ORB Extractor: Number of levels in the scale pyramid	
ORBextractor.nLevels: 5          # 金字塔层数

# ORB Extractor: Fast threshold
# iniThFAST 为首轮 FAST 阈值；若某网格检测不到角点，则退回到 minThFAST
# 场景对比度低时可降低二者以获得更多角点
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 8

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
# 仅影响 Pangolin 可视化窗口，与后端优化无关
Viewer.KeyFrameSize: 0.05       # 关键帧立方体尺寸
Viewer.KeyFrameLineWidth: 1.0   # 关键帧边框线宽
Viewer.GraphLineWidth: 0.9      # 图优化线宽
Viewer.PointSize: 2.0           # MapPoint 点云大小
Viewer.CameraSize: 0.08         # 当前相机模型尺寸
Viewer.CameraLineWidth: 3.0     # 相机模型线宽
Viewer.ViewpointX: 0.0          # 观察视角（世界坐标）
Viewer.ViewpointY: -0.6
Viewer.ViewpointZ: -2.8
Viewer.ViewpointF: 500.0        # 视点焦距
```

#### ORB_SLAM3-OCCUPANCY/src/orbslam3_dense_ros2/run_rgbd.sh
```bash
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    rgb_camera.color_profile:=640x480x30 \
    enable_depth:=true \
    depth_module.depth_profile:=640x480x30 \
    enable_infra:=false \
    enable_infra1:=false \
    enable_infra2:=false \
    depth_module.infra_profile:=640x480x30 \
    enable_rgbd:=true \
    align_depth.enable:=true \
    enable_sync:=true \
    enable_gyro:=false \
    enable_accel:=false \
    pointcloud.enable:=true
```

### 注意事项与建议
1. **平台兼容性**：Jetson等嵌入式平台使用ROS2 Humble时，需确保系统为Ubuntu 22.04
2. **OpenCV版本管理**：避免多个OpenCV版本共存，建议删除旧版本后安装兼容版本（如OpenCV 4.5.x/4.6.0）
3. **版本兼容性**：使用GPU加速时，确保CUDA、cuDNN、OpenCV、PyTorch等版本相互兼容
4. **容器化方案**：考虑使用Docker容器避免环境冲突，多个GitHub仓库提供相关Docker镜像
---

## 项目参考资源
- [https://github.com/Mechazo11/ros2_orb_slam3](https://github.com/Mechazo11/ros2_orb_slam3)
- https://github.com/LeonardoDiCaprio1/Map_ORBSLAM_ROS/
- [https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker](https://github.com/zgfbupt/orbslam3_dense_ros2)
- 
---
*注意：本指南基于Ubuntu 22.04和ROS2 Humble版本编写，其他系统版本可能需要相应调整。*
