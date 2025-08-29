# Galileo Navigation Workspace

这是一个基于ROS2的自主导航工作空间，集成了多种传感器和定位算法。

## 环境配置

### 1. 系统要求

- **操作系统**: Ubuntu 20.04 LTS 或 Ubuntu 22.04 LTS
- **ROS2版本**: Humble Hawksbill 或 Iron Irwini
- **编译器**: GCC 9.0+ 或 Clang 10.0+
- **CMake**: 3.16.3+

### 2. ROS2安装

#### Ubuntu 20.04 + ROS2 Humble
```bash
# 添加ROS2仓库
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop

# 源化环境
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Ubuntu 22.04 + ROS2 Iron
```bash
# 添加ROS2仓库
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS2 Iron
sudo apt update
sudo apt install -y ros-iron-desktop

# 源化环境
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. 系统依赖安装

```bash
# 更新包管理器
sudo apt update

# 基础开发工具
sudo apt install -y build-essential cmake git wget

# PCL和点云处理相关
sudo apt install -y libpcl-dev

# LCM通信库
sudo apt install -y liblcm-dev

# YAML解析库
sudo apt install -y libyaml-cpp-dev

# Eigen数学库
sudo apt install -y libeigen3-dev

# 网络工具（用于雷达通信）
sudo apt install -y libpcap-dev

# Python相关
sudo apt install -y python3-pip python3-colcon-common-extensions

# 可选：用于性能分析
sudo apt install -y libgoogle-perftools-dev
```

### 4. 第三方库安装

#### GTSAM (必选，用于galileo_sam)
```bash
# 安装依赖
sudo apt install -y libboost-all-dev libtbb-dev

# 下载并编译GTSAM
cd ~/Downloads
wget https://github.com/borglab/gtsam/archive/refs/tags/4.2.0.tar.gz
tar -xzf 4.2.0.tar.gz
cd gtsam-4.2.0
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

#### TEASER++ (必选，用于galileo_sam)
```bash
# 克隆并编译
cd ~/Downloads
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

#### OpenMP (通常已预装)
```bash
sudo apt install -y libomp-dev
```

### 5. ROS2包依赖安装

```bash
# 安装ROS2核心包
sudo apt install -y ros-$ROS_DISTRO-rclcpp \
                    ros-$ROS_DISTRO-std-msgs \
                    ros-$ROS_DISTRO-sensor-msgs \
                    ros-$ROS_DISTRO-nav-msgs \
                    ros-$ROS_DISTRO-geometry-msgs \
                    ros-$ROS_DISTRO-visualization-msgs \
                    ros-$ROS_DISTRO-tf2 \
                    ros-$ROS_DISTRO-tf2-ros \
                    ros-$ROS_DISTRO-tf2-geometry-msgs \
                    ros-$ROS_DISTRO-tf2-eigen \
                    ros-$ROS_DISTRO-pcl-ros \
                    ros-$ROS_DISTRO-pcl-conversions \
                    ros-$ROS_DISTRO-message-filters \
                    ros-$ROS_DISTRO-launch \
                    ros-$ROS_DISTRO-launch-ros \
                    ros-$ROS_DISTRO-ament-index-cpp \
                    ros-$ROS_DISTRO-ament-index-python
```

### 6. 可选中间件

#### Zenoh DDS中间件
```bash
# 安装Zenoh DDS for ROS2
sudo apt install -y ros-$ROS_DISTRO-rmw-zenoh-cpp
```

#### CycloneDDS (默认中间件)
```bash
# 安装CycloneDDS
sudo apt install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp
```

### 7. 工作空间设置

```bash
# 克隆项目（如果还没有）
cd ~/Code
git clone <repository_url> galileo_nav_ws
cd galileo_nav_ws

# 安装Python依赖（如果有requirements.txt）
pip3 install -r requirements.txt

# 创建符号链接（可选）
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
# 或者使用Zenoh
# echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> ~/.bashrc
source ~/.bashrc
```

### 8. 环境验证

```bash
# 验证ROS2安装
ros2 --version

# 验证关键依赖
pkg-config --modversion eigen3
pkg-config --modversion pcl
pkg-config --modversion yaml-cpp

# 验证GTSAM
pkg-config --modversion gtsam

# 验证LCM
pkg-config --modversion lcm

# 测试网络多播（用于LCM）
sudo ip link set dev lo multicast on
```

### 9. 常见问题解决

#### PCL版本问题
```bash
# 如果遇到PCL版本不匹配
sudo apt install -y libpcl-dev=1.10.0+dfsg-5ubuntu1
```

#### LCM编译问题
```bash
# LCM可能需要手动编译
cd ~/Downloads
git clone https://github.com/lcm-proj/lcm.git
cd lcm
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

#### 内存不足问题
```bash
# 如果编译时内存不足，可以减少并行任务数
export MAKEFLAGS=-j2
# 或者使用内存交换文件
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

## 项目结构

```
galileo_nav_ws/
├── src/
│   ├── galileo_assets/          # 机器人资产文件
│   ├── galileo_bringup/         # 启动配置
│   ├── galileo_data/            # 数据文件
│   ├── galileo_klio/            # KI算法实现
│   ├── galileo_lcm_ros2_bridge/ # LCM-ROS2桥接
│   ├── galileo_map/             # 地图模块
│   ├── galileo_sam/             # SAM算法实现
│   ├── rslidar_sdk/             # 速腾聚创雷达驱动
│   └── ...
├── build/                       # 编译输出目录
└── install/                     # 安装目录
```

## 编译配置

### 1. 启用坐标变换功能

编译rslidar_sdk时，需要将`ENABLE_TRANSFORM`选项设置为`ON`以启用坐标变换功能：

#### 直接编译
```bash
cmake -DENABLE_TRANSFORM=ON ..
```

#### ROS1 (catkin)
```bash
catkin_make -DENABLE_TRANSFORM=ON
```

#### ROS2 (colcon)
```bash
colcon build --cmake-args '-DENABLE_TRANSFORM=ON'
```

### 2. 完整编译工作空间

```bash
# 在工作空间根目录
colcon build --symlink-install --cmake-args '-DENABLE_TRANSFORM=ON'
```

## 雷达配置

### 1. 参数设置

在`src/rslidar_sdk/src/rslidar_sdk/config/config.yaml`中配置雷达参数：

```yaml
common:
  msg_source: 1                                       # 消息源: 1=在线雷达
  send_packet_ros: false                             # 是否通过ROS发送数据包
  send_point_cloud_ros: true                         # 是否通过ROS发送点云

lidar:
  - driver:
      # 坐标变换参数 (需要ENABLE_TRANSFORM=ON)
      x: 1.0                                        # X偏移
      y: 0.0                                        # Y偏移
      z: 2.5                                        # Z偏移
      roll: 0.1                                     # 翻滚角
      pitch: 0.2                                    # 俯仰角
      yaw: 1.57                                     # 偏航角
```

### 2. 网络配置

```yaml
# 网络参数
host_address: 192.168.99.51                       # 本机地址
group_address: 224.1.1.3                          # 组播地址
```

## LCM配置

### 1. 启用网口多播功能

在使用LCM进行通信前，需要启用网络接口的多播功能：

```bash
#!/bin/bash
# 启用多播功能的脚本

IFACE="your_network_interface"  # 替换为实际的网络接口名称

echo "启用网络接口 $IFACE 的多播功能..."
if ! sudo ip link set dev "$IFACE" multicast on; then
    echo "错误: 无法启用多播功能" >&2
    exit 1
fi

echo "多播功能已启用"
```

### 2. 查看网络接口

```bash
# 查看可用的网络接口
ip addr show

# 常用接口名称示例:
# - eth0 (有线网卡)
# - wlan0 (无线网卡)
# - enp0s3 (以太网接口)
```

### 3. 验证多播设置

```bash
# 检查多播是否启用
ip link show dev $IFACE

# 查看多播路由
ip route show | grep multicast
```

## 中间件配置

### 1. Zenoh DDS中间件

使用Zenoh中间件时，需要先启动Zenoh守护进程，然后设置RMW实现：

```bash
# 1. 首先启动Zenoh守护进程
ros2 run rmw_zenoh_cpp rmw_zenohd

# 2. 设置环境变量（在新终端中）
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# 3. 或者直接在运行命令时指定
RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 run your_package your_node
```

**注意**：Zenoh守护进程必须在后台运行，才能使用Zenoh DDS中间件进行通信。

### 2. 启动配置

在`src/galileo_bringup/config/bringup.yaml`中配置启动选项：

```yaml
bringup:
  ros__parameters:
    use_klio: true           # 使用KLIO算法
    use_sam: false           # 使用SAM算法
    use_map: false           # 使用地图模块
    use_bridge: true         # 使用桥接模块
    use_bag_play: false      # 使用bag文件播放
    bag_file_path: "src/galileo_data/8.12-1"  # bag文件路径
```

## 运行说明

### 1. 环境设置

```bash
# 源化工作空间
source install/setup.bash

# 或者使用本地设置
source install/local_setup.bash
```

### 2. 启动系统

```bash
# 启动bringup节点
ros2 launch galileo_bringup bringup.launch.py

# 或者直接运行节点
ros2 run galileo_klio galileo_klio_node
```

### 3. 查看话题

```bash
# 查看活跃话题
ros2 topic list

# 查看节点信息
ros2 node list

# 查看服务
ros2 service list
```

## 调试和监控

### 1. 日志级别设置

```bash
# 设置日志级别
export ROS_LOG_LEVEL=DEBUG

# 或者运行时设置
ros2 run galileo_klio galileo_klio_node --ros-args --log-level DEBUG
```

### 2. RViz可视化

```bash
# 启动RViz
rviz2

# 或者使用配置文件启动
rviz2 -d src/galileo_klio/rviz/galileo_klio.rviz
```

### 3. 常用调试命令

```bash
# 查看节点参数
ros2 param list /galileo_klio_node

# 查看话题信息
ros2 topic info /rslidar_points

# 查看话题数据
ros2 topic echo /klio/odometry

# 录制bag文件
ros2 bag record /rslidar_points /klio/odometry

# 播放bag文件
ros2 bag play src/galileo_data/your_bag_file/
```

## 故障排除

### 1. 编译问题

- 确保所有依赖已安装
- 检查ENABLE_TRANSFORM选项是否正确设置
- 确认ROS2版本与工作空间兼容

### 2. 网络问题

- 验证LCM多播设置
- 检查网络接口配置
- 确认防火墙设置

### 3. 雷达连接问题

- 检查雷达IP地址和端口配置
- 验证网络连接
- 确认雷达固件版本

## 贡献指南

1. Fork项目
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 创建Pull Request

## 许可证

本项目采用MIT许可证 - 查看[LICENSE](LICENSE)文件了解详情

## 联系方式

如有问题，请通过以下方式联系：
- 邮箱: your-email@example.com
- 项目主页: [项目地址]

---

**注意**: 请根据实际环境调整配置参数，特别是在生产环境中使用前务必进行充分测试。
