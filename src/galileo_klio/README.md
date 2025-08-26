# Galileo KLIO - ROS2 四足机器人激光雷达惯性里程计

这是一个基于ROS2的四足机器人激光雷达惯性里程计（LIO）系统，从Leg-KILO项目迁移而来。

## 🚀 功能特性

- **多传感器融合**: 激光雷达 + IMU + 运动学信息
- **实时定位**: 基于ESKF的状态估计
- **点云处理**: 体素地图和特征提取
- **ROS2标准**: 完全符合ROS2最佳实践
- **参数配置**: 支持多种传感器配置

## 📁 项目结构

```
Galileo_KLIO/
├── src/
│   └── galileo_klio_node.cc          # 主节点实现
├── include/
│   ├── galileo_klio_node.h           # 节点头文件
│   ├── common.hpp                    # 通用定义
│   ├── kinematics.h                  # 运动学处理
│   └── voxel_map.h                   # 体素地图
├── config/
│   ├── galileo_klio_params.yaml      # Robosense Airy 激光雷达配置
│   └── galileo_klio_ouster_params.yaml # 已统一为 Robosense Airy 配置示例
├── launch/
│   └── galileo_klio_launch.py        # 启动文件
├── msg/
│   └── RobotState.msg                # 自定义消息
├── package.xml                       # 包配置
├── CMakeLists.txt                    # 构建配置
└── README.md                         # 项目文档
```

## 🛠️ 安装和构建

### 依赖项

- ROS2 Humble
- Eigen3
- PCL (Point Cloud Library)
- yaml-cpp
  

### 构建

```bash
# 克隆项目
cd ~/ros2_ws/src
git clone <repository_url> galileo_klio

# 构建
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select galileo_klio

# 设置环境
source install/setup.bash
```

## 🎯 使用方法

### 1. 直接运行节点

```bash
# 使用默认参数
ros2 run galileo_klio galileo_klio_node

# 使用自定义参数文件
ros2 run galileo_klio galileo_klio_node --ros-args --params-file config/galileo_klio_params.yaml
```

### 2. 使用启动文件

```bash
# 使用默认配置（Robosense Airy 激光雷达）
ros2 launch galileo_klio galileo_klio_launch.py

# 使用另一份 Robosense Airy 配置示例
ros2 launch galileo_klio galileo_klio_launch.py config_file:=galileo_klio_ouster_params.yaml
```

### 3. 话题重映射

启动文件默认包含以下重映射：
- `/points_raw` → `/rslidar_points`
- `/imu_raw` → `/imu/data`
- `/high_state` → `/robot_state`

可以通过启动参数自定义：
```bash
ros2 launch galileo_klio galileo_klio_launch.py \
    lidar_topic:=/your_lidar_topic \
    imu_topic:=/your_imu_topic \
    kinematic_topic:=/your_kinematic_topic
```

## ⚙️ 配置文件

### 主要参数

#### 话题配置
- `lidar_topic`: 激光雷达话题名称
- `imu_topic`: IMU话题名称
- `kinematic_topic`: 运动学话题名称

#### 传感器选项
- `only_imu_use`: 是否仅使用IMU（不使用运动学信息）
- `redundancy`: 是否启用冗余处理

#### 外参标定
- `extrinsic_T`: IMU到激光雷达的平移向量 [x, y, z]
- `extrinsic_R`: IMU到激光雷达的旋转矩阵 [3x3]

#### 激光雷达参数
- `lidar_type`: 激光雷达类型（仅保留 1: Robosense Airy）
- `time_scale`: 时间尺度
- `blind`: 盲区距离
- `filter_num`: 滤波次数
- `point_stamp_correct`: 是否进行时间戳校正

#### 体素地图参数
- `voxel_size`: 体素大小
- `max_layer`: 最大层数
- `max_points_num`: 最大点数
- `min_eigen_value`: 最小特征值

#### 运动学参数
- `leg_offset_x/y`: 腿部偏移
- `leg_calf_length`: 小腿长度
- `leg_thigh_length`: 大腿长度
- `contact_force_threshold_up/down`: 接触力阈值

#### ESKF参数
- `vel_process_cov`: 速度过程协方差
- `imu_acc_process_cov`: IMU加速度过程协方差
- `imu_gyr_process_cov`: IMU陀螺仪过程协方差
- `imu_acc_meas_noise`: IMU加速度测量噪声
- `imu_gyr_meas_noise`: IMU陀螺仪测量噪声

### 配置文件选择

1. **`galileo_klio_params.yaml`**: 适用于 Robosense Airy 激光雷达
   - 默认话题: `/points_raw`, `/imu_raw`, `/high_state`
   - 外参: 零平移，单位旋转矩阵

2. **`galileo_klio_ouster_params.yaml`**: 另一份 Robosense Airy 配置示例
   - 话题: `/agent1/ouster/points`, `/agent1/ouster/imu`, `/agent1/high_state`
   - 外参: 非零平移，单位旋转矩阵
   - 时间尺度: 1e-9（纳秒）

## 📊 发布的话题

- `/pointcloud_body`: 机体坐标系点云
- `/pointcloud_world`: 世界坐标系点云
- `/path`: 轨迹路径
- `/odometry`: 里程计信息

## 🔧 开发说明

### 代码结构

- **节点类**: `GalileoKLIONode` 继承自 `rclcpp::Node`
- **定时器**: 使用100Hz定时器替代while循环
- **参数管理**: 使用ROS2参数系统
- **消息处理**: 支持自定义 `RobotState` 消息

### 添加新功能

1. 在 `include/galileo_klio_node.h` 中声明新方法
2. 在 `src/galileo_klio_node.cc` 中实现
3. 在配置文件中添加相关参数
4. 更新启动文件（如需要）

### 调试

```bash
# 查看参数
ros2 param list /galileo_klio_node
ros2 param get /galileo_klio_node lidar_topic

# 设置参数
ros2 param set /galileo_klio_node debug_mode true

# 查看话题
ros2 topic list
ros2 topic echo /odometry
```

## 🤝 贡献

欢迎提交Issue和Pull Request！

## 📄 许可证

本项目基于Leg-KILO项目开发，请遵守相应的许可证要求。 