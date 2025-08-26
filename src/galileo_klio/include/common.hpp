#ifndef COMMON_HPP
#define COMMON_HPP

#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <chrono>
#include <cmath>
#include <deque>
#include <thread>

// ROS2 相关头文件
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

// Eigen 相关头文件
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

// PCL 相关头文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// 其他库
#include <yaml-cpp/yaml.h>

// 命名空间
namespace galileo_klio {

// 常用类型定义
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;

// Leg-KILO 特定类型定义
using PointType = pcl::PointXYZINormal;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using CloudConstPtr = PointCloudType::ConstPtr;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

// 时间相关
using Clock = std::chrono::high_resolution_clock;
using TimePoint = Clock::time_point;
using Duration = std::chrono::duration<double>;

// 常量定义
constexpr double PI = M_PI;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;

// 宏定义
#define UNUSED(x) (void)(x)
#define THREAD_SLEEP(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]

namespace common {

struct LidarScan {
    double lidar_begin_time_;
    double lidar_end_time_;
    CloudPtr cloud_;
};

// leg order: FR FL RR RL
struct KinImuMeas {
    double time_stamp_;
    double foot_pos_[4][3];
    double foot_vel_[4][3];
    bool contact_[4];
    double acc_[3];
    double gyr_[3];
};

struct MeasGroup {
    LidarScan lidar_scan_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imus_;
    std::deque<KinImuMeas> kin_imus_;
};

enum class LidarType { ROBOSENSE_AIRY = 1 };

}  // namespace common
}  // namespace galileo_klio

#endif // COMMON_HPP 