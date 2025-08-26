#pragma once

#include <deque>
// #include <iomanip>
// #include <iostream>
#include <mutex>
#include <string>
// #include <thread>
// #include <utility>

#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <pcl/filters/voxel_grid.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_srvs/srv/trigger.hpp>
// #include "galileo_klio/msg/robot_state.hpp"

#include "common.hpp"
#include "lidar_processing.h"
// #include "options.h"
// #include "timer_utils.hpp"
// #include "voxel_grid.hpp"
#include "eskf.h"
#include "leg_utils.hpp"
#include "state_initial.hpp"
#include "voxel_map.h"
#include "leg_utils.hpp"


namespace galileo_klio {

// 使用其他头文件中已定义的结构体
using ESKFConfig = galileo_klio::ESKF::Config;
// 移除旧 Kinematics，改用 leg_utils 直接计算
using LidarProcessingConfig = galileo_klio::LidarProcessing::Config;
using VoxelMapConfig = ::VoxelMapConfig;

class GalileoKLIONode : public rclcpp::Node {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit GalileoKLIONode();
    ~GalileoKLIONode();

    void run();

   private:
    bool initParamAndReset();
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    // 旧的 RobotState 回调已移除
    void kinematicCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void timerCallback();
    
    // 简单时间对齐函数
    bool syncPackage();
    void initStateAndMap();
    bool predictUpdateImu(const sensor_msgs::msg::Imu::SharedPtr& imu);
    bool predictUpdatePoint(double current_time, size_t idx_i, size_t idx_j);
    bool predictUpdateKinImu(const common::KinImuMeas& kin_imu);
    void runReset();
    void cloudLidarToWorld(const CloudPtr& cloud_lidar, CloudPtr& cloud_world);
    void pointLidarToImu(const PointType& point_lidar, PointType& point_imu);
    void pointLidarToWorld(const PointType& point_lidar, PointType& point_world);
    void publishOdomTFPath(double end_time);
     void publishOdomTF(double stamp_sec);
    void publishPointcloudWorld(double end_time);
    void publishPointcloudWorldFull(double stamp_sec);
    void publishPointcloudBody(double end_time);  // without undistort
    // 发布静态TF变换
    void publishStaticTF();
    // 保存地图服务回调
    void handleSaveMap(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);
 
    // subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_raw_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_raw_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_kinematic_raw_;

    // publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_world_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_world_full_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_world_;
    // 机身速度（IMU系）发布器
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_body_velocity_imu_;
    // 接触检测结果发布器（四腿）
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> pub_contact_scores_;
    // 足端位置发布器（四腿）
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pub_foot_position_;
    // 足端速度发布器（四腿）
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> pub_foot_velocity_;
    
    // timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;

    nav_msgs::msg::Odometry odom_world_;
    nav_msgs::msg::Path path_world_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
    geometry_msgs::msg::Transform transform_;
    geometry_msgs::msg::Quaternion q_tf_;
    Eigen::Quaterniond q_eigen_;
    geometry_msgs::msg::PoseStamped pose_path_;
    
    // module
    std::unique_ptr<ESKF> eskf_;
    std::unique_ptr<LidarProcessing> lidar_processing_;
    std::unique_ptr<StateInitial> state_initial_;
    std::unique_ptr<VoxelMapManager> map_manager_;
    pcl::VoxelGrid<PointType> voxel_grid_;
    std::map<std::string, legutils::LegModel> legs_;
    

    // meaure
    std::deque<common::LidarScan> lidar_cache_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_cache_;
    std::deque<common::KinImuMeas> kin_cache_;
    common::MeasGroup measure_;

    // sync package
    std::mutex mutex_;
    double last_timestamp_imu_;
    double last_timestamp_kin_imu_;
    double lidar_end_time_;

    // initialization
    double init_time_ = 0.1;
    bool init_flag_ = true;

    // pcl
    CloudPtr cloud_raw_;
    CloudPtr cloud_down_body_;
    CloudPtr cloud_down_world_;

    // eskf
    double gravity_;
    double acc_norm_;
    double last_state_predict_time_;
    double last_state_update_time_;

    // sensor param
    Mat3D ext_rot_;
    Vec3D ext_t_;
    double satu_acc_;
    double satu_gyr_;
    // 输出坐标系帧名
    std::string output_frame_id_;
    // 机器人本体坐标系帧名
    std::string body_frame_id_;
    // 激光雷达坐标系帧名
    std::string lidar_frame_id_;
    
    // 静态TF发布器
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_br_;
    
    // 配置结构体
    ESKFConfig eskf_config_;
    // 移除 KinematicsConfig
    LidarProcessingConfig lidar_config_;
    VoxelMapConfig voxel_map_config_;
    
    // 话题配置
    std::string lidar_topic_;
    std::string imu_topic_;
    std::string kinematic_topic_;
    
    // 选项配置
    bool only_imu_use_;
    bool redundancy_;
    
    // 其他参数
    double voxel_grid_resolution_;
    std::string map_save_path_;

    // LOG
    size_t success_pts_size;
    
    // 频率统计
    double last_timer_time_;
    int timer_count_;
    double timer_frequency_;

    // 体素地图发布节流
    double last_voxel_map_pub_time_;

     // 高频里程计发布节流
     double last_odom_pub_time_;
     double odom_pub_period_;

     // 调试与监控
     bool debug_enable_;
     double debug_jump_pos_thresh_;
     double debug_jump_yaw_thresh_deg_;
     double debug_log_period_;
     double last_debug_log_time_;

     // 上一次里程计发布的状态
     Eigen::Vector3d last_pub_pos_;
     Eigen::Quaterniond last_pub_q_;
     bool has_last_pub_;

     // 最近一次点更新的信息
     size_t last_effect_num_;
     size_t last_points_size_;
     double last_cur_point_time_;
     double last_curvature_value_;
     double last_lidar_begin_time_;
     double last_lidar_end_time_;
     size_t last_cloud_raw_size_;
     size_t last_cloud_down_body_size_;
     double last_frame_curv_min_;
     double last_frame_curv_max_;

     // 最近一次 IMU 输入
     Vec3D last_imu_acc_;
     Vec3D last_imu_gyr_;
     double last_imu_time_;
     // 最近一次批次 IMU/kin 队列信息（进入处理时的首末时间与长度）
     size_t last_imus_size_;
     double last_imu_first_time_;
     double last_imu_last_time_;
     size_t last_kin_imus_size_;
     double last_kin_first_time_;
     double last_kin_last_time_;

      // 丢包检测参数
      double lidar_expected_hz_;
      double lidar_drop_warn_factor_;

     
     
         // 接触检测参数
    legutils::ContactSigmoidParams contact_params_;
    
    // 接触检测滑动窗口
    std::map<std::string, legutils::TorqueSlidingWindow> contact_torque_windows_;
    
    // 接触检测状态机
    std::map<std::string, legutils::ContactStateMachine> contact_state_machines_;
    
    // 时间对齐变量
    double first_joint_time_ = -1.0;
    double first_imu_time_ = -1.0;
    
    // 简单时间对齐变量
    double time_offset_ = 0.0;           // 关节时间与IMU时间的差值
    bool time_aligned_ = false;          // 是否已完成时间对齐
    

};

}  // namespace galileo_klio