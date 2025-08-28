#ifndef ROBOT_HW_RECORD__RECORDER_NODE_HPP_
#define ROBOT_HW_RECORD__RECORDER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <chrono>
#include <thread>
#include <memory>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

// LCM相关包含
#include <lcm/lcm-cpp.hpp>

// LCM类型包含
#include "lcm_types/ImuData.hpp"
#include "lcm_types/RobotDebugData.hpp"
#include "lcm_types/UserCommandData.hpp"

using namespace std::chrono_literals;

class RecorderNode : public rclcpp::Node
{
public:
  RecorderNode();
  ~RecorderNode();

private:
  void timer_callback();
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
  void pointcloud_callback();
  
  // LCM相关函数
  void lcm_thread();
  
  // LCM消息处理函数
  void handle_robot_debug_data(const lcm::ReceiveBuffer* rbuf, const std::string& channel);
  void handle_user_command_data(const lcm::ReceiveBuffer* rbuf, const std::string& channel);
  
  // 发布者
  // 转发发布者
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  
  // TF广播器
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  size_t count_;
  
  
  // 线程
  std::thread pointcloud_thread_;
  std::thread lcm_thread_;
  
  // LCM相关成员
  std::unique_ptr<lcm::LCM> lcm_interface_;
  std::string lcm_suffix_;
  int lcm_port_;
  int lcm_ttl_;
  
  // 消息对象
  sensor_msgs::msg::Imu imu_msg_;
  sensor_msgs::msg::JointState joint_state_msg_;
  sensor_msgs::msg::JointState joint_command_msg_;
  
  // LCM通道名称
  std::string robot_debug_channel_;
  std::string user_command_channel_;

  // 日志计数（仅打印前5条）
  int robot_debug_log_count_ = 0;
  int user_cmd_log_count_ = 0;
};

#endif  // ROBOT_HW_RECORD__RECORDER_NODE_HPP_