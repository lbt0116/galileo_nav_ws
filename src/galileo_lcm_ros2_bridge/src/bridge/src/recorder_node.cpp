#include "recorder_node.hpp"

#include <map>
#include <string>

#include "std_msgs/msg/float32_multi_array.hpp"

RecorderNode::RecorderNode()
    : Node("recorder_node"),
      count_(0)
{
    // 声明参数
    this->declare_parameter("lcm_suffix", "galileo");
    this->declare_parameter("lcm_port", 20000);
    this->declare_parameter("lcm_ttl", 1);
    this->declare_parameter("lcm_iface", "enxf8e43be98da6");  // ens33


    // 获取参数
    lcm_suffix_ = this->get_parameter("lcm_suffix").as_string();
    lcm_port_ = this->get_parameter("lcm_port").as_int();
    lcm_ttl_ = this->get_parameter("lcm_ttl").as_int();

    RCLCPP_INFO(this->get_logger(), "LCM channel: %s  port: %d  ttl: %d", lcm_suffix_.c_str(), lcm_port_, lcm_ttl_);

    // 创建IMU发布者
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());

    // 创建JointState发布者
    joint_state_publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
    joint_command_publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>("joint_commands", rclcpp::SensorDataQoS());

    // 创建UserCommand转发发布者（ROS2 cmd_vel）
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // 创建TF广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // 创建订阅者

    // 初始化LCM通道名称 - 匹配Python程序配置
    robot_debug_channel_ = "lcm_robot_debug_data_" + lcm_suffix_;
    user_command_channel_ = "lcm_user_command_data_" + lcm_suffix_;

    // 初始化LCM
    std::string lcm_provider = "udpm://239.255.76.67:" + std::to_string(lcm_port_) + "?ttl=" + std::to_string(lcm_ttl_);
    lcm_interface_ = std::make_unique<lcm::LCM>(lcm_provider);

    if (!lcm_interface_->good())
    {
        RCLCPP_ERROR(this->get_logger(), "LCM初始化失败");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "LCM初始化成功");

        // 订阅LCM通道 - 匹配Python程序配置
        lcm_interface_->subscribe(robot_debug_channel_, &RecorderNode::handle_robot_debug_data, this);
        lcm_interface_->subscribe(user_command_channel_, &RecorderNode::handle_user_command_data, this);

        RCLCPP_INFO(this->get_logger(), "已订阅LCM通道");
        RCLCPP_INFO(this->get_logger(), "  机器人调试通道: %s", robot_debug_channel_.c_str());
        RCLCPP_INFO(this->get_logger(), "  用户控制通道: %s", user_command_channel_.c_str());

        // 启动LCM线程
        lcm_thread_ = std::thread(&RecorderNode::lcm_thread, this);
    }

    // 这里移除了依赖外部未定义的 RS 雷达驱动初始化代码
    // 如需启用 RS 驱动，请在本包中加入对应实现后再恢复
}

RecorderNode::~RecorderNode()
{
    // 确保线程安全退出
    if (pointcloud_thread_.joinable())
    {
        pointcloud_thread_.join();
    }
    if (lcm_thread_.joinable())
    {
        lcm_thread_.join();
    }
}

void RecorderNode::lcm_thread()
{
    RCLCPP_INFO(this->get_logger(), "LCM线程已启动，开始监听LCM消息...");

    while (rclcpp::ok() && lcm_interface_->good())
    {
        // RCLCPP_INFO(this->get_logger(), "LCM线程已启动，开始监听LCM消息...");

        // 处理LCM消息
        lcm_interface_->handle();

        // 短暂休眠避免过度占用CPU
        // std::this_thread::sleep_for(std::chrono::microseconds(1));
    }

    RCLCPP_INFO(this->get_logger(), "LCM线程已退出");
}

void RecorderNode::handle_robot_debug_data(const lcm::ReceiveBuffer* rbuf, const std::string& channel)
{
    (void)channel;  // 避免未使用参数警告
    // std::cout << "handle_robot_debug_data" << std::endl;
    lcm_types::RobotDebugData robot_debug_data;
    if (robot_debug_data.decode(rbuf->data, 0, rbuf->data_size) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "解码机器人调试数据失败");
        return;
    }

    // 发布TF变换
    auto tf_msg = geometry_msgs::msg::TransformStamped();
    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "base_link";

    // 设置位置
    tf_msg.transform.translation.x = robot_debug_data.ground_truth.position[0];
    tf_msg.transform.translation.y = robot_debug_data.ground_truth.position[1];
    tf_msg.transform.translation.z = robot_debug_data.ground_truth.position[2];

    // 设置旋转（从IMU数据获取四元数）
    tf_msg.transform.rotation.x = robot_debug_data.imu_data.quaternion.x;
    tf_msg.transform.rotation.y = robot_debug_data.imu_data.quaternion.y;
    tf_msg.transform.rotation.z = robot_debug_data.imu_data.quaternion.z;
    tf_msg.transform.rotation.w = robot_debug_data.imu_data.quaternion.w;

    tf_broadcaster_->sendTransform(tf_msg);

    // 发布关节状态
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name = robot_debug_data.joint_state_data.joint_name;
    joint_state_msg.position = robot_debug_data.joint_state_data.position;
    joint_state_msg.velocity = robot_debug_data.joint_state_data.velocity;
    joint_state_msg.effort = robot_debug_data.joint_state_data.effort;
    joint_state_publisher_->publish(joint_state_msg);

    // 发布关节命令
    auto joint_command_msg = sensor_msgs::msg::JointState();
    joint_command_msg.header.stamp = this->now();
    joint_command_msg.name = robot_debug_data.joint_command_data.joint_name;
    joint_command_msg.position = robot_debug_data.joint_command_data.position;
    joint_command_msg.velocity = robot_debug_data.joint_command_data.velocity;
    joint_command_msg.effort = robot_debug_data.joint_command_data.effort;
    joint_command_publisher_->publish(joint_command_msg);

    // 发布IMU数据
    auto imu_msg = sensor_msgs::msg::Imu();
    rclcpp::Time stamp(robot_debug_data.imu_data.utime);//?? time

    // RCLCPP_INFO(this->get_logger(), "imu_data.utime: %ld", robot_debug_data.imu_data.utime);

    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "base_link";
    imu_msg.orientation.x = robot_debug_data.imu_data.quaternion.x;
    imu_msg.orientation.y = robot_debug_data.imu_data.quaternion.y;
    imu_msg.orientation.z = robot_debug_data.imu_data.quaternion.z;
    imu_msg.orientation.w = robot_debug_data.imu_data.quaternion.w;
    imu_msg.angular_velocity.x = robot_debug_data.imu_data.angular_velocity[0];
    imu_msg.angular_velocity.y = robot_debug_data.imu_data.angular_velocity[1];
    imu_msg.angular_velocity.z = robot_debug_data.imu_data.angular_velocity[2];
    imu_msg.linear_acceleration.x = robot_debug_data.imu_data.acceleration[0];
    imu_msg.linear_acceleration.y = robot_debug_data.imu_data.acceleration[1];
    imu_msg.linear_acceleration.z = robot_debug_data.imu_data.acceleration[2];
    imu_publisher_->publish(imu_msg);

    // 仅打印前5条调试信息
    if (robot_debug_log_count_ < 5)
    {
        RCLCPP_INFO(this->get_logger(), "[LCM Debug %d] pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)",
                    robot_debug_log_count_ + 1,
                    robot_debug_data.ground_truth.position[0],
                    robot_debug_data.ground_truth.position[1],
                    robot_debug_data.ground_truth.position[2],
                    robot_debug_data.imu_data.quaternion.x,
                    robot_debug_data.imu_data.quaternion.y,
                    robot_debug_data.imu_data.quaternion.z,
                    robot_debug_data.imu_data.quaternion.w);
        robot_debug_log_count_++;
    }
}

void RecorderNode::handle_user_command_data(const lcm::ReceiveBuffer* rbuf, const std::string& channel)
{
    (void)channel;
    lcm_types::UserCommandData user_cmd;
    if (user_cmd.decode(rbuf->data, 0, rbuf->data_size) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "解码用户控制数据失败");
        return;
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = user_cmd.velocity_x;
    twist.linear.y = user_cmd.velocity_y;
    twist.angular.z = user_cmd.velocity_yaw;
    cmd_vel_publisher_->publish(twist);

    // 仅打印前5条用户控制信息
    if (user_cmd_log_count_ < 5)
    {
        RCLCPP_INFO(this->get_logger(), "[LCM UserCmd %d] vx=%.3f vy=%.3f wz=%.3f",
                    user_cmd_log_count_ + 1,
                    user_cmd.velocity_x,
                    user_cmd.velocity_y,
                    user_cmd.velocity_yaw);
        user_cmd_log_count_++;
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RecorderNode>());
    rclcpp::shutdown();
    return 0;
}