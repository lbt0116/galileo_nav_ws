#include "galileo_klio_node.h"

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

#include "timer_utils.hpp"
// 移除 glog，统一使用 ROS2 日志
namespace galileo_klio
{

bool time_list(PointType &x, PointType &y)
{
    return (x.curvature < y.curvature);
}

GalileoKLIONode::GalileoKLIONode()
    : Node("galileo_klio_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    RCLCPP_INFO(this->get_logger(), "Galileo KLIO Node 启动");

    // 初始化TF广播器
    br_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    // 初始化静态TF广播器
    static_br_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    // 初始化PCL点云
    cloud_raw_ = std::make_shared<PointCloudType>();
    cloud_down_body_ = std::make_shared<PointCloudType>();
    cloud_down_world_ = std::make_shared<PointCloudType>();

    // 初始化其他成员变量
    success_pts_size = 0;
    gravity_ = 9.81;
    acc_norm_ = 0.0;
    last_state_predict_time_ = 0.0;
    last_state_update_time_ = 0.0;
    last_timestamp_imu_ = 0.0;
    last_timestamp_kin_imu_ = 0.0;
    lidar_end_time_ = 0.0;

    // 初始化频率统计
    last_timer_time_ = 0.0;
    timer_count_ = 0;
    timer_frequency_ = 0.0;
    last_voxel_map_pub_time_ = 0.0;
    last_odom_pub_time_ = 0.0;
    odom_pub_period_ = 0.005;  // 200 Hz 默认

    // 调试与监控初始化
    debug_enable_ = false;
    debug_jump_pos_thresh_ = 0.8;       // m，一次发布间距超过该阈值认为跳变
    debug_jump_yaw_thresh_deg_ = 25.0;  // deg
    debug_log_period_ = 0.2;            // s，周期性打印
    last_debug_log_time_ = 0.0;
    has_last_pub_ = false;
    last_effect_num_ = 0;
    last_points_size_ = 0;
    last_imu_acc_.setZero();
    last_imu_gyr_.setZero();
    last_imu_time_ = 0.0;
    last_imus_size_ = 0;
    last_imu_first_time_ = 0.0;
    last_imu_last_time_ = 0.0;
    last_kin_imus_size_ = 0;
    last_kin_first_time_ = 0.0;
    last_kin_last_time_ = 0.0;
    last_cur_point_time_ = 0.0;
    last_curvature_value_ = 0.0;
    last_lidar_begin_time_ = 0.0;
    last_lidar_end_time_ = 0.0;
    last_cloud_raw_size_ = 0;
    last_cloud_down_body_size_ = 0;
    last_frame_curv_min_ = 0.0;
    last_frame_curv_max_ = 0.0;

    // 初始化传感器参数
    ext_rot_ = Mat3D::Identity();
    ext_t_ = Vec3D::Zero();
    lidar_to_body_rot_ = Mat3D::Identity();
    satu_acc_ = 0.0;
    satu_gyr_ = 0.0;
    // 输出坐标系帧名和机器人本体坐标系帧名将在initParamAndReset中从参数服务器获取

    // 初始化消息头
    odom_world_.header.frame_id = "odom";  // 临时默认值
    odom_world_.child_frame_id = "body";   // 临时默认值
    path_world_.header.frame_id = "odom";  // 临时默认值
    pose_path_.header.frame_id = "odom";   // 临时默认值

    // 初始化ROS2接口
    RCLCPP_INFO(this->get_logger(), "初始化ROS2接口");

    // 初始化腿部模型
    legs_ = legutils::createGRQ20Legs();

    // 初始化参数
    if (!initParamAndReset())
    {
        RCLCPP_ERROR(this->get_logger(), "参数初始化失败");
        return;
    }

    // 订阅话题
    auto cb_group_lidar_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto cb_group_imu_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto cb_group_joint_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions opt_lidar;
    opt_lidar.callback_group = cb_group_lidar_;

    sub_lidar_raw_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, 10, std::bind(&GalileoKLIONode::lidarCallback, this, std::placeholders::_1), opt_lidar);
    RCLCPP_INFO(this->get_logger(), "订阅激光雷达话题: %s", lidar_topic_.c_str());

    rclcpp::SubscriptionOptions opt_imu;
    opt_imu.callback_group = cb_group_imu_;

    sub_imu_raw_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10, std::bind(&GalileoKLIONode::imuCallback, this, std::placeholders::_1), opt_imu);

    RCLCPP_INFO(this->get_logger(), "订阅IMU话题: %s", imu_topic_.c_str());

    // 订阅机载九轴IMU（仅保存最新消息）
    sub_imu_onboard_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_onboard_topic_, rclcpp::SensorDataQoS(), std::bind(&GalileoKLIONode::imuOnboardCallback, this, std::placeholders::_1), opt_imu);
    RCLCPP_INFO(this->get_logger(), "订阅机载IMU话题: %s", imu_onboard_topic_.c_str());

    rclcpp::SubscriptionOptions opt_joint;
    opt_joint.callback_group = cb_group_joint_;

    // 暂时禁用机器人状态订阅
    RCLCPP_INFO(this->get_logger(), "订阅关节状态话题: %s", kinematic_topic_.c_str());
    sub_kinematic_raw_ = this->create_subscription<sensor_msgs::msg::JointState>(
        kinematic_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&GalileoKLIONode::kinematicCallback, this, std::placeholders::_1),
        opt_joint);

    // 订阅用户命令话题
    RCLCPP_INFO(this->get_logger(), "订阅用户命令话题: %s", cmd_vel_topic_.c_str());
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10,
        std::bind(&GalileoKLIONode::userCommandCallback, this, std::placeholders::_1));

    // 发布话题
    pub_pointcloud_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pointcloud_body_, 10);
    pub_pointcloud_world_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pointcloud_world_, 10);
    pub_pointcloud_world_full_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pointcloud_world_full_, 10);
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>(topic_path_, 10);
    pub_path_to_map_ = this->create_publisher<nav_msgs::msg::Path>(topic_path_to_map_, 10);
    pub_odom_world_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_odom_world_, 10);
    pub_odom_to_map_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_odom_to_map_, 10);
    pub_pointcloud_to_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pointcloud_to_map_, 10);
    pub_body_velocity_imu_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(topic_body_velocity_imu_, 10);

    // 初始化四条腿的发布器
    std::vector<std::string> leg_names = {"FR", "FL", "RR", "RL"};
    for (const auto &leg_name : leg_names)
    {
        pub_contact_scores_[leg_name] = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_contact_scores_prefix_ + leg_name, 10);
        pub_foot_position_[leg_name] = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_foot_position_prefix_ + leg_name, 10);
        pub_foot_velocity_[leg_name] = this->create_publisher<geometry_msgs::msg::TwistStamped>(topic_foot_velocity_prefix_ + leg_name, 10);
    }
    // 体素地图 MarkerArray 发布器
    // 注意：实际发布由 VoxelMapManager::pubVoxelMap 完成，这里只创建发布器实例
    // 保存地图服务
    save_map_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "save_map", std::bind(&GalileoKLIONode::handleSaveMap, this, std::placeholders::_1, std::placeholders::_2));

    // 创建定时器（5000Hz）
    timer_ = this->create_wall_timer(std::chrono::microseconds(200),  //
                                     std::bind(&GalileoKLIONode::timerCallback, this));



    RCLCPP_INFO(this->get_logger(), "ROS2接口初始化完成");
}

GalileoKLIONode::~GalileoKLIONode()
{
    RCLCPP_INFO(this->get_logger(), "Galileo KLIO Node 关闭");
}

bool GalileoKLIONode::initParamAndReset()
{
    RCLCPP_INFO(this->get_logger(), "初始化ROS2参数");

    // 声明参数
    this->declare_parameter("lidar_topic", "/points_raw");
    this->declare_parameter("imu_topic", "/imu_raw");
    this->declare_parameter("kinematic_topic", "/high_state");
    this->declare_parameter("imu_onboard_topic", std::string("/imu_onboard"));
    this->declare_parameter("cmd_vel_topic", std::string("/cmd_vel"));

    this->declare_parameter("only_imu_use", false);
    this->declare_parameter("redundancy", true);

    this->declare_parameter("init_time", 0.05);
    this->declare_parameter("gravity", 9.81);
    // 输出坐标帧名
    this->declare_parameter("output_frame_id", std::string("odom"));
    // 机器人本体坐标系帧名
    this->declare_parameter("body_frame_id", std::string("body"));
    // 激光雷达坐标系帧名
    this->declare_parameter("lidar_frame_id", std::string("rslidar"));
    this->declare_parameter("map_frame_id", std::string("map"));

    this->declare_parameter("extrinsic_T", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter("extrinsic_R", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    // 新增：IMU->Body 旋转（9元数组）
    this->declare_parameter("lidar_to_body_R", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});

    // 默认仅支持 Robosense Airy，保留参数但默认值设为 3
    this->declare_parameter("lidar_type", 3);
    this->declare_parameter("time_scale", 1.0);
    this->declare_parameter("blind", 1.5);
    this->declare_parameter("filter_num", 3);
    this->declare_parameter("point_stamp_correct", true);
    this->declare_parameter("voxel_grid_resolution", 0.5);
    this->declare_parameter("map_save_path", std::string("map.pcd"));
    // 发布话题参数
    this->declare_parameter("topic_pointcloud_body", std::string("klio/pointcloud_body"));
    this->declare_parameter("topic_pointcloud_world", std::string("klio/pointcloud_world"));
    this->declare_parameter("topic_pointcloud_world_full", std::string("klio/pointcloud_world_full"));
    this->declare_parameter("topic_path", std::string("klio/path"));
    this->declare_parameter("topic_odom_world", std::string("klio/odometry"));
    this->declare_parameter("topic_odom_to_map", std::string("klio/odometry_to_map"));
    this->declare_parameter("topic_pointcloud_to_map", std::string("klio/pointcloud_to_map"));
    this->declare_parameter("topic_path_to_map", std::string("klio/path_to_map"));
    this->declare_parameter("topic_body_velocity_imu", std::string("klio/body_velocity_imu"));
    this->declare_parameter("topic_voxel_map", std::string("klio/voxel_map"));
    this->declare_parameter("topic_contact_scores_prefix", std::string("klio/contact_scores_"));
    this->declare_parameter("topic_foot_position_prefix", std::string("klio/foot_position_"));
    this->declare_parameter("topic_foot_velocity_prefix", std::string("klio/foot_velocity_"));

    this->declare_parameter("pub_plane_en", false);
    this->declare_parameter("max_layer", 2);
    this->declare_parameter("voxel_size", 0.5);
    this->declare_parameter("min_eigen_value", 0.01);
    this->declare_parameter("sigma_num", 3.0);
    this->declare_parameter("beam_err", 0.2);
    this->declare_parameter("dept_err", 0.04);
    this->declare_parameter("layer_init_num", std::vector<int>{5, 5, 5, 5, 5});
    this->declare_parameter("max_points_num", 50);
    this->declare_parameter("max_iterations", 10);  // 添加缺失的参数
    this->declare_parameter("map_sliding_en", false);
    this->declare_parameter("half_map_size", 100);
    this->declare_parameter("sliding_thresh", 8.0);

    // 移除旧 Kinematics 几何参数声明（使用内置 legutils 模型）
    // Pinocchio URDF 与足端帧
    this->declare_parameter("pin_urdf_path", std::string(""));
    this->declare_parameter("pin_foot_frame", std::string("FL_foot"));
    this->declare_parameter("contact_force_threshold_up", 220.0);
    this->declare_parameter("contact_force_threshold_down", 200.0);

    // 接触检测参数
    this->declare_parameter("contact_knee_torque_threshold_Nm", 10.0);
    this->declare_parameter("contact_foot_height_target_m", -0.4);
    this->declare_parameter("contact_foot_height_tolerance_m", 0.05);
    this->declare_parameter("contact_knee_torque_change_scale_Nm", 5.0);
    this->declare_parameter("contact_weight_torque", 10.0);
    this->declare_parameter("contact_weight_speed", 1.0);
    this->declare_parameter("contact_weight_height", 1.5);
    this->declare_parameter("contact_weight_torque_change", 0.1);
    this->declare_parameter("contact_bias", 0.0);
    this->declare_parameter("contact_decision_threshold", 0.5);
    this->declare_parameter("contact_sliding_window_size", 6);
    this->declare_parameter("contact_state_hold_time_s", 0.1);

    this->declare_parameter("satu_acc", 35.0);
    this->declare_parameter("satu_gyr", 30.0);

    this->declare_parameter("vel_process_cov", 20.0);
    this->declare_parameter("imu_acc_process_cov", 500.0);
    this->declare_parameter("imu_gyr_process_cov", 1000.0);
    this->declare_parameter("contact_process_cov", 20.0);
    this->declare_parameter("acc_bias_process_cov", 0.001);
    this->declare_parameter("gyr_bias_process_cov", 0.001);
    this->declare_parameter("kin_bias_process_cov", 0.001);

    this->declare_parameter("imu_acc_meas_noise", 0.1);
    this->declare_parameter("imu_acc_z_meas_noise", 1.0);
    this->declare_parameter("imu_gyr_meas_noise", 0.01);
    this->declare_parameter("kin_meas_noise", 0.1);
    this->declare_parameter("chd_meas_noise", 0.1);
    this->declare_parameter("contact_meas_noise", 0.001);
    this->declare_parameter("lidar_point_meas_ratio", 10.0);
    
    // 回环检测参数
    this->declare_parameter("loop_enable", false);
    this->declare_parameter("loop_frequency", 1.0);
    this->declare_parameter("history_search_radius", 25.0);
    this->declare_parameter("history_search_time_diff", 25.0);
    this->declare_parameter("history_search_num", 25);
    this->declare_parameter("history_fitness_score", 0.4);
    this->declare_parameter("surrounding_kf_search_radius", 30.0);
    this->declare_parameter("surrounding_kf_density", 2.0);
    this->declare_parameter("mapping_surf_leaf_size", 0.4);
    this->declare_parameter("global_map_visualization_search_radius", 1000.0);
    this->declare_parameter("global_map_visualization_pose_density", 10.0);
    this->declare_parameter("global_map_visualization_leaf_size", 0.5);
    
    // 调试参数
    this->declare_parameter("debug_enable", false);
    this->declare_parameter("debug_jump_pos_thresh", 0.8);
    this->declare_parameter("debug_jump_yaw_thresh_deg", 25.0);
    this->declare_parameter("debug_log_period", 0.2);



    // 获取话题配置
    lidar_topic_ = this->get_parameter("lidar_topic").as_string();
    imu_topic_ = this->get_parameter("imu_topic").as_string();
    imu_onboard_topic_ = this->get_parameter("imu_onboard_topic").as_string();
    kinematic_topic_ = this->get_parameter("kinematic_topic").as_string();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();

    // 获取选项配置
    only_imu_use_ = this->get_parameter("only_imu_use").as_bool();
    redundancy_ = this->get_parameter("redundancy").as_bool();

    // 获取基本参数
    init_time_ = this->get_parameter("init_time").as_double();
    gravity_ = this->get_parameter("gravity").as_double();
    satu_acc_ = this->get_parameter("satu_acc").as_double();
    satu_gyr_ = this->get_parameter("satu_gyr").as_double();
    voxel_grid_resolution_ = this->get_parameter("voxel_grid_resolution").as_double();
    map_save_path_ = this->get_parameter("map_save_path").as_string();
    // 读取话题名参数
    topic_pointcloud_body_ = this->get_parameter("topic_pointcloud_body").as_string();
    topic_pointcloud_world_ = this->get_parameter("topic_pointcloud_world").as_string();
    topic_pointcloud_world_full_ = this->get_parameter("topic_pointcloud_world_full").as_string();
    topic_path_ = this->get_parameter("topic_path").as_string();
    topic_odom_world_ = this->get_parameter("topic_odom_world").as_string();
    topic_odom_to_map_ = this->get_parameter("topic_odom_to_map").as_string();
    topic_pointcloud_to_map_ = this->get_parameter("topic_pointcloud_to_map").as_string();
    topic_path_to_map_ = this->get_parameter("topic_path_to_map").as_string();
    topic_body_velocity_imu_ = this->get_parameter("topic_body_velocity_imu").as_string();
    topic_voxel_map_ = this->get_parameter("topic_voxel_map").as_string();
    topic_contact_scores_prefix_ = this->get_parameter("topic_contact_scores_prefix").as_string();
    topic_foot_position_prefix_ = this->get_parameter("topic_foot_position_prefix").as_string();
    topic_foot_velocity_prefix_ = this->get_parameter("topic_foot_velocity_prefix").as_string();

    // 获取外参
    auto extrinsic_T = this->get_parameter("extrinsic_T").as_double_array();
    auto extrinsic_R = this->get_parameter("extrinsic_R").as_double_array();
    // 保留 IMU->Lidar 旋转矩阵（用于计算 IMU->Body）
    Eigen::Matrix3d extrinsic_R_LI;
    extrinsic_R_LI.setIdentity();
    auto lidar_to_body_R = this->get_parameter("lidar_to_body_R").as_double_array();
    // 获取输出帧名
    output_frame_id_ = this->get_parameter("output_frame_id").as_string();
    // 获取机器人本体坐标系帧名
    body_frame_id_ = this->get_parameter("body_frame_id").as_string();
    // 获取激光雷达坐标系帧名
    lidar_frame_id_ = this->get_parameter("lidar_frame_id").as_string();
    map_frame_id_ = this->get_parameter("map_frame_id").as_string();

    // 配置ESKF参数
    eskf_config_.vel_process_cov = this->get_parameter("vel_process_cov").as_double();
    eskf_config_.imu_acc_process_cov = this->get_parameter("imu_acc_process_cov").as_double();
    eskf_config_.imu_gyr_process_cov = this->get_parameter("imu_gyr_process_cov").as_double();
    eskf_config_.acc_bias_process_cov = this->get_parameter("acc_bias_process_cov").as_double();
    eskf_config_.gyr_bias_process_cov = this->get_parameter("gyr_bias_process_cov").as_double();
    eskf_config_.kin_bias_process_cov = this->get_parameter("kin_bias_process_cov").as_double();
    eskf_config_.contact_process_cov = this->get_parameter("contact_process_cov").as_double();

    eskf_config_.imu_acc_meas_noise = this->get_parameter("imu_acc_meas_noise").as_double();
    eskf_config_.imu_acc_z_meas_noise = this->get_parameter("imu_acc_z_meas_noise").as_double();
    eskf_config_.imu_gyr_meas_noise = this->get_parameter("imu_gyr_meas_noise").as_double();
    eskf_config_.kin_meas_noise = this->get_parameter("kin_meas_noise").as_double();
    eskf_config_.chd_meas_noise = this->get_parameter("chd_meas_noise").as_double();
    eskf_config_.contact_meas_noise = this->get_parameter("contact_meas_noise").as_double();
    eskf_config_.lidar_point_meas_ratio = this->get_parameter("lidar_point_meas_ratio").as_double();
    // 获取调试参数
    debug_enable_ = this->get_parameter("debug_enable").as_bool();
    debug_jump_pos_thresh_ = this->get_parameter("debug_jump_pos_thresh").as_double();
    debug_jump_yaw_thresh_deg_ = this->get_parameter("debug_jump_yaw_thresh_deg").as_double();
    debug_log_period_ = this->get_parameter("debug_log_period").as_double();
    eskf_ = std::make_unique<ESKF>(eskf_config_);

    // 根据only_imu_use选择状态初始化器
    if (only_imu_use_)
    {
        state_initial_ = std::make_unique<StateInitialByImu>(gravity_);
    }
    else
    {
        state_initial_ = std::make_unique<StateInitialByKinImu>(gravity_);
    }

    // 移除旧 Kinematics 使用，统一使用 leg_utils 直接计算

    // 配置接触检测参数
    contact_params_.knee_torque_threshold_Nm = this->get_parameter("contact_knee_torque_threshold_Nm").as_double();
    contact_params_.foot_height_target_m = this->get_parameter("contact_foot_height_target_m").as_double();
    contact_params_.foot_height_tolerance_m = this->get_parameter("contact_foot_height_tolerance_m").as_double();
    contact_params_.knee_torque_change_scale_Nm =
        this->get_parameter("contact_knee_torque_change_scale_Nm").as_double();
    contact_params_.weight_torque = this->get_parameter("contact_weight_torque").as_double();
    contact_params_.weight_speed = this->get_parameter("contact_weight_speed").as_double();
    contact_params_.weight_height = this->get_parameter("contact_weight_height").as_double();
    contact_params_.weight_torque_change = this->get_parameter("contact_weight_torque_change").as_double();
    contact_params_.bias = this->get_parameter("contact_bias").as_double();
    contact_params_.decision_threshold = this->get_parameter("contact_decision_threshold").as_double();
    contact_params_.sliding_window_size = this->get_parameter("contact_sliding_window_size").as_int();
    contact_params_.state_hold_time_s = this->get_parameter("contact_state_hold_time_s").as_double();

    // 配置激光雷达处理参数
    lidar_config_.blind_ = this->get_parameter("blind").as_double();
    lidar_config_.filter_num_ = this->get_parameter("filter_num").as_int();
    lidar_config_.time_scale_ = this->get_parameter("time_scale").as_double();
    lidar_config_.point_stamp_correct_ = this->get_parameter("point_stamp_correct").as_bool();
    {
        int lt = this->get_parameter("lidar_type").as_int();
        if (lt != static_cast<int>(common::LidarType::ROBOSENSE_AIRY))
        {
            RCLCPP_WARN(this->get_logger(), "仅保留 Robosense Airy 处理，强制将 lidar_type 覆写为 3");
            lt = static_cast<int>(common::LidarType::ROBOSENSE_AIRY);
        }
        lidar_config_.lidar_type_ = static_cast<common::LidarType>(lt);
    }
    lidar_processing_ = std::make_unique<LidarProcessing>(lidar_config_);

    // 配置体素网格
    voxel_grid_.setLeafSize(voxel_grid_resolution_, voxel_grid_resolution_, voxel_grid_resolution_);

    // 配置体素地图参数
    voxel_map_config_.is_pub_plane_map_ = this->get_parameter("pub_plane_en").as_bool();
    voxel_map_config_.max_layer_ = this->get_parameter("max_layer").as_int();
    voxel_map_config_.max_voxel_size_ = this->get_parameter("voxel_size").as_double();
    voxel_map_config_.planner_threshold_ = this->get_parameter("min_eigen_value").as_double();
    voxel_map_config_.sigma_num_ = this->get_parameter("sigma_num").as_double();
    voxel_map_config_.beam_err_ = this->get_parameter("beam_err").as_double();
    voxel_map_config_.dept_err_ = this->get_parameter("dept_err").as_double();
    auto layer_init_num_int64 = this->get_parameter("layer_init_num").as_integer_array();
    voxel_map_config_.layer_init_num_.clear();
    for (const auto &val : layer_init_num_int64)
    {
        voxel_map_config_.layer_init_num_.push_back(static_cast<int>(val));
    }
    voxel_map_config_.max_points_num_ = this->get_parameter("max_points_num").as_int();
    voxel_map_config_.max_iterations_ = this->get_parameter("max_iterations").as_int();
    voxel_map_config_.map_sliding_en = this->get_parameter("map_sliding_en").as_bool();
    voxel_map_config_.half_map_size = this->get_parameter("half_map_size").as_int();
    voxel_map_config_.sliding_thresh = this->get_parameter("sliding_thresh").as_double();
    map_manager_ = std::make_unique<VoxelMapManager>(voxel_map_config_);

    // 设置外参
    if (extrinsic_T.size() >= 3)
    {
        ext_t_ << extrinsic_T[0], extrinsic_T[1], extrinsic_T[2];
    }
    if (extrinsic_R.size() >= 9)
    {
        // IMU->Lidar 旋转
        extrinsic_R_LI << extrinsic_R[0], extrinsic_R[1], extrinsic_R[2],
                          extrinsic_R[3], extrinsic_R[4], extrinsic_R[5],
                          extrinsic_R[6], extrinsic_R[7], extrinsic_R[8];
        // 存储为 Lidar->IMU 旋转
        ext_rot_ = extrinsic_R_LI;
    }

    if (lidar_to_body_R.size() >= 9)
    {
        lidar_to_body_rot_ << lidar_to_body_R[0], lidar_to_body_R[1], lidar_to_body_R[2],
                            lidar_to_body_R[3], lidar_to_body_R[4], lidar_to_body_R[5],
                            lidar_to_body_R[6], lidar_to_body_R[7], lidar_to_body_R[8];
    }

    ext_rot_.transposeInPlace();
    // 覆写消息头帧名
    odom_world_.header.frame_id = output_frame_id_;
    odom_world_.child_frame_id = body_frame_id_;
    path_world_.header.frame_id = output_frame_id_;
    pose_path_.header.frame_id = output_frame_id_;

    // 设置外参到地图管理器
    map_manager_->extT_ = ext_t_;
    map_manager_->extR_ = ext_rot_;
    // 初始化体素地图发布器
    map_manager_->voxel_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_voxel_map_, 10);
    // 传递输出修正到地图发布器
    map_manager_->outR_pub_.setIdentity();
    map_manager_->output_frame_id_ = output_frame_id_;

    // 发布静态TF：rslidar到body的固定外参变换
    publishStaticTF();

    RCLCPP_INFO(this->get_logger(), "参数初始化完成");
    RCLCPP_INFO(this->get_logger(), "激光雷达话题: %s", lidar_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "IMU话题: %s", imu_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "运动学话题: %s", kinematic_topic_.c_str());

    return true;
}

void GalileoKLIONode::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    static double last_scan_time = rclcpp::Time(msg->header.stamp).seconds();
    static double last_header_stamp = -1.0;     // 上一帧点云的 Header 时间
    static double last_expected_period = -1.0;  // 上一帧估计的扫描周期（s）
    // 使用计时器测量处理时间
    // auto start_time = std::chrono::high_resolution_clock::now();

    if (rclcpp::Time(msg->header.stamp).seconds() < last_scan_time)
    {
        RCLCPP_WARN(this->get_logger(), "检测到激光雷达数据流中的时间不一致");
        lidar_cache_.clear();
    }

    common::LidarScan lidar_scan;

    lidar_processing_->processing(msg, lidar_scan);

    // 丢包检测：对异常 scan_duration 做鲁棒性处理（仅打印告警，不做处理）
    {
        const double current_header_stamp = rclcpp::Time(msg->header.stamp).seconds();
        const double scan_duration = lidar_scan.lidar_end_time_ - lidar_scan.lidar_begin_time_;
        const double header_dt = (last_header_stamp > 0.0) ? (current_header_stamp - last_header_stamp) : -1.0;

        // 合理的周期范围，避免极小/极大的异常值污染估计（可按需要调整）
        constexpr double kMinValidPeriod = 0.01;  // 100 Hz 上限
        constexpr double kMaxValidPeriod = 0.30;  // ~3.3 Hz 下限

        // 期望周期估计：优先使用有效的扫描持续时间，否则回退到 Header 间隔；仅在有效范围内更新历史期望
        double expected_period = last_expected_period;
        if (scan_duration > kMinValidPeriod && scan_duration < kMaxValidPeriod)
        {
            expected_period = scan_duration;
            last_expected_period = scan_duration;
        }
        else if (expected_period <= 0.0 && header_dt > kMinValidPeriod && header_dt < kMaxValidPeriod)
        {
            expected_period = header_dt;
            last_expected_period = header_dt;
        }

        if (header_dt > 0.0 && expected_period > 0.0)
        {
            if (header_dt > expected_period * 1.5)
            {
                int estimated_dropped = static_cast<int>((header_dt + 0.5 * expected_period) / expected_period) - 1;
                if (estimated_dropped < 1) estimated_dropped = 1;
                RCLCPP_WARN(this->get_logger(),
                            "点云可能丢包: 间隔=%.3f s, 期望=%.3f s, 估计丢失 %d 帧 (仅告警, 不做处理)",
                            header_dt,
                            expected_period,
                            estimated_dropped);
            }
        }
        else if (scan_duration > 0.0 && scan_duration < kMinValidPeriod)
        {
            // 扫描持续时间异常过小，提示但不用于检测
            RCLCPP_WARN(
                this->get_logger(), "点云帧内时间戳异常: 扫描持续时间=%.6f s，忽略本帧用于期望周期估计", scan_duration);
        }

        last_header_stamp = current_header_stamp;
    }

    lidar_cache_.push_back(lidar_scan);
    last_scan_time = rclcpp::Time(msg->header.stamp).seconds();

    // RCLCPP_INFO(this->get_logger(), "lidar time %f end %d", rclcpp::Time(msg->header.stamp).seconds(),
    // lidar_cache_.size()); auto end_time = std::chrono::high_resolution_clock::now(); auto duration =
    // std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time); RCLCPP_DEBUG(this->get_logger(),
    // "激光雷达处理耗时: %ld 微秒", duration.count());
    return;
}

void GalileoKLIONode::imuOnboardCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // 仅保存最新机载IMU消息
    imu_onboard_latest_ = msg;
}

void GalileoKLIONode::userCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // 保存最新的用户命令消息
    user_command_latest_ = msg;

    // 记录用户命令信息（仅前几次）
    static int user_cmd_log_count = 0;
    if (user_cmd_log_count < 5)
    {
        RCLCPP_INFO(this->get_logger(), "[UserCmd %d] vx=%.3f vy=%.3f wz=%.3f",
                    user_cmd_log_count + 1,
                    msg->linear.x,
                    msg->linear.y,
                    msg->angular.z);
        user_cmd_log_count++;
    }
}

void GalileoKLIONode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    static sensor_msgs::msg::Imu last_imu_msg;
    sensor_msgs::msg::Imu::SharedPtr imu_msg = std::make_shared<sensor_msgs::msg::Imu>(*msg);

    if (redundancy_)
    {
        if (imu_msg->linear_acceleration.z == last_imu_msg.linear_acceleration.z
            && imu_msg->angular_velocity.z == last_imu_msg.angular_velocity.z)
        {
            last_imu_msg = *imu_msg;
            return;
        }
    }

    double timestamp = rclcpp::Time(imu_msg->header.stamp).seconds();

    std::lock_guard<std::mutex> lock(mutex_);

    if (timestamp < last_timestamp_imu_)
    {
        RCLCPP_WARN(this->get_logger(), "检测到IMU数据流中的时间不一致");
        imu_cache_.clear();
    }

    imu_cache_.push_back(imu_msg);
    last_imu_msg = *imu_msg;
    last_timestamp_imu_ = timestamp;

    // 记录第一帧IMU时间
    if (!time_aligned_ && first_imu_time_ < 0)
    {
        first_imu_time_ = timestamp;
        RCLCPP_INFO(this->get_logger(), "记录第一帧IMU时间: %.6f", timestamp);
    }

    // RCLCPP_INFO(this->get_logger(), "imu time %f end %d", timestamp, imu_cache_.size());
    return;
}

// void GalileoKLIONode::kinematicCallback(const galileo_klio::msg::RobotState::SharedPtr msg)
// {
//     std::lock_guard<std::mutex> lock(mutex_);
//     static double last_kin_time = rclcpp::Time(msg->header.stamp).seconds();

//     if (rclcpp::Time(msg->header.stamp).seconds() < last_kin_time)
//     {
//         RCLCPP_WARN(this->get_logger(), "检测到机器人状态数据流中的时间不一致");
//         kin_cache_.clear();
//     }

//     common::KinImuMeas kin_imu_meas;
//     kinematics_->processing(*msg, kin_imu_meas);

//     kin_cache_.push_back(kin_imu_meas);
//     last_kin_time = rclcpp::Time(msg->header.stamp).seconds();
//     return;
// }

void GalileoKLIONode::kinematicCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 时间处理：检查时间一致性
    double current_kin_time = rclcpp::Time(msg->header.stamp).seconds();
    static double last_kin_time = current_kin_time;

    if (current_kin_time < last_kin_time)
    {
        RCLCPP_WARN(this->get_logger(), "检测到关节状态数据流中的时间不一致，清空缓存");
        kin_cache_.clear();
        // 重置时间对齐状态
        time_aligned_ = false;
        first_joint_time_ = -1.0;
        first_imu_time_ = -1.0;
        time_offset_ = 0.0;
    }
    last_kin_time = current_kin_time;

    // 初始化腿部模型
    if (legs_.empty())
    {
        legs_ = legutils::createGRQ20Legs();
    }

    // 检查数据完整性
    if (msg->position.size() < 12 || msg->velocity.size() < 12 || msg->effort.size() < 12)
    {
        RCLCPP_WARN(this->get_logger(), "JointState 数据不足，需要12个关节数据（4条腿×3个关节）");
        return;
    }

    // 定义四条腿的名称和关节索引
    std::vector<std::string> leg_names = {"FR", "FL", "RR", "RL"};
    std::vector<std::vector<int>> joint_indices = {
        {0, 1, 2},   // FR: hip, thigh, calf
        {3, 4, 5},   // FL: hip, thigh, calf
        {6, 7, 8},   // RR: hip, thigh, calf
        {9, 10, 11}  // RL: hip, thigh, calf
    };

    // 获取当前时间
    const double current_time = this->now().seconds();

    // 准备数据用于processing函数
    std::vector<Eigen::Vector3d> foot_positions(4);
    std::vector<Eigen::Vector3d> foot_velocities(4);
    std::vector<bool> contacts(4);

    // 处理每条腿
    for (size_t i = 0; i < leg_names.size(); ++i)
    {
        const std::string &leg_name = leg_names[i];
        const std::vector<int> &indices = joint_indices[i];

        // 获取腿部模型
        const auto it = legs_.find(leg_name);
        if (it == legs_.end())
        {
            RCLCPP_WARN(this->get_logger(), "未找到腿模型 %s", leg_name.c_str());
            continue;
        }
        const auto &lm = it->second;

        // 提取关节数据
        Eigen::VectorXd q(3), dq(3), tau(3);
        q << msg->position[indices[0]], msg->position[indices[1]], msg->position[indices[2]];
        dq << msg->velocity[indices[0]], msg->velocity[indices[1]], msg->velocity[indices[2]];
        tau << msg->effort[indices[0]], msg->effort[indices[1]], msg->effort[indices[2]];

        // 计算运动学：足端位置和速度
        const Eigen::Vector3d foot_pos = legutils::footPosition(lm.chain, q, lm.foot_offset_in_last_link);
        const Eigen::Vector3d foot_vel = legutils::footVelocity(lm.chain, q, dq, lm.foot_offset_in_last_link);

        // 接触检测（基于 sigmoid）：输入膝关节扭矩/速度 与 足端 z 高度
        const double knee_tau = tau[2];  // 小腿关节扭矩
        const double knee_dq = dq[2];    // 小腿关节速度
        const double foot_z_world = foot_pos.z();

        // 初始化滑动窗口（如果不存在）
        if (contact_torque_windows_.find(leg_name) == contact_torque_windows_.end())
        {
            contact_torque_windows_[leg_name] = legutils::TorqueSlidingWindow(contact_params_.sliding_window_size);
        }

        // 初始化状态机（如果不存在）
        if (contact_state_machines_.find(leg_name) == contact_state_machines_.end())
        {
            contact_state_machines_[leg_name] = legutils::ContactStateMachine();
        }

        // 获取用户命令期望速度（直接使用user_command_latest_的值）
        const double desired_vx = user_command_latest_ ? user_command_latest_->linear.x : 0.3;
        const double desired_vy = user_command_latest_ ? user_command_latest_->linear.y : 0.0;

        // 获取详细得分（包含状态机处理）
        const legutils::ContactScores scores =
            legutils::contactProbabilitySigmoidDetailed(knee_tau,
                                                        knee_dq,
                                                        foot_z_world,
                                                        foot_vel.x(),
                                                        foot_vel.y(),
                                                        contact_torque_windows_[leg_name],
                                                        contact_state_machines_[leg_name],
                                                        current_time,
                                                        contact_params_,
                                                        desired_vx,
                                                        desired_vy);

        // 发布接触检测结果
        if (pub_contact_scores_.find(leg_name) != pub_contact_scores_.end())
        {
            std_msgs::msg::Float64MultiArray msg_contact;
            msg_contact.data.resize(14);
            msg_contact.data[0] = scores.raw_contact ? 1.0 : 0.0;
            msg_contact.data[1] = scores.filtered_contact ? 1.0 : 0.0;
            msg_contact.data[2] = scores.probability;
            msg_contact.data[3] = scores.total_score;
            msg_contact.data[4] = scores.f_torque;
            msg_contact.data[5] = scores.f_speed;
            msg_contact.data[6] = scores.f_height;
            msg_contact.data[7] = scores.f_torque_change;
            msg_contact.data[8] = scores.f_foot_vx;
            msg_contact.data[9] = scores.f_foot_vy;
            msg_contact.data[10] = knee_tau;
            msg_contact.data[11] = static_cast<double>(scores.state_machine_state);
            msg_contact.data[12] = contact_state_machines_[leg_name].getStateHoldTime();
            msg_contact.data[13] = current_time;
            pub_contact_scores_[leg_name]->publish(msg_contact);
        }

        // 发布足端位置
        if (pub_foot_position_.find(leg_name) != pub_foot_position_.end())
        {
            geometry_msgs::msg::PoseStamped msg_foot_pos;
            msg_foot_pos.header.stamp = this->now();
            msg_foot_pos.header.frame_id = body_frame_id_;
            msg_foot_pos.pose.position.x = foot_pos.x();
            msg_foot_pos.pose.position.y = foot_pos.y();
            msg_foot_pos.pose.position.z = foot_pos.z();
            msg_foot_pos.pose.orientation.w = 1.0;
            msg_foot_pos.pose.orientation.x = 0.0;
            msg_foot_pos.pose.orientation.y = 0.0;
            msg_foot_pos.pose.orientation.z = 0.0;
            pub_foot_position_[leg_name]->publish(msg_foot_pos);
        }

        // 发布足端速度
        if (pub_foot_velocity_.find(leg_name) != pub_foot_velocity_.end())
        {
            geometry_msgs::msg::TwistStamped msg_foot_vel;
            msg_foot_vel.header.stamp = this->now();
            msg_foot_vel.header.frame_id = body_frame_id_;
            msg_foot_vel.twist.linear.x = foot_vel.x();
            msg_foot_vel.twist.linear.y = foot_vel.y();
            msg_foot_vel.twist.linear.z = foot_vel.z();
            msg_foot_vel.twist.angular.x = 0.0;
            msg_foot_vel.twist.angular.y = 0.0;
            msg_foot_vel.twist.angular.z = 0.0;
            pub_foot_velocity_[leg_name]->publish(msg_foot_vel);
        }

        // 填充数据用于processing函数
        foot_positions[i] = foot_pos;
        foot_velocities[i] = foot_vel;
        contacts[i] = scores.filtered_contact;

        // 打印运动学结果
        // RCLCPP_INFO(this->get_logger(),
        //             "%s: pos [%.3f %.3f %.3f] vel [%.3f %.3f %.3f] contact=%s (p=%.3f)",
        //             leg_name.c_str(),
        //             foot_pos.x(), foot_pos.y(), foot_pos.z(),
        //             foot_vel.x(), foot_vel.y(), foot_vel.z(),
        //             scores.filtered_contact ? "YES" : "NO",
        //             scores.probability);
    }

    // 获取关节时间戳并处理时间对齐
    double joint_timestamp = rclcpp::Time(msg->header.stamp).seconds();

    // 记录第一帧关节时间
    if (!time_aligned_ && first_joint_time_ < 0)
    {
        first_joint_time_ = joint_timestamp;
        RCLCPP_INFO(this->get_logger(), "记录第一帧关节时间: %.6f", joint_timestamp);
    }

    // 计算时间偏移
    if (!time_aligned_ && first_joint_time_ >= 0 && first_imu_time_ >= 0)
    {
        time_offset_ = first_joint_time_ - first_imu_time_;
        time_aligned_ = true;
        RCLCPP_INFO(this->get_logger(), "时间对齐完成，偏移量: %.6f", time_offset_);
        // 为避免未对齐与已对齐数据混入，清空旧的 kin 缓存
        if (!kin_cache_.empty())
        {
            RCLCPP_WARN(this->get_logger(),
                        "时间对齐完成后清空历史 kin_cache_（size=%zu）以避免时间基准混用",
                        kin_cache_.size());
            kin_cache_.clear();
        }
    }

    // IMU 数据占位（将于时间对齐后按 aligned_timestamp 插值填充）
    Eigen::Vector3d imu_acc(0.0, 0.0, 0.0);
    Eigen::Vector3d imu_gyr(0.0, 0.0, 0.0);

    // 未完成时间对齐前，不入队，避免混入 epoch 秒级时间戳
    if (!time_aligned_)
    {
        if (debug_enable_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "KIN DBG: not aligned yet, skip enqueue. joint=%.6f first_joint=%.6f first_imu=%.6f",
                        joint_timestamp, first_joint_time_, first_imu_time_);
        }
        return;
    }

    // 计算对齐后的时间戳
    double aligned_timestamp = joint_timestamp - time_offset_;

    // 基于 aligned_timestamp 从 imu_cache_ 中选择/插值 IMU 加计与陀螺
    if (!imu_cache_.empty())
    {
        double t = aligned_timestamp;
        // 边界：若 t 不在缓存范围内，使用最近端
        double t_front = rclcpp::Time(imu_cache_.front()->header.stamp).seconds();
        double t_back = rclcpp::Time(imu_cache_.back()->header.stamp).seconds();
        if (t <= t_front || imu_cache_.size() == 1)
        {
            const auto &m = imu_cache_.front();
            imu_acc << m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z;
            imu_gyr << m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z;
        }
        else if (t >= t_back)
        {
            const auto &m = imu_cache_.back();
            imu_acc << m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z;
            imu_gyr << m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z;
        }
        else
        {
            // 在线性时间上寻找 [m0, m1] 使 t0 <= t <= t1
            sensor_msgs::msg::Imu::SharedPtr m0, m1;
            double t0 = 0.0, t1 = 0.0;
            for (size_t i = 1; i < imu_cache_.size(); ++i)
            {
                double ti_prev = rclcpp::Time(imu_cache_[i - 1]->header.stamp).seconds();
                double ti = rclcpp::Time(imu_cache_[i]->header.stamp).seconds();
                if (ti_prev <= t && t <= ti)
                {
                    m0 = imu_cache_[i - 1];
                    m1 = imu_cache_[i];
                    t0 = ti_prev;
                    t1 = ti;
                    break;
                }
            }
            if (m0 && m1 && t1 > t0)
            {
                double w = (t - t0) / (t1 - t0);
                auto lerp = [w](double a, double b) { return a + w * (b - a); };
                imu_acc.x() = lerp(m0->linear_acceleration.x, m1->linear_acceleration.x);
                imu_acc.y() = lerp(m0->linear_acceleration.y, m1->linear_acceleration.y);
                imu_acc.z() = lerp(m0->linear_acceleration.z, m1->linear_acceleration.z);
                imu_gyr.x() = lerp(m0->angular_velocity.x, m1->angular_velocity.x);
                imu_gyr.y() = lerp(m0->angular_velocity.y, m1->angular_velocity.y);
                imu_gyr.z() = lerp(m0->angular_velocity.z, m1->angular_velocity.z);
            }
            else
            {
                // 保险：若未找到区间，退回最近端
                const auto &m = imu_cache_.front();
                imu_acc << m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z;
                imu_gyr << m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z;
            }
        }
    }
    // 调试：打印关节原始时间与对齐时间
    // RCLCPP_INFO(this->get_logger(),
    //             "KIN DBG: joint=%.6f offset=%.6f aligned=%.6f first_joint=%.6f first_imu=%.6f",
    //             joint_timestamp, time_offset_, aligned_timestamp, first_joint_time_, first_imu_time_);

    // 若对齐后的时间出现倒退，清空缓存以维持单调性
    if (last_timestamp_kin_imu_ > 0.0 && aligned_timestamp + 1e-6 < last_timestamp_kin_imu_)
    {
        if (debug_enable_)
        {
            RCLCPP_WARN(this->get_logger(),
                        "KIN DBG: aligned timestamp moved backwards (aligned=%.6f < last=%.6f), clear kin_cache_",
                        aligned_timestamp, last_timestamp_kin_imu_);
        }
        kin_cache_.clear();
    }

    // 调用processing函数进行数据包装和坐标转换
    galileo_klio::common::KinImuMeas kin_imu_meas;
    legutils::processing(
                 aligned_timestamp, foot_positions, foot_velocities, imu_acc, imu_gyr, contacts,
         kin_imu_meas, ext_t_, ext_rot_, lidar_to_body_rot_);

    // 由足端速度反推 IMU系机身速度（接触腿加权平均）并发布
    {
        int contact_count = 0;
        Eigen::Vector3d vel_sum = Eigen::Vector3d::Zero();
        // 观测模型：w_skew * foot_pos + foot_vel ≈ - R^T * vel_body
        // 这里仅用足端线速度近似：vel_body_imu ≈ - foot_vel_imu（接触条件近似足端为零速度）
        for (int i = 0; i < 4; ++i)
        {
            if (contacts[i])
            {
                Eigen::Vector3d foot_vel_imu(
                    kin_imu_meas.foot_vel_[i][0],
                    kin_imu_meas.foot_vel_[i][1],
                    kin_imu_meas.foot_vel_[i][2]);
                vel_sum += -foot_vel_imu;
                contact_count++;
            }
        }
        if (contact_count > 0)
        {
            Eigen::Vector3d vel_body_imu = eskf_->state().rot_ *vel_sum / static_cast<double>(contact_count);
            geometry_msgs::msg::TwistStamped msg_vel;
            msg_vel.header.stamp = this->now();
            msg_vel.header.frame_id = "imu";
            msg_vel.twist.linear.x = vel_body_imu.x();
            msg_vel.twist.linear.y = vel_body_imu.y();
            msg_vel.twist.linear.z = vel_body_imu.z();
            pub_body_velocity_imu_->publish(msg_vel);
        }
    }

    // 将处理后的数据添加到缓存中
    kin_cache_.push_back(kin_imu_meas);

    // 更新last_timestamp_kin_imu_为对齐后的时间戳
    last_timestamp_kin_imu_ = aligned_timestamp;

    return;
}

void GalileoKLIONode::runReset()
{
    cloud_raw_.reset(new PointCloudType());
    cloud_down_body_.reset(new PointCloudType());
    cloud_down_world_.reset(new PointCloudType());

    success_pts_size = 0;
}

void GalileoKLIONode::timerCallback()
{

    if (!this->syncPackage()) return;
    this->runReset();

    // RCLCPP_DEBUG(this->get_logger(), "syncPackage is finished");

    cloud_raw_ = measure_.lidar_scan_.cloud_;
    auto &imus = measure_.imus_;
    auto &kin_imus = measure_.kin_imus_;
    double begin_time = measure_.lidar_scan_.lidar_begin_time_;
    double end_time = measure_.lidar_scan_.lidar_end_time_;

    if (cloud_raw_->points.empty() || (only_imu_use_ && imus.empty()) || (!only_imu_use_ && kin_imus.empty()))
    {
        RCLCPP_WARN(this->get_logger(), "Data packet is not ready");
        if (debug_enable_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "DPKT DBG: cloud=%zu imus=%zu kin_imus=%zu",
                        cloud_raw_->points.size(),
                        imus.size(),
                        kin_imus.size());
        }
        return;
    }

    /* initialization */
    if (init_flag_)
    {
        RCLCPP_INFO(this->get_logger(), "init_flag_");

        state_initial_->processing(measure_, *eskf_);
        RCLCPP_INFO(this->get_logger(), "state_initial_processing");

        this->cloudLidarToWorld(cloud_raw_, cloud_down_world_);
        map_manager_->feats_down_body_ = cloud_raw_;
        map_manager_->feats_down_world_ = cloud_down_world_;
        map_manager_->BuildVoxelMap(
            eskf_->state().rot_, eskf_->cov().block<3, 3>(0, 0), eskf_->cov().block<3, 3>(3, 3));

        auto gravity = eskf_->state().grav_;
        auto bw = eskf_->state().bw_;
        RCLCPP_INFO(this->get_logger(), "Initialization is finished");
        RCLCPP_INFO(this->get_logger(), "Gravity is initialized to %.3f %.3f %.3f", gravity(0), gravity(1), gravity(2));
        RCLCPP_INFO(this->get_logger(), "IMU bw is initialized to %.3f %.3f %.3f", bw(0), bw(1), bw(2));

        init_flag_ = false;
        acc_norm_ = state_initial_->getAccNorm();
        last_state_predict_time_ = end_time;
        last_state_update_time_ = end_time;
        // 若有机载IMU消息，则记录初始化姿态（仅记录一次）
        if (!has_imu_onboard_init_ && imu_onboard_latest_)
        {
            imu_onboard_init_q_ = Eigen::Quaterniond(
                imu_onboard_latest_->orientation.w,
                imu_onboard_latest_->orientation.x,
                imu_onboard_latest_->orientation.y,
                imu_onboard_latest_->orientation.z);
            has_imu_onboard_init_ = true;
        }
        return;
    }

    /* downsampling */
    Timer::measure("Downsampling",
                   [&, this]()
                   {
                       // voxel_grid_->filter(cloud_raw_, cloud_down_body_);
                       voxel_grid_.setInputCloud(cloud_raw_);
                       voxel_grid_.filter(*cloud_down_body_);
                   });

    Timer::measure(
        "State predict/update & Map update",
        [&, this]()
        {
            /* pointcloud sort*/
            std::sort(cloud_down_body_->points.begin(), cloud_down_body_->points.end(), time_list);

            /* predict and update (point/imu/kin.)*/
            // points: [idx_i, idx_j)  left close right open, time compressing
            const auto &pts = cloud_down_body_->points;
            size_t pts_size = pts.size();
            size_t idx_i = 0;
            cloud_down_world_->points.resize(pts_size);
            if (debug_enable_)
            {
                last_cloud_raw_size_ = cloud_raw_->points.size();
                last_cloud_down_body_size_ = pts_size;
                if (pts_size > 0)
                {
                    // 计算该帧曲率最小/最大（作为时间偏移代理）
                    float cmin = pts.front().curvature, cmax = pts.front().curvature;
                    for (size_t k = 1; k < pts_size; ++k)
                    {
                        cmin = std::min(cmin, pts[k].curvature);
                        cmax = std::max(cmax, pts[k].curvature);
                    }
                    last_frame_curv_min_ = static_cast<double>(cmin);
                    last_frame_curv_max_ = static_cast<double>(cmax);
                }
                else
                {
                    last_frame_curv_min_ = 0.0;
                    last_frame_curv_max_ = 0.0;
                }
                last_lidar_begin_time_ = begin_time;
                last_lidar_end_time_ = end_time;
                // 记录当前可用 IMU/kin 队列窗口
                last_imus_size_ = imus.size();
                if (!imus.empty())
                {
                    last_imu_first_time_ = rclcpp::Time(imus.front()->header.stamp).seconds();
                    last_imu_last_time_ = rclcpp::Time(imus.back()->header.stamp).seconds();
                }
                else
                {
                    last_imu_first_time_ = last_imu_last_time_ = 0.0;
                }
                last_kin_imus_size_ = kin_imus.size();
                if (!kin_imus.empty())
                {
                    last_kin_first_time_ = kin_imus.front().time_stamp_;
                    last_kin_last_time_ = kin_imus.back().time_stamp_;
                }
                else
                {
                    last_kin_first_time_ = last_kin_last_time_ = 0.0;
                }
            }
            while (idx_i < pts_size)
            {
                double cur_point_time = begin_time + pts[idx_i].curvature;
                if (debug_enable_)
                {
                    last_cur_point_time_ = cur_point_time;
                    last_curvature_value_ = static_cast<double>(pts[idx_i].curvature);
                }
                size_t idx_j = idx_i + 1;
                while (idx_j < pts_size && pts[idx_i].curvature == pts[idx_j].curvature)
                {
                    idx_j++;
                }

                if (only_imu_use_)
                {
                    while (!imus.empty() && rclcpp::Time(imus.front()->header.stamp).seconds() < cur_point_time)
                    {
                        /* process imu*/
                        this->predictUpdateImu(imus.front());
                        imus.pop_front();
                    }
                }
                else
                {
                    while (!kin_imus.empty() && kin_imus.front().time_stamp_ < cur_point_time)
                    {
                        /* process kin imu*/
                        this->predictUpdateKinImu(kin_imus.front());
                        kin_imus.pop_front();
                    }
                }

                /* process points*/
                this->predictUpdatePoint(cur_point_time, idx_i, idx_j);

                // 高频发布里程计/TF（轻量），以点时间戳为准，节流到 odom_pub_period_
                double now_sec_od = cur_point_time;
                if (now_sec_od - last_odom_pub_time_ >= odom_pub_period_)
                {
                    this->publishOdomTF(now_sec_od);
                    last_odom_pub_time_ = now_sec_od;
                    if (debug_enable_ && (success_pts_size < 5))
                    {
                        // RCLCPP_WARN(this->get_logger(), "PUBLISH WITH FEW MATCH: success_pts=%zu at t=%.3f",
                        // static_cast<size_t>(success_pts_size), now_sec_od);
                    }
                }

                idx_i = idx_j;
            }
        });

    this->publishOdomTFPath(end_time);//slow pub
    this->publishPointcloudWorld(end_time);
    // 发布给地图节点的基于初始化IMU世界系的数据（不使用实时IMU旋转）
    this->publishMappingInputs(end_time);
    // 发布未降采样且已转换到世界系的点云，时间戳使用 begin_time（与原始点云保持一致的起始戳）
    // this->publishPointcloudWorldFull(begin_time);
    // 发布体素地图（节流，例如10Hz）
    if (voxel_map_config_.is_pub_plane_map_)
    {
        double now_sec = this->now().seconds();
        if (now_sec - last_voxel_map_pub_time_ > 0.1)
        {
            if (map_manager_ && map_manager_->voxel_map_pub_)
            {
                map_manager_->pubVoxelMap();
            }
            last_voxel_map_pub_time_ = now_sec;
        }
    }
    // this->publishPointcloudBody(end_time);  // 添加body坐标系点云发布



    return;
}

bool GalileoKLIONode::syncPackage()
{
    static bool lidar_push_ = false;

    std::lock_guard<std::mutex> lock(mutex_);

    // pack lidar and imu
    if (only_imu_use_)
    {
        if (lidar_cache_.empty() || imu_cache_.empty())
        {
            return false;
        }

        if (!lidar_push_)
        {
            measure_.lidar_scan_ = lidar_cache_.front();
            lidar_end_time_ = measure_.lidar_scan_.lidar_end_time_;
            lidar_push_ = true;
        }

        if (last_timestamp_imu_ < lidar_end_time_)
        {
            // RCLCPP_INFO(this->get_logger(), "lidar>imu %f %f", lidar_end_time_, last_timestamp_imu_);
            return false;
        }

        double imu_time = rclcpp::Time(imu_cache_.front()->header.stamp).seconds();
        measure_.imus_.clear();
        while ((!imu_cache_.empty()) && (imu_time < lidar_end_time_))
        {
            imu_time = rclcpp::Time(imu_cache_.front()->header.stamp).seconds();
            if (imu_time > lidar_end_time_) break;
            measure_.imus_.push_back(imu_cache_.front());
            imu_cache_.pop_front();
        }

        lidar_cache_.pop_front();
        lidar_push_ = false;

        return true;
    }

    // pack lidar, kin. and  imu
    if (!only_imu_use_)
    {
        if (lidar_cache_.empty() || kin_cache_.empty() || imu_cache_.empty())
        {
            return false;
        }

        // 保护：若检测到kin缓存时间基准混用（队头时间明显大于队尾时间），清空并等待新数据
        if (kin_cache_.size() >= 2)
        {
            double kin_front_ts = kin_cache_.front().time_stamp_;
            double kin_back_ts = kin_cache_.back().time_stamp_;
            if (kin_front_ts > kin_back_ts + 1.0)
            {
                RCLCPP_WARN(this->get_logger(),
                            "KIN DBG: Detected mixed time bases in kin_cache_ (front=%.6f back=%.6f). Clear cache.",
                            kin_front_ts, kin_back_ts);
                kin_cache_.clear();
                return false;
            }
        }

        if (!lidar_push_)
        {
            measure_.lidar_scan_ = lidar_cache_.front();
            lidar_end_time_ = measure_.lidar_scan_.lidar_end_time_;
            lidar_push_ = true;
        }

        // 调试：打印当前激光帧的时间窗口与kin缓存窗口
        if (debug_enable_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "SYNC DBG: lidar_end=%.6f last_kin=%.6f kin_cache_size=%zu front=%.6f back=%.6f",
                        lidar_end_time_,
                        last_timestamp_kin_imu_,
                        kin_cache_.size(),
                        kin_cache_.empty() ? -1.0 : kin_cache_.front().time_stamp_,
                        kin_cache_.empty() ? -1.0 : kin_cache_.back().time_stamp_);
        }

        if (last_timestamp_kin_imu_ < lidar_end_time_)
        {
            // RCLCPP_INFO(this->get_logger(),
            //             "syncPackage last_timestamp_kin_imu_ < lidar_end_time_ (last_kin=%.6f, lidar_end=%.6f)",
            //             last_timestamp_kin_imu_, lidar_end_time_);
            return false;
        }

        double kin_time = kin_cache_.front().time_stamp_;
        measure_.kin_imus_.clear();
        while ((!kin_cache_.empty()) && (kin_time < lidar_end_time_))
        {
            kin_time = kin_cache_.front().time_stamp_;
            if (kin_time > lidar_end_time_) break;
            measure_.kin_imus_.push_back(kin_cache_.front());
            kin_cache_.pop_front();
        }

        // 调试：统计本帧打包的kin条目
        if (measure_.kin_imus_.empty())
        {
            if (debug_enable_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "SYNC DBG: No kin entries < lidar_end (%.6f). kin_cache_size=%zu front=%.6f back=%.6f",
                            lidar_end_time_,
                            kin_cache_.size(),
                            kin_cache_.empty() ? -1.0 : kin_cache_.front().time_stamp_,
                            kin_cache_.empty() ? -1.0 : kin_cache_.back().time_stamp_);
            }
        }
        else
        {
            if (debug_enable_)
            {
                RCLCPP_INFO(this->get_logger(),
                            "SYNC DBG: Packed %zu kin entries in (%.6f, %.6f]",
                            measure_.kin_imus_.size(),
                            measure_.kin_imus_.front().time_stamp_,
                            measure_.kin_imus_.back().time_stamp_);
            }
        }

        lidar_cache_.pop_front();
        lidar_push_ = false;

        return true;
    }

    // 数据同步逻辑
    throw std::runtime_error("Error sync package");
    return false;
}


bool GalileoKLIONode::predictUpdateImu(const sensor_msgs::msg::Imu::SharedPtr &imu)
{
    double current_time = rclcpp::Time(imu->header.stamp).seconds();
    double dt_cov = current_time - last_state_update_time_;
    eskf_->predict(dt_cov, false, true);
    double dt = current_time - last_state_predict_time_;
    eskf_->predict(dt, true, false);
    last_state_predict_time_ = current_time;

    ObsShared obs_shared;
    obs_shared.ki_R.resize(6);
    obs_shared.ki_z.resize(6);
    Vec3D imu_acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
    Vec3D imu_gyr(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
    obs_shared.ki_z.block<3, 1>(0, 0) = (gravity_ / acc_norm_) * imu_acc - eskf_->state().imu_a_ - eskf_->state().ba_;
    obs_shared.ki_z.block<3, 1>(3, 0) = imu_gyr - eskf_->state().imu_w_ - eskf_->state().bw_;

    obs_shared.ki_R << eskf_->config().imu_acc_meas_noise, eskf_->config().imu_acc_meas_noise,
        eskf_->config().imu_acc_z_meas_noise, eskf_->config().imu_gyr_meas_noise, eskf_->config().imu_gyr_meas_noise,
        eskf_->config().imu_gyr_meas_noise;

    eskf_->updateByImu(obs_shared);
    last_state_update_time_ = current_time;
    if (debug_enable_)
    {
        last_imu_time_ = current_time;
        last_imu_acc_ = imu_acc;
        last_imu_gyr_ = imu_gyr;
    }
    return true;
}


bool GalileoKLIONode::predictUpdateKinImu(const common::KinImuMeas &kin_imu)
{
    double current_time = kin_imu.time_stamp_;
    double dt_cov = current_time - last_state_update_time_;
    eskf_->predict(dt_cov, false, true);
    double dt = current_time - last_state_predict_time_;
    eskf_->predict(dt, true, false);
    last_state_predict_time_ = current_time;

    int contact_nums = 0;
    for (int i = 0; i < 4; ++i)
    {
        if (kin_imu.contact_[i])
        {
            contact_nums++;
        }
    }

    ObsShared obs_shared;
    obs_shared.ki_R.resize(6 + 3 * contact_nums);
    obs_shared.ki_z.resize(6 + 3 * contact_nums);
    obs_shared.ki_h.resize(6 + 3 * contact_nums, DIM_STATE);
    obs_shared.ki_h.setZero();

    obs_shared.ki_h.block<6, 6>(0, 9) = Eigen::Matrix<double, 6, 6>::Identity();
    obs_shared.ki_h.block<6, 6>(0, 18) = Eigen::Matrix<double, 6, 6>::Identity();
    Vec3D imu_acc(kin_imu.acc_[0], kin_imu.acc_[1], kin_imu.acc_[2]);
    Vec3D imu_gyr(kin_imu.gyr_[0], kin_imu.gyr_[1], kin_imu.gyr_[2]);
    obs_shared.ki_z.block<3, 1>(0, 0) = (gravity_ / acc_norm_) * imu_acc - eskf_->state().imu_a_ - eskf_->state().ba_;
    obs_shared.ki_z.block<3, 1>(3, 0) = imu_gyr - eskf_->state().imu_w_ - eskf_->state().bw_;

    obs_shared.ki_R.block<6, 1>(0, 0) << eskf_->config().imu_acc_meas_noise, eskf_->config().imu_acc_meas_noise,
        eskf_->config().imu_acc_z_meas_noise, eskf_->config().imu_gyr_meas_noise, eskf_->config().imu_gyr_meas_noise,
        eskf_->config().imu_gyr_meas_noise;

    int idx = 0;
    Mat3D w_skew = SKEW_SYM_MATRIX(eskf_->state().imu_w_);
    for (int i = 0; i < 4; ++i)
    {
        if (kin_imu.contact_[i])
        {
            Vec3D foot_pos(kin_imu.foot_pos_[i][0], kin_imu.foot_pos_[i][1], kin_imu.foot_pos_[i][2]);
            Vec3D foot_vel(kin_imu.foot_vel_[i][0], kin_imu.foot_vel_[i][1], kin_imu.foot_vel_[i][2]);

            // Vec3D w_skew_pos_vel =  foot_vel;
            Vec3D w_skew_pos_vel = w_skew * foot_pos + foot_vel;

            obs_shared.ki_h.block<3, 3>(6 + 3 * idx, 0) = -eskf_->state().rot_ * SKEW_SYM_MATRIX(w_skew_pos_vel);
            obs_shared.ki_h.block<3, 3>(6 + 3 * idx, 6) = Mat3D::Identity();
            obs_shared.ki_h.block<3, 3>(6 + 3 * idx, 21) = -eskf_->state().rot_ * SKEW_SYM_MATRIX(foot_pos);

            obs_shared.ki_z.block<3, 1>(6 + 3 * idx, 0) = -eskf_->state().vel_ - eskf_->state().rot_ * w_skew_pos_vel;

            obs_shared.ki_R.block<3, 1>(6 + 3 * idx, 0) << eskf_->config().kin_meas_noise,
                eskf_->config().kin_meas_noise, eskf_->config().kin_meas_noise;
            idx++;
        }
    }

    eskf_->updateByKinImu(obs_shared);
    last_state_update_time_ = current_time;
    return true;
}


bool GalileoKLIONode::predictUpdatePoint(double current_time, size_t idx_i, size_t idx_j)
{
    /* 1. predict state*/
    double dt_cov = current_time - last_state_update_time_;
    eskf_->predict(dt_cov, false, true);
    double dt = current_time - last_state_predict_time_;
    eskf_->predict(dt, true, false);
    last_state_predict_time_ = current_time;

    /* 2. residual compute*/
    size_t points_size = idx_j - idx_i;
    std::vector<PointToPlane> ptpl_list;
    std::vector<pointWithVar> pv_list(points_size);
    ptpl_list.reserve(points_size);
    for (size_t i = 0; i < points_size; ++i)
    {
        PointType &cur_pt = cloud_down_body_->points[i + idx_i];

        /* 2.1 point var(body and world) compute*/
        pointWithVar &cur_pt_var = pv_list[i];
        cur_pt_var.point_b << cur_pt.x, cur_pt.y, cur_pt.z;
        cur_pt_var.point_i = ext_rot_ * cur_pt_var.point_b + ext_t_;
        cur_pt_var.point_w = eskf_->state().rot_ * cur_pt_var.point_i + eskf_->state().pos_;
        cloud_down_world_->points[idx_i + i].x = cur_pt_var.point_w(0);
        cloud_down_world_->points[idx_i + i].y = cur_pt_var.point_w(1);
        cloud_down_world_->points[idx_i + i].z = cur_pt_var.point_w(2);
        cloud_down_world_->points[idx_i + i].intensity = 0;
        calcBodyCov(cur_pt_var.point_b,
                    map_manager_->config_setting_.dept_err_,
                    map_manager_->config_setting_.beam_err_,
                    cur_pt_var.body_var);
        cur_pt_var.point_crossmat << SKEW_SYM_MATRIX(cur_pt_var.point_i);
        M3D rot_extR = eskf_->state().rot_ * ext_rot_;
        M3D rot_crossmat = eskf_->state().rot_ * cur_pt_var.point_crossmat;
        cur_pt_var.var = rot_extR * cur_pt_var.body_var * rot_extR.transpose()
                         + rot_crossmat * eskf_->cov().block<3, 3>(0, 0) * rot_crossmat.transpose()
                         + eskf_->cov().block<3, 3>(3, 3);

        /* 2.2 point residual */
        float loc_xyz[3];
        for (int j = 0; j < 3; j++)
        {
            loc_xyz[j] = cur_pt_var.point_w[j] / map_manager_->config_setting_.max_voxel_size_;
            if (loc_xyz[j] < 0)
            {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        auto iter = map_manager_->voxel_map_.find(position);
        if (iter != map_manager_->voxel_map_.end())
        {
            VoxelOctoTree *current_octo = iter->second;
            PointToPlane single_ptpl;
            bool is_success = false;
            double prob = 0;
            map_manager_->build_single_residual(cur_pt_var, current_octo, 0, is_success, prob, single_ptpl);
            if (!is_success)
            {
                VOXEL_LOCATION near_position = position;
                if (loc_xyz[0] > (current_octo->voxel_center_[0] + current_octo->quater_length_))
                {
                    near_position.x = near_position.x + 1;
                }
                else if (loc_xyz[0] < (current_octo->voxel_center_[0] - current_octo->quater_length_))
                {
                    near_position.x = near_position.x - 1;
                }
                if (loc_xyz[1] > (current_octo->voxel_center_[1] + current_octo->quater_length_))
                {
                    near_position.y = near_position.y + 1;
                }
                else if (loc_xyz[1] < (current_octo->voxel_center_[1] - current_octo->quater_length_))
                {
                    near_position.y = near_position.y - 1;
                }
                if (loc_xyz[2] > (current_octo->voxel_center_[2] + current_octo->quater_length_))
                {
                    near_position.z = near_position.z + 1;
                }
                else if (loc_xyz[2] < (current_octo->voxel_center_[2] - current_octo->quater_length_))
                {
                    near_position.z = near_position.z - 1;
                }
                auto iter_near = map_manager_->voxel_map_.find(near_position);
                if (iter_near != map_manager_->voxel_map_.end())
                {
                    map_manager_->build_single_residual(
                        cur_pt_var, iter_near->second, 0, is_success, prob, single_ptpl);
                }
            }
            if (is_success)
            {
                ++success_pts_size;
                ptpl_list.push_back(single_ptpl);
            }
        }
    }

    /* 3. kalman filter update*/
    size_t effect_num = ptpl_list.size();
    if (debug_enable_)
    {
        last_effect_num_ = effect_num;
        last_points_size_ = points_size;
        if (points_size == 0 || effect_num == 0)
        {
            // RCLCPP_WARN(this->get_logger(), "LOW MATCH: t=%.3f points=%zu effect=%zu", current_time, points_size,
            // effect_num);
        }
    }
    bool eskf_update = effect_num > 0;
    if (eskf_update)
    {
        ObsShared obs_shared;
        obs_shared.pt_h.resize(effect_num, 6);
        obs_shared.pt_R.resize(effect_num);
        obs_shared.pt_z.resize(effect_num);
        for (size_t k = 0; k < effect_num; ++k)
        {
            V3D crossmat_rotT_u = ptpl_list[k].point_crossmat_ * eskf_->state().rot_.transpose() * ptpl_list[k].normal_;
            obs_shared.pt_h.row(k) << crossmat_rotT_u(0), crossmat_rotT_u(1), crossmat_rotT_u(2),
                ptpl_list[k].normal_(0), ptpl_list[k].normal_(1), ptpl_list[k].normal_(2);

            obs_shared.pt_z(k) = -ptpl_list[k].dis_to_plane_;

            Eigen::Matrix<double, 1, 6> J_nq;
            J_nq.block<1, 3>(0, 0) = ptpl_list[k].point_w_ - ptpl_list[k].center_;
            J_nq.block<1, 3>(0, 3) = -ptpl_list[k].normal_;
            M3D var;
            var = eskf_->state().rot_ * ext_rot_ * ptpl_list[k].body_cov_ * ext_rot_.transpose()
                  * eskf_->state().rot_.transpose();
            double single_l = J_nq * ptpl_list[k].plane_var_ * J_nq.transpose();
            obs_shared.pt_R(k) = eskf_->config().lidar_point_meas_ratio
                                 * (single_l + ptpl_list[k].normal_.transpose() * var * ptpl_list[k].normal_);
        }
        eskf_->updateByPoints(obs_shared);
        last_state_update_time_ = current_time;
    }

    /* 4. voxelmap update*/
    if (eskf_update)
    {
        for (size_t i = 0; i < points_size; ++i)
        {
            pv_list[i].point_w = eskf_->state().rot_ * pv_list[i].point_i + eskf_->state().pos_;
            cloud_down_world_->points[idx_i + i].x = pv_list[i].point_w(0);
            cloud_down_world_->points[idx_i + i].y = pv_list[i].point_w(1);
            cloud_down_world_->points[idx_i + i].z = pv_list[i].point_w(2);
            cloud_down_world_->points[idx_i + i].intensity = 255;

            M3D rot_extR = eskf_->state().rot_ * ext_rot_;
            M3D rot_crossmat = eskf_->state().rot_ * pv_list[i].point_crossmat;
            pv_list[i].var = rot_extR * pv_list[i].body_var * rot_extR.transpose()
                             + rot_crossmat * eskf_->cov().block<3, 3>(0, 0) * rot_crossmat.transpose()
                             + eskf_->cov().block<3, 3>(3, 3);
        }
    }
    map_manager_->UpdateVoxelMap(pv_list);

    return effect_num > 0;
}

void GalileoKLIONode::cloudLidarToWorld(const CloudPtr &cloud_lidar, CloudPtr &cloud_world)
{
    cloud_world->clear();
    cloud_world->points.resize(cloud_lidar->points.size());
    for (size_t i = 0; i < cloud_lidar->points.size(); ++i)
    {
        pointLidarToWorld(cloud_lidar->points[i], cloud_world->points[i]);
    }
}
inline void GalileoKLIONode::pointLidarToImu(const PointType &point_lidar, PointType &point_imu)
{
    Eigen::Vector3d pt_lidar(point_lidar.x, point_lidar.y, point_lidar.z);
    Eigen::Vector3d pt_imu = ext_rot_ * pt_lidar + ext_t_;

    point_imu.x = static_cast<float>(pt_imu(0));
    point_imu.y = static_cast<float>(pt_imu(1));
    point_imu.z = static_cast<float>(pt_imu(2));
    point_imu.intensity = point_lidar.intensity;
}
inline void GalileoKLIONode::pointLidarToWorld(const PointType &point_lidar, PointType &point_world)
{
    Eigen::Vector3d pt_lidar(point_lidar.x, point_lidar.y, point_lidar.z);
    Eigen::Vector3d pt_imu = ext_rot_ * pt_lidar + ext_t_;
    Eigen::Vector3d pt_world = eskf_->state().rot_ * pt_imu + eskf_->state().pos_;

    point_world.x = static_cast<float>(pt_world(0));
    point_world.y = static_cast<float>(pt_world(1));
    point_world.z = static_cast<float>(pt_world(2));
    point_world.intensity = point_lidar.intensity;
}
void GalileoKLIONode::publishOdomTFPath(double end_time)
{
    // odometry (apply output rotation)
    odom_world_.header.stamp = rclcpp::Time(end_time);
    odom_world_.header.frame_id = output_frame_id_;
    Eigen::Vector3d pos = eskf_->state().pos_;
    Eigen::Matrix3d R = eskf_->getRot();
    Eigen::Vector3d pos_corr = pos;
    Eigen::Matrix3d R_corr = R;
    q_eigen_ = Eigen::Quaterniond(R_corr);
    odom_world_.pose.pose.position.x = pos_corr.x();
    odom_world_.pose.pose.position.y = pos_corr.y();
    odom_world_.pose.pose.position.z = pos_corr.z();
    odom_world_.pose.pose.orientation.w = q_eigen_.w();
    odom_world_.pose.pose.orientation.x = q_eigen_.x();
    odom_world_.pose.pose.orientation.y = q_eigen_.y();
    odom_world_.pose.pose.orientation.z = q_eigen_.z();
    pub_odom_world_->publish(odom_world_);

    // tf2
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = odom_world_.header.stamp;
    transform_stamped.header.frame_id = output_frame_id_;
    transform_stamped.child_frame_id = body_frame_id_;
    transform_stamped.transform.translation.x = odom_world_.pose.pose.position.x;
    transform_stamped.transform.translation.y = odom_world_.pose.pose.position.y;
    transform_stamped.transform.translation.z = odom_world_.pose.pose.position.z;
    transform_stamped.transform.rotation.w = odom_world_.pose.pose.orientation.w;
    transform_stamped.transform.rotation.x = odom_world_.pose.pose.orientation.x;
    transform_stamped.transform.rotation.y = odom_world_.pose.pose.orientation.y;
    transform_stamped.transform.rotation.z = odom_world_.pose.pose.orientation.z;
    // br_->sendTransform(transform_stamped);

    // path
    pose_path_.header.stamp = odom_world_.header.stamp;
    pose_path_.header.frame_id = output_frame_id_;
    pose_path_.pose = odom_world_.pose.pose;
    path_world_.poses.push_back(pose_path_);
    pub_path_->publish(path_world_);
}

void GalileoKLIONode::publishOdomTF(double stamp_sec)
{
    // odometry (apply output rotation)
    odom_world_.header.stamp = rclcpp::Time(stamp_sec);
    odom_world_.header.frame_id = output_frame_id_;
    Eigen::Vector3d pos = eskf_->state().pos_;
    Eigen::Vector3d vel = eskf_->state().vel_;
    Eigen::Matrix3d R = eskf_->getRot();
    Eigen::Vector3d pos_corr = pos;
    Eigen::Matrix3d R_corr = R;
    q_eigen_ = Eigen::Quaterniond(R_corr);
    odom_world_.pose.pose.position.x = pos_corr.x();
    odom_world_.pose.pose.position.y = pos_corr.y();
    odom_world_.pose.pose.position.z = pos_corr.z();
    odom_world_.pose.pose.orientation.w = q_eigen_.w();
    odom_world_.pose.pose.orientation.x = q_eigen_.x();
    odom_world_.pose.pose.orientation.y = q_eigen_.y();
    odom_world_.pose.pose.orientation.z = q_eigen_.z();
    odom_world_.twist.twist.linear.x = vel.x();
    odom_world_.twist.twist.linear.y = vel.y();
    odom_world_.twist.twist.linear.z = vel.z();
    if (debug_enable_)
    {
        // 跳变检测与周期性输出
        Eigen::Vector3d cur_pos = pos_corr;
        Eigen::Quaterniond cur_q = q_eigen_;
        double now_sec = stamp_sec;
        if (has_last_pub_)
        {
            double dt = now_sec - last_odom_pub_time_;
            double pos_jump = (cur_pos - last_pub_pos_).norm();
            double yaw_prev =
                std::atan2(2.0 * (last_pub_q_.w() * last_pub_q_.z() + last_pub_q_.x() * last_pub_q_.y()),
                           1.0 - 2.0 * (last_pub_q_.y() * last_pub_q_.y() + last_pub_q_.z() * last_pub_q_.z()));
            double yaw_curr = std::atan2(2.0 * (cur_q.w() * cur_q.z() + cur_q.x() * cur_q.y()),
                                         1.0 - 2.0 * (cur_q.y() * cur_q.y() + cur_q.z() * cur_q.z()));
            double dyaw_deg = std::abs((yaw_curr - yaw_prev) * 180.0 / M_PI);
            if (dyaw_deg > 180.0) dyaw_deg = 360.0 - dyaw_deg;
            if (pos_jump > debug_jump_pos_thresh_ || dyaw_deg > debug_jump_yaw_thresh_deg_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "ODOM JUMP: dt=%.4f pos_jump=%.3f yaw_jump=%.1fdeg effect=%zu/%zu imu_acc=[%.2f,%.2f,%.2f] "
                            "imu_gyr=[%.2f,%.2f,%.2f]",
                            dt,
                            pos_jump,
                            dyaw_deg,
                            static_cast<size_t>(last_effect_num_),
                            static_cast<size_t>(last_points_size_),
                            last_imu_acc_.x(),
                            last_imu_acc_.y(),
                            last_imu_acc_.z(),
                            last_imu_gyr_.x(),
                            last_imu_gyr_.y(),
                            last_imu_gyr_.z());
                RCLCPP_WARN(this->get_logger(),
                            "SENS CTX: cur_t=%.3f curv=%.6f lidar=[%.3f,%.3f] raw=%zu down=%zu curv[min=%.6f,max=%.6f]",
                            last_cur_point_time_,
                            last_curvature_value_,
                            last_lidar_begin_time_,
                            last_lidar_end_time_,
                            static_cast<size_t>(last_cloud_raw_size_),
                            static_cast<size_t>(last_cloud_down_body_size_),
                            last_frame_curv_min_,
                            last_frame_curv_max_);
                RCLCPP_WARN(this->get_logger(),
                            "IMU CTX: imus=%zu [%.3f,%.3f] kin=%zu [%.3f,%.3f]",
                            static_cast<size_t>(last_imus_size_),
                            last_imu_first_time_,
                            last_imu_last_time_,
                            static_cast<size_t>(last_kin_imus_size_),
                            last_kin_first_time_,
                            last_kin_last_time_);
            }
            // 周期性状态打印
            if (now_sec - last_debug_log_time_ >= debug_log_period_)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "ODOM DBG: t=%.3f pos=[%.3f %.3f %.3f] yaw=%.1fdeg cov_pos=[%.3g %.3g %.3g] success_pts=%zu/%zu",
                    now_sec,
                    cur_pos.x(),
                    cur_pos.y(),
                    cur_pos.z(),
                    yaw_curr * 180.0 / M_PI,
                    eskf_->cov()(3, 3),
                    eskf_->cov()(4, 4),
                    eskf_->cov()(5, 5),
                    static_cast<size_t>(last_effect_num_),
                    static_cast<size_t>(last_points_size_));
                last_debug_log_time_ = now_sec;
            }
        }
        else
        {
            last_debug_log_time_ = now_sec;
        }
        last_pub_pos_ = cur_pos;
        last_pub_q_ = cur_q;
        has_last_pub_ = true;
    }
    pub_odom_world_->publish(odom_world_);

    // tf2
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = odom_world_.header.stamp;
    transform_stamped.header.frame_id = output_frame_id_;
    transform_stamped.child_frame_id = body_frame_id_;
    transform_stamped.transform.translation.x = odom_world_.pose.pose.position.x;
    transform_stamped.transform.translation.y = odom_world_.pose.pose.position.y;
    transform_stamped.transform.translation.z = odom_world_.pose.pose.position.z;
    transform_stamped.transform.rotation.w = odom_world_.pose.pose.orientation.w;
    transform_stamped.transform.rotation.x = odom_world_.pose.pose.orientation.x;
    transform_stamped.transform.rotation.y = odom_world_.pose.pose.orientation.y;
    transform_stamped.transform.rotation.z = odom_world_.pose.pose.orientation.z;
    br_->sendTransform(transform_stamped);
}
void GalileoKLIONode::publishPointcloudWorld(double end_time)
{
    // 直接发布 world 点云（不做 out_rot_ 旋转修正）
    pcl::PointCloud<PointType> cloud_pub;
    cloud_pub.header.stamp = static_cast<uint64_t>(end_time * 1e9);
    cloud_pub.points.resize(cloud_down_world_->points.size());
    for (size_t i = 0; i < cloud_down_world_->points.size(); ++i)
    {
        const auto &pt = cloud_down_world_->points[i];
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        cloud_pub.points[i].x = static_cast<float>(p.x());
        cloud_pub.points[i].y = static_cast<float>(p.y());
        cloud_pub.points[i].z = static_cast<float>(p.z());
        cloud_pub.points[i].intensity = pt.intensity;
    }
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_pub, cloud_msg);
    cloud_msg.header.stamp = rclcpp::Time(end_time);
    cloud_msg.header.frame_id = output_frame_id_;
    pub_pointcloud_world_->publish(cloud_msg);
}
void GalileoKLIONode::publishPointcloudBody(double end_time)
{
    cloud_down_body_->header.stamp = static_cast<uint64_t>(end_time * 1e9);
    cloud_down_body_->header.frame_id = body_frame_id_;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_down_body_, cloud_msg);
    cloud_msg.header.stamp = rclcpp::Time(end_time);
    cloud_msg.header.frame_id = body_frame_id_;
    pub_pointcloud_body_->publish(cloud_msg);
}

void GalileoKLIONode::publishPointcloudWorldFull(double stamp_sec)
{
    if (!cloud_raw_ || cloud_raw_->points.empty())
    {
        return;
    }

    CloudPtr cloud_world_full(new PointCloudType());
    this->cloudLidarToWorld(cloud_raw_, cloud_world_full);

    cloud_world_full->header.stamp = static_cast<uint64_t>(stamp_sec * 1e9);
    cloud_world_full->header.frame_id = output_frame_id_;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_world_full, cloud_msg);
    cloud_msg.header.stamp = rclcpp::Time(stamp_sec);
    cloud_msg.header.frame_id = output_frame_id_;
    pub_pointcloud_world_full_->publish(cloud_msg);
}

void GalileoKLIONode::handleSaveMap(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    bool ok = false;
    if (map_manager_)
    {
        ok = map_manager_->saveMapToPCD(map_save_path_);
    }
    response->success = ok;
    response->message = ok ? (std::string("Saved to ") + map_save_path_) : std::string("Save failed");
}

void GalileoKLIONode::publishStaticTF()
{
    // 发布静态TF：rslidar到body的固定外参变换
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = body_frame_id_;
    transform_stamped.child_frame_id = lidar_frame_id_;
    
    // 设置平移（外参中的平移部分）
    transform_stamped.transform.translation.x = ext_t_(0);
    transform_stamped.transform.translation.y = ext_t_(1);
    transform_stamped.transform.translation.z = ext_t_(2);
    
    // 设置旋转（外参中的旋转矩阵转换为四元数）
    Eigen::Quaterniond q(ext_rot_);
    transform_stamped.transform.rotation.w = q.w();
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    
    static_br_->sendTransform(transform_stamped);
    
    RCLCPP_INFO(this->get_logger(), "发布静态TF: %s -> %s (外参变换)", 
                body_frame_id_.c_str(), lidar_frame_id_.c_str());
}

void GalileoKLIONode::publishMappingInputs(double stamp_sec)
{
    const Eigen::Vector3d t_W = eskf_->state().pos_;
    const Eigen::Matrix3d R_Wimu_W =  lidar_to_body_rot_ * ext_rot_.transpose();//
    const Eigen::Vector3d t_Wimu = R_Wimu_W * t_W;

    // 发布基于IMU世界系的里程计
    nav_msgs::msg::Odometry odom_to_map;
    odom_to_map.header.stamp = rclcpp::Time(stamp_sec);
    odom_to_map.header.frame_id = map_frame_id_.empty() ? std::string("map") : map_frame_id_;
    odom_to_map.child_frame_id = body_frame_id_;
    odom_to_map.pose.pose.position.x = t_Wimu.x();
    odom_to_map.pose.pose.position.y = t_Wimu.y();
    odom_to_map.pose.pose.position.z = t_Wimu.z();

    odom_to_map.pose.pose.orientation = imu_onboard_latest_->orientation;

    pub_odom_to_map_->publish(odom_to_map);

    // Path (Wimu frame)
    geometry_msgs::msg::PoseStamped pose_map;
    pose_map.header.stamp = odom_to_map.header.stamp;
    pose_map.header.frame_id = odom_to_map.header.frame_id;
    pose_map.pose = odom_to_map.pose.pose;
    if (path_map_.header.frame_id.empty())
    {
        path_map_.header.frame_id = odom_to_map.header.frame_id;
    }
    path_map_.header.stamp = odom_to_map.header.stamp;
    path_map_.poses.push_back(pose_map);
    pub_path_to_map_->publish(path_map_);
    // 发布基于IMU世界系的点云：使用未降采样点云 raw，先转到W，再转到 map（Wimu）
    if (cloud_raw_ && !cloud_raw_->points.empty())
    {
        pcl::PointCloud<PointType> cloud_pub;
        cloud_pub.header.stamp = static_cast<uint64_t>(stamp_sec * 1e9);
        cloud_pub.points.resize(cloud_raw_->points.size());
        for (size_t i = 0; i < cloud_raw_->points.size(); ++i)
        {
            const auto &pt = cloud_raw_->points[i];
            // raw 点先从 Lidar -> IMU
            Eigen::Vector3d p_L(pt.x, pt.y, pt.z);
            Eigen::Vector3d p_I = ext_rot_ * p_L + ext_t_;
            // 再从 IMU -> W
            Eigen::Vector3d p_W = eskf_->state().rot_ * p_I + eskf_->state().pos_;
            // 再从 W -> map(Wimu)
            Eigen::Vector3d p_Wimu = R_Wimu_W * p_W;
            cloud_pub.points[i].x = static_cast<float>(p_Wimu.x());
            cloud_pub.points[i].y = static_cast<float>(p_Wimu.y());
            cloud_pub.points[i].z = static_cast<float>(p_Wimu.z());
            cloud_pub.points[i].intensity = pt.intensity;
        }
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud_pub, cloud_msg);
        cloud_msg.header.stamp = rclcpp::Time(stamp_sec);
        cloud_msg.header.frame_id = map_frame_id_.empty() ? std::string("map") : map_frame_id_;
        pub_pointcloud_to_map_->publish(cloud_msg);
    }
}



}  // namespace galileo_klio

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // 统一使用 ROS2 日志系统，不再使用 glog 初始化

    // 声明多线程执行器，可指定线程数（默认为硬件并发数）
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);

    const auto node = std::make_shared<galileo_klio::GalileoKLIONode>();
    // 将节点添加到执行器
    executor.add_node(node);
    // 运行节点
    executor.spin();

    rclcpp::shutdown();

    galileo_klio::Timer::logAllAverTime();
    return 0;
}