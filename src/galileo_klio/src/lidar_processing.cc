#include "lidar_processing.h"

namespace galileo_klio
{
LidarProcessing::LidarProcessing(LidarProcessing::Config config)
    : config_(config)
{
    RCLCPP_INFO(rclcpp::get_logger("lidar_processing"), "Lidar Processing is Constructed");
    cloud_pcl_.reset(new PointCloudType());
}

LidarProcessing::~LidarProcessing()
{
    RCLCPP_INFO(rclcpp::get_logger("lidar_processing"), "Lidar Processing is Destructed");
}

common::LidarType LidarProcessing::getLidarType() const
{
    return config_.lidar_type_;
}

void LidarProcessing::processing(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, common::LidarScan& lidar_scan)
{

    // 仅保留 Robosense Airy 处理
    robosense_airy_handler(msg, lidar_scan);
}


void LidarProcessing::robosense_airy_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                                             common::LidarScan& lidar_scan)
{
    // std::cout << "robosense_airy_handler" << std::endl;

    lidar_scan.cloud_.reset(new PointCloudType());
    pcl::PointCloud<robosense_airy_ros::Point> cloud_pcl_raw;
    pcl::fromROSMsg(*msg, cloud_pcl_raw);


    float first_point_time = config_.time_scale_ * cloud_pcl_raw.points.front().timestamp;
    float last_point_time = config_.time_scale_ * cloud_pcl_raw.points.back().timestamp;

    // RCLCPP_INFO(rclcpp::get_logger("lidar_processing"), "first_point_time: %f", first_point_time);
    // RCLCPP_INFO(rclcpp::get_logger("lidar_processing"), "last_point_time: %f", last_point_time);
    // RCLCPP_INFO(rclcpp::get_logger("lidar_processing"), "msg->header.stamp: %f", msg->header.stamp.sec);

    lidar_scan.lidar_begin_time_ = first_point_time;
    lidar_scan.lidar_end_time_ = last_point_time;

    int cloud_size = cloud_pcl_raw.points.size();
    lidar_scan.cloud_->points.reserve(cloud_size);


    for (int i = 0; i < cloud_size; ++i)
    {
        if ((i % config_.filter_num_) || blindCheck(cloud_pcl_raw.points[i])) continue;
        PointType added_point;
        added_point.x = cloud_pcl_raw.points[i].x;
        added_point.y = cloud_pcl_raw.points[i].y;
        added_point.z = cloud_pcl_raw.points[i].z;
        added_point.intensity = cloud_pcl_raw.points[i].intensity;
        float cur_point_time = config_.time_scale_ * cloud_pcl_raw.points[i].timestamp;
        added_point.curvature = std::round((cur_point_time - first_point_time) * 500.0f) / 500.0f;

        lidar_scan.cloud_->points.push_back(added_point);
    }

}

}  // namespace galileo_klio
