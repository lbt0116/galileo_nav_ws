#ifndef LEG_KILO_LIDAR_PROCESSING_H
#define LEG_KILO_LIDAR_PROCESSING_H

#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "common.hpp"

// 已精简，仅保留 Robosense Airy 点类型

namespace robosense_airy_ros
{
struct EIGEN_ALIGN16 Point
{
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY
    uint16_t ring;
    double timestamp;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace robosense_airy_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_airy_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      uint16_t, ring, ring)(double, timestamp, timestamp))
namespace galileo_klio
{

class LidarProcessing
{
public:
    struct Config
    {
        float blind_ = 1.0;
        int filter_num_ = 1;
        bool point_stamp_correct_ = true;  // for leg kilo dataset
        common::LidarType lidar_type_ = common::LidarType::ROBOSENSE_AIRY;
        double time_scale_ = 1.0;
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LidarProcessing() = delete;
    LidarProcessing(LidarProcessing::Config config);
    ~LidarProcessing();

    common::LidarType getLidarType() const;
    void processing(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, common::LidarScan& lidar_scan);

    template <typename T>
    inline bool blindCheck(const T& p)
    {
        return config_.blind_ * config_.blind_ > p.x * p.x + p.y * p.y + p.z * p.z;
    }

private:
    // 仅保留 Robosense Airy 处理
    void robosense_airy_handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, common::LidarScan& lidar_scan);
    CloudPtr cloud_pcl_;
    Config config_;
};

}  // namespace galileo_klio

#endif  // LEG_KILO_LIDAR_PROCESSING_H