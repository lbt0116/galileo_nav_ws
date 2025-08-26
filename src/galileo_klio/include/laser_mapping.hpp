#pragma once

#ifndef GALILEO_KLIO_LASER_MAPPING_HPP
#define GALILEO_KLIO_LASER_MAPPING_HPP

#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "common.hpp"

// #include <gtsam/geometry/Rot3.h>
// #include <gtsam/geometry/Pose3.h>
// #include <gtsam/slam/PriorFactor.h>
// #include <gtsam/slam/BetweenFactor.h>
// #include <gtsam/navigation/GPSFactor.h>
// #include <gtsam/navigation/ImuFactor.h>
// #include <gtsam/navigation/CombinedImuFactor.h>
// #include <gtsam/nonlinear/NonlinearFactorGraph.h>
// #include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
// #include <gtsam/nonlinear/Marginals.h>
// #include <gtsam/nonlinear/Values.h>
// #include <gtsam/inference/Symbol.h>
// #include <gtsam/nonlinear/ISAM2.h>


namespace galileo_klio {

struct LaserMappingConfig {
    // 预留配置结构体，当前不包含功能项（后续按需扩展）
};

class LaserMapping {
public:
    explicit LaserMapping(const rclcpp::Node::SharedPtr &node,
                          const LaserMappingConfig &config);

private:
    rclcpp::Node::SharedPtr node_;
    LaserMappingConfig cfg_;
    std::mutex mtx_;
};

} // namespace galileo_klio

#endif


