#include "laser_mapping.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace galileo_klio {

LaserMapping::LaserMapping(const rclcpp::Node::SharedPtr &node,
                           const LaserMappingConfig &config)
    : node_(node), cfg_(config) {
    (void)node_;
}

} // namespace galileo_klio


