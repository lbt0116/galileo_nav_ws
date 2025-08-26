#include <rclcpp/rclcpp.hpp>
#include <rog_map_ros/rog_map_ros2.hpp>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("galileo_map_node");

  std::string cfg_path = node->declare_parameter<std::string>("config_path", "");

  auto rog_map = std::make_shared<rog_map::ROGMapROS>(node, cfg_path);
  (void)rog_map;

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

