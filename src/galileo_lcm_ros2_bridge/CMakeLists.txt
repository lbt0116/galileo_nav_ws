cmake_minimum_required(VERSION 3.10)
project(galileo_lcm_ros2_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -fpermissive)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

add_executable(recorder_node src/bridge/src/recorder_node.cpp)
ament_target_dependencies(recorder_node rclcpp std_msgs sensor_msgs tf2_ros nav_msgs geometry_msgs)

target_include_directories(recorder_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge/include
  ${LCM_INCLUDE_DIRS}
)
target_compile_options(recorder_node PUBLIC ${LCM_CFLAGS_OTHER})
target_link_libraries(recorder_node ${LCM_LIBRARIES})

install(TARGETS recorder_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY src/bridge/include/
  DESTINATION include)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()


