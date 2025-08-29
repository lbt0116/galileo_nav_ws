cmake_minimum_required(VERSION 3.8)
project(galileo_klio)

# 生成编译数据库，用于编辑器智能提示
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# 消息生成
# find_package(rosidl_default_generators REQUIRED)

# 查找外部依赖
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io)
find_package(yaml-cpp REQUIRED)
# 尝试查找 GTSAM（即便当前类不使用，先引入依赖环境）
find_package(GTSAM QUIET)

# glog/gflags 移除，统一使用 ROS2 日志

## 构建型态由 colcon mixin 控制（debug/release），包内不强行设置
# 显式强制本包使用 Release（无论上层如何指定）
get_property(GENERATOR_IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(GENERATOR_IS_MULTI_CONFIG)
  set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
else()
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# 添加定义
add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

## 全局编译标志改为目标级设置，避免污染其他包

#

## 目标级包含目录

# 头文件列表
set(HEADERS
    include/common.hpp
    include/eskf.h
    # include/glog_utils.hpp  已移除
    
    include/lidar_processing.h
    include/math_utils.hpp
    include/state_initial.hpp
    include/timer_utils.hpp
    include/voxel_map.h
    include/leg_utils.hpp
    include/laser_mapping.hpp
)

# 源文件列表
set(SOURCES
    src/eskf.cc
    src/lidar_processing.cc
    src/voxel_map.cc
    src/laser_mapping.cc
)

# 创建库
add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -O3)

# 链接库
target_link_libraries(${PROJECT_NAME}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    yaml-cpp
)
if (GTSAM_FOUND)
  # 兼容不同导出方式：GTSAM::gtsam 或 gtsam 或变量 GTSAM_LIBRARIES
  if (TARGET GTSAM::gtsam)
    target_link_libraries(${PROJECT_NAME} GTSAM::gtsam)
  elseif (TARGET gtsam)
    target_link_libraries(${PROJECT_NAME} gtsam)
  elseif (DEFINED GTSAM_LIBRARIES)
    target_link_libraries(${PROJECT_NAME} ${GTSAM_LIBRARIES})
  else()
    message(WARNING "GTSAM found but no exported CMake target; skipping explicit link")
  endif()
  target_compile_definitions(${PROJECT_NAME} PUBLIC HAVE_GTSAM)
endif()

# 创建可执行文件
add_executable(${PROJECT_NAME}_node
    src/galileo_klio_node.cc
)

# 链接可执行文件到库
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wpedantic -O3)

# 安装目标
install(TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
    DESTINATION include/
)

# 安装启动文件
install(DIRECTORY
    launch
    config
    rviz
    DESTINATION share/${PROJECT_NAME}
)

# 移除自定义消息 RobotState

# 导出依赖
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    pcl_ros
    tf2
    tf2_ros
    tf2_geometry_msgs
    visualization_msgs
    std_srvs
)

ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    pcl_ros
    tf2
    tf2_ros
    tf2_geometry_msgs
    visualization_msgs
    std_srvs
)

# 导出包含目录和库
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    pcl_ros
    tf2
    tf2_ros
    tf2_geometry_msgs
    visualization_msgs
)

# 无自定义消息，以下导出关闭

# 测试
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package() 