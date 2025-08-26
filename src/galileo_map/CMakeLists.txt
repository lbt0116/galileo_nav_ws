cmake_minimum_required(VERSION 3.5)
project(galileo_map)

# Enable compile_commands.json for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_BUILD_TYPE "Debug")
# 由 colcon mixin 控制构建型态与编译选项，避免在包内设置全局 Flags

# reduce warning
#add_compile_options(-Wno-dev)
cmake_policy(SET CMP0057 NEW)
cmake_policy(SET CMP0054 NEW)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE BOOL "Suppress developer warnings")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Define the voxelize and raycasting method
add_definitions(-DORIGIN_AT_CORNER)

string(TOUPPER $ENV{ROS_DISTRO} ROS_VERSION)
message(STATUS "ROS version: ${ROS_VERSION}")

message("Using ROS2")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(includes
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(third_party_libs
        ${PCL_LIBRARIES}
        yaml-cpp
)

set(ros_libs
        rclcpp
        std_msgs
        nav_msgs
        geometry_msgs
        visualization_msgs
        pcl_conversions
        tf2_ros
)

include_directories(
        ${includes}
)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp include/*.cpp)

add_library(${PROJECT_NAME} STATIC ${srcs})

target_include_directories(${PROJECT_NAME} PUBLIC include)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic $<$<CONFIG:Release>:-O3>)
target_link_libraries(${PROJECT_NAME} PUBLIC
        ${third_party_libs})


ament_target_dependencies(${PROJECT_NAME} PUBLIC
        ${ros_libs}
)

# 可执行节点
add_executable(${PROJECT_NAME}_node
        src/rog_map/rog_map_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node PUBLIC
        ${ros_libs}
)
target_link_libraries(${PROJECT_NAME}_node PUBLIC
        ${PROJECT_NAME}
)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wpedantic $<$<CONFIG:Release>:-O3>)


install(TARGETS
        ${PROJECT_NAME}
        ${PROJECT_NAME}_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(${includes})
ament_export_dependencies(${ros_libs} yaml-cpp)
install(DIRECTORY include/
        DESTINATION include/
)

# 安装资源
install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
