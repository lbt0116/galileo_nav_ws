cmake_minimum_required(VERSION 3.10)
project(galileo_sam)
get_property(GENERATOR_IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(GENERATOR_IS_MULTI_CONFIG)
  set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
else()
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenMP)
find_package(teaserpp REQUIRED)
find_package(nano_gicp REQUIRED)
find_package(quatro REQUIRED)

## 简化构建输出，去除第三方版本噪音

## 目标级 include，避免全局污染

add_library(${PROJECT_NAME}_modules
  ${PROJECT_SOURCE_DIR}/../third_party/scancontext_tro/Scancontext.cpp
  src/loop_closure.cpp
  src/galileo_sam.cpp
)
target_compile_features(${PROJECT_NAME}_modules PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_modules PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/../third_party/scancontext_tro
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)
target_compile_options(${PROJECT_NAME}_modules PRIVATE -Wall -Wextra -Wpedantic -O3)
ament_target_dependencies(${PROJECT_NAME}_modules
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  message_filters
  pcl_conversions
  ament_index_cpp
  nano_gicp
  quatro
)
target_link_libraries(${PROJECT_NAME}_modules
  ${PCL_LIBRARIES}
  gtsam
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}_modules OpenMP::OpenMP_CXX)
endif()

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wpedantic -O3)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  message_filters
  pcl_conversions
  ament_index_cpp
  nano_gicp
  quatro
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_modules
  ${PCL_LIBRARIES}
  gtsam
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}_node OpenMP::OpenMP_CXX)
endif()

install(TARGETS
  ${PROJECT_NAME}_modules
  ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
