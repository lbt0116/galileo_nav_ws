#############################################################
# Copyright (c) 2022, University of California, Los Angeles #
#                                                           #
# Modifier: Eungchang Mason Lee                             #
# Authors: Kenny J. Chen, Brett T. Lopez                    #
# Contact: kennyjchen@ucla.edu, btlopez@ucla.edu            #
#############################################################

cmake_minimum_required(VERSION 3.10.0)
project(nano_gicp)

### set compiler
get_property(GENERATOR_IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(GENERATOR_IS_MULTI_CONFIG)
  set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
else()
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()
# 仅设置通用标志，调试符号放到 Debug 专用变量
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -ggdb")
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
include(FindOpenMP)
if(OPENMP_FOUND)
  # 使用目标级链接以传播OpenMP编译选项
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)
## 目标级编译选项，避免全局污染
## 在库创建后追加

### get packages
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

### get packages' headers
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

###########
## Build ##
###########
### NanoFLANN
add_library(nanoflann src/nanoflann.cc)
target_link_libraries(nanoflann ${PCL_LIBRARIES})
target_compile_features(nanoflann PUBLIC cxx_std_17)
target_compile_options(nanoflann PRIVATE
  -Wall -Wextra
  $<$<AND:$<CONFIG:Release>,$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>>:-O3 -march=native -mtune=native>
)
### NanoGICP
add_library(${PROJECT_NAME} src/nano_gicp.cc)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} nanoflann)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall -Wextra
  $<$<AND:$<CONFIG:Release>,$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>>:-O3 -march=native -mtune=native>
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

install(TARGETS
  ${PROJECT_NAME}
  nanoflann
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(PCL Eigen3)

ament_package()
