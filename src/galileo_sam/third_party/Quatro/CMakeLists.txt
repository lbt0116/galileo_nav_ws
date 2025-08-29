cmake_minimum_required(VERSION 3.10)
project(quatro)

### set compiler
get_property(GENERATOR_IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(GENERATOR_IS_MULTI_CONFIG)
  set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
else()
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()
# 仅设置通用标志，调试符号放到 Debug 专用变量
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -ggdb")
include(FindOpenMP) #The best way to set proper compiler settings for using OpenMP in all platforms
if(OPENMP_FOUND) #The best way to set proper compiler settings for using OpenMP in all platforms
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -Wall")

### get packages
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(teaserpp REQUIRED)
## 简化构建输出

#### for tbb
if(QUATRO_TBB)
  list(APPEND CMAKE_MODULE_PATH /usr/local/include /usr/include)
  find_library(TBB_LIBRARY tbb HINTS /usr/lib /usr/local/lib)
  message(WARNING "TBB FOUND, ${TBB_LIBRARY}")
  add_definitions(-DTBB_EN)
  include(ProcessorCount)
  ProcessorCount(N)
  message(WARNING "Processer number: ${N}")
  if(N GREATER 4)
    add_definitions(-DTBB_PROC_NUM=${N})
  else()
    add_definitions(-DTBB_PROC_NUM=1)
  endif()
endif()

#### for debug output
if(QUATRO_DEBUG)
  add_definitions(-DQUATRO_DEBUG)
endif()

### get packages' headers
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

###########
## Build ##
###########
### main
add_library(${PROJECT_NAME} src/fpfh.cc src/matcher.cc src/quatro_module.cc)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall -Wextra
  $<$<AND:$<CONFIG:Release>,$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>>:-O3 -march=native -mtune=native>
)
if (QUATRO_TBB)
  target_link_libraries(${PROJECT_NAME} ${OpenMP_LIBS} ${EIGEN3_LIBS} ${PCL_LIBRARIES} teaserpp::teaser_registration teaserpp::teaser_io ${TBB_LIBRARY})
else()
  target_link_libraries(${PROJECT_NAME} ${OpenMP_LIBS} ${EIGEN3_LIBS} ${PCL_LIBRARIES} teaserpp::teaser_registration teaserpp::teaser_io)
endif()

install(TARGETS
  ${PROJECT_NAME}
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
ament_export_dependencies(Eigen3 PCL teaserpp)

ament_package()