cmake_minimum_required(VERSION 3.0.2)
project(lidar_point_pillars)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(CUDA)
find_package(Eigen3 REQUIRED)
# find_package(PCL 1.10 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  roslib
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

if(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL  "aarch64")    # arm64
  set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)
  set(CUDA_INSTALL_TARGET_DIR targets/aarch64-linux)
elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "x86_64")  # x86
  set(CMAKE_C_COMPILER /usr/bin/gcc)
  set(CMAKE_CXX_COMPILER /usr/bin/g++)
  set(CUDA_INSTALL_TARGET_DIR targets/x86_64-linux)
endif()

# cuda-11.8
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
set(CUDA_INCLUDE_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/${CUDA_INSTALL_TARGET_DIR}/include)
# tensorrt-8.6.1.6
set(TENSORRT_ROOT /home/user/NVIDIA/tensorRT_8.6.1.6/TensorRT-8.6.1.6)
set(TENSORRT_INCLUDE ${TENSORRT_ROOT}/include)
set(TENSORRT_LIBRARY ${TENSORRT_ROOT}/targets/x86_64-linux-gnu/lib/)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-Wno-deprecated-declarations -O2")
add_compile_options(-W)
# add_compile_options(-std=c++11)

set( SMS 30 32 35 37 50 52 53 60 61 62 70 72 75 87)
foreach(sm ${SMS})
	set(GENCODE ${GENCODE} -gencode arch=compute_${sm},code=sm_${sm})
endforeach()
set(HIGHEST_SM 87)
set(GENCODE ${GENCODE} -gencode arch=compute_${HIGHEST_SM},code=compute_${HIGHEST_SM})

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}
    -ccbin ${CMAKE_CXX_COMPILER}
    -Xcompiler -DWIN_INTERFACE_CUSTOM
    -Xcompiler -I/usr/aarch64-linux-gnu/include/
    -Xlinker -lsocket
    -Xlinker -rpath=/usr/lib/aarch64-linux-gnu/
    -Xlinker -rpath=/usr/aarch64-linux-gnu/lib/
    -Xlinker -L/usr/lib/aarch64-linux-gnu/
    -Xlinker -L/usr/aarch64-linux-gnu/lib/
)

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  message("Using Debug Mode")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -g -G --ptxas-options=-v)
endif()

# TensorRT arm64
set(TENSORRT_INCLUDE_DIRS /usr/include/aarch64-linux-gnu/)
set(TENSORRT_LIBRARY_DIRS /usr/lib/aarch64-linux-gnu/)



###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roslib
  pcl_ros
  autoware_msgs
  ${OPENCV_INCLUDE_DIRS}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${TENSORRT_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE}
)

link_directories(
	${TENSORRT_LIBRARY_DIRS}
  ${TENSORRT_LIBRARY}
	/usr/lib/aarch64-linux-gnu
  /usr/aarch64-linux-gnu/lib/
)


set(SOURCE_FILES
src/PointpillarsRos/PointPillarsRos.cpp
src/PointpillarsRos/Tracker.cpp
src/main.cpp
# src/main_old.cpp
)

add_executable(lidar_point_pillars
${SOURCE_FILES}
)

add_dependencies(lidar_point_pillars
    ${catkin_EXPORTED_TARGETS}
  )
  
cuda_add_library(gpu_point_pillars_lib
  src/Pointpillars/pillarScatterKernels.cu
  src/Pointpillars/preprocess_kernels.cu
  src/Pointpillars/postprocess_kernels.cu
)

target_link_libraries(gpu_point_pillars_lib
    ${CUDA_LIBRARIES}
  )

add_library(point_pillars_lib
  src/Pointpillars/postprocess.cpp
  src/Pointpillars/preprocess.cpp
  src/Pointpillars/pillarScatter.cpp
  src/Pointpillars/pointpillar.cpp
)  

target_link_libraries(point_pillars_lib
${catkin_LIBRARIES}
${CUDA_LIBRARIES}
${CUDA_CUBLAS_LIBRARIES}
${CUDA_curand_LIBRARY}
${CUDNN_LIBRARY}
gpu_point_pillars_lib
yaml-cpp
libnvinfer.so
libnvonnxparser.so
)
  
# file(GLOB_RECURSE SOURCE_FILES
#     src/Pointpillars/*.cu
#     src/Pointpillars/*.cpp
# )

# cuda_add_executable(${PROJECT_NAME} main.cpp ${SOURCE_FILES})

target_link_libraries(lidar_point_pillars
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  point_pillars_lib
)




