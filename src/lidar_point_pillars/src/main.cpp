#include <iostream>
#include <sstream>
#include <fstream>

#include "cuda_runtime.h"

#include "../include/lidar_point_pillars/PointPillarsRos.h"

// Class names for KITTI dataset
enum class_enum 
{
  Car = 0,         // 汽车
  Pedestrian = 1,  // 行人
  Cyclist = 2      // 自行车
}; 

// Class names for custom dataset { "Car","Bus","Pedestrian","Motorcycle",}
enum CLASS_ENUM 
{
  Car_ = 0,         
  Bus_ = 1,
  Pedestrian_ = 2,  
  Motorcycle_ = 3      
}; 

/**
 * @brief cudaGetErrorString(status) - 错误字符串
 * @brief __LINE__ - 行号
 * @brief __FILE__ - 出错文件名
 * @brief status - 状态码
 * @brief abort() - 终止程序
 * @brief \ - 跨行宏定义
 */
#define checkCudaErrors(status)                                   \
{                                                                 \
  if (status != 0)                                                \
  {                                                               \
    std::cout << "Cuda failure: " << cudaGetErrorString(status)   \
              << " at line " << __LINE__                          \
              << " in file " << __FILE__                          \
              << " error status: " << status                      \
              << std::endl;                                       \
              abort();                                            \
    }                                                             \
}                                                                 


int main(int argc, char **argv)
{
  std::string node_name = "pointpillars_node";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;
  ROS_INFO("Starting %s ...", node_name.c_str());
  std::shared_ptr<PointPillarsNS::PointPillarsROS> pointpillars_ros = std::make_shared<PointPillarsNS::PointPillarsROS>();
  pointpillars_ros->run();
  return 0;
}
