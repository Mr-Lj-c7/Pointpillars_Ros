/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>
#include <sstream>
#include <fstream>

// headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
// headers in PCL
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cuda_runtime.h"

#include "../include/Pointpillars/params.h"
#include "../include/Pointpillars/pointpillar.h"

// Class names for KITTI dataset
enum class_enum 
{
  Car = 0,         // 汽车
  Pedestrian = 1,  // 行人
  Cyclist = 2      // 自行车
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

std::string Data_File = "/home/user/ChangTing/Code/Openpcdet/Pointpillars_Ros/src/lidar_point_pillars/data/testing/";
std::string Save_Dir = "/home/user/ChangTing/Code/Openpcdet/Pointpillars_Ros/src/lidar_point_pillars/eval/kitti/object/pred_kitti/";
std::string Model_File = "/home/user/ChangTing/Code/Openpcdet/Pointpillars_Ros/src/lidar_point_pillars/model/pointpillar.onnx";
const int POINT_CLOUD_FRAME_NUM = 65;

cudaEvent_t start, stop; 
float elapsedTime = 0.0f;  // 推理时间
cudaStream_t stream = NULL;
std::vector<Bndbox> nms_pred;  // 保存预测框
// nms_pred.reserve(100);  // 保存预测框的容量
PointPillar pointpillar(Model_File, stream);  // 初始化PointPillar模型

// std::string Data_File = "../data/";
// std::string Save_Dir = "../eval/kitti/object/pred_velo/";
// std::string Model_File = "../model/pointpillar.onnx";

/**
 * @brief Getinfo() - 显示本地GPU设备信息
 */
void Getinfo(void)
{
  cudaDeviceProp prop;

  int count = 0;
  cudaGetDeviceCount(&count);
  printf("\nGPU has cuda devices: %d\n", count);
  for (int i = 0; i < count; ++i) {
    cudaGetDeviceProperties(&prop, i);
    printf("----device id: %d info----\n", i);
    printf("  GPU : %s \n", prop.name);
    printf("  Capbility: %d.%d\n", prop.major, prop.minor);
    printf("  Global memory: %luMB\n", prop.totalGlobalMem >> 20);
    printf("  Const memory: %luKB\n", prop.totalConstMem  >> 10);
    printf("  SM in a block: %luKB\n", prop.sharedMemPerBlock >> 10);
    printf("  warp size: %d\n", prop.warpSize);
    printf("  threads in a block: %d\n", prop.maxThreadsPerBlock);
    printf("  block dim: (%d,%d,%d)\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1], prop.maxThreadsDim[2]);
    printf("  grid dim: (%d,%d,%d)\n", prop.maxGridSize[0], prop.maxGridSize[1], prop.maxGridSize[2]);
  }
  printf("\n");
}

/**
 * @fn loadData() - 读取文件
 * @param file - 文件名
 * @param data - 数据指针
 * @param length - 数据长度
 * @return 0 - 成功
 */
int loadData(const char *file, void **data, unsigned int *length)
{
  std::fstream dataFile(file, std::ifstream::in);

  if (!dataFile.is_open())
  {
	  std::cout << "Can't open files: "<< file<<std::endl;
	  return -1;
  }

  //get length of file:
  unsigned int len = 0;
  dataFile.seekg (0, dataFile.end);
  len = dataFile.tellg();
  dataFile.seekg (0, dataFile.beg);

  //allocate memory:
  char *buffer = new char[len];
  if(buffer==NULL) {
	  std::cout << "Can't malloc buffer."<<std::endl;
    dataFile.close();
	  exit(-1);
  }

  //read data as a block:
  dataFile.read(buffer, len);
  dataFile.close();

  *data = (void*)buffer;
  *length = len;
  return 0;  
}

/**
 * @fn SaveBoxPred() - 保存预测框
 * @param boxes - 预测框
 * @param file_name - 保存的文件名
 */
void SaveBoxPred(std::vector<Bndbox> boxes, std::string file_name)
{
    std::ofstream ofs;
    ofs.open(file_name, std::ios::out);
    if (ofs.is_open()) {
        for (const auto box : boxes) {
          ofs << box.x << " ";      // 位置x, y, z
          ofs << box.y << " ";
          ofs << box.z << " ";
          ofs << box.w << " ";      // 宽高长w, l, h
          ofs << box.l << " ";
          ofs << box.h << " ";
          ofs << box.rt << " ";     // 旋转角rt
          ofs << box.id << " ";     // 类别id
          ofs << box.score << " ";  // 置信度
          ofs << "\n";
        }
    }
    else {
      std::cerr << "Output file cannot be opened!" << std::endl;
    }
    ofs.close();
    std::cout << "Saved prediction in: " << file_name << std::endl;
    return;
};


void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    size_t points_size = cloud->size();
    std::cout << "cloud size: " << cloud->size() << std::endl;
    
    float* points = new float[points_size * 4];
    for(size_t i = 0; i < points_size; i++)
    {
        points[i * 4 + 0] = cloud->points[i].x;
        points[i * 4 + 1] = cloud->points[i].y;
        points[i * 4 + 2] = cloud->points[i].z;
        points[i * 4 + 3] = 0.f;
    }
  
    float *points_data = nullptr;
    unsigned int points_data_size = points_size * 4 * sizeof(float);
    checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size));
    checkCudaErrors(cudaMemcpy(points_data, points, points_data_size, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaDeviceSynchronize());
    
    cudaEventRecord(start, stream);
    pointpillar.doinfer(points_data, points_size, nms_pred);
    cudaEventRecord(stop, stream);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&elapsedTime, start, stop);
    ROS_INFO("TIME: pointpillar: %f ms.", elapsedTime);
    
    checkCudaErrors(cudaFree(points_data));
    ROS_INFO("Bndbox objs: %lu", nms_pred.size());
    nms_pred.clear();
    delete[] points;
}

// int main(int argc, const char **argv)
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointpillars_node");
  ros::NodeHandle nh;
  int queue_size_ = 1;
  
  Getinfo();  // 显示本地GPU设备信息

  // cudaEvent_t start, stop;  
  // float elapsedTime = 0.0f;  // 推理时间
  // cudaStream_t stream = NULL;

  checkCudaErrors(cudaEventCreate(&start));
  checkCudaErrors(cudaEventCreate(&stop));
  checkCudaErrors(cudaStreamCreate(&stream));

  Params params_;

  // std::vector<Bndbox> nms_pred;  // 保存预测框
  // nms_pred.reserve(100);  // 保存预测框的容量

  // PointPillar pointpillar(Model_File, stream);  // 初始化PointPillar模型

  std::cout << "data_file :" << Data_File << std::endl;


  ros::Subscriber sub = nh.subscribe("/kitti/velo/pointcloud", queue_size_, pointsCallback);
  ros::spin();
  // for (int i = 0; i < POINT_CLOUD_FRAME_NUM; i++)
  // {
  //   std::string dataFile = Data_File;

  //   std::stringstream ss;

  //   ss<< i;

  //   int n_zero = 6;
  //   std::string _str = ss.str();
  //   std::string index_str = std::string(n_zero - _str.length(), '0') + _str;
  //   dataFile += index_str;
  //   dataFile +=".bin";

  //   std::cout << "<<<<<<<<<<<" <<std::endl;
  //   std::cout << "load file: "<< dataFile <<std::endl;

  //   //load points cloud
  //   unsigned int length = 0;
  //   void *data = NULL;
  //   std::shared_ptr<char> buffer((char *)data, std::default_delete<char[]>());  // 分配内存存储点云数据
  //   loadData(dataFile.data(), &data, &length);
  //   buffer.reset((char *)data);

  //   float* points = (float*)buffer.get();
  //   size_t points_size = length/sizeof(float)/4;

  //   std::cout << "find points num: "<< points_size <<std::endl;

  //   float *points_data = nullptr;
  //   unsigned int points_data_size = points_size * 4 * sizeof(float);
  //   checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size));
  //   checkCudaErrors(cudaMemcpy(points_data, points, points_data_size, cudaMemcpyDefault));  // 点云数据拷贝到GPU
  //   checkCudaErrors(cudaDeviceSynchronize());  // 同步GPU

  //   cudaEventRecord(start, stream);

  //   pointpillar.doinfer(points_data, points_size, nms_pred);  // 模型推理
  //   cudaEventRecord(stop, stream);
  //   cudaEventSynchronize(stop);
  //   cudaEventElapsedTime(&elapsedTime, start, stop);  // 计算推理时间
  //   std::cout<<"TIME: pointpillar: "<< elapsedTime <<" ms." <<std::endl;

  //   checkCudaErrors(cudaFree(points_data));  // 释放GPU内存

  //   std::cout<<"Bndbox objs: "<< nms_pred.size()<<std::endl;
  //   std::string save_file_name = Save_Dir + index_str + ".txt";
  //   SaveBoxPred(nms_pred, save_file_name);

  //   nms_pred.clear();  // 清空保存预测框的容器

  //   std::cout << ">>>>>>>>>>>" <<std::endl;
  // }
  // 释放CUDA事件和数据流
  checkCudaErrors(cudaEventDestroy(start));
  checkCudaErrors(cudaEventDestroy(stop));
  checkCudaErrors(cudaStreamDestroy(stream));

  return 0;
}
