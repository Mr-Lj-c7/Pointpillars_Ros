# Pointpillars - Ros 

## 项目说明
该项目主要用于实现将训练好的激光雷达感知检测模型（pointpillar）在NVIDIA GeoForce RTX 4060 GPU 上基于TensorRt库进行加速推理，并利用中间件ROS1进行模型推理结果的消息发布与可视化评估。

## 项目需求

1.    **系统软硬件配置：**
   - linux: ubuntu20.04 LTS
   - gpu: NVIDIA GeForce RTX 4060
   - driver: INVIDA Driver 535.183.01 
   - ros: noetic
   - tensorrt: TensorRT-8.6.1.6
   - cuda: cuda-11.8
   - pcl: pcl-1.10
   - opencv: opencv-4.5
   - eigen: Eigen3
   - yaml：yaml-cpp 


## 项目架构
``` bash
Pointpillars_Ros
|_build                           # 编译空间
|_devel                           # 开发空间，存放可执行文件
|_install                         # 安装空间(可弃用)
|_src
   |_autoware_msgs                # 检测结果消息功能包
   |_detected_objects_visualizer  # 检测结果可视化功能包
   |_lidar_point_pillars          # pointpillar模型推理，检测结果发布功能包
   |_result                       # 检测结果
   |_CMakeLists.txt               # 工作空间编译文件
|_start.sh                        # 项目启动脚本
|_PointpillarRecord.md            # pointpillar感知模型记录文件
|_ReadMe.md                       # 项目说明文件
```


## 项目启动
[说明]：将导出的onnx格式模型放在/Pointpillars_Ros/src/lidar_point_pillars/model/路径下，并在lidar_point_pillars.launch文件中对应修改模型路径（注意，如果修改自定义模型的类别和检测范围需要更改/Pointpillars_Ros/src/lidar_point_pillars/include/params.h文件参数）。

```bash

roscore

robag play -l xxx.bag    # 发布对应点云消息

catkin_make -j4 --pkg autoware_msgs

catkin_make -j4 --pkg detected_objects_visualizer  && catkin_make -j4 --pkg lidar_point_pillars

bash start.sh

```

