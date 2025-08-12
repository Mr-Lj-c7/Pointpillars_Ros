#ifndef _POINTPILLARS_ROS_H_
#define _POINTPILLARS_ROS_H_
#pragma once

// headers ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// headers pcl - 1.10
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>

// headers STL
#include <iostream>
#include <vector>
#include <memory>
// headers C++
#include <sstream>
#include <string>
#include <fstream>
#include <math.h>

// headers cuda
#include <cuda_runtime.h>
#include "../../include/Pointpillars/params.h"
#include "../../include/Pointpillars/pointpillar.h"

// headers autoware
#include "autoware_msgs/DetectedObjectArray.h"

// Tracker
#include "Tracker.h"

using namespace std;
namespace PointPillarsNS
{
    class PointPillarsROS
    {
        private: 
            const int NUM_POINT_FEATURE_;
        
            ros::NodeHandle private_nh_;
            ros::Subscriber sub_points_;
            ros::Publisher pub_objects_;
            ros::Publisher pub_tracker_objects_;
            
            const std::string BASELINK_FRAME_;
            const float TRAINED_SENSOR_HEIGHT_;
            const float offset_z_ = 0.f;
            
            // ros param
            bool use_tracking_;
            bool has_subscribed_baselink_;
            bool point_xyzi_;
            bool baselink_support_;
            bool use_onnx_;
            bool debug_ = true;
            
            // topics
            std::string input_topic_;
            std::string pfe_onnx_file_;
            std::string output_topic_;
            std::string tracker_output_topic_;
            
            float elapsedTime = 0.0f;  // 推理时间
            
            std::vector<Bndbox> nms_pred;  // 保存预测框,x, y, z, w, l, h, rt, id, score
            // nms_pred.reserve(100);      // 保存预测框的容量
            
            Params params_; 

            // Tracker
            Tracker tracker;
            Tracker_KF tracker_kf;  // 卡尔曼滤波
            bool use_kf_tracking_;
            
            // tf
            tf::TransformListener tf_listener_;
            tf::StampedTransform baselink2lidar_;
            tf::Transform angle_transform_;
            tf::Transform angle_transform_inversed_;
            float offset_z_from_trained_data_;
            
            std::string Data_File = "/home/user/ChangTing/Code/Openpcdet/Pointpillars_Ros/src/lidar_point_pillars/data/testing/";
            std::string Save_Dir = "/home/user/ChangTing/Code/Openpcdet/Pointpillars_Ros/src/lidar_point_pillars/eval/kitti/object/pred_kitti/";
 
            std::unique_ptr <PointPillar> pointpillar_ptr;

            void init();
            void Getinfo(void);
            void pclToArrayI(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr, 
                            float* out_points_array,
                            const float offset_z);
            void pclToArray(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_pcl_pc_ptr, 
                            float* out_points_array,
                            const float offset_z);
            void pclSave(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr, 
                         int suffix);
            void pubDetectedObject(const std::vector<Bndbox>& nms_pred,
                                   const std_msgs::Header& in_header);
            void analyzeTFInfo(tf::StampedTransform baselink2lidar);
            void getBaselinkToLidarTF(const std::string& target_frameid);
            geometry_msgs::Pose getTransformedPose(const geometry_msgs::Pose& in_pose, 
                                                   const tf::Transform& tf);
            
        public:
            PointPillarsROS();
            ~PointPillarsROS();
            void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
            void createROSPubSub();
            void run();
    };
}


# endif