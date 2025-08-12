#include <lidar_point_pillars/PointPillarsRos.h>
#include <iostream>
#include <fstream>

namespace PointPillarsNS
{
  // cuda event
  cudaEvent_t start, stop; 
  cudaStream_t stream = NULL;
  
  PointPillarsROS::PointPillarsROS()
  : private_nh_("~")
  , NUM_POINT_FEATURE_(5)
  , TRAINED_SENSOR_HEIGHT_(1.73f)
  , BASELINK_FRAME_("base_link")
  {
    //ros related param
    private_nh_.param<bool>("use_tracking", use_tracking_, false);
    private_nh_.param<bool>("use_kf_tracking", use_kf_tracking_, false);
    private_nh_.param<bool>("point_xyzi", point_xyzi_, false);
    private_nh_.param<bool>("baselink_support", baselink_support_, true);
    private_nh_.param<bool>("has_subscribed_baselink_", has_subscribed_baselink_, false);
    private_nh_.param<bool>("use_onnx", use_onnx_, false);

    //algorithm related params
    private_nh_.param<std::string>("input_topic", input_topic_, "");
    private_nh_.param<std::string>("output_topic", output_topic_, "");
    private_nh_.param<std::string>("tracker_output_topic", tracker_output_topic_, "");
    private_nh_.param<std::string>("pfe_onnx_file", pfe_onnx_file_, "");
    // load model
    double start = ros::Time::now().toSec();
    ROS_INFO("init start: %d %s", use_onnx_, pfe_onnx_file_.c_str());
    pointpillar_ptr.reset(new PointPillar(pfe_onnx_file_, stream));
    double end = ros::Time::now().toSec();
    ROS_WARN("init time: %f ms", (end - start)*1000);
  }

  PointPillarsROS::~PointPillarsROS(){}

  void PointPillarsROS::init()
  {
    Getinfo();
  }
  
  /**
   * @brief 显示GPU信息
   * @def Getinfo
   * @return void
   */
  void PointPillarsROS::Getinfo(void)
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
   * @brief 将点云坐标系转换到 baselink 坐标系
   * @def getTransformedPose
   * @param in_pose: 输入的点云坐标系下的pose
   * @param tf: baselink坐标系到点云坐标系的变换
   * @return transformed pose
   */
  geometry_msgs::Pose PointPillarsROS::getTransformedPose(const geometry_msgs::Pose& in_pose, const tf::Transform& tf)
  {
    tf::Transform transform;
    geometry_msgs::PoseStamped out_pose;
    transform.setOrigin(tf::Vector3(in_pose.position.x, in_pose.position.y, in_pose.position.z));
    transform.setRotation(tf::Quaternion(in_pose.orientation.x, in_pose.orientation.y, in_pose.orientation.z, in_pose.orientation.w));
    tf::poseTFToMsg(tf * transform, out_pose.pose);
    return out_pose.pose;
  }
  
  /**
   * @brief 发布pointpillars模型检测结果
   * @def pubDetectedObject
   * @param nms_pred: 检测结果
   * @param in_header: 点云header
   * @return void
   */
  void PointPillarsROS::pubDetectedObject(
    const std::vector<Bndbox>& nms_pred,
    const std_msgs::Header& in_header)
    {
      std::vector<autoware_msgs::DetectedObject> detection_objects;
      std::vector<autoware_msgs::DetectedObject> tracking_objects;
      int obj_id = 0;
      for(auto box : nms_pred)
      {
        autoware_msgs::DetectedObject obj;
        obj.header = in_header;
        obj.valid = true;
        obj.pose_reliable = true;
        // pose x, y, z 规则滤除
        // if ((box.x <= 1.75f && box.x >= -1.5f) || 
        //     (box.y <= 1.5f && box.y >= -1.5f)  || 
        //     (box.y > 15.f || box.y < -15.f)    ||
        //     (box.z > 2.25f || box.z < -1.5f)){continue;}
        // else
        // {
        //   obj.pose.position.x = box.x;
        //   obj.pose.position.y = box.y;
        //   obj.pose.position.z = box.z;
        // }
        obj.pose.position.x = box.x;
        obj.pose.position.y = box.y;
        obj.pose.position.z = box.z;
        // pose roll, pitch, yaw
        // float yaw = box.rt;
        /* 
        修正角度 - 根据KITTI数据集的坐标转换要求进行反向补偿
        KITTI格式转换: rotation_y = -yaw - M_PI / 2.0
        因此反向转换应该是: yaw = -rotation_y - M_PI / 2.0 
        */
        float yaw = box.rt + (M_PI / 2.0);
        yaw = std::atan2(std::sin(yaw), std::cos(yaw));  // (-pi, pi]
        // ROS_ERROR("yaw: %f", yaw);
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(-yaw);
        obj.pose.orientation = q;
        if (baselink_support_)
        {
          obj.pose = getTransformedPose(obj.pose, angle_transform_inversed_);
          // obj.pose = getTransformedPose(obj.pose, baselink2lidar_);  // lidar坐标系下obj的位姿变换到baselink坐标系
        }
        // w, l, h
        float width_, length_;
        if (box.w > box.l)
        {
          length_ = box.w;
          width_ = box.l;
        }
        else
        {
          length_ = box.l;
          width_ = box.w;
        }
        if ((length_ <= 0.f || width_ <= 0.f) || 
            (length_ > 6.f  || width_ > 3.f)  || 
            (box.h   <= 0.f || box.h  > 4.5f)) {continue;}
        else
        { 
          obj.dimensions.x = box.w;
          obj.dimensions.y = box.l;
          obj.dimensions.z = box.h;
        }
        // label: 0-car, 1-pedestrian, 2-cyclist
        if(box.id == 0){obj.label = "car";} 
        else if (box.id == 1){obj.label = "pedestrian";}
        else if (box.id == 2){obj.label = "cyclist";}
        else{continue;}
        obj.id = obj_id;
        obj.score = box.score;
        detection_objects.push_back(obj);
        obj_id++;
      }
      if (use_tracking_)
      {
        double start_trac = ros::Time::now().toSec();
        // ROS_WARN("Before tracking: %ld", detection_objects.size());
        if(use_kf_tracking_){    // kalman filter
          tracker_kf.track(detection_objects, tracking_objects, start_trac);
        } else{
          tracker.track(detection_objects, tracking_objects);
        }
        // tracker.track(detection_objects, tracking_objects);
        // ROS_WARN("After tracking: %ld", tracking_objects.size());
        double end_trac = ros::Time::now().toSec();
        ROS_WARN("tracking time: %f ms", (end_trac - start_trac)*1000);
        if (debug_)
        {
          autoware_msgs::DetectedObjectArray tracker_objects;
          tracker_objects.header = in_header;
          for (std::vector<autoware_msgs::DetectedObject>::iterator it = tracking_objects.begin(); it != tracking_objects.end(); it++)
          {
            tracker_objects.objects.push_back(*it);
          } 
          pub_tracker_objects_.publish(tracker_objects);
        }
      }
      autoware_msgs::DetectedObjectArray objects;
      objects.header = in_header;
      for(std::vector<autoware_msgs::DetectedObject>::iterator it = detection_objects.begin(); it != detection_objects.end(); it++) 
      {
        objects.objects.push_back(*it);
      }
      pub_objects_.publish(objects);
    }
  
  /**
   * @brief 从baselink到lidar的变换信息中分析出z轴的偏移量
   * @def analyzeTFInfo
   * @param baselink2lidar: baselink到lidar的变换
   * @return void
   */
  void PointPillarsROS::analyzeTFInfo(tf::StampedTransform baselink2lidar)
  {
    tf::Vector3 v = baselink2lidar.getOrigin();  // 获取baselink坐标系到lidar坐标系的偏移量position
    offset_z_from_trained_data_ = v.getZ() - TRAINED_SENSOR_HEIGHT_;
    // ROS_INFO("offset_z_from_trained_data_: %f", offset_z_from_trained_data_);
    // ROS_INFO("offset_x_: %f", v.getX());
    // ROS_INFO("offset_y_: %f", v.getY());
    // ROS_INFO("offset_z_: %f", v.getZ());

    tf::Quaternion q = baselink2lidar.getRotation();  // 获取baselink坐标系到lidar坐标系的旋转量orientation
    angle_transform_ = tf::Transform(q);
    angle_transform_inversed_ = angle_transform_.inverse();
  }
  
  /**
   * @brief 从tf监听中获取baselink到lidar的变换信息
   * @def getBaselinkToLidarTF
   * @param target_frameid: lidar坐标系
   * @return void
   */
  void PointPillarsROS::getBaselinkToLidarTF(const std::string& target_frameid)
  {
    try
    {
      std::cout << "BASELINK_FRAME_: " << BASELINK_FRAME_ << " " << "target_frameid: " << target_frameid << std::endl;
      tf_listener_.waitForTransform(BASELINK_FRAME_, target_frameid, ros::Time(0), ros::Duration(1.0));
      tf_listener_.lookupTransform(BASELINK_FRAME_, target_frameid, ros::Time(0), baselink2lidar_);
      analyzeTFInfo(baselink2lidar_);
      has_subscribed_baselink_ = true;
    }
    catch (tf::TransformException ex)
    {
      // std::cout << "getBaselinkToLidarTF TransformException" << std::endl;
      ROS_ERROR("%s", ex.what());
    }
  }
  
  /**
   * @brief 将点云数据保存到本地文件
   * @def pclSave
   * @param in_pcl_pc_ptr: 输入点云数据
   * @param suffix: 文件名后缀
   * @return void
   */
  void PointPillarsROS::pclSave(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr, 
    int suffix)
  {
    fstream ofile;
    string file_name = "/home/user/ChangTing/Code/Openpcdet/Pointpillars_Ros/src/lidar_point_pillars/PointCloudOut/point_cloud-" + std::to_string(suffix) + ".bin";
    // string file_name = "out-" + std::to_string(suffix) + ".bin";
    ofile.open(file_name, std::ios::out | std::ios::binary );

    float f = 0;
    for (size_t i = 0; i < in_pcl_pc_ptr->size(); i++)
    {
      pcl::PointXYZI point = in_pcl_pc_ptr->at(i);
      if (i == 0)
        std::cout << point;
      ofile.write(reinterpret_cast<const char*>(&point.x), sizeof(float));
      ofile.write(reinterpret_cast<const char*>(&point.y), sizeof(float));
      ofile.write(reinterpret_cast<const char*>(&point.z), sizeof(float));
      ofile.write(reinterpret_cast<const char*>(&point.intensity), sizeof(float));
      ofile.write(reinterpret_cast<const char*>(&f), sizeof(float));
    }
    std::cout << "Open Save File: " << file_name << " " <<  "points num: " << in_pcl_pc_ptr->size() << "\n";
    ofile.close();
  }
  
  /**
   * @brief 将点云数据转化为数组
   * @def pclToArray
   * @param in_pcl_pc_ptr: 输入点云数据
   * @param out_points_array: 输出数组
   * @param offset_z: 点云数据z轴偏移量
   * @return void
   */
  void PointPillarsROS::pclToArrayI(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr, 
    float* out_points_array,
    const float offset_z)
  {
    for (size_t i = 0; i < in_pcl_pc_ptr->size(); i++)
    {
      pcl::PointXYZI point = in_pcl_pc_ptr->at(i);
      out_points_array[i * NUM_POINT_FEATURE_ + 0] = point.x;
      out_points_array[i * NUM_POINT_FEATURE_ + 1] = point.y;
      out_points_array[i * NUM_POINT_FEATURE_ + 2] = point.z + offset_z;
      out_points_array[i * NUM_POINT_FEATURE_ + 3] = float(point.intensity);
      for (size_t j = 4; j < NUM_POINT_FEATURE_; j++)
      {
        out_points_array[i * NUM_POINT_FEATURE_ + j] = 0.f;
      }
    }
  }
  
  void PointPillarsROS::pclToArray(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_pcl_pc_ptr, 
    float* out_points_array,
    const float offset_z)
  {
    for (size_t i = 0; i < in_pcl_pc_ptr->size(); i++)
    {
      pcl::PointXYZ point = in_pcl_pc_ptr->at(i);
      out_points_array[i * (NUM_POINT_FEATURE_ - 1) + 0] = point.x;
      out_points_array[i * (NUM_POINT_FEATURE_ - 1) + 1] = point.y;
      out_points_array[i * (NUM_POINT_FEATURE_ - 1) + 2] = point.z + offset_z;
      out_points_array[i * (NUM_POINT_FEATURE_ - 1) + 3] = 0.f;
    }
  }
  
  /**
   * @brief 点云数据回调函数
   * @def pointsCallback
   * @param msg: 输入点云数据
   * @return void
   */
  void PointPillarsROS::pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  { 
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg, *cloud);
      
      if (baselink_support_)
      {
        if (!has_subscribed_baselink_)
        {
          getBaselinkToLidarTF(msg->header.frame_id);
        }
        pcl_ros::transformPointCloud(*cloud, *cloud, angle_transform_);
      }
      size_t points_size = cloud->size();
      std::cout << "cloud size: " << cloud->size() << std::endl;
      float* points = new float[points_size * NUM_POINT_FEATURE_];
      // for(size_t i = 0; i < points_size; i++)
      // {
      //     points[i * NUM_POINT_FEATURE_ + 0] = cloud->points[i].x;
      //     points[i * NUM_POINT_FEATURE_ + 1] = cloud->points[i].y;
      //     points[i * NUM_POINT_FEATURE_ + 2] = cloud->points[i].z;
      //     points[i * NUM_POINT_FEATURE_ + 3] = cloud->points[i].intensity;
      // }
      if (baselink_support_ && has_subscribed_baselink_)
      {
        pclToArrayI(cloud, points, offset_z_);
      // pclToArray(cloud, points, offset_z_from_trained_data_);
      }
      else
      {
        pclToArrayI(cloud, points, offset_z_);
      }
      // pclSave(cloud, msg->header.seq);
      std::cout << "<<<<<<<<<<<" << std::endl;
      float *points_data = nullptr;
      unsigned int points_data_size = points_size * 4 * sizeof(float);
      checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size));
      checkCudaErrors(cudaMemcpy(points_data, points, points_data_size, cudaMemcpyHostToDevice));
      checkCudaErrors(cudaDeviceSynchronize());
      
      cudaEventRecord(start, stream);
      
      pointpillar_ptr->doinfer(points_data, points_size, nms_pred); // 推理
      
      cudaEventRecord(stop, stream);
      cudaEventSynchronize(stop);
      cudaEventElapsedTime(&elapsedTime, start, stop);
      ROS_INFO("TIME: pointpillar: %f ms.", elapsedTime);
      
      checkCudaErrors(cudaFree(points_data));
      ROS_INFO("Bndbox objs: %lu", nms_pred.size());
      // publish detected objects
      pubDetectedObject(nms_pred, msg->header);
      nms_pred.clear();
      delete[] points;
      std::cout << ">>>>>>>>>>>" << std::endl;
  }
  
  /**
   * @brief create ROS pub and sub
   * @def createROSPubSub
   * @return void
   */
  void PointPillarsROS::createROSPubSub()
  {
    int queue_size_ = 1;
    // std::string input_topic_ =  "/kitti/velo/pointcloud";
    // std::string output_topic_ = "/detection/lidar_detector/objects";
    sub_points_ = private_nh_.subscribe<sensor_msgs::PointCloud2>(input_topic_, queue_size_, &PointPillarsROS::pointsCallback, this);
    pub_objects_ = private_nh_.advertise<autoware_msgs::DetectedObjectArray>(output_topic_, queue_size_);
    pub_tracker_objects_ = private_nh_.advertise<autoware_msgs::DetectedObjectArray>(tracker_output_topic_, queue_size_);
  }

  /**
   * @brief run
   * @def run
   * @return void
   */
  void PointPillarsROS::run()
  {
    init();
    // get uda events and stream
    checkCudaErrors(cudaEventCreate(&start));
    checkCudaErrors(cudaEventCreate(&stop));
    checkCudaErrors(cudaStreamCreate(&stream));
    
    createROSPubSub();
    ros::spin();
    
    // release cuda events and stream
    checkCudaErrors(cudaEventDestroy(start));
    checkCudaErrors(cudaEventDestroy(stop));
    checkCudaErrors(cudaStreamDestroy(stream));
  }
  
}


