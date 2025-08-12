#ifndef _TRACKER_H_
#define _TRACKER_H_
#pragma once

// headers ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <list>
#include <stdio.h>
#include <cmath>
#include <string>
#include <vector>
#include <assert.h>
#include <map>

#include "autoware_msgs/DetectedObjectArray.h"
#include "Kalman_filter.h"

#define RESET_OK nullptr // reset ok

using namespace std;
using namespace Eigen;

class Tracker;

class TrackedObj {
public:
    autoware_msgs::DetectedObject prev_obj;
    autoware_msgs::DetectedObject obj;
    int obj_id;        // 目标的ID
    int id;            // 追踪目标ID （status == TRACKING）
    int appearCnt;     // 目标出现次数
    int disappearCnt;  // 目标消失次数
    int trackedCnt;    // 目标被跟踪次数
    Tracker* tracker;  
    enum TrackingStatus {
        INSPECTING,
        TRACKING,
        LOSTING
    };

    TrackingStatus status;
    void init(const autoware_msgs::DetectedObject& obj, int obj_id, Tracker* tracker);
    void disappear();
    void appear(const autoware_msgs::DetectedObject& obj, int obj_id);
    void release();
    void smooth_update();

    // used for smoothing the object
    // void predict() {}
    // void correct(const autoware_msgs::DetectedObject& obj) {}

    ~TrackedObj() {}
};

class Tracker {
public:
    friend class TrackedObj;
    float dist_thr;      // 目标匹配阈值
    int appear_thr;      // 目标出现次数阈值
    int disappear_thr;   // 目标消失次数阈值
    bool debug_ = false;
protected:
    int object_index;    // 被追踪的目标索引
    std::list<TrackedObj*> tracked_objects;  // 被追踪的目标列表
public:
    Tracker();
    // void init();  
    bool similar(const autoware_msgs::DetectedObject& a, const autoware_msgs::DetectedObject& b);  // 判断两个目标是否匹配
    int track(const std::vector<autoware_msgs::DetectedObject> &objects,  std::vector<autoware_msgs::DetectedObject> &out_objects);  // 追踪
    ~Tracker();
};

class Tracker_KF;

class TrackedObj_KF { 
    public:
        autoware_msgs::DetectedObject obj;
        int id;                    // 最终目标的ID
        int obj_age = 0;           // 目标存活时间
        int obj_hits = 0;          // 目标匹配次数
        int time_since_update = 0; // 跟踪器未更新的时间
        std::shared_ptr<KalmanFilter> kf;

        // initializa the Kalman filter 
        void initKF(const autoware_msgs::DetectedObject& detection, double dt = 0.1);

        // predict the next state of the object
        void predict(double dt);

        // update the object state
        void update(const autoware_msgs::DetectedObject& detection);

        // get the current state of the object
        autoware_msgs::DetectedObject get_state() const;

        // caculate the distance iwth the other object
        double distanceTo(const autoware_msgs::DetectedObject& other) const;

};

class Tracker_KF {
private:  
    int next_id;
    double last_update_time; 
    double max_age;               // 目标最大存活时间 s
    double min_hits;              // 目标最小匹配次数
    double gating_threshold_;     // 目标匹配距离阈 m

    std::map<int, TrackedObj_KF> tracked_objects;  // 被追踪的目标列表

    // 数据关联（最邻近匹配）
    std::map<int, int> data_association(const std::vector<autoware_msgs::DetectedObject>& detections);

public:
    Tracker_KF();
    ~Tracker_KF();
    char* init_();
    
    void track(const std::vector<autoware_msgs::DetectedObject> &objects,  
        std::vector<autoware_msgs::DetectedObject> &tracked_objs, 
        double current_time);
};

# endif