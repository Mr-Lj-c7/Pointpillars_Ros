#include <lidar_point_pillars/Tracker.h>

//  目标匹配检测（距离阈值）
bool Tracker::similar(const autoware_msgs::DetectedObject& a, const autoware_msgs::DetectedObject& b) {
    float d_x = a.pose.position.x - b.pose.position.x;
    float d_y = a.pose.position.y - b.pose.position.y;
    float d_z = a.pose.position.z - b.pose.position.z;

    float dist = sqrt(d_x * d_x + d_y * d_y + d_z * d_z);
    return (dist < dist_thr);
}

// 初始化跟踪参数
Tracker::Tracker() {
    object_index = 0;
    dist_thr = 5;
    appear_thr = 3;
    disappear_thr = 3;
}

int Tracker::track(const std::vector<autoware_msgs::DetectedObject> &objects, std::vector<autoware_msgs::DetectedObject> &out_objects) {
    if (appear_thr <= 0) 
    {
        out_objects = objects;
    }
    else 
    {
        std::list<TrackedObj*>::iterator ii;  // 多目标跟踪
        // 保存上一帧跟踪结果
        for (ii = tracked_objects.begin(); ii != tracked_objects.end(); ++ii ) {
            TrackedObj* tobj = *ii;
            tobj->prev_obj = tobj->obj;
            tobj->obj_id = -1;  // 新一帧数据匹配前所有跟踪目标都未匹配
        }

        // 每一个目标和已跟踪目标进行匹配，新目标加入
        for (int i = 0; i < objects.size(); i++) 
        {
            const autoware_msgs::DetectedObject& obj = objects[i];
            bool isnew = true;

            for (ii = tracked_objects.begin(); ii != tracked_objects.end(); ++ii ) 
            {
                TrackedObj* tobj = *ii;
                if (similar(obj, tobj->obj)) 
                {
                    isnew = false;
                    tobj->appear(obj, i);
                    // tobj->correct(obj);
                    break;
                }
            }

            if (isnew) 
            {
                TrackedObj* new_obj = new TrackedObj();
                new_obj->init(obj, i, this);
                tracked_objects.push_back(new_obj);
                //printf("create Track Object with trackid %d\n", new_tobj.id);
            }
        }

        // Remove disappear objects 匹配失败的目标标记为消失
        for (ii = tracked_objects.begin(); ii != tracked_objects.end(); ++ii ) {
            TrackedObj* tobj = *ii;
            if (-1 == tobj->obj_id) {
                tobj->disappear();
                // tobj->predict();
            }
        }

        // Remove lost objects
        for (ii = tracked_objects.begin(); ii != tracked_objects.end();) {
            TrackedObj* tobj = *ii;
            if (tobj->status == TrackedObj::LOSTING) {
                tobj->release();
                delete tobj;
                ii = tracked_objects.erase(ii);  // 从跟踪列表中移除
            } else {
                ++ii ;  //  继续遍历跟踪列表
            }
        }

        //  更新目标被跟踪次数
        for (ii = tracked_objects.begin(); ii != tracked_objects.end(); ++ii ) {
            TrackedObj* tobj = *ii;
            tobj->trackedCnt += 1;
            // tobj->smooth_update();
        }

        out_objects.clear();
        // 输出跟踪结果
        for (ii = tracked_objects.begin(); ii != tracked_objects.end(); ++ii ) {
            TrackedObj* tobj = *ii;
            if (tobj->status == TrackedObj::TRACKING) {
                autoware_msgs::DetectedObject& obj = tobj->obj;
                obj.track_id = tobj->id;
                // tobj->output(&obj);
                out_objects.push_back(obj);
            }
        }
        
        if (debug_)
        {
            std::cout << "tracked_objects size: " << tracked_objects.size() << "\n";
            for (ii = tracked_objects.begin(); ii != tracked_objects.end(); ++ii ) {
                TrackedObj* tobj = *ii;
                std::cout << " Tracked Id: "   << tobj->id 
                          << " status: "       << tobj->status 
                          << " appearCnt: "    << tobj->appearCnt 
                          << " disappearCnt: " << tobj->disappearCnt 
                          << " trackedCnt: "   << tobj->trackedCnt 
                          << "\n";
            }
        }  
    }

    return 0;
}

// 清空跟踪器
Tracker::~Tracker() {
    std::list<TrackedObj*>::iterator ii = tracked_objects.begin();
    for (; ii != tracked_objects.end(); ++ii ) {
       TrackedObj* tobj = *ii ;
       delete tobj;
    }
}

void TrackedObj::init(const autoware_msgs::DetectedObject &obj, int obj_id, Tracker *tracker) {
    this->obj = obj;
    this->obj_id = obj_id;
    this->id = -1;
    this->appearCnt = 1;
    this->disappearCnt = 0;
    this->trackedCnt = 0;
    this->status = INSPECTING;
    this->tracker = tracker;
    switch (status) {
        case INSPECTING:
            if (appearCnt >= tracker->appear_thr) {
                this->status = TRACKING;
                this->id = tracker->object_index++;
            } else {
                this->status = INSPECTING;
            }
            break;
        default:
            break;
    }
}

void TrackedObj::disappear() {
    this->obj = this->prev_obj;
    this->obj_id = -1;
    this->disappearCnt++;
    if (disappearCnt >= tracker->disappear_thr) {
        status = LOSTING;
    }
}

void TrackedObj::appear(const autoware_msgs::DetectedObject &obj, int obj_id) {
    this->obj = obj;
    this->obj_id = obj_id;
    this->appearCnt++;
    this->disappearCnt = 0;
    switch (status) {
        case INSPECTING:
            if (appearCnt >= tracker->appear_thr) {
                this->status = TRACKING;
                this->id = tracker->object_index++;
            } else {
                this->status = INSPECTING;
            }
            break;
        default:
            break;
    }
}

void TrackedObj::release() {
}

void TrackedObj::smooth_update() {
}



/* KF跟踪器*/

void TrackedObj_KF::initKF(const autoware_msgs::DetectedObject& detection, double dt){
    kf = std::make_shared<KalmanFilter>();
    // 初始化状态向量x=[x, y, vx, vy]
    Eigen::Vector4d x_in;
    x_in << detection.pose.position.x, 
            detection.pose.position.y, 
            0.0, 
            0.0;  // 初始速度为0
    kf->init(x_in);

    // 状态转移矩阵F
    Eigen::Matrix4d F_in;
    F_in << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;
    kf->setF(F_in);

    // 估计误差协方差矩阵P
    Eigen::Matrix4d P_in;
    P_in << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 100, 0,
            0, 0, 0, 100;  // 初始位置
    kf->setP(P_in);

    // 过程噪声协方差矩阵Q
    Eigen::Matrix4d Q_in;
    Q_in << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;  // 简化过程噪声为单位矩阵
    kf->setQ(Q_in);

    // 观测矩阵H
    Eigen::MatrixXd H_in(2, 4);
    H_in << 1, 0, 0, 0,
            0, 1, 0, 0;  // 只观测位置变化
    kf->setH(H_in);

    // 观测噪声协方差矩阵R
    Eigen::Matrix2d R_in;
    R_in << 0.0225, 0,
            0, 0.0255;  // 观测噪声协方矩阵（传感器厂商给到）
    kf->setR(R_in);
}

void TrackedObj_KF::predict(double dt) {
    if (!kf || !kf->isInit()) return;
    // 更新状态转移矩阵
    Eigen::Matrix4d F_in;
    F_in << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;
    kf->setF(F_in);
    kf->prediction();
    time_since_update++;
}

void TrackedObj_KF::update(const autoware_msgs::DetectedObject& detection) {
    if (!kf || !kf->isInit()) return;
    // 观测向量z [x, y]
    Eigen::Vector2d z;
    z << detection.pose.position.x,
         detection.pose.position.y;
    kf->MeasurementUpdate(z);
    // 更新目标属性
    Eigen::Vector4d state = kf->getState();
    obj.pose.position.x = state[0];
    obj.pose.position.y = state[1];
    obj.pose.position.z = detection.pose.position.z; // 保持原始高度
    // 使用最新检测的尺寸和方向
    obj.dimensions = detection.dimensions;
    obj.pose.orientation = detection.pose.orientation;
    obj_hits++;             // 增加匹配次数
    time_since_update = 0;  // 更新成功重置时间
}

autoware_msgs::DetectedObject TrackedObj_KF::get_state() const {
    autoware_msgs::DetectedObject result = obj;
    result.track_id = id;
    return result;
}

double TrackedObj_KF::distanceTo(const autoware_msgs::DetectedObject& other) const {
    double dx = obj.pose.position.x - other.pose.position.x;
    double dy = obj.pose.position.y - other.pose.position.y;
    double dz = obj.pose.position.z - other.pose.position.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

Tracker_KF::Tracker_KF() {
    init_();
}

char* Tracker_KF::init_() {
    next_id = 1;
    last_update_time = 0.0; 
    max_age = 3.0;               // 目标最大存活时间 5.0s
    min_hits = 3;                // 目标最小匹配次数
    gating_threshold_ = 5.0;     // 目标匹配距离阈 5.0m
    return RESET_OK;
}

// 数据关联（最邻近匹配）
std::map<int, int> Tracker_KF::data_association(const std::vector<autoware_msgs::DetectedObject>& detections){
    std::map<int, int> matches;
    std::vector<bool> matched_detections(detections.size(), false);
    for (auto& track_pair : tracked_objects) {
        int track_id = track_pair.first;
        TrackedObj_KF& track = track_pair.second;

        double min_distance = std::numeric_limits<double>::max();
        int best_match_idx = -1;
        for (size_t i = 0; i < detections.size(); ++i) {
            if(matched_detections[i]) continue;                 // 跳过已匹配的检测
            double distance = track.distanceTo(detections[i]);  
            if (distance < gating_threshold_ && distance < min_distance) {  // 距离小于阈值且所有目标中距离最小
                min_distance = distance;
                best_match_idx = i;
            }
        }
        // 匹配成功
        if (best_match_idx != -1) {
            matches[track_id] = best_match_idx;
            matched_detections[best_match_idx] = true;
        }
    }
    return matches;
}

void Tracker_KF::track(const std::vector<autoware_msgs::DetectedObject> &objects,  
        std::vector<autoware_msgs::DetectedObject> &tracked_objs, 
        double current_time) 
{ 
    // 时间差
    double dt = 0.1;  // 默认值0.1s
    if (last_update_time > 0){
        dt = current_time - last_update_time;
    }
    last_update_time = current_time;
    // 预测
    for (auto& track_pair : tracked_objects) {
        TrackedObj_KF& track_ = track_pair.second;
        track_.predict(dt);
    }
    // 数据关联(邻近匹配)
    auto matches = data_association(objects);
    // 更新匹配目标
    for (auto& match : matches) {
        int track_id = match.first;
        int detection_idx = match.second;
        tracked_objects[track_id].update(objects[detection_idx]);
    }
    // 处理未匹配的检测（创建新目标）
    for(size_t i = 0; i < objects.size(); ++i){
        bool matched = false;
        for (auto& match : matches) {
            if (match.second == i) {
                matched = true;
                break;
            }
        }
        // 检测未被匹配，创建新的跟踪对象
        if (!matched) {
            TrackedObj_KF new_obj;
            new_obj.id = next_id++;
            new_obj.obj = objects[i];
            new_obj.initKF(objects[i], dt);
            tracked_objects[new_obj.id] = new_obj;
        }
    }
    // 清理丢失的目标
    std::vector<int> to_remove;
    for (const auto& track_pair : tracked_objects) {
        const TrackedObj_KF& track_ = track_pair.second;
        // 如果未更新时间超过最大时长5s，则标记为待移除
        if (track_.time_since_update * dt > max_age) {
            to_remove.push_back(track_pair.first);
        }
    }
    for (int id : to_remove) {
        tracked_objects.erase(id);
    }
    // 输出跟踪结果
    tracked_objs.clear();
    for (const auto& track_pair : tracked_objects) {
        const TrackedObj_KF& track_ = track_pair.second;
        if (track_.obj_hits >= min_hits) {  // 只输出匹配
            tracked_objs.push_back(track_.get_state());
        }
    }
}

Tracker_KF::~Tracker_KF() {
    tracked_objects.clear();
}