#ifndef KALMANFILTER_H
#define KALMANFILTER_H

# include <Eigen/Dense>
# include <Eigen/Core>
# include <math.h>
# include <cmath>
# include <vector>
# include <string>

using namespace std;
using namespace Eigen;

# define SET_OK nullptr

/**
*KF 卡尔曼滤波器的关键点：[理论解析](https://zhuanlan.zhihu.com/p/45238681)
    * 1. 状态向量 x_: 根据目标的运动学模型确定状态向量的维度，这里为4维，x_=[x,y,vx,vy],表示一个二维空间中物体的位置和速度；
    * 2. 状态转移矩阵 F_: 描述系统状态如何从一个时间步转移到下一个时间步，这里简化物体为匀速直线运动,F_需要根据时间步进行更新；
    * 3. 估计误差协方差矩阵 P_: 描述状态估计的不确定性，P_需要根据系统的噪声和状态转移进行更新；
    * 4. 过程噪声协方差矩阵 Q_: 描述物体运动过程中所导致的噪声，Q_对整个系统影响但无法确定影响程度，Q_一般为单位矩阵；
    * 5. 测量矩阵 H_: 描述如何从状态向量映射到观测值，这里只观测位置变化，H_为2x4矩阵，H_根据状态向量维度确定，计算时不需要进行更新；
    * 6. 测量噪声矩阵 R_: 描述传感器测量过程中所导致的噪声，一般由传感器厂商提供；
    * 7. 观测值 z_: 传感器测量的观测值，这里为二维位置坐标[x, y],z_需要根据时间步长进行更新；
*/
class KalmanFilter {
    private:
        Eigen::VectorXd x_; // 状态向量
        // prediction part
        Eigen::MatrixXd F_; // 状态转移矩阵
        Eigen::MatrixXd P_; // 估计误差矩阵
        Eigen::MatrixXd Q_; // 过程噪声矩阵
        // Measurement part
        Eigen::MatrixXd H_; // 测量矩阵
        Eigen::MatrixXd R_; // 测量噪声矩阵
        Eigen::VectorXd z_; // 观测值

        bool is_init;
        Eigen::Vector4d u = Eigen::Vector4d::Zero();  // 不考虑加速度
        
    public:
        KalmanFilter():is_init(false) {};
        ~KalmanFilter(){};
        
        bool isInit() {
            return is_init;
        };
        
        void init(const Eigen::VectorXd &x0) {
            if (x0.size() != 4) {
            throw std::invalid_argument("State vector must be size 4 [x, y, vx, vy]");  // 考虑为二维平面匀速运动
            }
            x_ = x0;
            is_init = true;
        };

        void setF(const Eigen::MatrixXd &F) {    // 4x4
            if (F.rows() != 4 || F.cols() != 4) {
            throw std::invalid_argument("F matrix must be 4x4");
            }
            F_ = F;
        };
        void setP(const Eigen::MatrixXd &P) {    // 4x4
            if (P.rows() != 4 || P.cols() != 4) {
            throw std::invalid_argument("P matrix must be 4x4");
            }
            P_ = P;
        };
        void setQ(const Eigen::MatrixXd &Q) {    // 4x4
            if (Q.rows() != 4 || Q.cols() != 4) {
            throw std::invalid_argument("Q matrix must be 4x4");
            }
            Q_ = Q;
        };

        void setH(const Eigen::MatrixXd &H) {    // 2x4
            if (H.rows() != 2 || H.cols() != 4) {
            throw std::invalid_argument("H matrix must be 2x4");
            }
            H_ = H;
        };

        void setR(const Eigen::MatrixXd &R) {    // 2x2
            if (R.rows() != 2 || R.cols() != 2) {
            throw std::invalid_argument("R matrix must be 2x2");
            }
            R_ = R;
        };

        void prediction() {
            if (!is_init) {
                throw std::runtime_error("KalmanFilter not initialized");
            }
            x_ = F_ * x_ + u;
            Eigen::MatrixXd Ft = F_.transpose();
            P_ = F_ * P_ * Ft + Q_;
        };

        void MeasurementUpdate(const Eigen::VectorXd &z) {
            Eigen::VectorXd y = z - H_ * x_; 
            MatrixXd S = H_ * P_ * H_.transpose() + R_;
            MatrixXd K = P_ * H_.transpose() * S.inverse();
            x_ = x_ + K * y;
            int n = x_.size();
            MatrixXd I = MatrixXd::Identity(n, n);
            P_ = (I - K * H_) * P_;
        };

        Eigen::VectorXd getState() {
            return x_;
        };
};

/**
* EKF 扩展卡尔曼滤波器[理论解析](https://zhuanlan.zhihu.com/p/63641680)
    * 1. EKF在KF的基础上，增加了非线性系统的处理能力，通过对状态转移和观测模型进行线性化来处理非线性问题；
    * 2. EKF的状态转移矩阵 F_和测量矩阵 H_需要根据当前状态进行雅可比矩阵计算，以适应非线性系统；
    * 3. EKF的过程噪声协方差矩阵 Q_和测量噪声协方差矩阵 R_同样需要根据系统特性进行调整。
    * 4. EKF的预测和更新步骤与KF类似，但需要使用雅可比矩阵来线性化非线性系统。
 * 卡尔曼滤波器的应用场景：毫米波雷达(24Ghz 79Ghz 92Ghz ARS408 ARS410) 多普勒效应，测量数据位于极坐标下，主要用测量目标速度
    * 1. 目标跟踪：在视频监控、无人驾驶等场景中，卡尔曼滤波器可以用于跟踪目标的位置和速度，处理传感器噪声和不确定性； 
 */
class ExtendedKalmanFilter { 
    private:
        bool is_init;
        Eigen::VectorXd u;  // 不考虑加速度a
        // 预测部分
        Eigen::VectorXd x_; // 状态向量
        Eigen::MatrixXd F_; // 状态转移矩阵
        Eigen::MatrixXd P_; // 估计误差矩阵
        Eigen::MatrixXd Q_; // 过程噪声矩阵
        // 状态更新
        Eigen::MatrixXd H_; // 测量矩阵
        Eigen::MatrixXd R_; // 测量噪声矩阵
        Eigen::VectorXd z_; // 观测值  [ρ， θ, vρ]  极坐标下的测量值：经向距离、方位角、径向速度

    public:
        ExtendedKalmanFilter():is_init(false) {};
        ~ExtendedKalmanFilter(){};
        bool isInit() {
            return is_init;
        };

        void init(const Eigen::VectorXd &x_in){    // x_in为4维向量[px, py, vx, vy]
            x_ = x_in;
        }

        char* setF(const Eigen::MatrixXd &F_in){
            F_ = F_in;
            return SET_OK;
        }

        char* setP(const Eigen::MatrixXd &P_in){
            P_ = P_in;
            return SET_OK;
        }

        char* setQ(const Eigen::MatrixXd &Q_in){
            Q_ = Q_in;
            return SET_OK;
        }

        char* setR(const Eigen::MatrixXd &R_in){
            R_ = R_in;
            return SET_OK;
        }

        void calculateJacobianMatrix()
        {
            Eigen::MatrixXd H_j(3, 4);
            // 状态参数
            float px = x_(0);
            float py = x_(1);
            float vx = x_(2);
            float vy = x_(3);
            float c1 = px * px + py * py;
            float c2 = sqrt(c1);
            float c3 = (c1 * c2);
            if (c1 < 0.0001) {
                H_ = H_j;
                return;
            }
            // 计算雅可比矩阵
            H_j <<  (px/c2),  (py/c2), 0, 0,
                    -(py/c1), (px/c1), 0, 0,
                    (py*(vx*py - vy*px))/c3, (px*(vy*px - vx*py))/c3, (px/c2), (py/c2);
            H_ = H_j;
            return;
        }
        
        // 预测更新
        void measurementUpdate(const Eigen::VectorXd &z_in)
        { 
            double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));  // 经向距离
            double theta = atan2(x_(1), x_(0));  // 方位角
            double rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
            VectorXd h_ = VectorXd(3);
            h_ << rho, theta, rho_dot;  // 预测值

            Eigen::VectorXd y = z_in - h_;

            calculateJacobianMatrix();

            Eigen::MatrixXd S = H_*P_*H_.transpose() + R_;
            Eigen::MatrixXd K = P_*H_.transpose()*S.inverse();
            x_ = x_ + (K*y);
            int n = x_.size();
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
            P_ = (I - (K*H_))*P_;
        }

};

# endif