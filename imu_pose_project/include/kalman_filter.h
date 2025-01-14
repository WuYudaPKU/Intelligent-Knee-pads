#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter();
    void predict(const Eigen::Vector3f& acceleration, float dt);
    void update(const Eigen::Vector3f& measured_position);
    Eigen::VectorXf getState() const;

private:
    Eigen::Matrix<float, 6, 6> A; // 状态转移矩阵
    Eigen::Matrix<float, 6, 3> B; // 控制输入矩阵
    Eigen::Matrix<float, 6, 6> Q; // 过程噪声
    Eigen::Matrix<float, 6, 6> P; // 状态协方差
    Eigen::Matrix<float, 6, 6> H; // 测量矩阵
    Eigen::Matrix<float, 6, 6> R; // 测量噪声
    Eigen::Vector<float, 6> state; // 当前状态
};

#endif // KALMAN_FILTER_H