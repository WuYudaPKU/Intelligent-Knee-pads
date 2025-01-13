#include "kalman_filter.h"

KalmanFilter::KalmanFilter() {
    A.setIdentity();
    B.setZero();
    Q.setIdentity() *= 0.01;
    P.setIdentity() *= 0.1;
    H.setIdentity();
    R.setIdentity() *= 0.05;
    state.setZero();
}

void KalmanFilter::predict(const Eigen::Vector3f& acceleration, float dt) {
    state = A * state + B * acceleration * dt;
    P = A * P * A.transpose() + Q;
}

void KalmanFilter::update(const Eigen::Vector3f& measured_position) {
    Eigen::Vector<float, 6> y = measured_position - H * state;
    Eigen::Matrix<float, 6, 6> S = H * P * H.transpose() + R;
    Eigen::Matrix<float, 6, 6> K = P * H.transpose() * S.inverse();

    state += K * y;
    P = (Eigen::Matrix<float, 6, 6>::Identity() - K * H) * P;
}

Eigen::VectorXf KalmanFilter::getState() const {
    return state;
}