#ifndef IMU_DATA_H
#define IMU_DATA_H

#include <Eigen/Dense>

// IMU 数据结构
struct IMUData {
    float ax, ay, az; // 加速度
    float wx, wy, wz; // 角速度
};

// IMU 位置信息
struct IMUPosition {
    Eigen::Vector3f position; // IMU 在刚体上的位置
};

#endif // IMU_DATA_H