#ifndef POSE_ESTIMATION_H
#define POSE_ESTIMATION_H

#include "imu_data.h"
#include <Eigen/Dense>

// 刚体位姿结构
struct Pose {
    Eigen::Vector3f position;  // 位置 (x, y, z)
    Eigen::Quaternionf orientation; // 四元数姿态
};

// 计算刚体的 3D 位姿
Pose calculatePose3D(const std::vector<IMUData>& imu_data, 
                     const std::vector<IMUPosition>& imu_positions, 
                     float dt);

#endif // POSE_ESTIMATION_H