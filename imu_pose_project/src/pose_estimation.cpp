#include "pose_estimation.h"

Pose calculatePose3D(const std::vector<IMUData>& imu_data, 
                     const std::vector<IMUPosition>& imu_positions, 
                     float dt) {
    Pose pose;
    pose.position = Eigen::Vector3f::Zero();
    pose.orientation = Eigen::Quaternionf::Identity();

    Eigen::Vector3f linear_acceleration = Eigen::Vector3f::Zero();
    Eigen::Vector3f angular_velocity = Eigen::Vector3f::Zero();

    for (size_t i = 0; i < imu_data.size(); ++i) {
        linear_acceleration += Eigen::Vector3f(imu_data[i].ax, imu_data[i].ay, imu_data[i].az);
        angular_velocity += Eigen::Vector3f(imu_data[i].wx, imu_data[i].wy, imu_data[i].wz);
    }

    linear_acceleration /= imu_data.size();
    angular_velocity /= imu_data.size();

    Eigen::Vector3f delta_angle = angular_velocity * dt;
    Eigen::Quaternionf delta_orientation(1, delta_angle.x() / 2, delta_angle.y() / 2, delta_angle.z() / 2);
    delta_orientation.normalize();
    pose.orientation = pose.orientation * delta_orientation;

    pose.position += 0.5 * linear_acceleration * dt * dt;

    return pose;
}