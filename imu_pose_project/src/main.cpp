#include "imu_data.h"
#include "pose_estimation.h"
#include "kalman_filter.h"
#include "serial_interface.h"
#include <iostream>

int main() {
    SerialInterface serial("/dev/ttyUSB0", 115200);
    KalmanFilter kf;

    std::vector<IMUPosition> imu_positions = {
        {Eigen::Vector3f(0, 1, 0)},
        {Eigen::Vector3f(-0.5, 0, 0)},
        {Eigen::Vector3f(0.5, 0, 0)}
    };

    while (true) {
        std::vector<IMUData> imu_data;
        for (size_t i = 0; i < imu_positions.size(); ++i) {
            imu_data.push_back(serial.readIMUData());
        }

        Pose pose = calculatePose3D(imu_data, imu_positions, 0.01f);
        kf.predict(pose.position, 0.01f);
        kf.update(pose.position);

        std::cout << "Position: " << kf.getState().transpose() << std::endl;
    }
    return 0;
}