#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include "imu_data.h"
#include <string>
#include <fstream>

/* SerialInterface class to handle IMU data input */
class SerialInterface {
public:
    // Constructor and Destructor
    SerialInterface(const std::string& test_file);
    ~SerialInterface();

    // Read IMU data
    IMUData readIMUData();

private:
    std::ifstream input_file; // File stream for reading test data
};

#endif // SERIAL_INTERFACE_H