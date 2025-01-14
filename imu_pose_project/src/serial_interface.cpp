#include "serial_interface.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdlib>

/* Constructor */
SerialInterface::SerialInterface(const std::string& test_file) {
    // Open the test data file
    input_file.open(test_file);
    if (!input_file.is_open()) {
        std::cerr << "Error: Unable to open test data file: " << test_file << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << "Test data file " << test_file << " successfully opened." << std::endl;
}

/* Destructor */
SerialInterface::~SerialInterface() {
    if (input_file.is_open()) {
        input_file.close();
    }
}

/* Read IMU data from file */
IMUData SerialInterface::readIMUData() {
    std::string line;
    IMUData imu_data;

    if (std::getline(input_file, line)) {
        std::istringstream iss(line);

        // Parse the CSV line into IMUData structure
        char delimiter;
        iss >> imu_data.ax >> delimiter
            >> imu_data.ay >> delimiter
            >> imu_data.az >> delimiter
            >> imu_data.wx >> delimiter
            >> imu_data.wy >> delimiter
            >> imu_data.wz;

        return imu_data;
    } else {
        // End of file reached
        std::cerr << "Error: End of test data file reached!" << std::endl;
        exit(EXIT_FAILURE);
    }
}