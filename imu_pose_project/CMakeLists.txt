cmake_minimum_required(VERSION 3.10)
project(IMUPoseProject)

set(CMAKE_CXX_STANDARD 14)

# 添加 Eigen 库
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 添加 Boost 库
find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})

# 添加头文件路径
include_directories(include)

# 源文件
set(SOURCES
    src/imu_data.cpp
    src/pose_estimation.cpp
    src/kalman_filter.cpp
    src/serial_interface.cpp
    src/main.cpp
)

add_executable(IMUPose ${SOURCES})
target_link_libraries(IMUPose ${Boost_LIBRARIES})