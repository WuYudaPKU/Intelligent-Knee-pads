cmake_minimum_required(VERSION 3.10)

# 设置项目名
project(Calculator)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)

# 指定源文件
set(SOURCE_FILES
    src/main.cpp
    src/calculator.cpp
)

# 创建可执行文件
add_executable(calculator ${SOURCE_FILES})