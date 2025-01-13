cmake_minimum_required(VERSION 3.10)

# Project name
project(Calculator)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add source files
add_executable(calculator
    src/main.cpp
    src/calculator.cpp
)