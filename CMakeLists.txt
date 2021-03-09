project(KF_Tracking)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/main.cpp src/tracking.cpp src/kalman_filter.cpp src/tracking.h src/kalman_filter.h)

add_executable(KF_Tracking ${sources})