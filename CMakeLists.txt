cmake_minimum_required(VERSION 2.8)

project(ParticleFilterTrackingSmallTarget)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(tracker ParticleFilterTrackingSmallTarget/main.cpp
        ParticleFilterTrackingSmallTarget/BinaryFileStream.cpp
        ParticleFilterTrackingSmallTarget/BinaryFileStream.h
        ParticleFilterTrackingSmallTarget/Tracker.cpp
        ParticleFilterTrackingSmallTarget/Tracker.h
        ParticleFilterTrackingSmallTarget/State.h
        ParticleFilterTrackingSmallTarget/Orientation.h
        ParticleFilterTrackingSmallTarget/Utils.cpp
        ParticleFilterTrackingSmallTarget/Utils.h)

target_link_libraries(tracker ${OpenCV_LIBS})