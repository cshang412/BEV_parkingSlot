cmake_minimum_required(VERSION 3.22)
project(stitch)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV 4 REQUIRED)
include_directories(include
        ${OpenCV_INCLUDE_DIRS}
        "/usr/include/eigen3"
        "/usr/include/jsoncpp/")

add_library(stitch SHARED
src/camera.cpp
src/camera_ipml.cpp
src/lut_info.cpp
src/pstitch_cpu.cpp
src/stitch_init.cpp)

target_link_libraries(stitch ${OpenCV_LIBRARIES})

add_executable(test_stitch main.cpp)
target_link_libraries(test_stitch stitch ${OpenCV_LIBRARIES}  libjsoncpp.a)