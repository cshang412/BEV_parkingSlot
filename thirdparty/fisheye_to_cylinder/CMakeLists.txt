cmake_minimum_required(VERSION 3.0)
project(camera_model)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -std=c++11 -O2 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -O2 -Wall")
add_compile_options(-fPIC)
add_compile_options(-g)

find_package(OpenCV 4 REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    "/usr/include/eigen3"
    src/
    interface/)

set(OUTPUT_MAIN_DIR ${PROJECT_BINARY_DIR}/)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_MAIN_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_MAIN_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_MAIN_DIR}/lib)

file(GLOB_RECURSE CPP_FILES src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${CPP_FILES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

add_executable(test_camera_model sample/test_camera_model.cpp)
target_link_libraries(test_camera_model ${OpenCV_LIBRARIES} camera_model)

