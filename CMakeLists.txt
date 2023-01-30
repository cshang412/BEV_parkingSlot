cmake_minimum_required(VERSION 3.22)
project(psd_mapping)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS         "$ENV{CXXFLAGS} -std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++17")

find_package(PCL      REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(GTSAM    REQUIRED)
find_package(Ceres    REQUIRED)
#find_package(fast_gicp    REQUIRED)
find_package(Pangolin     REQUIRED)

add_subdirectory(thirdparty/stitch)
add_subdirectory(thirdparty/fisheye_to_cylinder)

include_directories(${PCL_INCLUDE_DIRS}
                    ${GTSAM_INCLUDE_DIR}
                    ${CERES_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${fast_gicp_INCLUDE_DIRS}
                    ${Pangolin_INCLUDE_DIRS}
                    "/usr/include/eigen3"
                    "/usr/include/jsoncpp/"
                    thirdparty/stitch/include/
                    thirdparty/fisheye_to_cylinder/interface/
                    include)

add_definitions(${PCL_DEFINITIONS})

file(GLOB_RECURSE CPP_FILES src/*.cpp)

add_library(psd_mapping STATIC ${CPP_FILES} )
target_link_libraries(psd_mapping ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${fast_gicp_LIBRARIES} ${Pangolin_LIBRARIES} gtsam libjsoncpp.a)

add_executable(test_pcl test.cpp)
target_link_libraries(test_pcl ${PCL_LIBRARIES} glog ${OpenCV_LIBRARIES} ${fast_gicp_LIBRARIES} ${Pangolin_LIBRARIES} gtsam psd_mapping libjsoncpp.a stitch camera_model)
