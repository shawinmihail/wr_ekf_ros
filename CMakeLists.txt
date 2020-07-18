cmake_minimum_required(VERSION 2.8.3)
project(wr_ekf_ros)

add_definitions(-std=c++11)

find_package(
    catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs
)

# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
find_package (Eigen3 REQUIRED NO_MODULE)

# GAZEBO

set(PROTOBUF_IMPORT_DIRS "")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        sensor_msgs
    DEPENDS
        ${EIGEN_PACKAGE}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(wr_ekf src/run_wr_ekf_ros.cpp)
add_library(wr_ekf_lib src/WrEkfNode.cpp src/SREKF.cpp src/Utils.cc)

target_link_libraries(
    wr_ekf wr_ekf_lib ${catkin_LIBRARIES}
)
