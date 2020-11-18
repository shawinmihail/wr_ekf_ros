cmake_minimum_required(VERSION 2.8.3)
project(wr_ekf_ros)

add_definitions(-std=c++11)

find_package(
    catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs nav_msgs wr_msgs
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
    wr_c_ekf
    mshw_geo/include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_library(wr_c_ekf wr_c_ekf/EKF4.cpp wr_c_ekf/MatlabDataParser.cpp wr_c_ekf/Utils.cc)
add_library(wr_ekf_node_lib src/WrEkfNode.cpp)
add_executable(wr_ekf_node src/run_wr_ekf_ros.cpp)

target_link_libraries(wr_ekf_node wr_ekf_node_lib wr_c_ekf ${catkin_LIBRARIES})

install(TARGETS wr_ekf_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        
install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
