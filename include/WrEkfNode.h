#pragma once

#include <iostream>
#include <stdio.h>
#include <chrono>
#include <random>

#include <Eigen/Dense>

#include "ros/ros.h"

#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "wr_msgs/ninelives_triplet_stamped.h"
#include "wr_msgs/imu_stamped.h"


#include "Definitions.h"
#include "EKF4.h"
#include "mshw_geolib.h"

class WrEkfNode
{
public:
    WrEkfNode();
    void run();
    void stop();
    
private:
    void subscribe();
    void initPubs();
    void estimate();
    void pubEstState();
    void pubCtrlState();
    void pubTestM();
    void pubTestE();
    
private:
    void imuCb(const wr_msgs::imu_stamped& msg);
    bool imuReady;
    
    void gnnsTripletCb(const wr_msgs::ninelives_triplet_stamped& msg);
    bool gnnsTripletReady;
    
private:
    ros::NodeHandle nodeHandle;
    ros::Subscriber imuSub;
    ros::Subscriber gnnsTripletSub;
    ros::Publisher estStatePub;
    ros::Publisher ctrlStatePub;
    int wrEkfQuenueDepth;
    ros::Rate wrEkfNodeRate;
    
    ros::Publisher testMesTripletSub;
    ros::Publisher testEstTripletSub;
    
private:
    std::string  imuTopicName;
    std::string  gnnsTripletTopicName;
    std::string  estStatePubTopicName;
    std::string  ctrlStatePubTopicName;
    
        
private:
    Vector3 gnnsBasePosEnuMes;
    Vector3 gnnsBaseVelEnuMes;
    Vector3 gnnsSlave1PosEnuMes;
    Vector3 gnnsSlave2PosEnuMes;
    int statusBase;
    int statusSlave1;
    int statusSlave2;
    
    Vector3 imuAmes;
    Vector3 imuWmes;
    
private:
    mswhgeo::Geo geo;
    bool refGeoInited;
    Vector3d refLatLonAlt;
    Vector3d refEcefXYZ;
        
private:
    EKF4 ekf;
    bool ekfInited;
    std::chrono::high_resolution_clock::time_point predictionTimePoint;
    std::chrono::high_resolution_clock::time_point gnnsCorrectionTimePoint;
    Ekf4_fullState estState;
};
