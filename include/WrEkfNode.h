#pragma once

#include <iostream>
#include <stdio.h>
#include <chrono>

#include <Eigen/Dense>

#include "ros/ros.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32MultiArray.h"

#include "Definitions.h"
#include "SREKF.h"

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
    
private:
    void imuCb(const sensor_msgs::Imu& msg);
    bool imuReady;
    
    void gnnsBasePosCb(const sensor_msgs::NavSatFix& msg);
    bool gnnsBasePosReady;
    
    void gnnsBaseVelCb(const geometry_msgs::Vector3Stamped& msg);
    bool gnnsBaseVelReady;
    
    void gnnsLeftPosCb(const sensor_msgs::NavSatFix& msg);
    bool gnnsLeftPosReady;
    
    void gnnsRightPosCb(const sensor_msgs::NavSatFix& msg);
    bool gnnsRightPosReady;

private:
    ros::NodeHandle nodeHandle;
    
    ros::Subscriber imuSub;
    ros::Subscriber gnnsBasePosSub;
    ros::Subscriber gnnsBaseVelSub;
    ros::Subscriber gnnsLeftPosSub;
    //ros::Subscriber gnnsLeftVelSub;
    ros::Subscriber gnnsRightPosSub;
    //ros::Subscriber gnnsRightVelSub;
    
    ros::Publisher estStatePub;
    
    int wrEkfQuenueDepth;

    ros::Rate wrEkfNodeRate;
    
private:
    std::string  imuTopicName;
    std::string  gnnsBasePosTopicName;
    std::string  gnnsBaseVelTopicName;
    std::string  gnnsLeftPosTopicName;
    //std::string  gnnsLeftVelTopicName;
    std::string  gnnsRightPosTopicName;
    //std::string  gnnsRightVelTopicName;
    
    std::string  estStatePubTopicName;
    
private:
    Vector3 gnnsBasePosEnuMes;
    Vector3 gnnsLeftPosEnuMes;
    Vector3 gnnsRightPosEnuMes;
    
    Vector3 gnnsBaseVelEnuMes;
    
    Vector3 imuAmes;
    Vector3 imuWmes;
    
private:
    Vector3 refWgsPoint;
    bool refWgsPointInited;
        
private:
    SREKF srekf;
    std::chrono::high_resolution_clock::time_point stateEstimationTimePoint;
    EkfStateVector estState;
};
