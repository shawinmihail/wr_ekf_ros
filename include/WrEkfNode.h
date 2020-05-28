#pragma once

#include <iostream>
#include <stdio.h>
#include <chrono>
#include <random>

#include <Eigen/Dense>

#include "ros/ros.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkStates.h"

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
    void pubCtrlState();
    
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
    ros::Publisher ctrlStatePub;
    
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
    std::string  ctrlStatePubTopicName;
    

/* GAZEBO STATE*/
private:
    std::string  modelStateTopicName;
    ros::Subscriber modelStateSub;
    void modelStateCb(const gazebo_msgs::ModelStates& msg);
    EkfStateVector modelState;
    bool modelStateReady;
    
    std::string  linkStateTopicName;
    ros::Subscriber linkStateSub;
    void linkStateCb(const gazebo_msgs::LinkStates& msg);
    bool linkStateReady;
    Vector3 linkBasePos;
    Vector3 linkBaseVel;
    Vector3 linkLeftPos;
    Vector3 linkRightPos;
    
/* noise */
private:
    std::default_random_engine random_generator;
    std::normal_distribution<double> normal_distribution;
    
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
    std::chrono::high_resolution_clock::time_point predictionTimePoint;
    std::chrono::high_resolution_clock::time_point gnnsCorrectionTimePoint;
    EkfStateVector estState;
};

/*
#include "std_msgs/Float32MultiArray.h"
void ctrlStateCb(const std_msgs::Float32MultiArray msg){
    float x = msg.data.at(0);
    float y = msg.data.at(1);
    float vx = msg.data.at(2);
    float vy = msg.data.at(3);
    float yaw = msg.data.at(4);

}
*/
