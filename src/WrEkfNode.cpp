#include "WrEkfNode.h"

#include <cmath>

WrEkfNode::WrEkfNode():
     imuTopicName("/imu/data")
    ,gnnsBasePosTopicName("/gnss_base/fix")
    ,gnnsBaseVelTopicName("/gnss_base/vel")
    ,gnnsLeftPosTopicName("/gnss_left/fix")
    //,gnnsLeftVelTopicName("/gnss_left/vel")
    ,gnnsRightPosTopicName("/gnss_right/fix")
    //,gnnsRightVelTopicName("/gnss_right/vel")
    ,estStatePubTopicName("/wr_ekf/est_state")
    ,ctrlStatePubTopicName("/wr_ekf/ctrl_state")
    ,refWgsPointInited(false)
    ,wrEkfNodeRate(200)
    ,wrEkfQuenueDepth(1)
    ,predictionTimePoint(std::chrono::high_resolution_clock::now())
    ,gnnsCorrectionTimePoint(std::chrono::high_resolution_clock::now())
    ,imuReady(false)
    ,gnnsBasePosReady(false)
    ,gnnsBaseVelReady(false)
    ,gnnsLeftPosReady(false)
    ,gnnsRightPosReady(false)
    ,normal_distribution(0.0, 1.0)
{
// //
}

void WrEkfNode::run()
{
    std::cout.precision(4);
    subscribe();
    initPubs();
    while(ros::ok)
    {
        estimate();
        pubEstState();
        ros::spinOnce();
        wrEkfNodeRate.sleep();
    }
}

void WrEkfNode::estimate()
{
    /* predict */
    if (imuReady)
    {
        uint64_t dtMs =  std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::high_resolution_clock::now() - predictionTimePoint).count();
        predictionTimePoint = std::chrono::high_resolution_clock::now();
        double dt = dtMs / 1e3f;
        
        srekf.predictImu(imuAmes, imuWmes, dt);
        imuReady = false;
    }

    /*husky gnns corrections*/
    if (gnnsBasePosReady && gnnsBaseVelReady)
    {
        predictionTimePoint = std::chrono::high_resolution_clock::now();
        
        // pv
        Vector6 pv;
        pv << gnnsBasePosEnuMes, gnnsBaseVelEnuMes;
        srekf.correctPv(pv);
        
        // v
        srekf.correctV(gnnsBaseVelEnuMes);
        
        // q
        
        Vector3 dr1 = gnnsLeftPosEnuMes - gnnsBasePosEnuMes;
        Vector3 dr2 = gnnsRightPosEnuMes - gnnsBasePosEnuMes;
        
        /*
        std::cout << "gnnsBasePosEnuMes:\n" << gnnsBasePosEnuMes << std::endl;
        std::cout << "gnnsLeftPosEnuMes:\n" << gnnsLeftPosEnuMes << std::endl;
        std::cout << "gnnsRightPosEnuMes:\n" << gnnsRightPosEnuMes << std::endl;
        std::cout << "dr1:\n" << dr1 << std::endl;
        std::cout << "dr2:\n" << dr2 << std::endl;
        */
        
        
        srekf.correctP3(dr1, dr2);
        
        gnnsBasePosReady = false;
        gnnsBaseVelReady = false;
    }
}

void WrEkfNode::pubEstState()
{
    EkfStateVector state = srekf.getEstState();
    
    /*
    std::cout << "r:\n" << state.segment(0,3) << std::endl;
    std::cout << "v:\n" << state.segment(3,3) << std::endl;
    std::cout << "a:\n" << state.segment(6,3) << std::endl;
    std::cout << "q:\n" << state.segment(9,4) << std::endl;
    std::cout << "w:\n" << state.segment(13,3) << std::endl;
    */
    
    nav_msgs::Odometry msg;
    msg.pose.pose.position.x = state[0];
    msg.pose.pose.position.y = state[1];
    msg.pose.pose.position.z = state[2];
    
    msg.pose.pose.orientation.w = state[9];
    msg.pose.pose.orientation.x = state[10];
    msg.pose.pose.orientation.y = state[11];
    msg.pose.pose.orientation.z = state[12];
    
    msg.twist.twist.linear.x = state[4];
    msg.twist.twist.linear.y = state[5];
    msg.twist.twist.linear.z = state[6];
    
    msg.twist.twist.angular.x = state[13];
    msg.twist.twist.angular.y = state[14];
    msg.twist.twist.angular.z = state[15];
    

    estStatePub.publish(msg);
}

void WrEkfNode::pubCtrlState()
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(estState[0]);
    msg.data.push_back(estState[1]);
    msg.data.push_back(estState[3]);
    msg.data.push_back(estState[4]);
    Vector4 q = estState.segment(9, 4);
    Vector3 eul = quat2Eul(q);
    
    msg.data.push_back(eul[2]);
    //std::cout << eul << std::endl << std::endl;
    ctrlStatePub.publish(msg);
}

void WrEkfNode::initPubs()
{
    estStatePub = nodeHandle.advertise<nav_msgs::Odometry>(estStatePubTopicName, wrEkfQuenueDepth);
    //ctrlStatePub = nodeHandle.advertise<std_msgs::Float32MultiArray>(ctrlStatePubTopicName, wrEkfQuenueDepth);
}

void WrEkfNode::subscribe()
{
    imuSub = nodeHandle.subscribe(imuTopicName, wrEkfQuenueDepth, &WrEkfNode::imuCb, this);
    gnnsBasePosSub = nodeHandle.subscribe(gnnsBasePosTopicName, wrEkfQuenueDepth, &WrEkfNode::gnnsBasePosCb, this);
    gnnsBaseVelSub = nodeHandle.subscribe(gnnsBaseVelTopicName, wrEkfQuenueDepth, &WrEkfNode::gnnsBaseVelCb, this);
    gnnsLeftPosSub = nodeHandle.subscribe(gnnsLeftPosTopicName, wrEkfQuenueDepth, &WrEkfNode::gnnsLeftPosCb, this);
    gnnsRightPosSub = nodeHandle.subscribe(gnnsRightPosTopicName, wrEkfQuenueDepth, &WrEkfNode::gnnsRightPosCb, this);
}

void WrEkfNode::imuCb(const sensor_msgs::Imu& msg)
{
    imuAmes = Vector3(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    imuWmes = Vector3(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    imuReady = true;
}

void WrEkfNode::gnnsBasePosCb(const sensor_msgs::NavSatFix& msg)
{
    double ne[2];
    LLtoNE(msg.latitude, msg.longitude, ne);
    
    if (!refWgsPointInited)
    {
        refWgsPoint = Vector3(ne[0], ne[1], msg.altitude);
        refWgsPointInited = true;
    }
    
    gnnsBasePosEnuMes << ne[0], ne[1], msg.altitude;
    gnnsBasePosEnuMes -= refWgsPoint;
    gnnsBasePosReady = true;
    //std::cout << "base pos:\n" << gnnsBasePosEnuMes << std::endl;
    //std::cout << "base pos:\n" << ne[0] << std::endl;
}

void WrEkfNode::gnnsBaseVelCb(const geometry_msgs::Vector3Stamped& msg)
{
    if (refWgsPointInited)
    {
        Vector3 vEnu(-msg.vector.y, msg.vector.x, msg.vector.z); // ???
        //gnnsBaseVelEnuMes = ecef2Enu(vEcef, refWgsPoint);
        gnnsBaseVelEnuMes = vEnu;
        
        gnnsBaseVelReady = true;
        //std::cout << "base vel:\n" << gnnsBaseVelEnuMes << std::endl;
    }
}

void WrEkfNode::gnnsLeftPosCb(const sensor_msgs::NavSatFix& msg)
{
    if (refWgsPointInited)
    {        
        double ne[2];
        LLtoNE(msg.latitude, msg.longitude, ne);
        gnnsLeftPosEnuMes << ne[0], ne[1], msg.altitude;
        gnnsLeftPosEnuMes -= refWgsPoint;
        
        gnnsLeftPosReady = true;
        //std::cout << "left pos:\n" << gnnsLeftPosEnuMes << std::endl;
        //std::cout << "left pos:\n" << ne[0] << std::endl;
    }
}

void WrEkfNode::gnnsRightPosCb(const sensor_msgs::NavSatFix& msg)
{
    if (refWgsPointInited)
    {        
        double ne[2];
        LLtoNE(msg.latitude, msg.longitude, ne);
        gnnsRightPosEnuMes << ne[0], ne[1], msg.altitude;
        gnnsRightPosEnuMes -= refWgsPoint;
        
        gnnsRightPosReady = true;
        //std::cout << "right pos:\n" << gnnsRightPosEnuMes << std::endl;
        //std::cout << "right pos:\n" << ne[0] << std::endl;
    }
}

