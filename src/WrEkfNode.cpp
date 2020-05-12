#include "WrEkfNode.h"

#include <cmath>

Vector3 wgs2Enu(const Vector3& LatLonAlt, const Vector3& LatLonAlt0)
{
    float e = LatLonAlt[1] - LatLonAlt0[1];
    float n = LatLonAlt[0] - LatLonAlt0[0];
    float u = LatLonAlt[2] - LatLonAlt0[2];
    float refLat = LatLonAlt[0];
    float nm = n * 333400.0f / 3.0f;
    float em = e * 1001879.0f * cos(refLat * 3.1415f / 180.0f) / 9.0f;
    return Vector3(em, nm, u);
}

Vector3 ecef2Enu(const Vector3& v, const Vector3& LatLonAlt0)
{
    float l = LatLonAlt0[1] * 180.f / 3.1415f;
    float f = LatLonAlt0[0] * 180.f / 3.1415f;
    Eigen::Matrix<float, 3, 3> A_enu_ecef;
    A_enu_ecef << -sin(l), -cos(l) * sin(f), cos(l) * cos(f), 
                   cos(l), -sin(l) * sin(f), sin(l) * cos(f),
                   0.f, cos(f), sin(f);
    return A_enu_ecef.transpose() * v;
}

WrEkfNode::WrEkfNode():
     imuTopicName("/imu/data")
    ,gnnsBasePosTopicName("/gnss_base/fix")
    ,gnnsBaseVelTopicName("/gnss_base/vel")
    ,gnnsLeftPosTopicName("/gnss_left/fix")
    //,gnnsLeftVelTopicName("/gnss_left/vel")
    ,gnnsRightPosTopicName("/gnss_right/fix")
    //,gnnsRightVelTopicName("/gnss_right/vel")
    ,estStatePubTopicName("/wr_ekf/est_state")
    
    ,refWgsPointInited(false)
    
    ,wrEkfNodeRate(200)
    ,wrEkfQuenueDepth(1)
    
    ,stateEstimationTimePoint(std::chrono::high_resolution_clock::now())
    
    ,imuReady(false)
    ,gnnsBasePosReady(false)
    ,gnnsBaseVelReady(false)
    ,gnnsLeftPosReady(false)
    ,gnnsRightPosReady(false)

{
// //
}

void WrEkfNode::run()
{
    subscribe();
    initPubs();
    while (ros::ok)
    {
        estimate();
        pubEstState();
        ros::spinOnce();
        wrEkfNodeRate.sleep();
    }
}

void WrEkfNode::estimate()
{
    if (imuReady)
    {
        uint64_t dtMs =  std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::high_resolution_clock::now() - stateEstimationTimePoint).count();
        stateEstimationTimePoint = std::chrono::high_resolution_clock::now();
        
        float dt = dtMs / 1e3f;
        srekf.predictImu(imuAmes, imuWmes, dt);
        imuReady = false;
        
        estState = srekf.getEstState();
        pubEstState();
    }
    
    if (gnnsBasePosReady && gnnsBaseVelReady)
    {
        stateEstimationTimePoint = std::chrono::high_resolution_clock::now();
        Vector6 pv;
        pv << gnnsBasePosEnuMes, gnnsBaseVelEnuMes;
        srekf.correctPv(pv);
        
        gnnsBasePosReady = false;
        gnnsBaseVelReady = false;
    }
}

void WrEkfNode::pubEstState()
{
    std_msgs::Float32MultiArray msg;
    for (int i = 0; i < 16; ++i)
    {
        msg.data.push_back(estState[i]);
    }
    estStatePub.publish(msg);
}

void WrEkfNode::initPubs()
{
    estStatePub = nodeHandle.advertise<std_msgs::Float32MultiArray>(estStatePubTopicName, wrEkfQuenueDepth);
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
    Vector3 LatLonAlt(msg.latitude, msg.longitude, msg.altitude);
    if (!refWgsPointInited)
    {
        refWgsPoint = LatLonAlt;
        refWgsPointInited = true;
    }
    gnnsBasePosEnuMes = wgs2Enu(LatLonAlt, refWgsPoint);
    
    gnnsBasePosReady = true;
    //std::cout << "pos:\n" << gnnsBasePosEnuMes << std::endl;
}

void WrEkfNode::gnnsBaseVelCb(const geometry_msgs::Vector3Stamped& msg)
{
    if (refWgsPointInited)
    {
        Vector3 vEnu(-msg.vector.y, msg.vector.x, msg.vector.z); // ???
        //gnnsBaseVelEnuMes = ecef2Enu(vEcef, refWgsPoint);
        gnnsBaseVelEnuMes = vEnu;
        
        gnnsBaseVelReady = true;
        //std::cout << "vel:\n" << gnnsBaseVelEnuMes << std::endl;
    }
}

void WrEkfNode::gnnsLeftPosCb(const sensor_msgs::NavSatFix& msg)
{
    if (refWgsPointInited)
    {
        Vector3 LatLonAlt(msg.latitude, msg.longitude, msg.altitude);
        gnnsLeftPosEnuMes = wgs2Enu(LatLonAlt, refWgsPoint);
        gnnsLeftPosReady = true;
    }
}

void WrEkfNode::gnnsRightPosCb(const sensor_msgs::NavSatFix& msg)
{
    if (refWgsPointInited)
    {
        Vector3 LatLonAlt(msg.latitude, msg.longitude, msg.altitude);
        gnnsRightPosEnuMes = wgs2Enu(LatLonAlt, refWgsPoint);
        gnnsRightPosReady = true;
    }
}

