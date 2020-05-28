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
    ,ctrlStatePubTopicName("/wr_ekf/ctrl_state")
    ,modelStateTopicName("/gazebo/model_states")
    ,linkStateTopicName("/gazebo/link_states")
    
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
    ,modelStateReady(false)
    ,linkStateReady(false)
    
    ,normal_distribution(0.0, 1.0)

{
// //
}

void WrEkfNode::run()
{
    subscribe();
    initPubs();
    while(ros::ok)
    {
        estimate();
        pubEstState();
        pubCtrlState();
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
        float dt = dtMs / 1e3f;
        
        srekf.predictImu(imuAmes, imuWmes, dt);
        imuReady = false;
    }
    
    /* husky gnns corrections */
    /*
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
        std::cout << "gnnsBasePosEnuMes:\n" << gnnsBasePosEnuMes << std::endl;
        std::cout << "gnnsLeftPosEnuMes:\n" << gnnsLeftPosEnuMes << std::endl;
        std::cout << "gnnsRightPosEnuMes:\n" << gnnsRightPosEnuMes << std::endl;
        srekf.correctP3(dr1, dr2);
        
        gnnsBasePosReady = false;
        gnnsBaseVelReady = false;
    }
    */
    
    /* links state corrections */
    float gnnsCorrectionPeriod = 0.05f; // s
    if (linkStateReady)
    {
        uint64_t dtMs =  std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::high_resolution_clock::now() - gnnsCorrectionTimePoint).count();
        float dt = dtMs / 1e3f;
        if (dt > gnnsCorrectionPeriod)
        {
            gnnsCorrectionTimePoint = std::chrono::high_resolution_clock::now();
            
            // pv
            Vector3 linkBasePosNoise(0.005 * normal_distribution(random_generator),
                                     0.005 * normal_distribution(random_generator),
                                     0.007 * normal_distribution(random_generator));
            
            Vector3 linkLeftPosNoise(0.005 * normal_distribution(random_generator),
                                     0.005 * normal_distribution(random_generator),
                                     0.007 * normal_distribution(random_generator));
            
            Vector3 linkRightPosNoise(0.005 * normal_distribution(random_generator),
                                      0.005 * normal_distribution(random_generator),
                                      0.007 * normal_distribution(random_generator));
            
            Vector3 linkBaseVelNoise(0.01 * normal_distribution(random_generator),
                                     0.01 * normal_distribution(random_generator),
                                     0.01 * normal_distribution(random_generator));
            
            Vector3 linkBasePosNoised = linkBasePos + linkBasePosNoise;
            Vector3 linkLeftPosNoised = linkBasePos + linkLeftPosNoise;
            Vector3 linkRightPosNoised = linkBasePos + linkRightPosNoise;
            Vector3 linkBaseVelNoised = linkBaseVel + linkBaseVelNoise;
            
            Vector6 pv;
            pv << linkBasePosNoised, linkBaseVelNoised;
            srekf.correctPv(pv);
            
            // v
            srekf.correctV(linkBaseVelNoised);
            
            // q
            Vector3 dr1 = linkBasePos - linkLeftPos;
            Vector3 dr2 = linkBasePos - linkRightPos;
            srekf.correctP3(dr1, dr2);
            
            estState = srekf.getEstState();
            
            /*
            
            Vector3 estPos = estState.segment(0, 3);
            Vector3 estVel = estState.segment(3, 3);
            Vector4 estQua = estState.segment(9, 4);
             
            Vector3 gpsAttachmentShift(-0.06f, 0.0f, 0.53f);
            Vector3 dP = estPos - (linkBasePos - gpsAttachmentShift);
            Vector3 dV = estVel - linkBaseVel;
            Vector4 mQ = modelState.segment(9, 4);
            Vector4 dQ = estQua - mQ;
            
            //std::cout << "dP:\n" << dP.norm() << std::endl;
            //std::cout << "dV:\n" << dV.norm() << std::endl;
            //std::cout << "mQ:\n" << mQ << std::endl;
            //std::cout << "eQ:\n" << estQua << std::endl;
            //std::cout << "dQ:\n" << dQ.norm() << std::endl;
            */

            
            linkStateReady = false;
        }
    }
}

void WrEkfNode::pubEstState()
{
    if (!modelStateReady)
    {
        return;
    }
    std_msgs::Float32MultiArray msg;
    for (int i = 0; i < 16; ++i)
    {
        msg.data.push_back(modelState[i]);
    }
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
    estStatePub = nodeHandle.advertise<std_msgs::Float32MultiArray>(estStatePubTopicName, wrEkfQuenueDepth);
    ctrlStatePub = nodeHandle.advertise<std_msgs::Float32MultiArray>(ctrlStatePubTopicName, wrEkfQuenueDepth);
}

void WrEkfNode::subscribe()
{
    imuSub = nodeHandle.subscribe(imuTopicName, wrEkfQuenueDepth, &WrEkfNode::imuCb, this);
    gnnsBasePosSub = nodeHandle.subscribe(gnnsBasePosTopicName, wrEkfQuenueDepth, &WrEkfNode::gnnsBasePosCb, this);
    gnnsBaseVelSub = nodeHandle.subscribe(gnnsBaseVelTopicName, wrEkfQuenueDepth, &WrEkfNode::gnnsBaseVelCb, this);
    gnnsLeftPosSub = nodeHandle.subscribe(gnnsLeftPosTopicName, wrEkfQuenueDepth, &WrEkfNode::gnnsLeftPosCb, this);
    gnnsRightPosSub = nodeHandle.subscribe(gnnsRightPosTopicName, wrEkfQuenueDepth, &WrEkfNode::gnnsRightPosCb, this);
    
    modelStateSub = nodeHandle.subscribe(modelStateTopicName, wrEkfQuenueDepth, &WrEkfNode::modelStateCb, this);
    linkStateSub = nodeHandle.subscribe(linkStateTopicName, wrEkfQuenueDepth, &WrEkfNode::linkStateCb, this);
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

void WrEkfNode::modelStateCb(const gazebo_msgs::ModelStates& msg)
{
    Vector3 modelPos(msg.pose.at(1).position.x, msg.pose.at(1).position.y, msg.pose.at(1).position.z);
    Vector3 modelVel(msg.twist.at(1).linear.x, msg.twist.at(1).linear.y, msg.twist.at(1).linear.z);
    Vector3 modelAcc(0, 0, 0);
    Vector4 modelQua(msg.pose.at(1).orientation.w, msg.pose.at(1).orientation.x, msg.pose.at(1).orientation.y, msg.pose.at(1).orientation.z);
    Vector3 modelRve(msg.twist.at(1).angular.x, msg.twist.at(1).angular.y, msg.twist.at(1).angular.z);
    modelState << modelPos, modelVel, modelAcc, modelQua, modelRve;
    
    modelStateReady = true;
    
    //std::cout << modelState << std::endl;
}

void WrEkfNode::linkStateCb(const gazebo_msgs::LinkStates& msg)
{
    /*base  5*/
    /*left  4*/
    /*right 6*/
    linkBasePos = Vector3(msg.pose.at(5).position.x, msg.pose.at(5).position.y, msg.pose.at(5).position.z);
    linkLeftPos = Vector3(msg.pose.at(4).position.x, msg.pose.at(4).position.y, msg.pose.at(4).position.z);
    linkRightPos = Vector3(msg.pose.at(6).position.x, msg.pose.at(6).position.y, msg.pose.at(6).position.z);
    
    linkBaseVel = Vector3(msg.twist.at(5).linear.x, msg.twist.at(5).linear.y, msg.twist.at(5).linear.z);
    linkStateReady = true;
    //std::cout << "b-l:\n" << linkBasePos-linkLeftPos << std::endl;
    //std::cout << "b-r:\n" << linkBasePos-linkRightPos << std::endl;
}

