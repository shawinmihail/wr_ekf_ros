#include "WrEkfNode.h"
#include <cmath>

WrEkfNode::WrEkfNode():
     imuTopicName("wr_sensors/imu")
    ,gnnsTripletTopicName("/wr_sensors/nl_triplet")
    ,estStatePubTopicName("/wr_ekf/est_state")
    ,ctrlStatePubTopicName("/wr_ekf/ctrl_state")
    ,wrEkfNodeRate(100)
    ,wrEkfQuenueDepth(1)
    ,predictionTimePoint(std::chrono::high_resolution_clock::now())
    ,gnnsCorrectionTimePoint(std::chrono::high_resolution_clock::now())
    
    ,imuReady(false)
    ,gnnsTripletReady(false)
    ,statusBase(0)
    ,statusSlave1(0)
    ,statusSlave2(0)
    
    ,geo(mswhgeo_constants::WGS84)
    ,refGeoInited(false)
    
    ,ekfInited(false)
{
    /* init ref */
    if (!refGeoInited)
    {
        refLatLonAlt = Vector3d(55.751244 * 3.1415 / 180.0, 37.618423 * 3.1415 / 180.0, 200);
        double x = 0, y = 0, z = 0;
        geo.Wgs2Ecef(refLatLonAlt[0], refLatLonAlt[1], refLatLonAlt[2], x, y, z);
        refEcefXYZ = Vector3d(x,y,z);
        refGeoInited = true;    
    }
}

void WrEkfNode::run()
{
    //std::cout.precision(4);
    subscribe();
    initPubs();
    while(ros::ok)
    {
        estimate();
        //pubEstState();
        //pubCtrlState();
        ros::spinOnce();
        wrEkfNodeRate.sleep();
    }
}

void WrEkfNode::estimate()
{
    /* time */
    uint64_t dtMs =  std::chrono::duration_cast<std::chrono::milliseconds>
    (std::chrono::high_resolution_clock::now() - predictionTimePoint).count();
    predictionTimePoint = std::chrono::high_resolution_clock::now();
    double dt = dtMs / 1e3f;
    
    ekf.predict(dt);
    if (imuReady)
    {
        ekf.setImu(imuAmes, imuWmes);
        imuReady = false;
    }

    /* gnns corrections */
    if (gnnsTripletReady)
    {
        gnnsTripletReady = false;
        
        if(!ekfInited)
        {
            if (!statusBase == 4 || !StatusSlave1 == 4 || !StatusSlave1 == 4)
            {
                return;
            }
            ekf.reset(gnnsBasePosEnuMes);
            vector4 qImuCalib(1.0f/sqrtf(2.0f), 0.f, 0.f, 1.0f/sqrtf(2.0f));
            ekf.setQImuCalib(qImuCalib);
            ekf.calibSlavesWithSample(gnnsSlave1PosEnuMes, gnnsSlave2PosEnuMes);
            ekfInited = true;
        }
        
        // rv
        if (statusBase == 4)
        {
            ekf.correctRV(gnnsBasePosEnuMes, gnnsBaseVelEnuMes);
        }
        
        //if (statusBase == 4)
        //{
        // u
        //ekf.correctU(gnnsBaseVelEnuMes);
        //}
        
        // q
        if (statusSlave1 == 4 && statusSlave2 == 4)
        {
            ekf.correctQ2(gnnsSlave1PosEnuMes, gnnsSlave2PosEnuMes);
        }
        
        pubTestM();
    }
    
    estState = ekf.getEstTargetState();
    pubTestE();
    
    /*
    std::cout << "r_e:\n" << estState.segment(0,3) << std::endl;
    std::cout << "r_m:\n" << gnnsBasePosEnuMes.segment(0,3) << std::endl;
    std::cout << std::endl;
    */
}

void WrEkfNode::pubEstState()
{    
    nav_msgs::Odometry msg;
    msg.pose.pose.position.x = estState[0];
    msg.pose.pose.position.y = estState[1];
    msg.pose.pose.position.z = estState[2];
    
    msg.twist.twist.linear.x = estState[3];
    msg.twist.twist.linear.y = estState[4];
    msg.twist.twist.linear.z = estState[5];
    
    msg.pose.pose.orientation.w = estState[6];
    msg.pose.pose.orientation.x = estState[7];
    msg.pose.pose.orientation.y = estState[8];
    msg.pose.pose.orientation.z = estState[9];
    
    estStatePub.publish(msg);
}

void WrEkfNode::pubCtrlState()
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(estState[0]);
    msg.data.push_back(estState[1]);
    msg.data.push_back(estState[3]);
    msg.data.push_back(estState[4]);
    Vector4 q = estState.segment(6, 4);
    Vector3 eul = quatToEul(q);
    msg.data.push_back(eul[2]);
    //std::cout << eul << std::endl << std::endl;
    ctrlStatePub.publish(msg);
}

void WrEkfNode::pubTestM()
{
        
        wr_msgs::ninelives_triplet_stamped msg;
        
        msg.stamp = ros::Time::now();
        msg.r_ecef_master.x = gnnsBasePosEnuMes[0];
        msg.r_ecef_master.y = gnnsBasePosEnuMes[1];
        msg.r_ecef_master.z = gnnsBasePosEnuMes[2];
        
        msg.v_ecef_master.x = gnnsBaseVelEnuMes[0];
        msg.v_ecef_master.y = gnnsBaseVelEnuMes[1];
        msg.v_ecef_master.z = gnnsBaseVelEnuMes[2];
        
        msg.dr_ecef_slave1.x = gnnsSlave1PosEnuMes[0];
        msg.dr_ecef_slave1.y = gnnsSlave1PosEnuMes[1];
        msg.dr_ecef_slave1.z = gnnsSlave1PosEnuMes[2];
        
        msg.dr_ecef_slave2.x = gnnsSlave2PosEnuMes[0];
        msg.dr_ecef_slave2.y = gnnsSlave2PosEnuMes[1];
        msg.dr_ecef_slave2.z = gnnsSlave2PosEnuMes[2];
    
        testMesTripletSub.publish(msg);
}

void WrEkfNode::pubTestE()
{
        wr_msgs::ninelives_triplet_stamped msg;
        
        Vector3 r = estState.segment(0,3);
        Vector3 v = estState.segment(3,3);
        Vector4 q = estState.segment(6,4);
        Vector3 drSlave1;
        Vector3 drSlave2;
        Vector3 drImuGnns;
        drSlave1 << 0.73, 0.23, 0.0;
        drSlave2 << 0.73, -0.23, 0.0;
        drImuGnns << -0.4, 0.0, 0.4;
        
        r = r + quatRotate(q, drImuGnns);
        drSlave1 = quatRotate(q, drSlave1);
        drSlave2 = quatRotate(q, drSlave2);
        
        msg.stamp = ros::Time::now();
        msg.r_ecef_master.x = r[0];
        msg.r_ecef_master.y = r[1];
        msg.r_ecef_master.z = r[2];
        
        msg.v_ecef_master.x = v[0];
        msg.v_ecef_master.y = v[1];
        msg.v_ecef_master.z = v[2];
        
        msg.dr_ecef_slave1.x = drSlave1[0];
        msg.dr_ecef_slave1.y = drSlave1[1];
        msg.dr_ecef_slave1.z = drSlave1[2];
        
        msg.dr_ecef_slave2.x = drSlave2[0];
        msg.dr_ecef_slave2.y = drSlave2[1];
        msg.dr_ecef_slave2.z = drSlave2[2];
    
        testEstTripletSub.publish(msg);
}

void WrEkfNode::initPubs()
{
    estStatePub = nodeHandle.advertise<nav_msgs::Odometry>(estStatePubTopicName, wrEkfQuenueDepth);
    ctrlStatePub = nodeHandle.advertise<std_msgs::Float32MultiArray>(ctrlStatePubTopicName, wrEkfQuenueDepth);
    
    ctrlStatePub = nodeHandle.advertise<std_msgs::Float32MultiArray>(ctrlStatePubTopicName, wrEkfQuenueDepth);
    
    testMesTripletSub = nodeHandle.advertise<wr_msgs::ninelives_triplet_stamped>("wr_ekf_ros/test/mes_triplet", wrEkfQuenueDepth);
    testEstTripletSub = nodeHandle.advertise<wr_msgs::ninelives_triplet_stamped>("wr_ekf_ros/test/est_triplet", wrEkfQuenueDepth);
}

void WrEkfNode::subscribe()
{
    imuSub = nodeHandle.subscribe(imuTopicName, wrEkfQuenueDepth, &WrEkfNode::imuCb, this);
    gnnsTripletSub = nodeHandle.subscribe(gnnsTripletTopicName, wrEkfQuenueDepth, &WrEkfNode::gnnsTripletCb, this);
}


void WrEkfNode::imuCb(const wr_msgs::imu_stamped& msg)
{
    imuAmes = Vector3(msg.acc.x, msg.acc.y, msg.acc.z);
    imuWmes = Vector3(msg.ang_vel.x, msg.ang_vel.y, msg.ang_vel.z);
    imuReady = true;
}

void WrEkfNode::gnnsTripletCb(const wr_msgs::ninelives_triplet_stamped& msg)
{
    //std::cout << "triplet_cb" << std::endl;
    
    /* read */
    Vector3d poseBaseEcef(msg.r_ecef_master.x, msg.r_ecef_master.y, msg.r_ecef_master.z);
    Vector3 velBaseEcef(msg.v_ecef_master.x, msg.v_ecef_master.y, msg.v_ecef_master.z);
    Vector3 slave1Ecef(msg.dr_ecef_slave1.x, msg.dr_ecef_slave1.y, msg.dr_ecef_slave1.z);
    Vector3 slave2Ecef(msg.dr_ecef_slave2.x, msg.dr_ecef_slave2.y, msg.dr_ecef_slave2.z);
    statusBase = msg.status_master;
    statusSlave1 = msg.status_slave1;
    statusSlave2 = msg.status_slave2;
    
    Vector3d drBase = poseBaseEcef - refEcefXYZ;
    double e = 0, n = 0, u = 0;
    geo.Ecef2Enu(refLatLonAlt[0], refLatLonAlt[1], refLatLonAlt[2], drBase[0], drBase[1], drBase[2],  e, n, u);
    
    //std::cout << "dp: " << gnnsBasePosEnuMes - Vector3(e,n,u) << std::endl;
    
    gnnsBasePosEnuMes << e, n, u;
    geo.Ecef2Enu(refLatLonAlt[0], refLatLonAlt[1], refLatLonAlt[2], velBaseEcef[0], velBaseEcef[1], velBaseEcef[2],  e, n, u);
    gnnsBaseVelEnuMes << e, n, u;
    geo.Ecef2Enu(refLatLonAlt[0], refLatLonAlt[1], refLatLonAlt[2], slave1Ecef[0], slave1Ecef[1], slave1Ecef[2],  e, n, u);
    gnnsSlave1PosEnuMes << e, n, u;
    geo.Ecef2Enu(refLatLonAlt[0], refLatLonAlt[1], refLatLonAlt[2], slave2Ecef[0], slave2Ecef[1], slave2Ecef[2],  e, n, u);
    gnnsSlave2PosEnuMes << e, n, u;
    
    gnnsTripletReady = true;
}
