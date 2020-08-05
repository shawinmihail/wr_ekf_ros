#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_GPS_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_GPS_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <hector_gazebo_plugins/sensor_model.h>
#include <hector_gazebo_plugins/update_timer.h>

#include <dynamic_reconfigure/server.h>
#include <hector_gazebo_plugins/GNSSConfig.h>

namespace gazebo
{

class WrEkfSesors : public ModelPlugin
{
public:
  WrEkfSesors();
  virtual ~WrEkfSesors();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();

private:
    
  physics::WorldPtr world;
  physics::LinkPtr link;

  ros::NodeHandle* node_handle_;
  ros::Publisher fix_publisher_;
  ros::Publisher velocity_publisher_;

  sensor_msgs::NavSatFix fix_;
  geometry_msgs::Vector3Stamped velocity_;

  std::string namespace_;
  std::string link_name_;
  std::string frame_id_;
  std::string fix_topic_;
  std::string velocity_topic_;

  UpdateTimer updateTimer;
  event::ConnectionPtr updateConnection;
};

} // namespace gazebo

#endif // HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_GPS_H
