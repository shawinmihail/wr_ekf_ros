#include <hwr_ekf_sensors.h>
#include <gazebo/physics/physics.hh>


namespace gazebo {

WrEkfSesors::WrEkfSesors()
{
  // default parameters
  frame_id_ = "/world";
  fix_topic_ = "fix";
  velocity_topic_ = "fix_velocity";

  reference_latitude_  = DEFAULT_REFERENCE_LATITUDE;
  reference_longitude_ = DEFAULT_REFERENCE_LONGITUDE;
  reference_altitude_  = DEFAULT_REFERENCE_ALTITUDE;
}

WrEkfSesors::~WrEkfSesors()
{
  updateTimer.Disconnect(updateConnection);
  node_handle_->shutdown();
  delete node_handle_;
}

void GazeboRosGps::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();

  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    namespace_.clear();
  else
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
    link = _model->GetLink(link_name_);
  }

  if (!link)
  {
    ROS_FATAL("WrEkfSesors plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  fix_.status.status  = sensor_msgs::NavSatStatus::STATUS_FIX;
  fix_.status.service = 0;
  fix_.header.frame_id = frame_id_;
  velocity_.header.frame_id = frame_id_;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("GazeboRosGps error (!ros::isInitialized)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  fix_publisher_ = node_handle_->advertise<sensor_msgs::NavSatFix>(fix_topic_, 1);
  velocity_publisher_ = node_handle_->advertise<geometry_msgs::Vector3Stamped>(velocity_topic_, 1);

  // connect Update function
  updateTimer.setUpdateRate(4.0);
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRosGps::Update, this));
}

void GazeboRosGps::Update()
{

  common::Time sim_time = world->SimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  ignition::math::Pose3d pose = link->WorldPose();
  ignition::math::Pose3d pose = link->WorldLinearVel();
  
  ignition::math::Vector3d velocity = velocity_error_model_(link->WorldLinearVel(), dt);
  ignition::math::Vector3d position = position_error_model_(pose.Pos(), dt);


  fix_.latitude  = 0;
  fix_.longitude = 0;
  fix_.altitude  = 0;
  velocity_.vector.x =  0;
  velocity_.vector.y = 0;
  velocity_.vector.z = 0;

  fix_publisher_.publish(fix_);
  velocity_publisher_.publish(velocity_);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosGps)

} // namespace gazebo
