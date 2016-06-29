/*
  2016 05 16
  1. the prioripty of the sensor => 1) pluginazation, 2) order=> profile
  2. uniform the state(set, get , etc, no "sate_xc", no "outer state")
  3. independent catkin packgage for sensor modules(px_comm is bad )
*/

#include "aerial_robot_base/state_estimation.h"

RigidEstimator::RigidEstimator(ros::NodeHandle nh, ros::NodeHandle nh_private) : BasicEstimator(nh, nh_private)
{
  rosParamInit();
  //br_ = new tf::TransformBroadcaster();
}

RigidEstimator::~RigidEstimator()
{
  //delete br_;

}

void RigidEstimator::tfPublish()
{
  //set the states broadcast
  statesBroadcast();
  //TODO mutex

#if 0
  //ros::Time sys_stamp = getSystemTimeStamp();
  ros::Time sys_stamp = ros::Time::now();

  tf::Transform laser_to_baselink;
  tf::Transform footprint_to_laser;
  tf::Transform laser_to_camera;
  tf::Quaternion tmp;

  //send the laser -> quadcopter_base
  laser_to_baselink.setOrigin(tf::Vector3(0.0, 0.0, laser_to_baselink_distance_));
  tmp.setRPY(0.0 , 0.0 , 0.0);
  laser_to_baselink.setRotation(tmp);
  br_->sendTransform(tf::StampedTransform(laser_to_baselink, sys_stamp, laser_frame_,
                                          baselink_frame_));

  //send the laser -> camera
  laser_to_camera.setOrigin(tf::Vector3(0.02, 0.0, -0.04));
  tmp.setRPY(0.0 , 0.0 , 0.0);
  laser_to_camera.setRotation(tmp);
  br_->sendTransform(tf::StampedTransform(laser_to_camera, sys_stamp, laser_frame_,
                                          camera_frame_));

  tmp.setRPY((getStatePhy()), getStateTheta(), 0); 
  footprint_to_laser.setRotation(tmp);
  footprint_to_laser.setOrigin(tf::Vector3(0.0, 0.0, getStatePosZ() + getPosZOffset() - mirror_module_arm_length_));

  br_->sendTransform(tf::StampedTransform(footprint_to_laser, sys_stamp,
                                          base_footprint_frame_, laser_frame_));
#endif
}


void RigidEstimator::statesBroadcast()
{
  aerial_robot_base::States full_states;
  //full_states.header.stamp = getSystemTimeStamp();
  full_states.header.stamp = ros::Time::now();
  aerial_robot_base::State x_state;
  x_state.id = "x";
  x_state.pos = getGTState(X_W, 0);
  x_state.vel = getGTState(X_W, 1);
  x_state.raw_pos = getEEState(X_W, 0);
  x_state.raw_vel = getEEState(X_W, 1);
  x_state.kf_pos = getEXState(X_W, 0);
  x_state.kf_vel = getEXState(X_W, 1);
  x_state.reserves.push_back(getEEState(X_B, 0));
  x_state.reserves.push_back(getEEState(X_B, 1));
  x_state.reserves.push_back(getEXState(X_B, 0));
  x_state.reserves.push_back(getEXState(X_B, 1));


  aerial_robot_base::State y_state;
  y_state.id = "y";
  y_state.pos = getGTState(Y_W, 0);
  y_state.vel = getGTState(Y_W, 1);
  y_state.raw_pos = getEEState(Y_W, 0);
  y_state.raw_vel = getEEState(Y_W, 1);
  y_state.kf_pos = getEXState(Y_W, 0);
  y_state.kf_vel = getEXState(Y_W, 1);
  y_state.reserves.push_back(getEEState(Y_B, 0));
  y_state.reserves.push_back(getEEState(Y_B, 1));
  y_state.reserves.push_back(getEXState(Y_B, 0));
  y_state.reserves.push_back(getEXState(Y_B, 1));

  aerial_robot_base::State z_state;
  z_state.id = "z";
  z_state.pos = getGTState(Z_W, 0);
  z_state.vel = getGTState(Z_W, 1);
  z_state.raw_pos = getEEState(Z_W, 0);
  z_state.raw_vel = getEEState(Z_W, 1);
  z_state.kf_pos = getEXState(Z_W, 0);
  z_state.kf_vel = getEXState(Z_W, 1);
  z_state.reserves.push_back(getEEState(Z_W, 2));

  aerial_robot_base::State yaw_state;
  yaw_state.id = "yaw";
  yaw_state.pos = getGTState(YAW_W_B, 0);
  yaw_state.vel = getGTState(YAW_W_B, 1);
  yaw_state.raw_pos = getEEState(YAW_W_B, 0);
  yaw_state.raw_vel = getEEState(YAW_W_B, 1);
  yaw_state.kf_pos = getEXState(YAW_W_B, 0);
  yaw_state.kf_vel = getEXState(YAW_W_B, 1);
  yaw_state.reserves.push_back(getGTState(YAW_W_COG, 0));
  yaw_state.reserves.push_back(getGTState(YAW_W_COG, 1));
  yaw_state.reserves.push_back(getEEState(YAW_W_COG, 0));
  yaw_state.reserves.push_back(getEEState(YAW_W_COG, 1));
  aerial_robot_base::State pitch_state;
  pitch_state.id = "pitch";
  pitch_state.pos = getGTState(PITCH_W, 0);
  pitch_state.raw_pos = getEEState(PITCH_W, 0);
  aerial_robot_base::State roll_state;
  roll_state.id = "roll";
  roll_state.pos = getGTState(ROLL_W, 0);
  roll_state.raw_pos = getEEState(ROLL_W, 0);

  full_states.states.push_back(x_state);
  full_states.states.push_back(y_state);
  full_states.states.push_back(z_state);
  full_states.states.push_back(roll_state);  
  full_states.states.push_back(pitch_state);
  full_states.states.push_back(yaw_state);


  full_states_pub_.publish(full_states);
}


void RigidEstimator::rosParamInit()
{

  std::string ns = nhp_.getNamespace();
  printf("%s\n", ns.c_str());

  sensor_fusion_loader_ptr_ = boost::shared_ptr< pluginlib::ClassLoader<kf_base_plugin::KalmanFilter> >(new pluginlib::ClassLoader<kf_base_plugin::KalmanFilter>("kalman_filter", "kf_base_plugin::KalmanFilter"));


  if (!nhp_.getParam ("fuser_egomotion_no", fuser_egomotion_no_))
    fuser_egomotion_no_ = 0;
  printf("fuser_egomotion_no_ is %d\n", fuser_egomotion_no_);
  fuser_egomotion_.resize(fuser_egomotion_no_);
  fuser_egomotion_id_.resize(fuser_egomotion_no_);
  fuser_egomotion_name_.resize(fuser_egomotion_no_);
  fuser_egomotion_plugin_name_.resize(fuser_egomotion_no_);

  if (!nhp_.getParam ("fuser_experiment_no", fuser_experiment_no_))
    fuser_experiment_no_ = 0;
  printf("fuser_experiment_no_ is %d\n", fuser_experiment_no_);
  fuser_experiment_.resize(fuser_experiment_no_);
  fuser_experiment_id_.resize(fuser_experiment_no_);
  fuser_experiment_name_.resize(fuser_experiment_no_);
  fuser_experiment_plugin_name_.resize(fuser_experiment_no_);

  for(int i = 0; i < fuser_egomotion_no_; i++)
    {
      std::stringstream fuser_no;
      fuser_no << i + 1;

      if (!nhp_.getParam ("fuser_egomotion_id" + fuser_no.str() , fuser_egomotion_id_[i]))
        ROS_ERROR("%d, no param in fuser egomotion id", i);
      printf("fuser_egomotion_id%d is %d\n", i + 1, fuser_egomotion_id_[i]);

      if (!nhp_.getParam ("fuser_egomotion_name" + fuser_no.str(), fuser_egomotion_name_[i]))
        ROS_ERROR("%d, no param in fuser egomotion name", i);
      printf("fuser_egomotion_name%d is %s\n", i + 1, fuser_egomotion_name_[i].c_str());

      if (!nhp_.getParam ("fuser_egomotion_plugin_name" + fuser_no.str(), fuser_egomotion_plugin_name_[i]))
        ROS_ERROR("%d, no param in fuser egomotion plugin_name", i);
      printf("fuser_egomotion_plugin_name%d is %s\n", i + 1, fuser_egomotion_plugin_name_[i].c_str());

      //fuser_egomotion_[i]  = sensor_fusion_loader_ptr_->createInstance(fuser_egomotion_plugin_name_[i]);
      fuser_egomotion_[i]  = sensor_fusion_loader_ptr_->createInstance(fuser_egomotion_plugin_name_[i]);
      fuser_egomotion_[i]->initialize(nh_, fuser_egomotion_name_[i], fuser_egomotion_id_[i]);
    }

  for(int i = 0; i < fuser_experiment_no_; i++)
    {
      std::stringstream fuser_no;
      fuser_no << i + 1;

      if (!nhp_.getParam ("fuser_experiment_id" + fuser_no.str(), fuser_experiment_id_[i]))
        ROS_ERROR("%d, no param in fuser experiment id", i);
      printf("fuser_experiment_id%d is %d\n", i+1, fuser_experiment_id_[i]);

      if (!nhp_.getParam ("fuser_experiment_name" + fuser_no.str(), fuser_experiment_name_[i]))
        ROS_ERROR("%d, no param in fuser experiment name", i);
      printf("fuser_experiment_name%d is %s\n", i+1, fuser_experiment_name_[i].c_str());

      if (!nhp_.getParam ("fuser_experiment_plugin_name" + fuser_no.str(), fuser_experiment_plugin_name_[i]))
        ROS_ERROR("%d, no param in fuser experiment plugin_name", i);
      printf("fuser_experiment_plugin_name%d is %s\n", i+1, fuser_experiment_plugin_name_[i].c_str());

      //fuser_experiment_[i]  = sensor_fusion_loader_ptr_->createInstance(fuser_experiment_plugin_name_);
      fuser_experiment_[i]  = sensor_fusion_loader_ptr_->createInstance(fuser_experiment_plugin_name_[i]);
      fuser_experiment_[i]->initialize(nh_, fuser_experiment_name_[i], fuser_experiment_id_[i]);
    }

  sensor_loader_ptr_ =  boost::shared_ptr< pluginlib::ClassLoader<sensor_base_plugin::SensorBase> >( new pluginlib::ClassLoader<sensor_base_plugin::SensorBase>("aerial_robot_base", "sensor_base_plugin::SensorBase"));


  if (!nh_.getParam ("sensor_no", sensor_no_)) sensor_no_ = 0;
  printf("sensor_no_ is %d\n", sensor_no_);
  sensor_plugin_name_.resize(sensor_no_);
  sensors_.resize(sensor_no_);

  for(int i = 0; i < sensor_no_; i++)
    {
      std::stringstream sensor_no;
      sensor_no << i + 1;

      if (!nh_.getParam ("sensor_plugin_name" + sensor_no.str(), sensor_plugin_name_[i]))
        ROS_ERROR("%d, no param in sensor plugin_name", i);
      printf("sensor_plugin_name%d is %s\n", i+1, sensor_plugin_name_[i].c_str());

      sensors_[i]  = sensor_loader_ptr_->createInstance(sensor_plugin_name_[i]);
      sensors_[i]->initialize(nh_, ros::NodeHandle("sensor"), this);
    }
}
