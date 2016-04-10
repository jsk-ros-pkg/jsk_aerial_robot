/*
  2015 05 16 
  1. the prioripty of the sensor => 1) pluginazation, 2) order=> profile
  2. uniform the state(set, get , etc, no "sate_xc", no "outer state")
  3. independent catkin packgage for sensor modules(px_comm is bad )

  +*+*+*+* 最重要事項：関数のstatic変数はすべて同じポインタに保存されるので、同じ関数を複数呼ぶ場合、中身のstatic変数はお互い
>  
  1. kalman filters of optical flow(vel_x, vel_y, pos_z) do not use accurate timestamp => for throwing?
   on the other hand, the kalman filters for laser-imu do use accurate timestamp => use queue
  ==> the timestamp do not have any important effect on the accuracy of kalman filter, in the case of 100Hz   
*/

#include "aerial_robot_base/state_estimation.h"

RigidEstimator::RigidEstimator(ros::NodeHandle nh,
                               ros::NodeHandle nh_private,
                               bool simulation_flag) : BasicEstimator(nh, nh_private)
{
  rosParamInit(nhp_);

  lpf_acc_x_ = new FirFilter(128);
  lpf_acc_y_ = new FirFilter(128);
  lpf_acc_z_ = new FirFilter(32);



  //kalman filter
  if (kalman_filter_flag_)
    {
      std::string x("x");
      std::string y("y");
      std::string z("z");
      std::string x_vel("x_vel");
      std::string y_vel("y_vel");
      std::string z2("z2");

      std::string debug1("debug1");
      std::string debug2("debug2");

      kf_x_ = new KalmanFilterPosVelAcc(nh, nh_private, x, CORRECT_POS);
      kf_y_ = new KalmanFilterPosVelAcc(nh, nh_private, y, CORRECT_POS);
      kf_z_ = new KalmanFilterPosVelAcc(nh, nh_private, z, CORRECT_POS);
      kf_bias_x_ = new KalmanFilterPosVelAccBias(nh, nh_private, x, CORRECT_POS); 
      kf_bias_y_ = new KalmanFilterPosVelAccBias(nh, nh_private, y, CORRECT_POS);
      kf_bias_z_ = new KalmanFilterPosVelAccBias(nh, nh_private, z, CORRECT_POS);

      // for velocity
      kf_vel_x_ = new KalmanFilterPosVelAcc(nh, nh_private,x_vel, CORRECT_VEL);
      kf_vel_y_ = new KalmanFilterPosVelAcc(nh, nh_private,y_vel, CORRECT_VEL);
      kf_pos_z2_ = new KalmanFilterPosVelAcc(nh, nh_private,z2, CORRECT_POS);
      kf_vel_bias_x_ = new KalmanFilterPosVelAccBias(nh, nh_private, x_vel, CORRECT_VEL); 
      kf_vel_bias_y_ = new KalmanFilterPosVelAccBias(nh, nh_private, y_vel, CORRECT_VEL);

      if (kalman_filter_debug_)
        {
          kf1_ = new KalmanFilterPosVelAccBias(nh, nh_private, debug1);
          kf2_ = new KalmanFilterPosVelAccBias(nh, nh_private, debug2);
        }
      else
        {
          kf1_ = NULL; kf2_ = NULL;
        }
    }
  else
    {
      kf_x_ = NULL; kf_y_ = NULL; kf_z_ = NULL; kf_bias_x_ = NULL; kf_bias_y_ = NULL; kf_bias_z_ = NULL;
      kf_vel_x_ = NULL; kf_vel_y_ = NULL; kf_pos_z2_ = NULL;  kf_vel_bias_x_ = NULL; kf_vel_bias_y_ = NULL;
      kf1_ = NULL; kf2_ = NULL;
    }


  br_          = new tf::TransformBroadcaster();


  imu_data_     = new ImuData(nh, 
                              nh_private, 
                              this,
                              kalman_filter_flag_,
                              kf_x_, kf_y_, kf_z_,
                              kf_bias_x_, kf_bias_y_, kf_bias_z_,
                              kf_vel_x_, kf_vel_y_, kf_pos_z2_,
                              kf_vel_bias_x_, kf_vel_bias_y_,
                              kalman_filter_debug_,
                              kalman_filter_axis_,
                              kf1_, kf2_,
                              lpf_acc_x_,
                              lpf_acc_y_,
                              lpf_acc_z_,
                              simulation_flag_);

  if(px4flow_flag_)
    {
      optical_flow_data_   = new OpticalFlowData(nh,
                                                 nh_private,
                                                 this,
                                                 kalman_filter_flag_,
                                                 kf_vel_x_, kf_vel_y_, kf_pos_z2_,
                                                 kf_vel_bias_x_, kf_vel_bias_y_);
    }

  if(hokuyo_flag_)
    {
      mirror_module_ = new MirrorModule(nh, nh_private,
                                        this,
                                        kalman_filter_flag_, kalman_filter_debug_,
                                        kf_z_, kf_bias_z_);

      slam_data_     = new SlamData(nh,
                                    nh_private,
                                    this,
                                    kalman_filter_flag_,
                                    kf_x_, kf_y_,
                                    kf_bias_x_, kf_bias_y_,
                                    kalman_filter_debug_,
                                    kalman_filter_axis_,
                                    kf1_, kf2_);

    }

  if(mocap_flag_)
      mocap_data_ = new MocapData(nh, nh_private, this);
  
}

RigidEstimator::~RigidEstimator()
{
  //delete br_;

}




void RigidEstimator::setStateCorrectFlag(bool flag)
{ 
  if(px4flow_flag_) optical_flow_data_->setKFCorrectFlag(flag);
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

void RigidEstimator::rosParamInit()
{
  std::string ns = nhp_.getNamespace();

  sensor_fusion_loader_ptr_ = new pluginlib::ClassLoader<kf_base_plugin::KalmanFilter>("kalman_filter", "kf_base_plugin::KalmanFilter");

  if (!nhp_.getParam ("fuser_egomotion_no", fuser_egomotion_no_))
    fuser_egomotion_no_ = 0;
  printf("fuser_egomotion_no_ is %d\n", fuser_egomotion_no_);
  fuser_egomotion_.resize(fuser_egomotion_no_);
  fuser_egomotion_id_.resize(fuser_egomotion_no_);
  fuser_egomotion_plugin_name_.resize(fuser_egomotion_no_);

  if (!nhp_.getParam ("fuser_experiment_no", fuser_experiment_no_))
    fuser_experiment_no_ = 0;
  printf("fuser_experiment_no_ is %d\n", fuser_experiment_no_);
  fuser_experiment_.resize(fuser_experiment_no_);
  fuser_experiment_id_.resize(fuser_experiment_no_);
  fuser_experiment_plugin_name_.resize(fuser_experiment_no_);

  for(int i = 0; i < fuser_egomotion_no_; i++)
    {
      std::stringstream fuser_no;
      fuser_no << i + 1;

      if (!nhp_.getParam ("fuser_egomotion_id", fuser_egomotion_id_[i]))
        ROS_ERROR("%d, no param in fuser egomotion id");
      printf("fuser_egomotion_id_ is %d\n", fuser_egomotion_id_[i]);

      if (!nhp_.getParam ("fuser_egomotion_name", fuser_egomotion_name_[i]))
        ROS_ERROR("%d, no param in fuser egomotion name");
      printf("fuser_egomotion_name_ is %s\n", fuser_egomotion_name_[i]);

      if (!nhp_.getParam ("fuser_egomotion_plugin_name", fuser_egomotion_plugin_name_[i]))
        ROS_ERROR("%d, no param in fuser egomotion plugin_name");
      printf("fuser_egomotion_plugin_name_ is %s\n", fuser_egomotion_plugin_name_[i].c_str());

      fuser_egomotion_[i]  = sensor_fusion_loader_ptr_->createInstance(fuser_egomotion_plugin_name_[i]);
      fuser_egomotion_[i]->initialize(nh_, fuser_egomotion_name_[i], fuser_egomotion_id_[i]);
    }

  for(int i = 0; i < fuser_experiment_no_; i++)
    {
      std::stringstream fuser_no;
      fuser_no << i + 1;

      if (!nhp_.getParam ("fuser_experiment_id", fuser_experiment_id_[i]))
        ROS_ERROR("%d, no param in fuser experiment id");
      printf("fuser_experiment_id_ is %s\n", fuser_experiment_id_[i].c_str());

      if (!nhp_.getParam ("fuser_experiment_name", fuser_experiment_name_[i]))
        ROS_ERROR("%d, no param in fuser experiment name");
      printf("fuser_experiment_name_ is %s\n", fuser_experiment_name_[i].c_str());

      if (!nhp_.getParam ("fuser_experiment_plugin_name", fuser_experiment_plugin_name_))
        ROS_ERROR("%d, no param in fuser experiment plugin_name");
      printf("fuser_experiment_plugin_name_ is %s\n", fuser_experiment_plugin_name_.c_str());

      fuser_experiment_[i]  = sensor_fusion_loader_ptr_->createInstance(fuser_experiment_plugin_name_);
      fuser_experiment_[i]->initialize(nh_, fuser_experiment_name_[i], fuser_experiment_id_[i]);
    }


}


void RigidEstimator::statesBroadcast()
{
  aerial_robot_base::States full_states;
  //full_states.header.stamp = getSystemTimeStamp();
  full_states.header.stamp = ros::Time::now();
  aerial_robot_base::State x_state;
  x_state.id = "x";
  x_state.pos = getStatePosX();
  x_state.vel = getStateVelX();
  aerial_robot_base::State y_state;
  y_state.id = "y";
  y_state.pos = getStatePosY();
  y_state.vel = getStateVelY();
  aerial_robot_base::State z_state;
  z_state.id = "z";
  z_state.pos = getStatePosZ();
  z_state.vel = getStateVelZ();

  aerial_robot_base::State yaw_state;
  yaw_state.id = "yaw";
  yaw_state.pos = getStatePsiBoard();
  yaw_state.vel = getStateVelPsiBoard();
  aerial_robot_base::State pitch_state;
  pitch_state.id = "pitch";
  pitch_state.pos = getStateTheta();
  aerial_robot_base::State roll_state;
  roll_state.id = "roll";
  roll_state.pos = getStatePhy();
  full_states.states.push_back(x_state);
  full_states.states.push_back(y_state);
  full_states.states.push_back(z_state);
  full_states.states.push_back(yaw_state);
  full_states.states.push_back(pitch_state);
  full_states.states.push_back(roll_state);

  full_states_pub_.publish(full_states);
}

