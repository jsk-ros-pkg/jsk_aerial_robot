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

  old1. X/Y axis の問題: フィルタ遅延によるもの？ --実は係数をあげればよい(Z axisとX/Y axixの周波数応答が違う)
  old2. the input of slam data should be after the input imu data?

  
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
      std::string x("X");
      std::string y("Y");
      std::string z("Z");
      std::string x_opt("XOpt");
      std::string y_opt("YOpt");
      std::string z_opt("ZOpt");

      std::string debug1("Debug1");
      std::string debug2("Debug2");

      bool use_dynamic_reconfigure = true;
      kf_x_ = new KalmanFilterPosVelAcc(nh, nh_private, x);
      kf_y_ = new KalmanFilterPosVelAcc(nh, nh_private, y);
      kf_z_ = new KalmanFilterPosVelAcc(nh, nh_private, z, use_dynamic_reconfigure);
      kf_bias_x_ = new KalmanFilterPosVelAccBias(nh, nh_private, x, use_dynamic_reconfigure); 
      kf_bias_y_ = new KalmanFilterPosVelAccBias(nh, nh_private, y, use_dynamic_reconfigure);
      kf_bias_z_ = new KalmanFilterPosVelAccBias(nh, nh_private, z);

      // for optical flow
      kf_opt_x_ = new KalmanFilterPosVelAcc(nh, nh_private,x_opt);
      kf_opt_y_ = new KalmanFilterPosVelAcc(nh, nh_private,y_opt);
      kf_opt_z_ = new KalmanFilterPosVelAcc(nh, nh_private,z_opt);
      kf_opt_bias_x_ = new KalmanFilterPosVelAccBias(nh, nh_private, x_opt, use_dynamic_reconfigure); 
      kf_opt_bias_y_ = new KalmanFilterPosVelAccBias(nh, nh_private, y_opt, use_dynamic_reconfigure);
      kf_opt_bias_z_ = new KalmanFilterPosVelAccBias(nh, nh_private, z_opt, use_dynamic_reconfigure);

      if (kalman_filter_debug_)
        {
          kf1_ = new KalmanFilterPosVelAccBias(nh, nh_private, debug1, use_dynamic_reconfigure);
          kf2_ = new KalmanFilterPosVelAccBias(nh, nh_private, debug2, use_dynamic_reconfigure);
        }
      else
        {
          kf1_ = NULL; kf2_ = NULL;
        }
    }
  else
    {
      kf_x_ = NULL; kf_y_ = NULL; kf_z_ = NULL; kf_bias_x_ = NULL; kf_bias_y_ = NULL; kf_bias_z_ = NULL;
      kf_opt_x_ = NULL; kf_opt_y_ = NULL; kf_opt_z_ = NULL;
      kf_opt_bias_x_ = NULL; kf_opt_bias_y_ = NULL; kf_opt_bias_z_ = NULL;
      kf1_ = NULL; kf2_ = NULL;
    }

  br_          = new tf::TransformBroadcaster();

  imu_data_     = new ImuData(nh, 
                              nh_private, 
                              this,
                              kalman_filter_flag_,
                              kf_x_, kf_y_, kf_z_,
                              kf_bias_x_, kf_bias_y_, kf_bias_z_,
                              kf_opt_x_, kf_opt_y_, kf_opt_z_,
                              kf_opt_bias_x_, kf_opt_bias_y_, kf_opt_bias_z_,
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
                                                 kf_opt_x_, kf_opt_y_, kf_opt_z_,
                                                 kf_opt_bias_x_, kf_opt_bias_y_, kf_opt_bias_z_);
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
  
  simulation_flag_ = simulation_flag;

}

RigidEstimator::~RigidEstimator()
{
  if(kalman_filter_flag_)
    {
      delete kf_x_; delete kf_y_; delete kf_z_;
      delete kf_bias_x_; delete kf_bias_y_; delete kf_bias_z_;
      delete kf_opt_x_; delete kf_opt_y_; delete kf_opt_z_;
      delete kf_opt_bias_x_; delete kf_opt_bias_y_; delete kf_opt_bias_z_;

    }
  if(kalman_filter_debug_)
    {
      delete kf1_;
      delete kf2_;
    }

  delete br_;
  delete imu_data_;
  delete optical_flow_data_;
  delete slam_data_;
  delete mirror_module_;

  delete lpf_acc_x_;
  delete lpf_acc_y_;
  delete lpf_acc_z_;

  printf("   deleted br_, imu_data_, slam_data_, mirror_module_, kalmanFilters from rigid estimator");
}


float RigidEstimator::getStatePosX()
{
  if(use_outer_pose_estimate_ & X_AXIS)
    return  outer_estimate_pos_x_;
  else
    {
      if(hokuyo_flag_) return  (kf_bias_x_->getEstimateState())[0];
      else if(px4flow_flag_) return (kf_opt_bias_x_->getEstimateState())[0];
      else return 0;
    }
}

float RigidEstimator::getStatePosXc()
{
  if(use_outer_pose_estimate_ & X_AXIS)
    {
      float state_pos_xc
        = outer_estimate_pos_x_ * cos(outer_estimate_psi_board_) + outer_estimate_pos_y_ * sin(outer_estimate_psi_board_);
      return  state_pos_xc;
    }
  else
    return  (kf_opt_bias_x_->getEstimateState())[0];
}

float RigidEstimator::getStateVelX()
{
  if(use_outer_vel_estimate_ & X_AXIS)
      return  outer_estimate_vel_x_;
  else
    {
      if(hokuyo_flag_) return  (kf_bias_x_->getEstimateState())[1];
      else if(px4flow_flag_) return  (kf_opt_bias_x_->getEstimateState())[1];
      else return 0;
    }

}

float RigidEstimator::getStateVelXc()
{
  if(use_outer_vel_estimate_ & X_AXIS)
    {
      float state_vel_xc
        = outer_estimate_vel_x_ * cos(outer_estimate_psi_board_) + outer_estimate_vel_y_ * sin(outer_estimate_psi_board_);
      return  state_vel_xc;
    }
  else
    return  (kf_opt_bias_x_->getEstimateState())[1];
}

float RigidEstimator::getStateAccXb()
{
  return imu_data_->getAccXbValue();
}

float RigidEstimator::getStatePosY()
{
 if(use_outer_pose_estimate_ & Y_AXIS)
      return  outer_estimate_pos_y_;
 else
   {
     if(hokuyo_flag_) return  (kf_bias_y_->getEstimateState())[0];
     else if (px4flow_flag_) return (kf_opt_bias_y_->getEstimateState())[0];
     else return 0;
   }
}

float RigidEstimator::getStatePosYc()
{
 if(use_outer_pose_estimate_ & Y_AXIS)
   {
      float state_pos_yc
        = -outer_estimate_pos_x_ * sin(outer_estimate_psi_board_) + outer_estimate_pos_y_ * cos(outer_estimate_psi_board_);
      return  state_pos_yc;
   }
  else
    return  (kf_opt_bias_y_->getEstimateState())[0];
}

float RigidEstimator::getStateVelY()
{
 if(use_outer_vel_estimate_ & Y_AXIS)
     return  outer_estimate_vel_y_;
 else 
   {
     if(hokuyo_flag_)  return  (kf_bias_y_->getEstimateState())[1];
     else if(px4flow_flag_) return  (kf_opt_bias_y_->getEstimateState())[1];
     else return 0;
   }
}

float RigidEstimator::getStateVelYc()
{
 if(use_outer_vel_estimate_ & Y_AXIS)
   {
      float state_vel_yc
        = -outer_estimate_vel_x_ * sin(outer_estimate_psi_board_) + outer_estimate_vel_y_ * cos(outer_estimate_psi_board_);
      return  state_vel_yc;
   }
  else
    return  (kf_opt_bias_y_->getEstimateState())[1];
}

inline float RigidEstimator::getStateAccYb(){  return imu_data_->getAccYbValue(); }

float RigidEstimator::getStatePosZ()
{
  if(use_outer_pose_estimate_ & Z_AXIS)
      return  outer_estimate_pos_z_;
  else
    {
      if(hokuyo_flag_) return (kf_z_->getEstimateState())[0];
      else if(px4flow_flag_) return (kf_opt_z_->getEstimateState())[0];
      else return 0;
    }
}
float RigidEstimator::getStateVelZ()
{
  if(use_outer_vel_estimate_ & Z_AXIS)
      return  outer_estimate_vel_z_;
  else
    {
      if(hokuyo_flag_) return (kf_z_->getEstimateState())[1];
      else if(px4flow_flag_) return (kf_opt_z_->getEstimateState())[1];
      else return 0;
    }
}

inline float RigidEstimator::getStateAccZb(){  return imu_data_->getAccZbValue(); }

float RigidEstimator::getStateTheta()
{
  if(use_outer_pose_estimate_ & PITCH_AXIS)
    return  outer_estimate_theta_;
  else
    return imu_data_->getPitchValue();
}
float RigidEstimator::getStatePhy()
{
  if(use_outer_pose_estimate_ & ROLL_AXIS)
    return  outer_estimate_phy_;
  else
    return imu_data_->getRollValue();
}


//bad
float RigidEstimator::getStatePsiCog()
{
  if(use_outer_pose_estimate_ & YAW_AXIS)
    {      
      return  outer_estimate_psi_cog_;
    }
  else
    {
      if(use_outer_yaw_est_)
        return slam_data_->getPsiValue();
      else
        return 0;  //+*+*+ fixed point
    }
}
float RigidEstimator::getStateVelPsiCog()
{
  if(use_outer_vel_estimate_ & YAW_AXIS)
    {      
      return  outer_estimate_vel_psi_cog_;
    }
  else
    {
      if(use_outer_yaw_est_)
        return   slam_data_->getVelPsiValue();
      else 
        return 0;   //+*+*+ fixed point
    }
}

float RigidEstimator::getStatePsiBoard()
{
  if(use_outer_pose_estimate_ & YAW_AXIS)
    {      
      return  outer_estimate_psi_board_;
    }
  else
    {
      if(use_outer_yaw_est_)
        return slam_data_->getPsiValue();
      else
        return 0;  //+*+*+ fixed point
    }
}

float RigidEstimator::getStateVelPsiBoard()
{
  if(use_outer_vel_estimate_ & YAW_AXIS)
    {      
      return  outer_estimate_vel_psi_board_;
    }
  else
    {
      if(use_outer_yaw_est_)
        return   slam_data_->getVelPsiValue();
      else 
        return 0;   //+*+*+ fixed point
    }
}


void RigidEstimator::setStateCorrectFlag(bool flag)
{ 
  if(px4flow_flag_) optical_flow_data_->setKFCorrectFlag(flag);
}

inline float RigidEstimator::getStateVelXOpt(){  return optical_flow_data_->getRawVelX();}
inline float RigidEstimator::getStateVelYOpt(){  return optical_flow_data_->getRawVelY();}

void RigidEstimator::tfPublish()
{
  //set the states broadcast
  statesBroadcast();

  //TODO mutex

  ros::Time sys_stamp = getSystemTimeStamp();

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

}

float RigidEstimator::getLaserToImuDistance()
{
  return laser_to_baselink_distance_;
}

void RigidEstimator::rosParamInit(ros::NodeHandle nh)
{
  std::string ns = nhp_.getNamespace();

  if (!nhp_.getParam ("altitude_control_mode", altitude_control_mode_))
    altitude_control_mode_ = 0;
  printf("%s: altitude_control_mode_ is %d\n", ns.c_str(), altitude_control_mode_);

  if (!nhp_.getParam ("useOuter_yaw_est", use_outer_yaw_est_))
    use_outer_yaw_est_ = false;
  printf("%s: use_outer_yaw_est is %s\n", ns.c_str(), use_outer_yaw_est_ ? ("true") : ("false"));

  if (!nhp_.getParam ("baselink_frame", baselink_frame_))
    baselink_frame_ = "unknown";
  printf("%s: baselink_frame_ is %s\n", ns.c_str(), baselink_frame_.c_str());

  if (!nhp_.getParam ("base_footprint_frame", base_footprint_frame_))
    base_footprint_frame_ = "unknown";
  printf("%s: base_footprint_frame_ is %s\n", ns.c_str(), base_footprint_frame_.c_str());

  if (!nhp_.getParam ("laser_frame", laser_frame_))
    laser_frame_ = "unknown";
  printf("%s: laser_frame_ is %s\n", ns.c_str(), laser_frame_.c_str());

  if (!nhp_.getParam ("camera_frame", camera_frame_))
    camera_frame_ = "unknown";
  printf("%s: camera_frame_ is %s\n", ns.c_str(), camera_frame_.c_str());

  if (!nhp_.getParam ("laser_to_baselink_distance", laser_to_baselink_distance_))
    laser_to_baselink_distance_ = 0;
  printf("%s: laser_to_baselink_distance_ is %.3f\n", ns.c_str(), laser_to_baselink_distance_);

  if (!nhp_.getParam ("mirror_module_arm_length", mirror_module_arm_length_))
    mirror_module_arm_length_ = 0;
  printf("%s: mirror_module_arm_length_ is %.3f\n", ns.c_str(), mirror_module_arm_length_);

  //*** kalman filter
  if (!nhp_.getParam ("kalman_filter_flag", kalman_filter_flag_))
    kalman_filter_flag_ = false;
  printf("%s: kalman_filter_flag is %s\n", ns.c_str(), kalman_filter_flag_ ? ("true") : ("false"));

  //*** kalman filter debug
  if (!nhp_.getParam ("kalman_filter_debug", kalman_filter_debug_))
    kalman_filter_debug_ = false;
  printf("%s: kalman_filter_debug is %s\n", ns.c_str(), kalman_filter_debug_ ? ("true") : ("false"));

  if (!nhp_.getParam ("kalman_filter_axis", kalman_filter_axis_))
    kalman_filter_axis_ = 0;
  printf("%s: kalman_filter_axis_ is %d\n", ns.c_str(), kalman_filter_axis_);

  //*** sensors
  if (!nhp_.getParam ("hokuyo_flag", hokuyo_flag_))
    hokuyo_flag_ = false;
  printf("%s: hokuyo_flag is %s\n", ns.c_str(), hokuyo_flag_ ? ("true") : ("false"));
  if (!nhp_.getParam ("px4flow_flag", px4flow_flag_))
    px4flow_flag_ = false;
  printf("%s: px4flow_flag is %s\n", ns.c_str(), px4flow_flag_ ? ("true") : ("false"));
 
  if (!nhp_.getParam ("mocap_flag", mocap_flag_))
    mocap_flag_ = false;
  printf("%s: mocap_flag is %s\n", ns.c_str(), mocap_flag_ ? ("true") : ("false"));
}


void RigidEstimator::statesBroadcast()
{
  aerial_robot_base::States full_states;
  full_states.header.stamp = getSystemTimeStamp();
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

