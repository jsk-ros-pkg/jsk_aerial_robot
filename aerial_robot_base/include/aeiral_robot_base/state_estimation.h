#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

//* ros
#include <ros/ros.h>

//* for state estimate
//TODO:  plugin 
#include <aerial_robot_base/sensor/mirror_module.h>
#include <aerial_robot_base/sensor/slam_data.h>
#include <aerial_robot_base/sensor/imu_module.h>
#include <aerial_robot_base/sensor/mocap.h>
#include <aerial_robot_base/sensor/optical_flow_module.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
//* filter
#include <aerial_robot_base/kalman_filter.h>
#include <aerial_robot_base/digital_filter.h>


class BasicEstimator
{
 public:
 BasicEstimator(ros::NodeHandle nh, ros::NodeHandle nh_private):
  : nh_(nh, "estimator"), nhp_(nh_private, "estimator")
    {
      outer_estimate_pos_x_ = 0;
      outer_estimate_vel_x_ = 0;
      outer_estimate_pos_y_ = 0;
      outer_estimate_vel_y_ = 0;
      outer_estimate_pos_z_ = 0;
      outer_estimate_vel_z_ = 0;
      outer_estimate_theta_ = 0;
      outer_estimate_vel_theta_ = 0;
      outer_estimate_phy_ = 0;
      outer_estimate_vel_phy_ = 0;
      outer_estimate_psi_cog_ = 0;
      outer_estimate_vel_psi_cog_ = 0;
      outer_estimate_psi_board_ = 0;
      outer_estimate_vel_psi_board_ = 0;

      state_pos_z_offset_ = 1.0; //1m

      use_outer_pose_estimate_ = 0;
      use_outer_vel_estimate_ = 0;
    }

  virtual ~BasicEstimator(){}

  static const uint8_t X_AXIS = 1;
  static const uint8_t Y_AXIS = 2;
  static const uint8_t Z_AXIS = 4;
  static const uint8_t PITCH_AXIS = 8;
  static const uint8_t ROLL_AXIS = 16;
  static const uint8_t YAW_AXIS = 32;

  virtual float getStatePosX(){ return 0;}
  virtual float getStatePosXc(){ return 0;}
  virtual float getStateVelX(){ return 0;}
  virtual float getStateVelXc(){ return 0;}
  virtual float getStateAccXb(){ return 0;}
  virtual float getStatePosY(){ return 0;}
  virtual float getStatePosYc(){ return 0;}
  virtual float getStateVelY(){ return 0;}
  virtual float getStateVelYc(){ return 0;}
  virtual float getStateAccYb(){ return 0;}
  virtual float getStatePosZ(){ return 0;}
  virtual float getStateVelZ(){ return 0;}
  virtual float getStateAccZb(){ return 0;}
  virtual float getStateTheta(){ return 0;}
  virtual float getStatePhy(){ return 0;}
  virtual float getStatePsiBoard(){ return 0;}
  virtual float getStateVelPsiBoard(){ return 0;}
  virtual float getStatePsiCog(){ return 0;}
  virtual float getStateVelPsiCog(){ return 0;}

  virtual void setStatePosX(float value)
  {
    if(use_outer_pose_estimate_ & X_AXIS)
      outer_estimate_pos_x_ = value;
  }

  virtual void setStateVelX(float value)
  {
    if(use_outer_vel_estimate_ & X_AXIS)
      outer_estimate_vel_x_ = value;
  }

  virtual void setStatePosY(float value)
  {
    if(use_outer_pose_estimate_ & Y_AXIS)
      outer_estimate_pos_y_ = value;
  }

  virtual void setStateVelY(float value)
  {
    if(use_outer_vel_estimate_ & Y_AXIS)
      outer_estimate_vel_y_ = value;
  }

  virtual void setStatePosZ(float value)
  {
    if(use_outer_pose_estimate_ & Z_AXIS)
      outer_estimate_pos_z_ = value;
  }

  virtual void setStateVelZ(float value)
  {
    if(use_outer_vel_estimate_ & Z_AXIS)
      outer_estimate_vel_z_ = value;
  }

  virtual void setStateTheta(float value)
  {
    if(use_outer_pose_estimate_ & PITCH_AXIS)
      outer_estimate_theta_ = value;
  }

  virtual void setStateVelTheta(float value)
  {
    if(use_outer_vel_estimate_ & PITCH_AXIS)
      outer_estimate_vel_theta_ = value;
  }

  virtual void setStatePhy(float value)
  {
    if(use_outer_pose_estimate & ROLL_AXIS)
      outer_estimate_phy_ = value;
  }

  virtual void setStateVelPhy(float value)
  {
    if(use_outer_vel_estimate_ & ROLL_AXIS)
      outer_estimate_vel_phy_ = value;
  }


  virtual void setStatePsiCog(float value) //cog frame, for hydra
  {
    if(use_outer_pose_estimate_ & YAW_AXIS)
      outer_estimate_psi_cog_ = value;
  }

  virtual void setStateVelPsiCog(float value)
  {
    if(use_outer_vel_estimate_ & YAW_AXIS)
      outer_estimate_vel_psi_cog_ = value;
  }

  virtual void setStatePsiBoard(float value) //body frame
  {
    if(use_outer_pose_estimate_ & YAW_AXIS)
      outer_estimate_psi_board_ = value;
  }

  virtual void setStateVelPsiBoard(float value)
  {
    if(use_outer_vel_estimate_ & YAW_AXIS)
      outer_estimate_vel_psi_board_ = value;
  }

  virtual void setStatePsi(float value) //both cog and body
  {
    if(use_outer_pose_estimate_ & YAW_AXIS)
      {
        outer_estimate_psi_cog_ = value;
        outer_estimate_psi_board_ = value;
      }
  }

  virtual void setStateVelPsi(float value)
  {
    if(use_outer_vel_estimate_ & YAW_AXIS)
      {
        outer_estimate_vel_psi_cog_ = value;
        outer_estimate_vel_psi_board_ = value;
      }
  }


  virtual float getPosZOffset() {  return  state_pos_z_offset_;}
  virtual void setPosZOffset(float pos_z_offset){  state_pos_z_offset_ = pos_z_offset;}

  virtual float getLaserToImuDistance() {  return 0; }

  virtual bool getRocketStartFlag(){ return true; }

  virtual void setRocketStartFlag(){}
  virtual void setOuterEstimatePoseFlag(uint8_t axis)
  {
    if(axis == 0)
      use_outer_pose_estimate_ = 0;
    else
      use_outer_pose_estimate_ |= axis;
  }

  virtual void setOuterEstimateVelFlag(uint8_t axis)
  {
    if(axis == 0)
      use_outer_vel_estimate_ = 0;
    else
      use_outer_vel_estimate_ |= axis;
  }

  //+*+*+* option, bad
  virtual float getStateVelXOpt() { return 0;}
  virtual float getStateVelYOpt() { return 0;}
  virtual void setKFMeaureFlag(int axis, bool flag){}

 protected:  

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  float outer_estimate_pos_x_;
  float outer_estimate_vel_x_;
  float outer_estimate_pos_y_;
  float outer_estimate_vel_y_;
  float outer_estimate_pos_z_;
  float outer_estimate_vel_z_;
  float outer_estimate_theta_;
  float outer_estimate_vel_theta_;
  float outer_estimate_phy_;
  float outer_estimate_vel_phy_;
  float outer_estimate_psi_cog_;
  float outer_estimate_vel_psi_cog_;
  float outer_estimate_psi_board_;
  float outer_estimate_vel_psi_board_;

  float state_pos_z_offset_; 
  uint8_t use_outer_pose_estimate_;
  uint8_t use_outer_vel_estimate_;
};



class RigidEstimator : public BasicEstimator
{
 public:
  RigidEstimator (ros::NodeHandle nh,
              ros::NodeHandle nh_private,
              bool simulation_flag);
  ~RigidEstimator ();

  //deprecated
  static const int LASER_MIRROR = 0;
  static const int SONAR = 1;

  void tfPublish();
  float getLaserToImuDistance();

 private:

  tf::TransformBroadcaster* br_;
  ros::Time tf_stamp_;

  //+*+*+*+* add as members, but not superclass 
  ImuData* imu_data_ ;
  SlamData* slam_data_ ;
  OpticalFlowData* optical_flow_data_;
  MirrorModule* mirror_module_ ;
  MocapData* mocap_data_;


  KalmanFilterImuLaser* kf_x_;
  KalmanFilterImuLaser* kf_y_;
  KalmanFilterImuLaser* kf_z_;
  KalmanFilterImuLaserBias *kf_bias_x_;
  KalmanFilterImuLaserBias *kf_bias_y_;
  KalmanFilterImuLaserBias *kf_bias_Z_;

  KalmanFilterImuLaserBias *kf1_;
  KalmanFilterImuLaserBias *kf2_;

  //IIR filter for acceleration
  FirFilter *lpf_acc_x_;
  FirFilter *lpf_acc_y_;
  FirFilter *lpf_acc_z_;

  //for optical flow
  KalmanFilterImuLaser* kf_opt_x_;
  KalmanFilterImuLaser* kf_opt_y_;
  KalmanFilterImuLaser* kf_opt_z_;
  KalmanFilterImuLaserBias *kf_opt_bias_x_;
  KalmanFilterImuLaserBias *kf_opt_bias_y_;
  KalmanFilterImuLaserBias *kf_opt_bias_z_;

  //simulation flag
  bool simulation_flag_;

  //*** Kalma Filter 
  bool   kalman_filter_flag_;
  bool   kalman_filter_debug_;
  int    kalman_filter_axis_;

  bool mocap_flag_;

  int altitude_control_mode_;
  bool use_outer_yaw_est_;
  double laser_to_baselink_distance_; 
  double mirror_module_arm_length_;

  std::string laser_frame_;
  std::string camera_frame_;
  std::string baselink_frame_;
  std::string base_footprint_frame_;

  float getStatePosX();
  float getStatePosXc();
  float getStateVelX();
  float getStateVelXc();
  float getStateAccXb();
  float getStatePosY();
  float getStatePosYc();
  float getStateVelY();
  float getStateVelYc();
  float getStateAccYb();
  float getStatePosZ();
  float getStateVelZ();
  float getStateAccZb();
  float getStateTheta();
  float getStatePhy();
  float getStatePsiCog();
  float getStateVelPsiCog();
  float getStatePsiBoard();
  float getStateVelPsiBoard();


  //+*+*+* option
  float getStateVelXOpt();
  float getStateVelYOpt();

  bool getRocketStartFlag();
  void setRocketStartFlag();

  void rosParamInit(ros::NodeHandle nh);

};



#endif
