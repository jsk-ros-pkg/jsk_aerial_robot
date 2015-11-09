#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

//* ros
#include <ros/ros.h>

#include <aerial_robot_base/basic_state_estimation.h>
//* for state estimate
//TODO:  plugin 
#include <aerial_robot_base/sensor/mirror_module.h>
#include <aerial_robot_base/sensor/laser_2dslam.h>
#include <aerial_robot_base/sensor/imu_module.h>
#include <aerial_robot_base/sensor/mocap.h>
#include <aerial_robot_base/sensor/optical_flow_module.h>

#include <tf/transform_broadcaster.h>

//* filter
#include <aerial_robot_base/kalman_filter.h>
#include <aerial_robot_base/digital_filter.h>


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



  //+*+*+*+* add as members, but not superclass 
  ImuData* imu_data_ ;
  SlamData* slam_data_ ;
  OpticalFlowData* optical_flow_data_;
  MirrorModule* mirror_module_ ;
  MocapData* mocap_data_;


  KalmanFilterPosVelAcc* kf_x_;
  KalmanFilterPosVelAcc* kf_y_;
  KalmanFilterPosVelAcc* kf_z_;
  KalmanFilterPosVelAccBias *kf_bias_x_;
  KalmanFilterPosVelAccBias *kf_bias_y_;
  KalmanFilterPosVelAccBias *kf_bias_z_;

  KalmanFilterPosVelAccBias *kf1_;
  KalmanFilterPosVelAccBias *kf2_;

  //IIR filter for acceleration
  FirFilter *lpf_acc_x_;
  FirFilter *lpf_acc_y_;
  FirFilter *lpf_acc_z_;

  //for optical flow
  KalmanFilterPosVelAcc* kf_opt_x_;
  KalmanFilterPosVelAcc* kf_opt_y_;
  KalmanFilterPosVelAcc* kf_opt_z_;
  KalmanFilterPosVelAccBias *kf_opt_bias_x_;
  KalmanFilterPosVelAccBias *kf_opt_bias_y_;
  KalmanFilterPosVelAccBias *kf_opt_bias_z_;

  //simulation flag
  bool simulation_flag_;

  //*** Kalma Filter 
  bool   kalman_filter_flag_;
  bool   kalman_filter_debug_;
  int    kalman_filter_axis_;

  bool hokuyo_flag_;
  bool px4flow_flag_;
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

  void setStateCorrectFlag(bool flag);

  void rosParamInit(ros::NodeHandle nh);

  void statesBroadcast();

};



#endif
