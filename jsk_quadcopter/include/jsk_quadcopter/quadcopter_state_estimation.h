/*
pitch : theta roll  : phy yaw   : psi
using tf/Transform.h to deal with tranformation 
相互参照の解消方法1: supter class
*/

#ifndef QUADCOPTER_STATE_ESTIMATION_H
#define QUADCOPTER_STATE_ESTIMATION_H

//* ros
#include <ros/ros.h>
#include <jsk_quadcopter/state_estimation.h>
#include <jsk_quadcopter/sensor/mirror_module.h>
#include <jsk_quadcopter/sensor/slam_data.h>
#include <jsk_quadcopter/sensor/imu_module.h>
#include <jsk_quadcopter/sensor/mocap_data.h>
#include <jsk_quadcopter/sensor/optical_flow_module.h>
//#include <jsk_quadcopter/sensor/ar_marker_data.h>
#include <tf/transform_broadcaster.h>
//#include <jsk_quadcopter/QuadcopterStatus.h>
#include <iostream>
//* filter
#include <jsk_quadcopter/kalman_filter.h>
#include <jsk_quadcopter/digital_filter.h>


class Estimator1 : public Estimator
{
 public:
  Estimator1 (ros::NodeHandle nh,
              ros::NodeHandle nh_private,
              bool simulation_flag);
  ~Estimator1 ();

  //+*+*+*+* add as members, but not superclass 
  ImuData* imuData_ ;
  SlamData* slamData_ ;
  OpticalFlowData* opticalFlowData_;
  MirrorModule* mirrorModule_ ;
  MocapData* mocapData_;

  void tfPublish();
  float getLaserToImuDistance();
  void setKalmanFilterBoardToAsctec();

  //simulation flag
  bool simulationFlag;

  //kalman filter
  bool kalmanFilterFlag;
  
  KalmanFilterImuLaser* kfX_;
  KalmanFilterImuLaser* kfY_;
  KalmanFilterImuLaser* kfZ_;
  KalmanFilterImuLaserBias *kfbX_;
  KalmanFilterImuLaserBias *kfbY_;
  KalmanFilterImuLaserBias *kfbZ_;

  bool kalmanFilterDebug; 
  KalmanFilterImuLaserBias *kf1_;
  KalmanFilterImuLaserBias *kf2_;


  //IIR filter for acceleration
  FirFilter *filterAccX_;
  FirFilter *filterAccY_;
  FirFilter *filterAccZ_;

  //for optical flow
  KalmanFilterImuLaser* kfXForOpt_;
  KalmanFilterImuLaser* kfYForOpt_;
  KalmanFilterImuLaser* kfZForOpt_;
  KalmanFilterImuLaserBias *kfbXForOpt_;
  KalmanFilterImuLaserBias *kfbYForOpt_;
  KalmanFilterImuLaserBias *kfbZForOpt_;

  static const int LASER_MIRROR = 0;
  static const int SONAR = 1;


 private:

  tf::TransformBroadcaster* tfB_;
  ros::Time tfStamp_;

  //*** Kalma Filter 
  bool   kalmanFilterFlag_;
  bool   kalmanFilterDebug_;
  int    kalmanFilterAxis_;

  bool mocapFlag_;

  int altitudeControlMode_;
  bool useOuterYawEst_;
  double laserToBaselinkDistance_; 
  double mirrorModuleArmLength_;

  std::string laserFrame_;
  std::string cameraFrame_;
  std::string baselinkFrame_;
  std::string baseFootprintFrame_;



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
  float getStatePsiBody();
  float getStateVelPsiBody();


  //+*+*+* option
  float getStateVelXOpt();
  float getStateVelYOpt();

  bool getRocketStartFlag();
  void setRocketStartFlag();

  void rosParamInit();

};

#endif
