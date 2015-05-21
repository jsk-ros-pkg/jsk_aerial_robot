#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

//* ros
#include <ros/ros.h>
//* for dynamic reconfigure
#include <aerial_robot_msgs/DynamicReconfigureLevels.h>
#include <dynamic_reconfigure/server.h>
#include <aerial_robot_base/StateKalmanFilterConfig.h>

//* for state estimate
#include <aerial_robot_base/sensor/mirror_module.h>
#include <aerial_robot_base/sensor/slam_data.h>
#include <aerial_robot_base/sensor/imu_module.h>
#include <aerial_robot_base/sensor/mocap_data.h>
#include <aerial_robot_base/sensor/optical_flow_module.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
//* filter
#include <aerial_robot_base/kalman_filter.h>
#include <aerial_robot_base/digital_filter.h>


class BasicEstimator
{
 public:
  BasicEstimator()
    {
      outerEstimatePosX = 0;
      outerEstimateVelX = 0;
      outerEstimatePosY = 0;
      outerEstimateVelY = 0;
      outerEstimatePosZ = 0;
      outerEstimateVelZ = 0;
      outerEstimateTheta = 0;
      outerEstimateVelTheta = 0;
      outerEstimatePhy = 0;
      outerEstimateVelPhy = 0;
      outerEstimatePsiCog = 0;
      outerEstimateVelPsiCog = 0;
      outerEstimatePsiBoard = 0;
      outerEstimateVelPsiBoard = 0;


      statePosZOffset = 1.0; //1m

      useOuterPoseEstimate = 0;
      useOuterVelEstimate = 0;
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
    if(useOuterPoseEstimate & X_AXIS)
      outerEstimatePosX = value;
  }

  virtual void setStateVelX(float value)
  {
    if(useOuterVelEstimate & X_AXIS)
      outerEstimateVelX = value;
  }

  virtual void setStatePosY(float value)
  {
    if(useOuterPoseEstimate & Y_AXIS)
      outerEstimatePosY = value;
  }

  virtual void setStateVelY(float value)
  {
    if(useOuterVelEstimate & Y_AXIS)
      outerEstimateVelY = value;
  }

  virtual void setStatePosZ(float value)
  {
    if(useOuterPoseEstimate & Z_AXIS)
      outerEstimatePosZ = value;
  }

  virtual void setStateVelZ(float value)
  {
    if(useOuterVelEstimate & Z_AXIS)
      outerEstimateVelZ = value;
  }

  virtual void setStateTheta(float value)
  {
    if(useOuterPoseEstimate & PITCH_AXIS)
      outerEstimateTheta = value;
  }

  virtual void setStateVelTheta(float value)
  {
    if(useOuterVelEstimate & PITCH_AXIS)
      outerEstimateVelTheta = value;
  }

  virtual void setStatePhy(float value)
  {
    if(useOuterPoseEstimate & ROLL_AXIS)
      outerEstimatePhy = value;
  }

  virtual void setStateVelPhy(float value)
  {
    if(useOuterVelEstimate & ROLL_AXIS)
      outerEstimateVelPhy = value;
  }


  virtual void setStatePsiCog(float value) //cog frame, for hydra
  {
    if(useOuterPoseEstimate & YAW_AXIS)
      outerEstimatePsiCog = value;
  }

  virtual void setStateVelPsiCog(float value)
  {
    if(useOuterVelEstimate & YAW_AXIS)
      outerEstimateVelPsiCog = value;
  }

  virtual void setStatePsiBoard(float value) //body frame
  {
    if(useOuterPoseEstimate & YAW_AXIS)
      outerEstimatePsiBoard = value;
  }

  virtual void setStateVelPsiBoard(float value)
  {
    if(useOuterVelEstimate & YAW_AXIS)
      outerEstimateVelPsiBoard = value;
  }

  virtual void setStatePsi(float value) //both cog and body
  {
    if(useOuterPoseEstimate & YAW_AXIS)
      {
        outerEstimatePsiCog = value;
        outerEstimatePsiBoard = value;
      }
  }

  virtual void setStateVelPsi(float value)
  {
    if(useOuterVelEstimate & YAW_AXIS)
      {
        outerEstimateVelPsiCog = value;
        outerEstimateVelPsiBoard = value;
      }
  }


  virtual float getPosZOffset() {  return  statePosZOffset;}
  virtual void setPosZOffset(float pos_z_offset){  statePosZOffset = pos_z_offset;}

  virtual float getLaserToImuDistance() {  return 0; }

  virtual bool getRocketStartFlag(){ return true; }

  virtual void setRocketStartFlag(){}
  virtual void setOuterEstimatePoseFlag(uint8_t axis)
  {
    if(axis == 0)
      useOuterPoseEstimate = 0;
    else
      useOuterPoseEstimate |= axis;
  }

  virtual void setOuterEstimateVelFlag(uint8_t axis)
  {
    if(axis == 0)
      useOuterVelEstimate = 0;
    else
      useOuterVelEstimate |= axis;
  }


  //+*+*+* option
  virtual float getStateVelXOpt() { return 0;}
  virtual float getStateVelYOpt() { return 0;}
  virtual void setKFMeaureFlag(int axis, bool flag){}


 protected:  

  ros::NodeHandle estimatorNodeHandle_;
  ros::NodeHandle estimatorNodeHandlePrivate_;

  float outerEstimatePosX;
  float outerEstimateVelX;
  float outerEstimatePosY;
  float outerEstimateVelY;
  float outerEstimatePosZ;
  float outerEstimateVelZ;
  float outerEstimateTheta;
  float outerEstimateVelTheta;
  float outerEstimatePhy;
  float outerEstimateVelPhy;
  float outerEstimatePsiCog;
  float outerEstimateVelPsiCog;
  float outerEstimatePsiBoard;
  float outerEstimateVelPsiBoard;

  float statePosZOffset; 
  uint8_t useOuterPoseEstimate;
  uint8_t useOuterVelEstimate;
};



class RigidEstimator : public BasicEstimator
{
 public:
  RigidEstimator (ros::NodeHandle nh,
              ros::NodeHandle nh_private,
              bool simulation_flag);
  ~RigidEstimator ();

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
  float getStatePsiBoard();
  float getStateVelPsiBoard();


  //+*+*+* option
  float getStateVelXOpt();
  float getStateVelYOpt();

  bool getRocketStartFlag();
  void setRocketStartFlag();

  void rosParamInit();

};



#endif
