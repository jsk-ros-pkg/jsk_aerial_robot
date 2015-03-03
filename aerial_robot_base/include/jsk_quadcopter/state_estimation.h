#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

//* ros
#include <ros/ros.h>
//* for dynamic reconfigure
#include <jsk_quadcopter_common/DynamicReconfigureLevels.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_quadcopter/StateKalmanFilterConfig.h>

class Estimator
{
 public:
  Estimator();
  virtual ~Estimator();

  static const uint8_t X_AXIS = 1;
  static const uint8_t Y_AXIS = 2;
  static const uint8_t Z_AXIS = 4;
  static const uint8_t PITCH_AXIS = 8;
  static const uint8_t ROLL_AXIS = 16;
  static const uint8_t YAW_AXIS = 32;

  virtual float getStatePosX();
  virtual float getStatePosXc();
  virtual float getStateVelX();
  virtual float getStateVelXc();
  virtual float getStateAccXb();
  virtual float getStatePosY();
  virtual float getStatePosYc();
  virtual float getStateVelY();
  virtual float getStateVelYc();
  virtual float getStateAccYb();
  virtual float getStatePosZ();
  virtual float getStateVelZ();
  virtual float getStateAccZb();
  virtual float getStateTheta();
  virtual float getStatePhy();
  virtual float getStatePsiBody();
  virtual float getStateVelPsiBody();
  virtual float getStatePsiCog();
  virtual float getStateVelPsiCog();

  virtual void setStatePosX(float value);
  virtual void setStateVelX(float value);
  virtual void setStatePosY(float value);
  virtual void setStateVelY(float value);
  virtual void setStatePosZ(float value);
  virtual void setStateVelZ(float value);
  virtual void setStateTheta(float value);
  virtual void setStateVelTheta(float value);
  virtual void setStatePhy(float value);
  virtual void setStateVelPhy(float value);

  virtual void setStatePsiCog(float value); //cog frame, for hydra
  virtual void setStateVelPsiCog(float value);
  virtual void setStatePsiBody(float value); //body frame
  virtual void setStateVelPsiBody(float value);
  virtual void setStatePsi(float value); //both cog and body
  virtual void setStateVelPsi(float value);


  virtual float getPosZOffset();
  virtual void setPosZOffset(float pos_z_offset);
  virtual float getLaserToImuDistance();
  virtual bool getRocketStartFlag();  
  virtual void setRocketStartFlag();
  virtual void setOuterEstimatePoseFlag(uint8_t axis);
  virtual void setOuterEstimateVelFlag(uint8_t axis);


  //+*+*+* option
  virtual float getStateVelXOpt();
  virtual float getStateVelYOpt();
  virtual void setKFMeaureFlag(int axis, bool flag);


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
  float outerEstimatePsiBody;
  float outerEstimateVelPsiBody;

  float statePosZOffset; 
  uint8_t useOuterPoseEstimate;
  uint8_t useOuterVelEstimate;
};

#endif
