#ifndef BASIC_STATE_ESTIMATION_H
#define BASIC_STATE_ESTIMATION_H

//* ros
#include <ros/ros.h>
#include <aerial_robot_base/States.h>
#include <iostream>


class BasicEstimator
{
 public:
 BasicEstimator(ros::NodeHandle nh, ros::NodeHandle nh_private)
  : nh_(nh, "estimator"), nhp_(nh_private, "estimator")
    {
      full_states_pub_ = nh_.advertise<aerial_robot_base::States>("full_states", 1); 

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

      sys_stamp_ = ros::Time::now();

    }

  virtual ~BasicEstimator(){}

  static const uint8_t X_AXIS = 1;
  static const uint8_t Y_AXIS = 2;
  static const uint8_t Z_AXIS = 4;
  static const uint8_t PITCH_AXIS = 8;
  static const uint8_t ROLL_AXIS = 16;
  static const uint8_t YAW_AXIS = 32;

  inline ros::Time getSystemTimeStamp(){ return sys_stamp_;}
  inline void setSystemTimeStamp(ros::Time sys_stamp){ sys_stamp_ = sys_stamp;}

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
    if(use_outer_pose_estimate_ & ROLL_AXIS)
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

  virtual void setStatePsiBoard(float value) //board frame
  {
    if(use_outer_pose_estimate_ & YAW_AXIS)
      outer_estimate_psi_board_ = value;
  }

  virtual void setStateVelPsiBoard(float value)
  {
    if(use_outer_vel_estimate_ & YAW_AXIS)
      outer_estimate_vel_psi_board_ = value;
  }

  //recommandicate
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

  virtual void setStateCorrectFlag(bool flag){ return;}

 protected:  

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher full_states_pub_;

  ros::Time sys_stamp_;

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





#endif
