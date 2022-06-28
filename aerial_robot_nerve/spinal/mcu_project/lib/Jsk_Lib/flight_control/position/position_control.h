/*
******************************************************************************
* File Name          : attitude_control.h
* Description        : attitude control interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __POSITION_CONTROL_H
#define __POSITION_CONTROL_H

#include "config.h"
#include <ros.h>

/* state estimate  */
#include "state_estimate/state_estimate.h"

class PositionController
{
public:
  PositionController(){}
  void init(StateEstimate* estimator, AttitudeController* att_controller, ros::NodeHandle* nh)
  {
    nh_ = nh;
    estimator_ = estimator;
    att_controller_ = att_controller;
    start_control_flag_ = false;
  }
  ~PositionController(){}

  void update();

private:
  ros::NodeHandle* nh_;
  StateEstimate* estimator_;
  AttitudeController* att_controller_;

  //PID Control
  float p_gain_[3];
  float i_gain_[3];
  float d_gain_[3];
  float p_term_[3];
  float i_term_[3];
  float d_term_[3];
  float pid_value_[3];

  //Control Output
  float target_angle_[3];
  float throttle_force_;

  //Control flag
  bool start_control_flag_;
};
#endif

