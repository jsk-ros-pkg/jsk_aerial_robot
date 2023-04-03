/*
******************************************************************************
* File Name          : flight_control.h
* Description        : flight control(attitude, position)  interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __FLIGHT_CONTROL_H
#define __FLIGHT_CONTROL_H

#ifndef SIMULATION
#include "config.h"
#include <ros.h>
#else
#include <ros/ros.h>
#endif

#ifndef SIMULATION
/* state estimate  */
#include "state_estimate/state_estimate.h"

/* battery status */
#include "battery_status/battery_status.h"
#endif

/* controller  */
#include "flight_control/attitude/attitude_control.h"

/* ros */
#include <spinal/FlightConfigCmd.h>
#include <spinal/UavInfo.h>

class FlightControl
{
public:
  ~FlightControl(){}

#ifdef SIMULATION
  FlightControl(): att_controller_() {}

  void init(ros::NodeHandle* nh, StateEstimate* estimator)
  {
    nh_ = nh;
    config_ack_pub_ = nh_->advertise<std_msgs::UInt8>("flight_config_ack", 1);
    uav_info_sub_ = nh_->subscribe("uav_info", 1, &FlightControl::uavInfoConfigCallback, this);
    flight_config_sub_ = nh_->subscribe("flight_config_cmd", 1, &FlightControl::flightConfigCallback, this);

    att_controller_.init(nh, estimator);

    start_control_flag_ = false;
    pwm_test_flag_ = false;
    integrate_flag_ = false;
  }

  inline bool getStartControlFlag() { return start_control_flag_; }
  inline AttitudeController& getAttController(){ return att_controller_;}

  void useGroundTruth(bool flag) { att_controller_.useGroundTruth(flag); }
#else
  FlightControl():
    config_ack_pub_("flight_config_ack", &config_ack_msg_),
    uav_info_sub_("uav_info", &FlightControl::uavInfoConfigCallback, this ),
    flight_config_sub_("flight_config_cmd", &FlightControl::flightConfigCallback, this ),
    att_controller_()
  {
  }

  void init(TIM_HandleTypeDef* htim1, TIM_HandleTypeDef* htim2, StateEstimate* estimator, BatteryStatus* bat, ros::NodeHandle* nh, osMutexId* mutex = NULL)
  {
    nh_ = nh;

    /* config ack to ROS */
    nh_->advertise(config_ack_pub_);
    /* uav & motor type */
    nh_->subscribe(uav_info_sub_);
    /* flight control base config */
    nh_->subscribe(flight_config_sub_);

    estimator_ = estimator;

    bat_ = bat;

    pwm_htim1_ = htim1;
    pwm_htim2_ = htim2;

    mutex_ = mutex;

    att_controller_.init(htim1, htim2, estimator, bat, nh, mutex);
    //pos_controller_.init(estimator_, &att_controller_, nh_);

    start_control_flag_ = false;
    pwm_test_flag_ = false;
    integrate_flag_ = false;
  }
#endif

  void update()
  {
    att_controller_.update();

    /* check the failsafe from attitude controller */
    if(!force_landing_flag_ && att_controller_.getForceLandingFlag())
      {
        force_landing_flag_ = true;
        config_ack_msg_.data = spinal::FlightConfigCmd::FORCE_LANDING_CMD;
#ifdef SIMULATION
        config_ack_pub_.publish(config_ack_msg_);
#else
        config_ack_pub_.publish(&config_ack_msg_);
#endif
      }
  }

  void setMotorNumber(uint8_t motor_number)
  {
    att_controller_.setMotorNumber(motor_number);
  }

  void setUavModel(int8_t uav_model)
  {
    att_controller_.setUavModel(uav_model);
  }

  static const uint8_t MOTOR_START_MSG = 0x00;
  static const uint8_t MOTOR_STOP_MSG = 0x01;
  static const uint8_t FORCE_LANDING_MSG = 0x02;
  static const uint8_t PWM_TEST_MODE_MSG = 0x10;
  static const uint8_t ATT_INTEGRATE_CMD_MSG = 0xA0; //integrate flag

private:
  ros::NodeHandle* nh_;
  ros::Publisher config_ack_pub_;
  std_msgs::UInt8 config_ack_msg_;
#ifdef SIMULATION
  ros::Subscriber uav_info_sub_;
  ros::Subscriber flight_config_sub_;
#else
  ros::Subscriber<spinal::UavInfo, FlightControl> uav_info_sub_;
  ros::Subscriber<spinal::FlightConfigCmd, FlightControl> flight_config_sub_;
#endif

  bool start_control_flag_;
  bool pwm_test_flag_;
  bool integrate_flag_;
  bool force_landing_flag_;

  AttitudeController att_controller_;
#ifndef SIMULATION
  StateEstimate* estimator_;
  BatteryStatus* bat_;
  TIM_HandleTypeDef* pwm_htim1_;
  TIM_HandleTypeDef*  pwm_htim2_;
  osMutexId* mutex_;
#endif

  void flightConfigCallback(const spinal::FlightConfigCmd& config_msg)
  {
    switch(config_msg.cmd)
      {
      case spinal::FlightConfigCmd::ARM_ON_CMD:

        /* check whether motor_number is assigned */
        if(!att_controller_.activated()) break;

        /* clear  force landing */
        force_landing_flag_ = false;
        att_controller_.setForceLandingFlag(force_landing_flag_);

        start_control_flag_ = true;
        att_controller_.setStartControlFlag(start_control_flag_);

        /* ack to ROS */
        config_ack_msg_.data = spinal::FlightConfigCmd::ARM_ON_CMD;
#ifdef SIMULATION
        ROS_ERROR("[d_board] motor armed, motor number is %d", att_controller_.getMotorNumber());
        config_ack_pub_.publish(config_ack_msg_);
#else
        config_ack_pub_.publish(&config_ack_msg_);
#endif
        break;
      case spinal::FlightConfigCmd::ARM_OFF_CMD:
        start_control_flag_ = false;
        att_controller_.setStartControlFlag(start_control_flag_);

        /* ack to ROS */
        config_ack_msg_.data = spinal::FlightConfigCmd::ARM_OFF_CMD;
#ifdef SIMULATION
        config_ack_pub_.publish(config_ack_msg_);
#else
        config_ack_pub_.publish(&config_ack_msg_);
#endif
        /* clear  force landing */
        force_landing_flag_ = false;
        att_controller_.setForceLandingFlag(force_landing_flag_);

        break;
      case spinal::FlightConfigCmd::FORCE_LANDING_CMD:
        force_landing_flag_ = true;
        att_controller_.setForceLandingFlag(force_landing_flag_);
        config_ack_msg_.data = spinal::FlightConfigCmd::FORCE_LANDING_CMD;
#ifdef SIMULATION
        config_ack_pub_.publish(config_ack_msg_);
#else
        config_ack_pub_.publish(&config_ack_msg_);
#endif

        break;
      case spinal::FlightConfigCmd::INTEGRATION_CONTROL_ON_CMD:
        integrate_flag_ = true;
        att_controller_.setIntegrateFlag(integrate_flag_);
        break;
      case spinal::FlightConfigCmd::INTEGRATION_CONTROL_OFF_CMD:
        integrate_flag_ = false;
        att_controller_.setIntegrateFlag(integrate_flag_);
        break;
      default:
        break;
      }
  }

/* get the UAV type from ros, which is necessary for simulation and general multirotor */
void uavInfoConfigCallback(const spinal::UavInfo& config_msg)
  {
    setUavModel(config_msg.uav_model);
    setMotorNumber(config_msg.motor_num);
  }

};

#endif
