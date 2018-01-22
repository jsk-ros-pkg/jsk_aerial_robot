/**
  ******************************************************************************
  * File Name          : spine.h
  * Description        : can-based internal comm network, spine side interface
 Includes ------------------------------------------------------------------*/

#ifndef __SPINE_H
#define __SPINE_H

#include "stm32f7xx_hal.h"
#include <config.h>
#include <ros.h>
#include <CAN/can_device_manager.h>
#include <sensors/imu/imu_ros_cmd.h>
#include <Neuron/neuron.h>

/* state estimate  */
#include <state_estimate/state_estimate.h>

#include "math/AP_Math.h"

/* ros */
#include <std_msgs/Empty.h>
#include <aerial_robot_base/UavInfo.h>
#include <CANDevice/initializer/can_initializer.h>
#include <hydrus/ServoStates.h>
#include <hydrus/ServoControlCmd.h>
#include <hydrus/ServoTorqueCmd.h>
#include <hydrus/Gyro.h>
#include <spinal/GetBoardInfo.h>
#include <spinal/SetBoardConfig.h>

/* STL */
#include <algorithm>
#include <functional>

#define SEND_GYRO 0

// main subrutine for update enach instance
namespace Spine
{
  void send(void);
  void update(void);
  void  init(CAN_HandleTypeDef* hcan, ros::NodeHandle* nh, StateEstimate* estimator, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
  void setMotorPwm(uint16_t pwm, uint8_t motor);
  void convertGyroFromJointvalues();
  uint8_t getSlaveNum();
  int8_t getUavModel();
  void servoControlCallback(const hydrus::ServoControlCmd& control_msg);
  void servoTorqueControlCallback(const hydrus::ServoTorqueCmd& control_msg);
  void boardInfoCallback(const spinal::GetBoardInfo::Request& req, spinal::GetBoardInfo::Response& res);
  void boardConfigCallback(const spinal::SetBoardConfig::Request& req, spinal::SetBoardConfig::Response& res);
};

#endif
