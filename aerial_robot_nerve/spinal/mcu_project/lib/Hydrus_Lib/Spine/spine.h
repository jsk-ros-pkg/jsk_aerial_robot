/**
  ******************************************************************************
  * File Name          : spine.h
  * Description        : can-based internal comm network, spine side interface
 Includes ------------------------------------------------------------------*/

#ifndef __SPINE_H
#define __SPINE_H

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
#include <spinal/UavInfo.h>
#include <CANDevice/initializer/can_initializer.h>
#include <spinal/ServoStates.h>
#include <spinal/ServoControlCmd.h>
#include <spinal/ServoTorqueCmd.h>
#include <spinal/ServoTorqueStates.h>
#include <spinal/Gyro.h>
#include <spinal/GetBoardInfo.h>
#include <spinal/SetBoardConfig.h>

/* STL */
#include <algorithm>
#include <functional>

/* RTOS */
#include "cmsis_os.h"

#ifdef STM32F7
using CAN_GeranlHandleTypeDef = CAN_HandleTypeDef;
#endif

#ifdef STM32H7
using CAN_GeranlHandleTypeDef = FDCAN_HandleTypeDef;
#endif


#define SEND_GYRO 0

// main subrutine for update enach instance
namespace Spine
{
  void send(void);
  void update(void);
  void init(CAN_GeranlHandleTypeDef* hcan, ros::NodeHandle* nh, StateEstimate* estimator, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
  void setMotorPwm(uint16_t pwm, uint8_t motor);
  void convertGyroFromJointvalues();
  uint8_t getSlaveNum();
  int8_t getUavModel();
  void useRTOS(osMailQId* handle);
  void setServoControlFlag(bool flag);
  void servoPositionCallback(const spinal::ServoControlCmd& control_msg);
  void servoCurrentCallback(const spinal::ServoControlCmd& control_msg);
  void servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg);
  void boardInfoCallback(const spinal::GetBoardInfo::Request& req, spinal::GetBoardInfo::Response& res);
  void boardConfigCallback(const spinal::SetBoardConfig::Request& req, spinal::SetBoardConfig::Response& res);
};

#endif
