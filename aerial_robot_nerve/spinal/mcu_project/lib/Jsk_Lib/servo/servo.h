/**
******************************************************************************
* File Name          : servo.h
* Description        : universal servo control interface for Spinal
* Author             : J.Sugihara (2024/3/1)
includes ------------------------------------------------------------------*/


#ifndef APPLICATION_SERVO_TEMP_SERVO_H_
#define APPLICATION_SERVO_TEMP_SERVO_H_

#include "Dynamixel/dynamixel_serial.h"
#include <ros.h>
#include <spinal/ServoControlCmd.h>
#include <spinal/ServoStates.h>
#include <spinal/ServoTorqueStates.h>
#include <spinal/ServoTorqueCmd.h>
#include <string.h>
#include <config.h>
class Initializer;

class DirectServo
{
public:
  DirectServo():
    servo_ctrl_sub_("servo/target_states", &DirectServo::servoControlCallback,this),
    servo_torque_ctrl_sub_("servo/torque_enable", &DirectServo::servoTorqueControlCallback,this),
    servo_state_pub_("servo/states", &servo_state_msg_),
    servo_torque_state_pub_("servo/torque_states", &servo_torque_state_msg_)
  {
  }
  ~DirectServo(){}

  void init(UART_HandleTypeDef* huart, ros::NodeHandle* nh, osMutexId* mutex);
  void update();
  void sendData();

private:
  /* ROS */
  ros::NodeHandle* nh_;
  ros::Subscriber<spinal::ServoControlCmd, DirectServo> servo_ctrl_sub_;
  ros::Subscriber<spinal::ServoTorqueCmd, DirectServo> servo_torque_ctrl_sub_;
  ros::Publisher servo_state_pub_;
  ros::Publisher servo_torque_state_pub_;
  spinal::ServoStates servo_state_msg_;
  spinal::ServoTorqueStates servo_torque_state_msg_;

  uint32_t servo_last_pub_time_;
  uint32_t servo_torque_last_pub_time_;
  
  /* Servo state */
  struct ServoState{
    int16_t angle;
    uint8_t temperature;
    uint8_t moving;
    int16_t current;
    uint8_t error;
    ServoState(uint16_t angle, uint8_t temperature, uint8_t moving, int16_t current, uint8_t error)
      :angle(angle), temperature(temperature), moving(moving), current(current), error(error){}
  };
  void servoControlCallback(const spinal::ServoControlCmd& control_msg);
  void servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg);

  DynamixelSerial servo_handler_;
  friend class Initializer;
};


#endif /* APPLICATION_SERVO_SERVO_H_ */
