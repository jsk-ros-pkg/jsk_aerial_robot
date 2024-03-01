/*
 * servo.h
 *
 *  Created on: 2024/3/1
 *      Author: J.Sugihara
 */

#ifndef APPLICATION_SERVO_TEMP_SERVO_H_
#define APPLICATION_SERVO_TEMP_SERVO_H_

#include "Dynamixel/dynamixel_serial.h"
#include <algorithm>
#include <ros.h>
#include <spinal/ServoControlCmd.h>
#include <spinal/ServoStates.h>
#include <spinal/ServoTorqueCmd.h>
#include <string.h>
#include <config.h>

class Initializer;

class Servo
{
public:
  Servo():
    servo_ctrl_sub_("servo/target_states", &Servo::servoControlCallback,this),
    servo_torque_ctrl_sub_("servo/torque_enable", &Servo::servoTorqueControlCallback,this)
  {
  }
  ~Servo(){}

  void init(UART_HandleTypeDef* huart, I2C_HandleTypeDef* hi2c, ros::NodeHandle* nh, osMutexId* mutex);
  void update();
  void sendData();

private:
  ros::NodeHandle* nh_;
  ros::Subscriber<spinal::ServoControlCmd, Servo> servo_ctrl_sub_;
  ros::Subscriber<spinal::ServoTorqueCmd, Servo> servo_torque_ctrl_sub_;
  struct ServoData{
    int16_t angle;
    uint8_t temperature;
    uint8_t moving;
    int16_t current;
    uint8_t error;
    ServoData(uint16_t angle, uint8_t temperature, uint8_t moving, int16_t current, uint8_t error)
      :angle(angle), temperature(temperature), moving(moving), current(current), error(error){}
  };
  void servoControlCallback(const spinal::ServoControlCmd& control_msg);
  void servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg);

  DynamixelSerial servo_handler_;
  friend class Initializer;
};


#endif /* APPLICATION_SERVO_SERVO_H_ */
