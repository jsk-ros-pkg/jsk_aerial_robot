/*
 * servo.cpp
 *
 *  Created on: 2024/3/1
 *      Author: J.Sugihara
 *
 */

#include "servo.h"

void Servo::init(UART_HandleTypeDef* huart, I2C_HandleTypeDef* hi2c, ros::NodeHandle* nh, osMutexId* mutex = NULL)
{
  nh_ = nh;
  nh_->subscribe(servo_ctrl_sub_);
  nh_->subscribe(servo_torque_ctrl_sub_);
  servo_handler_.init(huart, hi2c, mutex);
}

void Servo::update()
{
  servo_handler_.update();
}

void Servo::sendData()
{
  for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
    const ServoData& s = servo_handler_.getServo()[i];
    if (s.send_data_flag_ != 0) {
      ServoData data(static_cast<int16_t>(s.getPresentPosition()),
                     s.present_temp_,
                     s.moving_,
                     s.present_current_,
                     s.hardware_error_status_);
      // sendMessage(CAN::MESSAGEID_SEND_SERVO_LIST[i], m_slave_id, 8, reinterpret_cast<uint8_t*>(&data), 1);
    }
  }
}

void Servo::servoControlCallback(const spinal::ServoControlCmd& control_msg)
{
  if (control_msg.index_length != control_msg.angles_length) return;
  for (unsigned int i = 0; i < control_msg.index_length; i++) {
    ServoData& s = servo_handler_.getServo()[control_msg.index[i]];
    int32_t goal_pos = static_cast<int32_t>(control_msg.angles[i]);
    s.setGoalPosition(goal_pos);
    if (! s.torque_enable_) {
      s.torque_enable_ = true;
      servo_handler_.setTorque(i);
    }
  }
}

void Servo::servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg)
{
  if (control_msg.index_length != control_msg.torque_enable_length) return;
  for (unsigned int i = 0; i < control_msg.index_length; i++) {
    ServoData& s = servo_handler_.getServo()[control_msg.index[i]];
    if (! s.torque_enable_) {
      s.torque_enable_ = (control_msg.torque_enable[i] != 0) ? true : false;
      servo_handler_.setTorque(i);
    }
  }
}
