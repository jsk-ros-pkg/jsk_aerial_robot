/**
******************************************************************************
* File Name          : servo.cpp
* Description        : universal servo control interface for Spinal
* Author             : J.Sugihara (2024/3/1)
 ------------------------------------------------------------------*/


#include "servo.h"

void DirectServo::init(UART_HandleTypeDef* huart,  ros::NodeHandle* nh, osMutexId* mutex = NULL) //TODO: support encoder
{
  nh_ = nh;
  nh_->subscribe(servo_ctrl_sub_);
  nh_->subscribe(servo_torque_ctrl_sub_);
  nh_->advertise(servo_state_pub_);
  nh_->advertise(servo_torque_state_pub_);

  // servo_state_msg_.servos_length = servo_with_send_flag_.size();
  // servo_state_msg_.servos = new spinal::ServoState[servo_with_send_flag_.size()];
  // servo_torque_state_msg_.torque_enable_length = servo_.size();
  // servo_torque_state_msg_.torque_enable = new uint8_t[servo_.size()];

  servo_handler_.init(huart, mutex);
}

void DirectServo::update()
{
  servo_handler_.update();
}

void DirectServo::sendData()
{
  for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
    const ServoData& s = servo_handler_.getServo()[i];
    if (s.send_data_flag_ != 0) {
      ServoState data(static_cast<int16_t>(s.getPresentPosition()),
                     s.present_temp_,
                     s.moving_,
                     s.present_current_,
                     s.hardware_error_status_);
      // sendMessage(CAN::MESSAGEID_SEND_SERVO_LIST[i], m_slave_id, 8, reinterpret_cast<uint8_t*>(&data), 1);
    }
  }
}

void DirectServo::servoControlCallback(const spinal::ServoControlCmd& control_msg)
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

void DirectServo::servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg)
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
