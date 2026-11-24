/*
 * can_servo.cpp
 *
 *  Created on: 2016/11/01
 *      Author: anzai
 */

#include "can_servo.h"

void Servo::setTorqueEnable(bool torque_enable)
{
  if (force_servo_off_) return;

  torque_enable_ = torque_enable;
}

void CANServo::sendData()
{
  uint16_t target_angle[4];
  unsigned int send_servo_num = std::min(static_cast<int>(servo_.size()), 4);

  for (unsigned int i = 0 ; i < send_servo_num; i++) {
    target_angle[i] = ((servo_[i].goal_position_ack_ ? 1 : 0) << 15) | (servo_[i].goal_position_ & 0x7FFF);
    servo_[i].goal_position_ack_ = false;
  }
  sendMessage(CAN::MESSAGEID_RECEIVE_SERVO_ANGLE, m_slave_id, send_servo_num * 2, reinterpret_cast<uint8_t*>(target_angle), 0);

  uint16_t target_current[4];
  for (unsigned int i = 0 ; i < send_servo_num; i++) {
    target_current[i] = ((servo_[i].torque_enable_ ? 1 : 0) << 15) | (servo_[i].goal_current_ & 0x7FFF);
  }
  sendMessage(CAN::MESSAGEID_RECEIVE_SERVO_CURRENT, m_slave_id, send_servo_num * 2, reinterpret_cast<uint8_t*>(target_current), 0);

  if (servo_.size() <= 4) return;

  uint16_t extra_target_angle[4];
  send_servo_num = std::min(static_cast<int>(servo_.size() - 4), 4);
  for (unsigned int i = 0 ; i < send_servo_num; i++) {
    extra_target_angle[i] = ((servo_[i + 4].goal_position_ack_ ? 1 : 0) << 15) | (servo_[i + 4].goal_position_ & 0x7FFF);
  }
  sendMessage(CAN::MESSAGEID_RECEIVE_EXTRA_SERVO_ANGLE, m_slave_id, send_servo_num * 2, reinterpret_cast<uint8_t*>(extra_target_angle), 0);

  uint16_t extra_target_current[4];
  for (unsigned int i = 0 ; i < send_servo_num; i++) {
    extra_target_current[i] = ((servo_[i + 4].torque_enable_ ? 1 : 0) << 15) | (servo_[i + 4].goal_current_ & 0x7FFF);
  }
  sendMessage(CAN::MESSAGEID_RECEIVE_EXTRA_SERVO_CURRENT, m_slave_id, send_servo_num * 2, reinterpret_cast<uint8_t*>(extra_target_current), 0);
}

void CANServo::receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data)
{
  CANServoData servo_data = *(reinterpret_cast<CANServoData*>(data));
  servo_[message_id].present_temperature_ = servo_data.temperature;
  servo_[message_id].moving_ = (servo_data.status >> 1) & 0x01;
  servo_[message_id].force_servo_off_ = ((servo_data.status & 0x01) != 0)? true: false;
  bool receive_target_position = (servo_data.status >> 2) & 0x01;
  if (receive_target_position) { // tricky process to save slot
    servo_[message_id].goal_position_ = servo_data.angle;
    servo_[message_id].goal_position_ack_ = true;
  } else {
    servo_[message_id].present_position_ = servo_data.angle;
    servo_[message_id].goal_position_ack_ = false;
  }

  servo_[message_id].present_current_ = servo_data.current;
  servo_[message_id].error_ = servo_data.error;

  if (servo_[message_id].force_servo_off_) {
    servo_[message_id].torque_enable_ = false;
  }
}
