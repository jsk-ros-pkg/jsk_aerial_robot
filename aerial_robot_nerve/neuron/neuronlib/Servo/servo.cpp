/*
 * servo.cpp
 *
 *  Created on: 2016/10/28
 *      Author: anzai
 *  Maintainer: bakui chou(2016/11/20)
 */

#include "servo.h"

void Servo::init(UART_HandleTypeDef* huart, I2C_HandleTypeDef* hi2c, osMutexId* mutex = NULL)
{
  servo_handler_.init(huart, hi2c, mutex);
  connect_ = false;
}

void Servo::update()
{
  servo_handler_.update();
}

void Servo::sendData()
{
  for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
    const ServoData& s = servo_handler_.getServo()[i];

    if (!s.send_data_flag_) {
      continue;
    }

    int32_t position = s.getPresentPosition();
    if (s.send_goal_position_) {
      position = s.getGoalPosition();
    }

    CANServoData data(static_cast<int16_t>(position),
                      s.present_temp_,
                      s.moving_,
                      !connect_ || s.force_servo_off_,
                      s.send_goal_position_,
                      s.present_current_,
                      s.hardware_error_status_);
    sendMessage(CAN::MESSAGEID_SEND_SERVO_LIST[i], m_slave_id, 8, reinterpret_cast<uint8_t*>(&data), 1);
  }
}

void Servo::receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data)
{
  if (!connect_) return;

  unsigned int max_servo_num = 4;
  switch (message_id) {
  case CAN::MESSAGEID_RECEIVE_SERVO_ANGLE:
    {
      unsigned int servo_num = std::min(servo_handler_.getServoNum(), max_servo_num);
      for (unsigned int i = 0; i < servo_num; i++) {
        ServoData& s = servo_handler_.getServo()[i];
        //convert int15 to int32
        int sign = data[i * 2 + 1] & 0x40;
        int32_t goal_pos = ((data[i * 2 + 1]  & 0x7F) << 8) | data[i * 2];
        if (sign != 0) {
          goal_pos = 0xFFFF8000 | goal_pos;
        }

        if (!s.torque_enable_) continue;

        if (s.send_goal_position_)
          {
            bool ack = ((data[i * 2 + 1] >> 7) & 0x01);
            if (!ack) continue;;

            s.send_goal_position_ = false;
          }

        s.setGoalPosition(goal_pos);
      }
      break;
    }
  case CAN::MESSAGEID_RECEIVE_EXTRA_SERVO_ANGLE:
    {
      unsigned int servo_num = std::min(servo_handler_.getServoNum() - 4, max_servo_num);
      for (unsigned int i = 0; i < servo_num; i++) {
        ServoData& s = servo_handler_.getServo()[i + 4];
        //convert int15 to int32
        int sign = data[i * 2 + 1] & 0x40;
        int32_t goal_pos = ((data[i * 2 + 1]  & 0x7F) << 8) | data[i * 2];
        if (sign != 0) {
          goal_pos = 0xFFFF8000 | goal_pos;
        }

        if (!s.torque_enable_) continue;

        if (s.send_goal_position_)
          {
            bool ack = ((data[i * 2 + 1] >> 7) & 0x01);
            if (!ack) continue;;

            s.send_goal_position_ = false;
          }

        s.setGoalPosition(goal_pos);
      }
      break;
    }
  case CAN::MESSAGEID_RECEIVE_SERVO_CURRENT:
    {
      unsigned int servo_num = std::min(servo_handler_.getServoNum(), max_servo_num);
      for (unsigned int i = 0; i < servo_num; i++) {
        ServoData& s = servo_handler_.getServo()[i];
        //convert int15 to int32
        int sign = data[i * 2 + 1] & 0x40;
        int16_t goal_current = (int16_t)((((data[i * 2 + 1] & 0x7F) << 8) & 0xFF00) | (data[i * 2] & 0xFF));
        if (sign != 0) {
          goal_current = 0x8000 | goal_current;
        }
        s.setGoalCurrent(goal_current);
        bool torque_enable = (((data[i * 2 + 1] >> 7) & 0x01) != 0) ? true : false;
        servo_handler_.setTorque(i, torque_enable);
      }
      break;
    }
    case CAN::MESSAGEID_RECEIVE_EXTRA_SERVO_CURRENT:
    {
      unsigned int servo_num = std::min(servo_handler_.getServoNum() - 4, max_servo_num);
      for (unsigned int i = 0; i < servo_num; i++) {
        ServoData& s = servo_handler_.getServo()[i + 4];
        //convert int15 to int32
        int sign = data[i * 2 + 1] & 0x40;
        int16_t goal_current = (int16_t)((((data[i * 2 + 1] & 0x7F) << 8) & 0xFF00) | (data[i * 2] & 0xFF));
        if (sign != 0) {
          goal_current = 0x8000 | goal_current;
        }
        s.setGoalCurrent(goal_current);
        bool torque_enable = (((data[i * 2 + 1] >> 7) & 0x01) != 0) ? true : false;
        servo_handler_.setTorque(i + 4, torque_enable);
      }
      break;
    }
  }
}
