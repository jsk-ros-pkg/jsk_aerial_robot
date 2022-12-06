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
			CANServoData data(static_cast<int16_t>(s.getPresentPosition()),
							  s.present_temp_,
							  s.moving_,
							  s.force_servo_off_,
							  s.present_current_,
							  s.hardware_error_status_);
			sendMessage(CAN::MESSAGEID_SEND_SERVO_LIST[i], m_slave_id, 8, reinterpret_cast<uint8_t*>(&data), 1);
		}
	}
}

void Servo::receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data)
{
	switch (message_id) {
		case CAN::MESSAGEID_RECEIVE_SERVO_ANGLE:
		{
			for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
				ServoData& s = servo_handler_.getServo()[i];
				//convert int15 to int32
				int sign = data[i * 2 + 1] & 0x40;
				int32_t goal_pos = ((data[i * 2 + 1]  & 0x7F) << 8) | data[i * 2];
				if (sign != 0) {
					goal_pos = 0xFFFF8000 | goal_pos;
				}
				s.setGoalPosition(goal_pos);
				bool torque_enable = (((data[i * 2 + 1] >> 7) & 0x01) != 0) ? true : false;
                                if (!s.force_servo_off_) {
                                  if (s.torque_enable_ != torque_enable) {
                                    s.torque_enable_ = torque_enable;
                                    servo_handler_.setTorque(i);
                                  }
                                }
			}
			break;
		}
		case CAN::MESSAGEID_RECEIVE_SERVO_CURRENT:
		{
			for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
				ServoData& s = servo_handler_.getServo()[i];
				int16_t goal_current = (int16_t)(((data[i * 2 + 1] << 8) & 0xFF00) | (data[i * 2] & 0xFF));
				s.setGoalCurrent(goal_current);
			}
			break;
		}
	}
}
