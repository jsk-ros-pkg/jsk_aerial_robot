/*
 * can_servo.cpp
 *
 *  Created on: 2016/11/01
 *      Author: anzai
 */

#include "can_servo.h"

void CANServo::sendData()
{
	uint16_t target_angle[4];
	for (unsigned int i = 0 ; i < servo_.size(); i++) {
		target_angle[i] = ((servo_[i].torque_enable_ ? 1 : 0) << 15) | (servo_[i].goal_position_ & 0xFFF);
	}
	setMessage(CAN::MESSAGEID_RECEIVE_SERVO_ANGLE, m_slave_id, servo_.size() * 2, reinterpret_cast<uint8_t*>(target_angle));
	sendMessage(0);
}

void CANServo::receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data)
{
	CANServoData servo_data = *(reinterpret_cast<CANServoData*>(data));
	servo_[message_id].present_position_ = servo_data.angle;
	servo_[message_id].present_temperature_ = servo_data.temperature;
	servo_[message_id].moving_ = servo_data.moving;
	servo_[message_id].present_current_ = servo_data.current;
	servo_[message_id].error_ = servo_data.error;
}
