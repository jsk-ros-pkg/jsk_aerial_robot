/*
 * can_motor.cpp
 *
 *  Created on: 2016/11/01
 *      Author: anzai
 */

#include "can_motor.h"

void CANMotor::sendData()
{
	/*

	*/
}

void CANMotor::receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data)
{
	return;
}

void CANMotorSendDevice::sendData()
{
	uint8_t pwm_data[8] = {};
	uint16_t motor_pwms[6] = {};

	auto parse = [&]{
		pwm_data[0] = motor_pwms[0] & 0xFF;
		pwm_data[1] = ((motor_pwms[1] << 2) & 0xFC) | ((motor_pwms[0] >> 8) & 0x03);
		pwm_data[2] = ((motor_pwms[2] << 4) & 0xF0) | ((motor_pwms[1] >> 6) & 0x0F);
		pwm_data[3] = ((motor_pwms[2] >> 4) & 0x3F);
		pwm_data[4] = motor_pwms[3] & 0xFF;
		pwm_data[5] = ((motor_pwms[4] << 2) & 0xFC) | ((motor_pwms[3] >> 8) & 0x03);
		pwm_data[6] = ((motor_pwms[5] << 4) & 0xF0) | ((motor_pwms[4] >> 6) & 0x0F);
		pwm_data[7] = ((motor_pwms[5] >> 4) & 0x3F);
	};

	unsigned int send_motor_num = std::min(static_cast<int>(can_motor_.size()), 6);
	for (unsigned int i = 0; i < send_motor_num; i++) {
		motor_pwms[i] = can_motor_.at(i).get().getPwm();
	}

	parse();
	sendMessage(CAN::MESSAGEID_RECEIVE_PWM_0_5, CAN::BROADCAST_ID, 8, pwm_data, 0);


	if (can_motor_.size() <= 6) return;

	send_motor_num = std::min(static_cast<int>(can_motor_.size() - 6), 6);
	for (unsigned int i = 0; i < send_motor_num; i++) {
		motor_pwms[i] = can_motor_.at(i + 6).get().getPwm();
	}

	parse();
	sendMessage(CAN::MESSAGEID_RECEIVE_PWM_6_11, CAN::BROADCAST_ID, 8, pwm_data, 0);
}

void CANMotorSendDevice::receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data)
{
	return;
}
