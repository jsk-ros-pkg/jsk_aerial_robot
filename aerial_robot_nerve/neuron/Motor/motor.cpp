/*
 * motor.cpp
 *
 *  Created on: 2016/10/28
 *      Author: anzai
 *  Maintainer : Bakui Chou
 */

#include "motor.h"


void Motor::init(TIM_HandleTypeDef* htim)
{
  pwm_htim_ = htim;
  HAL_TIM_PWM_Start(pwm_htim_,TIM_CHANNEL_1);
}

void Motor::sendData()
{
	return;
}

void Motor::receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data)
{
	auto getPwmFromData = [=](int index)->uint16_t {
		switch (index)
		{
		case 1:
			return ((data[1] << 8) & 0x300) | (data[0] & 0xFF);
		case 2:
			return ((data[2] << 6) & 0x3C0) | ((data[1] >> 2) & 0x3F);
		case 3:
			return ((data[3] << 4) & 0x3F0) | ((data[2] >> 4) & 0x0F);
		case 4:
			return ((data[5] << 8) & 0x300) | (data[4] & 0xFF);
		case 5:
			return ((data[6] << 6) & 0x3C0) | ((data[5] >> 2) & 0x3F);
		case 6:
			return ((data[7] << 4) & 0x3F0) | ((data[6] >> 4) & 0x0F);
		default:
			return 0;
		}
	};
	if (message_id == CAN::MESSAGEID_RECEIVE_PWM_0_5 && 1 <= m_slave_id && m_slave_id <= 6) {
		setPwm(getPwmFromData(m_slave_id));
	} else if (message_id == CAN::MESSAGEID_RECEIVE_PWM_6_11 && 7 <= m_slave_id && m_slave_id <= 12) {
		setPwm(getPwmFromData(m_slave_id - 6));
	}	
}

void Motor::setPwm(uint16_t pwm)
{
	pwm_htim_->Instance->CCR1 = (uint32_t)((pwm + 1000.0f) / 2000.0f *  MAX_PWM);
	return;
}
