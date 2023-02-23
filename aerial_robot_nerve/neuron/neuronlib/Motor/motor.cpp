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

void Motor::setPwm(uint16_t pwm)
{
	pwm_htim_->Instance->CCR1 = (uint32_t)((pwm + 1000.0f) / 2000.0f *  pwm_htim_->Init.Period);
	return;
}

void Motor::update()
{
}
