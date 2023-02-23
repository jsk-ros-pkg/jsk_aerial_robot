/*
 * motor.h
 *
 *  Created on: 2016/10/28
 *      Author: anzai
 *  Maintainer: bakui chou
 */

#ifndef APPLICATION_MOTOR_TEMP_MOTOR_H_
#define APPLICATION_MOTOR_TEMP_MOTOR_H_

#if defined(STM32F103xx)
#include "stm32f1xx_hal.h"
#elif defined(STM32F413xx)
#include "stm32f4xx_hal.h"
#elif defined(STM32G473xx)
#include "stm32g4xx_hal.h"
#else
#error "please specify the STM32 series"
#endif


#define IDLE_DUTY 0.5f // 1000[usec] / 2000[usec]
#define MIN_DUTY_DEFAULT  0.55f   //1100 / 2000
#define MAX_DUTY_DEFAULT 0.95f // 1900 / 2000

class Motor
{
private:
	  TIM_HandleTypeDef* pwm_htim_;
	  void setPwm(uint16_t pwm);
public:
	Motor(){}
	Motor(uint8_t slave_id){}
	void init(TIM_HandleTypeDef* htim);
	void update();
};

#endif /* APPLICATION_MOTOR_H_ */
