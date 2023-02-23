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
