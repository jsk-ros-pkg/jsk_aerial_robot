/*
 * motor.h
 *
 *  Created on: 2016/10/28
 *      Author: anzai
 *  Maintainer: bakui chou
 */

#ifndef APPLICATION_MOTOR_TEMP_MOTOR_H_
#define APPLICATION_MOTOR_TEMP_MOTOR_H_

#include "CAN/can_device.h"

#define IDLE_DUTY 0.5f // 1000[usec] / 2000[usec]
#define MIN_DUTY_DEFAULT  0.55f   //1100 / 2000
#define MAX_DUTY_DEFAULT 0.95f // 1900 / 2000

class Motor : public CANDevice
{
private:
	  TIM_HandleTypeDef* pwm_htim_;
	  void setPwm(uint16_t pwm);
public:
	Motor(){}
	Motor(uint8_t slave_id):CANDevice(CAN::DEVICEID_MOTOR, slave_id){}
	void init(TIM_HandleTypeDef* htim);
	void update();
	void sendData() override;
	void receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data) override;
};

#endif /* APPLICATION_MOTOR_H_ */
