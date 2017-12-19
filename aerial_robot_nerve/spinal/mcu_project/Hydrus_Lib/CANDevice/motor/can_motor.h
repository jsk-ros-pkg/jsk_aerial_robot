/*
 * can_motor.h
 *
 *  Created on: 2016/11/01
 *      Author: anzai
 */

#ifndef APPLICATION_HYDRUS_LIB_CANDEVICE_MOTOR_CAN_MOTOR_H_
#define APPLICATION_HYDRUS_LIB_CANDEVICE_MOTOR_CAN_MOTOR_H_

#include "CAN/can_device.h"
#include <functional>

class CANMotor : public CANDevice
{
private:
	uint16_t m_pwm;
public:
	CANMotor(){}
	CANMotor(uint8_t slave_id) : CANDevice(CAN::DEVICEID_MOTOR, slave_id), m_pwm(0){}
	void sendData() override;
	void receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data) override;
	void setPwm(uint16_t pwm){m_pwm = pwm;}
	uint16_t getPwm()const {return m_pwm;}
};

class CANMotorSendDevice : public CANDevice
{
private:
	std::vector<std::reference_wrapper<CANMotor>> can_motor_;
public:
	CANMotorSendDevice():CANDevice(CAN::DEVICEID_MOTOR, CAN::BROADCAST_ID) {}
	void sendData() override;
	void receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data) override;
	void addMotor(CANMotor& motor) {can_motor_.push_back(motor);}
};



#endif /* APPLICATION_HYDRUS_LIB_CANDEVICE_MOTOR_CAN_MOTOR_H_ */
