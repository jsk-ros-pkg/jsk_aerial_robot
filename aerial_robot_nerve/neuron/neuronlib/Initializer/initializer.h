/*
 * configurator.h
 *
 *  Created on: 2017/10/14
 *      Author: anzai
 */

#ifndef APPLICATION_CONFIGURATOR_CONFIGURATOR_H_
#define APPLICATION_CONFIGURATOR_CONFIGURATOR_H_

#include "CAN/can_device_manager.h"
#include "Servo/servo.h"
#include "IMU/imu_mpu9250.h"


class Initializer : public CANDevice
{
private:
	uint16_t& slave_id_;
	Servo& servo_;
	IMU& imu_;
public:
	Initializer(uint16_t& slave_id, Servo& servo, IMU& imu):CANDevice(CAN::DEVICEID_INITIALIZER, slave_id), slave_id_(slave_id), servo_(servo), imu_(imu){}
	void sendData() override;
	void sendBoardConfig();
	void receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data) override;
};



#endif /* APPLICATION_CONFIGURATOR_CONFIGURATOR_H_ */
