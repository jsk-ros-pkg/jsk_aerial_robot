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
#include "Motor/motor.h"


class Initializer : public CANDevice
{
private:
	uint16_t& slave_id_;
	Servo& servo_;
	IMU& imu_;
	Motor& motor_;
	uint8_t board_state_sequence_;
	uint32_t board_state_last_send_tick_;
	std::array<uint32_t, CAN::CANFD_MAX_SERVO_NUM>
	  last_servo_control_count_;
public:
	Initializer(uint16_t& slave_id, Servo& servo, IMU& imu, Motor& motor):
	  CANDevice(CAN::DEVICEID_INITIALIZER, slave_id), slave_id_(slave_id),
	  servo_(servo), imu_(imu), motor_(motor), board_state_sequence_(0),
	  board_state_last_send_tick_(0), last_servo_control_count_{} {}
	void sendData() override;
	bool sendBoardConfig();
	void receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data) override;
};



#endif /* APPLICATION_CONFIGURATOR_CONFIGURATOR_H_ */
