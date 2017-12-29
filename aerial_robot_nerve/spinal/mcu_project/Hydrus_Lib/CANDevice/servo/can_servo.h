/*
 * can_servo.h
 *
 *  Created on: 2016/11/01
 *      Author: anzai
 */

#ifndef APPLICATION_HYDRUS_LIB_CANDEVICE_SERVO_CAN_SERVO_H_
#define APPLICATION_HYDRUS_LIB_CANDEVICE_SERVO_CAN_SERVO_H_

#include "CAN/can_device.h"

class Servo
{
private:
	uint8_t id_;
	uint8_t index_;
	uint16_t p_gain_, i_gain_, d_gain_;
	uint16_t present_position_;
	uint16_t goal_position_;
	uint16_t profile_velocity_;
	uint16_t present_current_;
	uint16_t current_limit_;
	uint8_t present_temperature_;
	uint8_t moving_;
	uint8_t error_;
	bool send_data_flag_;
	bool torque_enable_;

	friend class CANServo;
	friend class CANInitializer;
public:
	uint8_t getId() const {return id_;}
	uint8_t getIndex () const {return index_;}
	uint16_t getPGain() const {return p_gain_;}
	uint16_t getIGain() const {return i_gain_;}
	uint16_t getDGain() const {return d_gain_;}
	uint16_t getPresentPosition() const {return present_position_;}
	uint16_t getProfileVelocity() const {return profile_velocity_;}
	int16_t getPresentCurrent() const {return present_current_;}
	uint16_t getCurrentLimit() const {return current_limit_;}
	bool getSendDataFlag() const {return send_data_flag_;}
	uint8_t getPresentTemperature() const {return present_temperature_;}
	uint8_t getMoving() const {return moving_;}
	uint8_t getError() const {return error_;}
	void setIndex(uint8_t index) {index_ = index;}
	void setGoalPosition(uint16_t goal_position) {goal_position_ = goal_position;}
	void setTorqueEnable(bool torque_enable) {torque_enable_ = torque_enable;}
};

class CANServo : public CANDevice
{
public:
	CANServo(){}
	CANServo(uint8_t slave_id, unsigned int servo_num):CANDevice(CAN::DEVICEID_SERVO, slave_id), servo_(servo_num){}
	void setServoNum(unsigned int servo_num) {servo_.resize(servo_num);}
	void sendServoConfigRequest();
	void sendData() override;
	void receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data) override;
	std::vector<Servo> servo_;
private:
	struct CANServoData{
		uint16_t angle;
		uint8_t temperature;
		uint8_t moving;
		int16_t current;
		uint8_t error;
	};
};

#endif /* APPLICATION_HYDRUS_LIB_CANDEVICE_SERVO_CAN_SERVO_H_ */
