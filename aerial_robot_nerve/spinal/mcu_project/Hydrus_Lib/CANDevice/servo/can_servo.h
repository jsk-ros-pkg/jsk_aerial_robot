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
	int16_t present_position_;
	int16_t goal_position_;
	uint16_t profile_velocity_;
	uint16_t present_current_;
	uint16_t current_limit_;
	uint8_t present_temperature_;
	uint8_t moving_;
	uint8_t error_;
	bool send_data_flag_;
	bool torque_enable_;
	bool external_encoder_flag_;
	uint16_t joint_resolution_;
	uint16_t servo_resolution_;

	friend class CANServo;
	friend class CANInitializer;
public:
	Servo():torque_enable_(true) {}
	uint8_t getId() const {return id_;}
	uint8_t getIndex () const {return index_;}
	uint16_t getPGain() const {return p_gain_;}
	uint16_t getIGain() const {return i_gain_;}
	uint16_t getDGain() const {return d_gain_;}
	int16_t getPresentPosition() const {return present_position_;}
	uint16_t getProfileVelocity() const {return profile_velocity_;}
	int16_t getPresentCurrent() const {return present_current_;}
	uint16_t getCurrentLimit() const {return current_limit_;}
	bool getSendDataFlag() const {return send_data_flag_;}
	bool getExternalEncoderFlag() const {return external_encoder_flag_;}
	uint16_t getJointResolution() const {return joint_resolution_;}
	uint16_t getServoResolution() const {return servo_resolution_;}
	uint8_t getPresentTemperature() const {return present_temperature_;}
	uint8_t getMoving() const {return moving_;}
	uint8_t getError() const {return error_;}
	bool getTorqueEnable();
	void setIndex(uint8_t index) {index_ = index;}
	void setGoalPosition(int16_t goal_position) {goal_position_ = goal_position;}
	void setTorqueEnable(bool torque_enable) {torque_enable_ = torque_enable;}
};

class CANServo : public CANDevice
{
public:
	CANServo(){}
	CANServo(uint8_t slave_id, unsigned int servo_num, bool dynamixel_ttl_rs485_mixed):CANDevice(CAN::DEVICEID_SERVO, slave_id), servo_(servo_num), dynamixel_ttl_rs485_mixed_(dynamixel_ttl_rs485_mixed){}
	void setServoNum(unsigned int servo_num) {servo_.resize(servo_num);}
	void sendServoConfigRequest();
	void sendData() override;
	void receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data) override;
	std::vector<Servo> servo_;
	bool getDynamixelTTLRS485Mixed() const {return dynamixel_ttl_rs485_mixed_;}
	void setDynamixelTTLRS485Mixed(bool dynamixel_ttl_rs485_mixed) {dynamixel_ttl_rs485_mixed_ = dynamixel_ttl_rs485_mixed;}
private:
	bool dynamixel_ttl_rs485_mixed_;
	struct CANServoData{
		int16_t angle;
		uint8_t temperature;
		uint8_t moving;
		int16_t current;
		uint8_t error;
	};
};

#endif /* APPLICATION_HYDRUS_LIB_CANDEVICE_SERVO_CAN_SERVO_H_ */
