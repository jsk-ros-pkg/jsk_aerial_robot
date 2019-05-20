/*
 * neuron.h
 *
 *  Created on: 2017/10/26
 *      Author: anzai
 */

#ifndef APPLICATION_HYDRUS_LIB_NEURON_NEURON_H_
#define APPLICATION_HYDRUS_LIB_NEURON_NEURON_H_

#include <CANDevice/imu/can_imu_mpu9250.h>
#include <CANDevice/motor/can_motor.h>
#include <CANDevice/servo/can_servo.h>

class Neuron {
private:
	uint8_t slave_id_;
	bool initialized_;
public:
	Neuron(uint8_t slave_id):slave_id_(slave_id), initialized_(false){}

	CANMotor can_motor_;
	CANIMU can_imu_;
	CANServo can_servo_;

	bool operator<(const Neuron& right) const
	{
		return this->slave_id_ < right.slave_id_;
	}

	bool operator==(const Neuron& right) const
	{
		return this->slave_id_ == right.slave_id_;
	}

	uint8_t getSlaveId() const {return slave_id_;}

	void setInitialized() {initialized_ = true;}
	bool getInitialized() {return initialized_;}
};



#endif /* APPLICATION_HYDRUS_LIB_NEURON_NEURON_H_ */
