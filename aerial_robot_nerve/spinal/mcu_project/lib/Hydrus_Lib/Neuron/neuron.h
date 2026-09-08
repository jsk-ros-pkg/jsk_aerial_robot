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
#include <array>

class Neuron {
private:
	uint8_t slave_id_;
	bool initialized_;
	bool config_receiving_;
	uint8_t expected_config_mask_;
	std::array<uint8_t, CAN::CANFD_MAX_SERVO_NUM> config_fragment_mask_;
	uint32_t config_generation_;
	bool pending_imu_send_data_;
	CANServo pending_can_servo_;

	bool commitConfiguration()
	{
	  if (initialized_ &&
	      can_servo_.servo_.size() != pending_can_servo_.servo_.size()) {
	    config_receiving_ = false;
	    return false;
	  }

	  if (!initialized_) {
	    can_motor_ = CANMotor(slave_id_);
	    can_imu_ = CANIMU(slave_id_, pending_imu_send_data_);
	    can_servo_ = pending_can_servo_;
	  } else {
	    can_imu_.setSendDataFlag(pending_imu_send_data_);
	    can_servo_.setDynamixelTTLRS485Mixed(
	      pending_can_servo_.getDynamixelTTLRS485Mixed());
	    can_servo_.setPulleySkipThresh(
	      pending_can_servo_.getPulleySkipThresh());
	    can_servo_.setServoFeedbackFrequency(
	      pending_can_servo_.getServoFeedbackFrequency());
	    can_servo_.setTargetWatchdogTimeoutMs(
	      pending_can_servo_.getTargetWatchdogTimeoutMs());
	    for (unsigned int i = 0; i < can_servo_.servo_.size(); ++i) {
	      can_servo_.servo_[i].copyConfigurationFrom(
	        pending_can_servo_.servo_[i]);
	    }
	  }

	  initialized_ = true;
	  config_receiving_ = false;
	  config_generation_++;
	  return true;
	}
public:
	Neuron(uint8_t slave_id):slave_id_(slave_id), initialized_(false),
	  config_receiving_(false), expected_config_mask_(0),
	  config_fragment_mask_{}, config_generation_(0),
	  pending_imu_send_data_(false),
	  pending_can_servo_(slave_id, CAN::CANFD_MAX_SERVO_NUM, false, 0){}

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

	void beginConfiguration(uint8_t servo_num, bool imu_send_data,
	                        bool dynamixel_ttl_rs485_mixed,
	                        uint16_t pulley_skip_thresh,
	                        uint16_t feedback_frequency,
	                        uint32_t watchdog_timeout,
	                        bool extended_config)
	{
	  pending_imu_send_data_ = imu_send_data;
	  pending_can_servo_.setServoNum(servo_num);
	  pending_can_servo_.setDynamixelTTLRS485Mixed(
	    dynamixel_ttl_rs485_mixed);
	  pending_can_servo_.setPulleySkipThresh(pulley_skip_thresh);
	  pending_can_servo_.setServoFeedbackFrequency(feedback_frequency);
	  pending_can_servo_.setTargetWatchdogTimeoutMs(watchdog_timeout);
	  for (unsigned int i = 0; i < servo_num; ++i) {
	    pending_can_servo_.servo_[i] = Servo();
	  }
	  config_fragment_mask_.fill(0);
	  expected_config_mask_ = extended_config ? 0x0F : 0x07;
	  config_receiving_ = true;
	  if (servo_num == 0) commitConfiguration();
	}

	Servo* pendingServo(uint8_t servo_index)
	{
	  if (!config_receiving_ ||
	      servo_index >= pending_can_servo_.servo_.size()) return nullptr;
	  return &pending_can_servo_.servo_[servo_index];
	}

	bool markConfigurationFragment(uint8_t servo_index,
	                               uint8_t fragment_bit)
	{
	  if (!config_receiving_ ||
	      servo_index >= pending_can_servo_.servo_.size() ||
	      (fragment_bit & expected_config_mask_) == 0) return false;
	  config_fragment_mask_[servo_index] |= fragment_bit;
	  for (unsigned int i = 0;
	       i < pending_can_servo_.servo_.size(); ++i) {
	    if (config_fragment_mask_[i] != expected_config_mask_) return true;
	  }
	  return commitConfiguration();
	}

	bool getInitialized() const {return initialized_;}
	bool isConfigurationPending() const {return config_receiving_;}
	uint32_t getConfigGeneration() const {return config_generation_;}
};



#endif /* APPLICATION_HYDRUS_LIB_NEURON_NEURON_H_ */
