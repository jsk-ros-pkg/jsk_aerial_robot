/*
 * can_servo.h
 *
 *  Created on: 2016/11/01
 *      Author: anzai
 */

#ifndef APPLICATION_HYDRUS_LIB_CANDEVICE_SERVO_CAN_SERVO_H_
#define APPLICATION_HYDRUS_LIB_CANDEVICE_SERVO_CAN_SERVO_H_

#include "CAN/can_device.h"
#include "cmsis_os.h"

class Servo
{
private:
		uint8_t id_;
		uint8_t index_;
		uint8_t bus_type_;
		uint8_t mode_;
		uint8_t operating_mode_;
		int16_t mit_kp_integer_, mit_kp_decimal_;
		int16_t mit_kd_integer_, mit_kd_decimal_;
	uint16_t mit_current_limit_;
	uint16_t p_gain_, i_gain_, d_gain_;
	int32_t present_position_;
	int32_t present_velocity_;
	int32_t goal_position_;
	int32_t goal_velocity_;
	int16_t goal_current_;
	uint16_t profile_velocity_;
	int16_t present_current_;
	uint16_t current_limit_;
	uint8_t present_temperature_;
	uint8_t moving_;
	uint8_t error_;
	bool send_data_flag_;
	bool torque_command_;
	bool torque_feedback_;
	bool goal_position_ack_;
	bool force_servo_off_;
	volatile bool pvc_updated_;
		bool external_encoder_flag_;
		bool configured_;
		bool watchdog_timed_out_;
	uint16_t joint_resolution_;
	uint16_t servo_resolution_;

	friend class CANServo;
	friend class CANInitializer;
public:
	  Servo():id_(0), index_(0), bus_type_(0xFF), mode_(0xFF),
	          operating_mode_(0xFF), mit_kp_integer_(0), mit_kp_decimal_(0),
	          mit_kd_integer_(0), mit_kd_decimal_(0), mit_current_limit_(0),
	          p_gain_(0), i_gain_(0),
	          d_gain_(0), present_position_(0), present_velocity_(0),
	          goal_position_(0), goal_velocity_(0), goal_current_(0),
	          profile_velocity_(0), present_current_(0), current_limit_(0),
	          present_temperature_(0), moving_(0), error_(0),
	          send_data_flag_(false), torque_command_(false),
	          torque_feedback_(false),
	          goal_position_ack_(false), force_servo_off_(false),
	          pvc_updated_(false),
	          external_encoder_flag_(false), configured_(false),
	          watchdog_timed_out_(false), joint_resolution_(0),
	          servo_resolution_(0) {}
	uint8_t getId() const {return id_;}
	uint8_t getIndex () const {return index_;}
	uint16_t getPGain() const {return p_gain_;}
	uint16_t getIGain() const {return i_gain_;}
	uint16_t getDGain() const {return d_gain_;}
	int32_t getPresentPosition() const {return present_position_;}
	int32_t getPresentVelocity() const {return present_velocity_;}
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
		bool getTorqueCommand() const {return torque_command_;}
		bool getTorqueFeedback() const {return torque_feedback_;}
		bool getForceServoOffFlag() const {return force_servo_off_;}
		bool getPvcUpdated() const {return pvc_updated_;}
		uint8_t getBusType() const {return bus_type_;}
		uint8_t getMode() const {return mode_;}
		uint8_t getOperatingMode() const {return operating_mode_;}
		int16_t getMitKpInteger() const {return mit_kp_integer_;}
		int16_t getMitKpDecimal() const {return mit_kp_decimal_;}
		int16_t getMitKdInteger() const {return mit_kd_integer_;}
		int16_t getMitKdDecimal() const {return mit_kd_decimal_;}
		uint16_t getMitCurrentLimit() const {return mit_current_limit_;}
		bool getConfigured() const {return configured_;}
		bool getWatchdogTimedOut() const {return watchdog_timed_out_;}
	void setIndex(uint8_t index) {index_ = index;}
	void setGoalPosition(int32_t goal_position) {goal_position_ = goal_position;}
	void setGoalVelocity(int32_t goal_velocity) {goal_velocity_ = goal_velocity;}
	void setGoalCurrent(int16_t goal_current) {goal_current_ = goal_current;}
	void clearPvcUpdated() {pvc_updated_ = false;}
	void setTorqueEnable(bool torque_enable);
	void copyConfigurationFrom(const Servo& source)
	{
	  id_ = source.id_;
	  bus_type_ = source.bus_type_;
	  mode_ = source.mode_;
	  operating_mode_ = source.operating_mode_;
	  mit_kp_integer_ = source.mit_kp_integer_;
	  mit_kp_decimal_ = source.mit_kp_decimal_;
	  mit_kd_integer_ = source.mit_kd_integer_;
	  mit_kd_decimal_ = source.mit_kd_decimal_;
	  mit_current_limit_ = source.mit_current_limit_;
	  p_gain_ = source.p_gain_;
	  i_gain_ = source.i_gain_;
	  d_gain_ = source.d_gain_;
	  present_position_ = source.present_position_;
	  goal_position_ = source.goal_position_;
	  goal_current_ = source.goal_current_;
	  profile_velocity_ = source.profile_velocity_;
	  current_limit_ = source.current_limit_;
	  error_ = source.error_;
	  send_data_flag_ = source.send_data_flag_;
	  external_encoder_flag_ = source.external_encoder_flag_;
	  configured_ = source.configured_;
	  joint_resolution_ = source.joint_resolution_;
	  servo_resolution_ = source.servo_resolution_;
	}
};

class CANServo : public CANDevice
{
public:
		CANServo():target_sequence_(0), config_sequence_(0),
		           servo_feedback_frequency_hz_(200),
		           target_watchdog_timeout_ms_(100),
		           config_pending_(false), config_result_(CAN::SERVO_CONFIG_RESULT_BUSY),
		           pending_config_sequence_(0), pending_config_command_(0){}
	  CANServo(uint8_t slave_id, unsigned int servo_num, bool dynamixel_ttl_rs485_mixed, uint16_t thresh):CANDevice(CAN::DEVICEID_SERVO, slave_id), servo_(servo_num), dynamixel_ttl_rs485_mixed_(dynamixel_ttl_rs485_mixed), pulley_skip_thresh_(thresh), target_sequence_(0), config_sequence_(0), servo_feedback_frequency_hz_(200), target_watchdog_timeout_ms_(100), config_pending_(false), config_result_(CAN::SERVO_CONFIG_RESULT_BUSY), pending_config_sequence_(0), pending_config_command_(0){}
	void setServoNum(unsigned int servo_num) {servo_.resize(servo_num);}
		void sendServoConfigRequest();
		uint8_t requestConfig(uint8_t command, const int32_t* data,
		                      uint8_t data_length, uint32_t timeout_ms);
		uint8_t requestConfigByIndex(uint8_t servo_index, uint8_t command,
		                             const int32_t* data, uint8_t data_length,
		                             uint32_t timeout_ms);
	void sendData() override;
	void receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data) override;
	std::vector<Servo> servo_;
	bool getDynamixelTTLRS485Mixed() const {return dynamixel_ttl_rs485_mixed_;}
	void setDynamixelTTLRS485Mixed(bool dynamixel_ttl_rs485_mixed) {dynamixel_ttl_rs485_mixed_ = dynamixel_ttl_rs485_mixed;}
		uint16_t getPulleySkipThresh() const {return pulley_skip_thresh_;}
		void setPulleySkipThresh(uint16_t value) {pulley_skip_thresh_ = value;}
		uint16_t getServoFeedbackFrequency() const {return servo_feedback_frequency_hz_;}
		void setServoFeedbackFrequency(uint16_t value) {servo_feedback_frequency_hz_ = value;}
		uint32_t getTargetWatchdogTimeoutMs() const {return target_watchdog_timeout_ms_;}
		void setTargetWatchdogTimeoutMs(uint32_t value) {target_watchdog_timeout_ms_ = value;}

private:
	bool dynamixel_ttl_rs485_mixed_;
		uint16_t pulley_skip_thresh_;
		uint8_t target_sequence_;
		uint8_t config_sequence_;
		uint16_t servo_feedback_frequency_hz_;
		uint32_t target_watchdog_timeout_ms_;
		volatile bool config_pending_;
		volatile uint8_t config_result_;
		uint8_t pending_config_sequence_;
		uint8_t pending_config_command_;
	struct CANServoData{
		int16_t angle;
		uint8_t temperature;
		uint8_t status;
		int16_t current;
		uint8_t error;
	};
};

#endif /* APPLICATION_HYDRUS_LIB_CANDEVICE_SERVO_CAN_SERVO_H_ */
