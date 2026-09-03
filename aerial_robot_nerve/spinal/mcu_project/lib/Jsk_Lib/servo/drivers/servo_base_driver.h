/*
******************************************************************************
* File Name          : servo_base_driver.h
* Description        : Base device driver for servo motors
* Author             : J.Sugihara 2025/3/31
******************************************************************************
*/

#include <array>
#include <stdint.h>

#include "sensors/encoder/mag_encoder.h"
#include "flashmemory/flashmemory.h"

#ifndef Base_Servo_driver_h
#define Base_Servo_driver_h

#define MAX_SERVO_NUM 10

enum ServoResult : int8_t
{
  SERVO_RESULT_OK = 0,
  SERVO_RESULT_BUSY = -1,
  SERVO_RESULT_QUEUE_FULL = -2,
  SERVO_RESULT_TIMEOUT = -3,
  SERVO_RESULT_INVALID_RESPONSE = -4,
  SERVO_RESULT_CRC_ERROR = -5,
  SERVO_RESULT_INVALID_ARGUMENT = -6,
  SERVO_RESULT_UART_ERROR = -7,
};

struct ServoCommCounters
{
  uint32_t transaction_success_count = 0;
  uint32_t timeout_count = 0;
  uint32_t crc_error_count = 0;
  uint32_t invalid_packet_count = 0;
  uint32_t recovery_count = 0;
  uint32_t deadline_miss_count = 0;
  uint32_t queue_overflow_count = 0;
  uint32_t unexpected_completion_count = 0;
  uint16_t queue_high_water_mark = 0;
};

class ServoData {
public:
	ServoData() = default;
  explicit ServoData(uint8_t id): id_(id){}

	uint8_t id_ = 255;
  int32_t present_position_ = 0;
  int32_t present_velocity = 0;
	int32_t goal_position_ = 0;
  int32_t goal_velocity_ = 0;
  int16_t goal_current_ = 0;
  int32_t calib_value_ = 0;
	int32_t homing_offset_ = 0;
  int32_t internal_offset_ = 0;
  uint8_t present_temp_ = 0;
	int16_t present_current_ = 0;
	uint8_t moving_ = 0;
	uint8_t hardware_error_status_ = 0;
	uint16_t p_gain_ = 0, i_gain_ = 0, d_gain_ = 0;
	uint16_t profile_velocity_ = 0;
	uint16_t current_limit_ = 0;
	uint16_t send_data_flag_ = 0;
  uint16_t external_encoder_flag_ = 0;
  int32_t joint_offset_ = 0;
  uint16_t joint_resolution_ = 65535;
  uint16_t servo_resolution_ = 65535;
  float resolution_ratio_ = 1.0f;
	bool led_ = false;
	bool torque_enable_ = false;
	bool first_get_pos_flag_ = true;
  float angle_scale_ = 3.1416f / 2047.0f;
  uint16_t zero_point_offset_ = 2047;
  uint32_t cur_vel_pos_update_count_ = 0;
  uint32_t goal_write_count_ = 0;

	void updateHomingOffset() { homing_offset_ = calib_value_ - present_position_;}
	void setPresentPosition(int32_t present_position) {present_position_ = present_position + internal_offset_;}
	int32_t getPresentPosition() const {return present_position_;}
	void setGoalPosition(int32_t goal_position) {goal_position_ = resolution_ratio_ * goal_position - internal_offset_;}
  int32_t getGoalPosition() const {return goal_position_;}
  int32_t getGoalVelocity() const {return goal_velocity_;}
  int16_t getGoalCurrent() const {return goal_current_;}
  float getAngleScale() const {return angle_scale_;}
  uint16_t getZeroPointOffset() const {return zero_point_offset_;}
  

	bool operator==(const ServoData& r) const {return this->id_ == r.id_;}
};

class ServoBase
{
public:
  ServoBase(){}

  virtual void init(UART_HandleTypeDef* huart,  osMutexId* mutex = NULL) = 0;
  virtual void pinReconfig() = 0;
  virtual void ping() = 0;
  virtual HAL_StatusTypeDef read(uint8_t* data,  uint32_t timeout) = 0;
  virtual void reboot(uint8_t servo_index) = 0;
  virtual void setTorque(uint8_t servo_index) = 0;
  virtual void setTorqueFromPresetnPos(uint8_t servo_index) = 0;
  virtual void setHomingOffset(uint8_t servo_index) = 0;
  virtual void setRoundOffset(uint8_t servo_index, int32_t ref_value) = 0;
  virtual void setPositionGains(uint8_t servo_index) = 0;
  virtual void setProfileVelocity(uint8_t servo_index) = 0;
  virtual void setCurrentLimit(uint8_t servo_index) = 0;
  virtual void update() = 0;
  virtual ServoResult requestConfig(uint8_t command, const int32_t* data,
                                    uint8_t data_length, uint32_t timeout_ms)
  {
    (void)command;
    (void)data;
    (void)data_length;
    (void)timeout_ms;
    return SERVO_RESULT_INVALID_ARGUMENT;
  }
  unsigned int getServoNum() const {return servo_num_;}
  std::array<ServoData, MAX_SERVO_NUM>& getServo() {return servo_;}
  const std::array<ServoData, MAX_SERVO_NUM>& getServo() const {return servo_;}
  const ServoCommCounters& getCommCounters() const {return comm_counters_;}

  void setROSCommFlag(bool flag) {flag_send_ros_ = flag;}
  bool getROSCommFlag() const {return flag_send_ros_;}

protected:
  UART_HandleTypeDef* huart_; // uart handlercmdReadPresentPosition
  osMutexId* mutex_; // for UART (RS485) I/O mutex
  MagEncoder encoder_handler_;
  unsigned int servo_num_;
  std::array<ServoData, MAX_SERVO_NUM> servo_;
  uint32_t set_pos_tick_;
  uint32_t get_pos_tick_;
  uint32_t get_load_tick_;
  uint32_t get_temp_tick_;
  uint32_t get_move_tick_;
  uint32_t get_error_tick_;
  uint16_t ttl_rs485_mixed_;
  ServoCommCounters comm_counters_;

  /* uart rx */
  uint32_t rd_ptr_;

  /* ros comm */
  bool flag_send_ros_ = false;

  virtual inline void setStatusReturnLevel() = 0;
  virtual inline void getHomingOffset() = 0;
  virtual inline void getCurrentLimit() = 0;
  virtual inline void getPositionGains() = 0;
  virtual inline void getProfileVelocity() = 0;
};

#endif
