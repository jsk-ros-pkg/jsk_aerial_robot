/*
******************************************************************************
* File Name          : servo_base_driver.h
* Description        : Base device driver for servo motors
* Author             : J.Sugihara 2025/3/31
******************************************************************************
*/

#include "sensors/encoder/mag_encoder.h"
#include "flashmemory/flashmemory.h"

#ifndef Base_Servo_driver_h
#define Base_Servo_driver_h

#define MAX_SERVO_NUM 10

class ServoData {
public:
	ServoData(){}
  ServoData(uint8_t id): id_(id), torque_enable_(false), first_get_pos_flag_(true), internal_offset_(0){}

	uint8_t id_;
  	int32_t present_position_;
	int32_t goal_position_;
        int32_t calib_value_;
	int32_t homing_offset_;
        int32_t internal_offset_;
        uint8_t present_temp_;
	int16_t present_current_;
	uint8_t moving_;
	uint8_t hardware_error_status_;
	uint16_t p_gain_, i_gain_, d_gain_;
	uint16_t profile_velocity_;
	uint16_t current_limit_;
	uint16_t send_data_flag_;
        uint16_t external_encoder_flag_;
        int32_t joint_offset_;
        uint16_t joint_resolution_;
        uint16_t servo_resolution_;
        float resolution_ratio_;
	bool led_;
	bool torque_enable_;
	bool first_get_pos_flag_;
        float angle_scale_;
        uint16_t zero_point_offset_;

	void updateHomingOffset() { homing_offset_ = calib_value_ - present_position_;}
	void setPresentPosition(int32_t present_position) {present_position_ = present_position + internal_offset_;}
	int32_t getPresentPosition() const {return present_position_;}
	void setGoalPosition(int32_t goal_position) {goal_position_ = resolution_ratio_ * goal_position - internal_offset_;}
        int32_t getGoalPosition() const {return goal_position_;}
        float getAngleScale() const {return angle_scale_;}
        uint16_t getZeroPointOffset() const {return zero_point_offset_;}
  

	bool operator==(const ServoData& r) const {return this->id_ == r.id_;}
};

class ServoBase
{
public:
  ServoBase(){}

  virtual void init(UART_HandleTypeDef* huart,  osMutexId* mutex = NULL) = 0;
  virtual void ping() = 0;
  virtual HAL_StatusTypeDef read(uint8_t* data,  uint32_t timeout) = 0;
  virtual void reboot(uint8_t servo_index) = 0;
  virtual void setTorque(uint8_t servo_index) = 0;
  virtual void setHomingOffset(uint8_t servo_index) = 0;
  virtual void setRoundOffset(uint8_t servo_index, int32_t ref_value) = 0;
  virtual void setPositionGains(uint8_t servo_index) = 0;
  virtual void setProfileVelocity(uint8_t servo_index) = 0;
  virtual void setCurrentLimit(uint8_t servo_index) = 0;
  virtual void update() = 0;
  unsigned int getServoNum() const {return servo_num_;}
  std::array<ServoData, MAX_SERVO_NUM>& getServo() {return servo_;}
  const std::array<ServoData, MAX_SERVO_NUM>& getServo() const {return servo_;}

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
