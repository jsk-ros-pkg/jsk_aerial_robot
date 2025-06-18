/*
  How Dynamixel work can be found
  http://www.besttechnology.co.jp/modules/knowledge
  http://www.besttechnology.co.jp/modules/knowledge/?MX%20Series%20Control%20table
*/

#ifndef Dynamixel_Serial_h
#define Dynamixel_Serial_h


#include <inttypes.h>
#include <stdlib.h>
#include <array>
#include <algorithm>
#include <utility>
#include <cmath>
#include "flashmemory.h"
#include "can_core.h"
#include "Encoder/mag_encoder.h"
#include "cmsis_os.h"


//#########################################################################

//################ define - Dynamixel Model table ############################
#define XC330_M077	0x4A6
#define XC330_M288	0x4B0
#define XC330_M181	0x4B0
#define XC330_T181	0x4BA
#define XC330_T288	0x4C4
#define XL430_W250	0x4CE
#define DXL430_W250	0x442 // 2XL430_W250
#define XC430_W150	0x42E
#define XC430_W250	0x438
#define DXC430_W250	0x488 // 2XC430_W250
#define XM430_W210	0x406
#define XH430_W210	0x3F2
#define XH430_V210	0x41A
#define XD430_T210	0x3F3
#define XM430_W350	0x3FC
#define XH430_W350	0x3E8
#define XH430_V350	0x410
#define XD430_T350	0x3E9
#define XW430_T200	0x500
#define XW430_T333	0x4F6
#define XM540_W150	0x46A
#define XH540_W150	0x456
#define XH540_V150	0x47E
#define XM540_W270	0x460
#define XH540_W270	0x44C
#define XH540_V270	0x474
#define XW540_T140	0x49C
#define XW540_T260	0x492
#define MX_28	0x1E
#define MX_64	0x137
#define MX_106	0x141

//################ define - Dynamixel operating mode table ######################
#define CURRENT_CONTROL_MODE 0
#define VELOCITY_CONTROL_MODE 1
#define POSITION_CONTROL_MODE 3
#define EXTENDED_POSITION_CONTROL_MODE 4
#define CURRENT_BASE_POSITION_CONTROL_MODE 5
#define PWM_CONTROL_MODE 16

//################ define - Dynamixel Hex control table ######################

#define CTRL_MODEL_NUMBER             	0x00
#define CTRL_MODEL_INFORMATION        	0x02
#define CTRL_VERSTION_OF_FIRMWARE     	0x06
#define CTRL_ID                       	0x07
#define CTRL_BAUD_RATE                	0x08
#define CTRL_RETURN_DELAY_TIME        	0x09
#define CTRL_DRIVE_MODE 		      	0x0a
#define CTRL_OPERATING_MODE           	0x0b
#define CTRL_SECONDARY_ID             	0x0c
#define CTRL_PROTOCOL_VERSTION        	0x0d
#define CTRL_HOMING_OFFSET            	0x14
#define CTRL_MOVING_THRESHOLD	      	0x18
#define CTRL_TEMPERATURE_LIMIT        	0x1f
#define CTRL_MAX_VOLTAGE_LIMIT        	0x20
#define CTRL_MIN_VOLTAGE_LIMIT        	0x22
#define CTRL_PWM_LIMIT                	0x24
#define CTRL_CURRENT_LIMIT            	0x26
#define CTRL_ACCELERATION_LIMIT       	0x28
#define CTRL_VELOCITY_LIMIT           	0x2c
#define CTRL_MAX_POSITION_LIMIT       	0x30
#define CTRL_MIN_POSITION_LIMIT       	0x34
#define CTRL_SHUTDOWN                 	0x3F
#define CTRL_TORQUE_ENABLE            	0x40
#define CTRL_LED					  	0x41
#define CTRL_STATUS_RETURN_LEVEL      	0x44
#define CTRL_REGISTERED_INSTRUCTION   	0x45
#define CTRL_HARDWARE_ERROR_STATUS    	0x46
#define CTRL_VELOCITY_I_GAIN          	0x4c
#define CTRL_VELOCITY_P_GAIN          	0x4e
#define CTRL_POSITION_D_GAIN          	0x50
#define CTRL_POSITION_I_GAIN          	0x52
#define CTRL_POSITION_P_GAIN          	0x54
#define CTRL_FEEDFORWARD_SECONDGAIN   	0x58
#define CTRL_FEEDFORWARD_FIRSTGAIN    	0x5a
#define CTRL_BUS_WATCHDOG		      	0x62
#define CTRL_GOAL_PWM				  	0x64
#define CTRL_GOAL_CURRENT			  	0x66
#define CTRL_GOAL_VELOCITY			  	0x68
#define CTRL_PROFILE_ACCELERATION	  	0x6c
#define CTRL_PROFILE_VELOCITY		  	0x70
#define CTRL_GOAL_POSITION			  	0x74
#define CTRL_REALTIME_TICK			  	0x78
#define CTRL_MOVING					  	0x7a
#define CTRL_MOVING_STATUS			  	0x7b
#define CTRL_PRESENT_PWM			  	0x7c
#define CTRL_PRESENT_CURRENT		  	0x7e
#define CTRL_PRESENT_VELOCITY		  	0x80
#define CTRL_PRESENT_POSITION	      	0x84
#define CTRL_VELOCITY_TRAJECTORY      	0x88
#define CTRL_POSITION_TRAJECTORY	  	0x8c
#define CTRL_PRESENT_INPUT_VOLTAGE    	0x90
#define CTRL_PRESENT_TEMPERATURE	  	0x92

//################ Instruction commands Set ###############################
#define COMMAND_PING               		0x01
#define COMMAND_READ                 	0x02
#define COMMAND_WRITE   	        	0x03
#define COMMAND_REG_WRITE		      	0x04
#define COMMAND_ACTION              	0x05
#define COMMAND_FACTORY_RESET           0x06
#define COMMAND_REBOOT					0x08
#define COMMAND_SYNC_READ				0x82
#define COMMAND_SYNC_WRITE          	0x83
#define COMMAND_BULK_READ				0x92
#define COMMAND_BULK_WRITE				0x93

//################ Instruction packet lengths #############################
#define PING_LEN				3
#define READ_INST_LEN				7
#define HOMING_OFFSET_BYTE_LEN			4
#define SET_STATUS_RETURN_LEVEL_LEN		3
#define REBOOT_LEN				3
#define TORQUE_ENABLE_BYTE_LEN			1
#define LED_BYTE_LEN				1
#define STATUS_RETURN_LEVEL_BYTE_LEN		1
#define GOAL_POSITION_BYTE_LEN			4
#define GOAL_CURRENT_BYTE_LEN			2
#define PRESENT_POSITION_BYTE_LEN		4
#define PRESENT_CURRENT_BYTE_LEN		2
#define PRESENT_TEMPERATURE_BYTE_LEN		1
#define MOVING_BYTE_LEN 			1
#define OPERATING_MODE_BYTE_LEN 		1
#define MODEL_NUMBER_BYTE_LEN 			2
#define HARDWARE_ERROR_STATUS_BYTE_LEN		1
#define POSITION_GAINS_BYTE_LEN			6
#define PROFILE_VELOCITY_BYTE_LEN		4
#define CURRENT_LIMIT_BYTE_LEN			2
#define SHUTDOWN_BYTE_LEN			1
#define RETURN_DELAY_TIME_BYTE_LEN		1
#define TEMPERATURE_LIMIT_BYTE_LEN		1

//#########################################################################
//############################ Specials ###################################

#define NONE					0x00
#define READ					0x01
#define ALL					0x02

#define BROADCAST_ID           	0xFE

#define HEADER0 				0xFF
#define HEADER1 				0xFF
#define HEADER2 				0xFD
#define HEADER3 				0x00
#define EXCEPTION_ADDITIONAL_BYTE 0xFD

#define INSTRUCTION_PACKET_SIZE 64
#define STATUS_PACKET_SIZE 		64

#define MAX_SERVO_NUM			4
#define PING_TRIAL_NUM			100

//read status packet
#define READ_HEADER0				0
#define READ_HEADER1				1
#define READ_HEADER2				2
#define READ_HEADER3				3
#define READ_SERVOID				4
#define READ_LENL				5
#define READ_LENH				6
#define READ_INSTRUCTION			7
#define READ_ERROR				8
#define READ_PARAMETER				9
#define READ_CHECKSUML 				10
#define READ_CHECKSUMH				11
#define STATUS_PACKET_INSTRUCTION		0x55
#define NO_ERROR				0

//error
#define INPUT_VOLTAGE_ERROR			0
#define OVERHEATING_ERROR			2
#define MOTOR_ENCODER_ERROR			3
#define ELECTRICAL_SHOCK_ERROR			4
#define OVERLOAD_ERROR				5
//addintional error status for external encoder
#define PULLEY_SKIP_ERROR			1
#define RESOLUTION_RATIO_ERROR			6
#define ENCODER_CONNECT_ERROR			7


//for instruction buffer
#define INST_GET_CURRENT_LIMIT			0
#define INST_GET_HARDWARE_ERROR_STATUS		1
#define INST_GET_HOMING_OFFSET			2
#define INST_GET_POSITION_GAINS			3
#define INST_GET_PRESENT_CURRENT		4
#define INST_GET_PRESENT_MOVING			5
#define INST_GET_PRESENT_POS			6
#define INST_GET_PRESENT_TEMPERATURE		7
#define INST_GET_PROFILE_VELOCITY		8
#define INST_PING				9
#define INST_SET_CURRENT_LIMIT			10
#define INST_SET_GOAL_COMMAND			11
#define INST_SET_HOMING_OFFSET			12
#define INST_SET_POSITION_GAINS			13
#define INST_SET_PROFILE_VELOCITY		14
#define INST_SET_TORQUE				15
#define INST_GET_MODEL_NUMBER			16
#define INST_GET_OPERATING_MODE			17

//instruction frequency: 0 means no process
#define SET_COMMAND_DU 20 //[msec], 20ms => 50Hz
#define SET_COMMAND_OFFSET 0 // offset from SET_COMMAND
#define GET_POS_DU 20 //[msec], 20ms => 50Hz
#define GET_POS_OFFSET 10 //offset from SET_COMMAND
#define GET_LOAD_DU 200 //[msec], 200ms => 5Hz
#define GET_LOAD_OFFSET 0 //offset from SET_COMMAND
#define GET_TEMP_DU 200 //[msec], 200ms => 5Hz
#define GET_TEMP_OFFSET 50 //offset from SET_COMMAND
#define GET_MOVE_DU 200 //[msec], 200ms => 5Hz
#define GET_MOVE_OFFSET 100 //offset from SET_COMMAND
#define GET_HARDWARE_ERROR_STATUS_DU 200 //[msec], 200ms => 5Hz
#define GET_HARDWARE_ERROR_STATUS_OFFSET 150 //offset from SET_COMMAND

// pre-defined configuration for dynamixel
#define TEMPERATURE_LIMIT			60
#define RETURN_DELAY_TIME			25
#define SHUTDOWN_BIT				1 << OVERHEATING_ERROR | 1 << ELECTRICAL_SHOCK_ERROR


/* please define the gpio which control the IO direction */
#define WE HAL_GPIO_WritePin(RS485EN_GPIO_Port, RS485EN_Pin, GPIO_PIN_SET);
#define RE HAL_GPIO_WritePin(RS485EN_GPIO_Port, RS485EN_Pin, GPIO_PIN_RESET);

/* DMA circular rx buffer size */
#define RX_BUFFER_SIZE 512

template <typename T,  int SIZE>
class RingBuffer
{
public:
  RingBuffer()
  {
    byte_in_progress_ = 0;
    byte_to_add_ = 0;
    buffer_length_ = (uint16_t)SIZE;
  }
  ~RingBuffer(){  }

  bool pop(T& pop_value)
  {
    if (byte_in_progress_ != byte_to_add_)
      {
        pop_value =  buf_[byte_in_progress_];

        byte_in_progress_++;
        if (byte_in_progress_ == buffer_length_)
          byte_in_progress_ = 0;

        return true;
      }
    return false;
  }

  bool push(T new_value)
  {
    // the pop process should have higher priority than the push process
    if ((byte_in_progress_ == (byte_to_add_ + 1)) || ( (byte_to_add_ == (buffer_length_ - 1) )&& (byte_in_progress_ == 0)) ) return false;

    buf_[byte_to_add_] = new_value;

    byte_to_add_++;

    if (byte_to_add_ == buffer_length_)
      {
        byte_to_add_ = 0;
      }
    return true;
  }

  uint16_t length()
  {
    if(byte_to_add_ - byte_in_progress_ >= 0)
      return (byte_to_add_ - byte_in_progress_);
    else 
      return (byte_to_add_ + (buffer_length_ - byte_in_progress_));
  }

private:
  T buf_[SIZE];
  int16_t byte_in_progress_, byte_to_add_;
  uint16_t buffer_length_;
};

class ServoData {
public:
	ServoData(){}
	ServoData(uint8_t id):
          id_(id),
          goal_current_(0),
          internal_offset_(0),
          pulley_skip_time_(0),
          pulley_skip_reset_du_(1000),
          operating_mode_(0),
          hardware_error_status_(0),
          torque_enable_(false),
          force_servo_off_(false),
          send_goal_position_(false),
          first_get_pos_flag_(true)
  {}

	uint8_t id_;
  	int32_t present_position_;
	int32_t goal_position_;
  	int16_t goal_current_;
        int32_t calib_value_;
	int32_t homing_offset_;
        int32_t internal_offset_;
	uint32_t pulley_skip_time_;
	uint32_t pulley_skip_reset_du_;
	uint8_t present_temp_;
	int16_t present_current_;
	uint8_t moving_;
  	uint16_t model_number_;
	uint8_t shutdown_bit_;
	uint8_t operating_mode_;
	uint8_t hardware_error_status_;
	uint16_t p_gain_, i_gain_, d_gain_;
	uint16_t profile_velocity_;
	uint16_t current_limit_;
	uint16_t send_data_flag_;
        uint16_t external_encoder_flag_;
        int32_t joint_offset_;
        int16_t joint_resolution_;
        int16_t servo_resolution_;
        float resolution_ratio_;
	bool led_;
	bool torque_enable_;
  	bool force_servo_off_;
	bool send_goal_position_;
	bool first_get_pos_flag_;

	void updateHomingOffset() { homing_offset_ = calib_value_ - present_position_;}
	int32_t getPresentPosition() const {return present_position_;}
	void setGoalPosition(int32_t goal_position) {goal_position_ = goal_position;}
  	int32_t getInternalGoalPosition() const {return resolution_ratio_ * goal_position_ - internal_offset_;}
	int32_t getGoalPosition() const {return goal_position_;}
	void setGoalCurrent(int16_t goal_current) {goal_current_ = goal_current;}
	int16_t getGoalCurrent() const { return goal_current_;}
	bool operator==(const ServoData& r) const {return this->id_ == r.id_;}
};

class DynamixelSerial
{
public:
  DynamixelSerial(){}

  void init(UART_HandleTypeDef* huart, I2C_HandleTypeDef* hi2c, osMutexId* mutex = NULL);
  void ping();
  HAL_StatusTypeDef read(uint8_t* data,  uint32_t timeout);
  void reboot(uint8_t servo_index);
  void setTorque(uint8_t servo_index, bool torque_enable);
  void setHomingOffset(uint8_t servo_index);
  void setRoundOffset(uint8_t servo_index, int32_t ref_value);
  void setPositionGains(uint8_t servo_index);
  void setProfileVelocity(uint8_t servo_index);
  void setCurrentLimit(uint8_t servo_index);
  void update();
  unsigned int getServoNum() const {return servo_num_;}
  uint16_t getTTLRS485Mixed() const {return ttl_rs485_mixed_;}
  void setTTLRS485Mixed(uint16_t flag) {ttl_rs485_mixed_ = flag;}
  uint16_t getPulleySkipThresh() const {return pulley_skip_thresh_;}
  void setPulleySkipThresh(uint16_t value) {pulley_skip_thresh_ = value;}
  std::array<ServoData, MAX_SERVO_NUM>& getServo() {return servo_;}
  const std::array<ServoData, MAX_SERVO_NUM>& getServo() const {return servo_;}

private:
  UART_HandleTypeDef* huart_; // uart handlercmdReadPresentPosition
  osMutexId* mutex_; // for UART (RS485) I/O mutex
  MagEncoder encoder_handler_;
  RingBuffer<std::pair<uint8_t, uint8_t>, 64> instruction_buffer_;
  unsigned int servo_num_;
  std::array<ServoData, MAX_SERVO_NUM> servo_;
  uint16_t ttl_rs485_mixed_;
  uint16_t pulley_skip_thresh_;
  uint32_t set_command_tick_;
  uint32_t get_pos_tick_;
  uint32_t get_load_tick_;
  uint32_t get_temp_tick_;
  uint32_t get_move_tick_;
  uint32_t get_error_tick_;

  /* uart rx */
  uint8_t rx_buf_[RX_BUFFER_SIZE];
  uint32_t rd_ptr_;

  void transmitInstructionPacket(uint8_t id, uint16_t len, uint8_t instruction, uint8_t* parameters);
  int8_t readStatusPacket(uint8_t status_packet_instruction);

  void cmdPing(uint8_t id);
  void cmdReboot(uint8_t id);
  void cmdRead(uint8_t id, uint16_t address, uint16_t byte_size);
  void cmdWrite(uint8_t id, uint16_t address, uint8_t* param, int param_len);
  void cmdSyncRead(uint16_t address, uint16_t byte_size, bool send_all);
  void cmdSyncWrite(uint16_t address, uint8_t* param, int param_len);

  inline void cmdReadCurrentLimit(uint8_t servo_index);
  inline void cmdReadHardwareErrorStatus(uint8_t servo_index);
  inline void cmdReadHomingOffset(uint8_t servo_index);
  inline void cmdReadMoving(uint8_t servo_index);
  inline void cmdReadModelNumber(uint8_t servo_index);
  inline void cmdReadOperatingMode(uint8_t servo_index);
  inline void cmdReadPositionGains(uint8_t servo_index);
  inline void cmdReadPresentCurrent(uint8_t servo_index);
  inline void cmdReadPresentPosition(uint8_t servo_index);
  inline void cmdReadPresentTemperature(uint8_t servo_index);
  inline void cmdReadProfileVelocity(uint8_t servo_index);
  inline void cmdWriteCurrentLimit(uint8_t servo_index);
  inline void cmdWriteHomingOffset(uint8_t servo_index);
  inline void cmdWritePositionGains(uint8_t servo_index);
  inline void cmdWriteProfileVelocity(uint8_t servo_index);
  inline void cmdWriteStatusReturnLevel(uint8_t id, uint8_t set);
  inline void cmdWriteTorqueEnable(uint8_t servo_index);
  inline void cmdWriteGoalCurrent(uint8_t servo_index);
  inline void cmdSyncReadCurrentLimit(bool send_all = true);
  inline void cmdSyncReadHardwareErrorStatus(bool send_all = true);
  inline void cmdSyncReadHomingOffset(bool send_all = true);
  inline void cmdSyncReadMoving(bool send_all = true);
  inline void cmdSyncReadModelNumber(bool send_all = true);
  inline void cmdSyncReadOperatingMode(bool send_all = true);
  inline void cmdSyncReadPositionGains(bool send_all = true);
  inline void cmdSyncReadPresentCurrent(bool send_all = true);
  inline void cmdSyncReadPresentPosition(bool send_all = true);
  inline void cmdSyncReadPresentTemperature(bool send_all = true);
  inline void cmdSyncReadProfileVelocity(bool send_all = true);
  inline void cmdSyncWriteGoalPosition();
  inline void cmdSyncWriteGoalCurrent();
  inline void cmdSyncWriteLed();
  inline void cmdSyncWritePositionGains();
  inline void cmdSyncWriteProfileVelocity();
  inline void cmdSyncWriteTorqueEnable();
  inline void cmdSyncWriteShutdownBit();
  inline void cmdSyncWriteReturnDelayTime();
  inline void cmdSyncWriteTemperatureLimit();

  inline void setStatusReturnLevel();
  inline void getHomingOffset();
  inline void getCurrentLimit();
  inline void getPositionGains();
  inline void getProfileVelocity();
  inline void getModelNumber();
  inline void getOperatingMode();

  uint16_t calcCRC16(uint16_t crc_accum, uint8_t *data_blk_ptr, int data_blk_size);
};

#endif
