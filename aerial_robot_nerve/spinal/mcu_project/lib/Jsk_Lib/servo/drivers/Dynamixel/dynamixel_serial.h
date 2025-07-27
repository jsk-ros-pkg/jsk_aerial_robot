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
// #include "flashmemory.h"
// #include "sensors/encoder/mag_encoder.h"
#include "sensors/encoder/mag_encoder.h"
#include "cmsis_os.h"

#include <string.h>
#include "config.h"

#include "servo/drivers/servo_base_driver.h"

/* first should set the baudrate to 1000000*/
/* uncomment following macro, and set the uart baudrate to 57143(M
 * X28), then the buadrate will change */
/* then comment following macro, change the baudrate back to 1000000 */

/* second, should set some setting to the servo */
/* 1. the responce delay: which is very important for MX28AR,
 *  uncomment the INIT_SETTING marcro, and comment again after the setting
 *  we observed that the RX of uart will turn to 0, after the tx process
 *  therefore, we set the delay of rx after rx to be 0 => no 0V after rx
 *  2. the voltage: we set the voltage limit according to the battery(4s: 17V)
 */

//#define SET_BAUD
//#define SET_ID
//#define PING_TEST
//#define LED_TEST
//#define INIT_SETTING
//#define SERVO_TEST
//#define WHEEL_TEST
//#define SET_HOMING_OFFSET

//#########################################################################
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
#define PING_LEN						3
#define READ_INST_LEN					7
#define HOMING_OFFSET_BYTE_LEN			4
#define SET_STATUS_RETURN_LEVEL_LEN		3
#define REBOOT_LEN						3
#define TORQUE_ENABLE_BYTE_LEN			1
#define LED_BYTE_LEN					1
#define STATUS_RETURN_LEVEL_BYTE_LEN	1
#define GOAL_POSITION_BYTE_LEN			4
#define PRESENT_POSITION_BYTE_LEN		4
#define PRESENT_CURRENT_BYTE_LEN		2
#define PRESENT_TEMPERATURE_BYTE_LEN	1
#define MOVING_BYTE_LEN 				1
#define HARDWARE_ERROR_STATUS_BYTE_LEN	1
#define POSITION_GAINS_BYTE_LEN			6
#define PROFILE_VELOCITY_BYTE_LEN		4
#define CURRENT_LIMIT_BYTE_LEN			2

//#########################################################################
//############################ Specials ###################################

#define NONE					0x00
#define READ					0x01
#define ALL						0x02

#define DX_BROADCAST_ID           	0xFE

#define HEADER0 				0xFF
#define HEADER1 				0xFF
#define HEADER2 				0xFD
#define HEADER3 				0x00
#define EXCEPTION_ADDITIONAL_BYTE 0xFD

#define INSTRUCTION_PACKET_SIZE 64
#define STATUS_PACKET_SIZE 		64
#define PING_TRIAL_NUM			100

//read status packet
#define READ_HEADER0					0
#define READ_HEADER1					1
#define READ_HEADER2					2
#define READ_HEADER3					3
#define READ_SERVOID					4
#define READ_LENL						5
#define READ_LENH						6
#define READ_INSTRUCTION				7
#define READ_ERROR						8
#define READ_PARAMETER		 			9
#define READ_CHECKSUML 					10
#define READ_CHECKSUMH					11
#define STATUS_PACKET_INSTRUCTION		0x55

//error
#define ERROR_NO_ERROR					0
#define ERROR_RESULT_FAIL				1
#define ERROR_INSTRUCTION_ERROR			2
#define ERROR_CRC_ERROR					3
#define ERROR_DATA_RANGE_ERROR			4
#define ERROR_DATA_LENGTH_ERROR			5
#define ERROR_DATA_LIMIT_ERROR			6
#define ERROR_ACCESS_ERROR				7
//addintional error status for external encoder
#define RESOLUTION_RATIO_ERROR 6
#define ENCODER_CONNECT_ERROR 7


//for instruction buffer
#define INST_GET_CURRENT_LIMIT			0
#define INST_GET_HARDWARE_ERROR_STATUS  1
#define INST_GET_HOMING_OFFSET			2
#define INST_GET_POSITION_GAINS			3
#define INST_GET_PRESENT_CURRENT		4
#define INST_GET_PRESENT_MOVING			5
#define INST_GET_PRESENT_POS			6
#define INST_GET_PRESENT_TEMPERATURE	7
#define INST_GET_PROFILE_VELOCITY		8
#define INST_PING						9
#define INST_SET_CURRENT_LIMIT			10
#define INST_SET_GOAL_POS 				11
#define INST_SET_HOMING_OFFSET			12
#define INST_SET_POSITION_GAINS			13
#define INST_SET_PROFILE_VELOCITY		14
#define INST_SET_TORQUE					15

//instruction frequency: 0 means no process
#define SET_POS_DU 20 //[msec], 20ms => 50Hz
#define SET_POS_OFFSET 0 // offset from SET_POS
#define GET_POS_DU 20 //[msec], 20ms => 50Hz
#define GET_POS_OFFSET 10 //offset from GET_POS
#define GET_LOAD_DU 200 //[msec], 200ms => 5Hz
#define GET_LOAD_OFFSET 0 //offset from GET_LOAD
#define GET_TEMP_DU 200 //[msec], 200ms => 5Hz
#define GET_TEMP_OFFSET 50 //offset from GET_TEMP
#define GET_MOVE_DU 200 //[msec], 200ms => 5Hz
#define GET_MOVE_OFFSET 100 //offset from GET_MOVE
#define GET_HARDWARE_ERROR_STATUS_DU 200 //[msec], 200ms => 5Hz
#define GET_HARDWARE_ERROR_STATUS_OFFSET 150

/* please define the gpio which control the IO direction */
// #define WE HAL_GPIO_WritePin(RS485EN_GPIO_Port, RS485EN_Pin, GPIO_PIN_SET);
// #define RE HAL_GPIO_WritePin(RS485EN_GPIO_Port, RS485EN_Pin, GPIO_PIN_RESET);

/* DMA circular rx buffer size */
#define RX_BUFFER_SIZE 512

template <typename T,  int SIZE>
class RingBufferDx
{
public:
  RingBufferDx()
  {
    byte_in_progress_ = 0;
    byte_to_add_ = 0;
    buffer_length_ = (uint16_t)SIZE;
  }
  ~RingBufferDx(){  }

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

class DynamixelSerial : public ServoBase
{
public:
  DynamixelSerial(){}

  void init(UART_HandleTypeDef* huart, osMutexId* mutex = NULL) override;
  void pinReconfig() override;
  void ping() override;
  HAL_StatusTypeDef read(uint8_t* data,  uint32_t timeout) override;
  void reboot(uint8_t servo_index) override;
  void setTorque(uint8_t servo_index) override;
  void setTorqueFromPresetnPos(uint8_t servo_index) override;
  void setHomingOffset(uint8_t servo_index) override;
  void setRoundOffset(uint8_t servo_index, int32_t ref_value) override;
  void setPositionGains(uint8_t servo_index) override;
  void setProfileVelocity(uint8_t servo_index) override;
  void setCurrentLimit(uint8_t servo_index) override;
  void update() override;
  
  uint16_t getTTLRS485Mixed() const {return ttl_rs485_mixed_;}
  void setTTLRS485Mixed(uint16_t flag) {ttl_rs485_mixed_ = flag;}
  ServoData& getOneServo(uint8_t id);
  uint8_t getServoIndex(uint8_t id);

private:
  RingBufferDx<std::pair<uint8_t, uint8_t>, 64> instruction_buffer_;

  // a new and quicker method to read servo data
  bool read_status_packet_flag_ = false;
  std::pair<uint8_t, uint8_t> instruction_last_ = std::make_pair(255, 255);


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
  inline void cmdSyncReadCurrentLimit(bool send_all = true);
  inline void cmdSyncReadHardwareErrorStatus(bool send_all = true);
  inline void cmdSyncReadHomingOffset(bool send_all = true);
  inline void cmdSyncReadMoving(bool send_all = true);
  inline void cmdSyncReadPositionGains(bool send_all = true);
  inline void cmdSyncReadPresentCurrent(bool send_all = true);
  inline void cmdSyncReadPresentPosition(bool send_all = true);
  inline void cmdSyncReadPresentTemperature(bool send_all = true);
  inline void cmdSyncReadProfileVelocity(bool send_all = true);
  inline void cmdSyncWriteGoalPosition();
  inline void cmdSyncWriteLed();
  inline void cmdSyncWritePositionGains();
  inline void cmdSyncWriteProfileVelocity();
  inline void cmdSyncWriteTorqueEnable();

  inline void setStatusReturnLevel() override;
  inline void getHomingOffset() override;
  inline void getCurrentLimit() override;
  inline void getPositionGains() override;
  inline void getProfileVelocity() override;

  uint16_t calcCRC16(uint16_t crc_accum, uint8_t *data_blk_ptr, int data_blk_size);
};


#endif
