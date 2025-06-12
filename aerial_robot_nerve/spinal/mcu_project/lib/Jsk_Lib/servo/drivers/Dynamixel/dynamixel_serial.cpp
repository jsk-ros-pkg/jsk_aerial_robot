#include "dynamixel_serial.h"
// #include "flashmemory/flashmemory.h"

namespace
{

#if STM32H7_V2
  uint8_t rx_buf_[RX_BUFFER_SIZE] __attribute__((section(".ServoRxBufferSection")));
#else
#ifdef STM32H7
  uint8_t rx_buf_[RX_BUFFER_SIZE] __attribute__((section(".GpsRxBufferSection")));
#else
  uint8_t rx_buf_[RX_BUFFER_SIZE];
#endif
#endif
}
void DynamixelSerial::init(UART_HandleTypeDef* huart, osMutexId* mutex)
{
	huart_ = huart;
  mutex_ = mutex;
	servo_num_ = 0;
	set_pos_tick_ = 0;
	get_pos_tick_ = 0;
	get_load_tick_ = 0;
	get_temp_tick_ = 0;
	get_move_tick_ = 0;
	get_error_tick_ = 0;

  pinReconfig();

        /* rx */
  __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
  #if DYNAMIXEL_BOARDLESS_CONTROL
    HAL_HalfDuplex_EnableReceiver(huart_);
  #endif
  HAL_UART_Receive_DMA(huart, rx_buf_, RX_BUFFER_SIZE);
  rd_ptr_ = 0;
  memset(rx_buf_, 0, sizeof(rx_buf_));

	std::fill(servo_.begin(), servo_.end(), ServoData(255));

	//initialize servo motors
	HAL_Delay(500);
	ping();
	HAL_Delay(500);
	for (unsigned int i = 0; i < servo_num_; i++) {
		reboot(i);
	}
	HAL_Delay(2000);

	setStatusReturnLevel();
	//Successfully detected servo's led will be turned on 1 seconds

	for (unsigned int i = 0; i < servo_num_; i++) {
		servo_[i].led_ = true;
	}
	cmdSyncWriteLed();

	HAL_Delay(1000);
	for (unsigned int i = 0; i < servo_num_; i++) {
		servo_[i].led_ = false;
	}
	cmdSyncWriteLed();

	for (int i = 0; i < MAX_SERVO_NUM; i++) {
		FlashMemory::addValue(&(servo_[i].p_gain_), 2);
		FlashMemory::addValue(&(servo_[i].i_gain_), 2);
		FlashMemory::addValue(&(servo_[i].d_gain_), 2);
		FlashMemory::addValue(&(servo_[i].profile_velocity_), 2);
		FlashMemory::addValue(&(servo_[i].send_data_flag_), 2);
                FlashMemory::addValue(&(servo_[i].external_encoder_flag_), 2);
                FlashMemory::addValue(&(servo_[i].joint_resolution_), 2);
                FlashMemory::addValue(&(servo_[i].servo_resolution_), 2);
                FlashMemory::addValue(&(servo_[i].joint_offset_), 4);
	}
	FlashMemory::addValue(&(ttl_rs485_mixed_), 2);

	FlashMemory::read();

	cmdSyncWritePositionGains();
	cmdSyncWriteProfileVelocity();

	getHomingOffset(); // This operation always fails after the servo motor is ignited first
	getHomingOffset(); // So this line is necessary
	getCurrentLimit();
	getPositionGains();
	getProfileVelocity();


        //initialize encoder: only can support one encoder
#ifndef SPINAL
        encoder_handler_.init(hi2c); // Magnetic Encoder
#endif
	for (int i = 0; i < MAX_SERVO_NUM; i++) {
          servo_[i].hardware_error_status_ = 0;
          if(servo_[i].servo_resolution_ == 65535 || servo_[i].joint_resolution_ == 65535){
#ifndef SPINAL
            if(servo_[i].external_encoder_flag_)
              servo_[i].hardware_error_status_ = 1 << RESOLUTION_RATIO_ERROR;  // 0b01000000;
#endif
            servo_[i].resolution_ratio_ = 1;
          }
          else{
            servo_[i].resolution_ratio_ = (float)servo_[i].servo_resolution_ / (float)servo_[i].joint_resolution_;
#ifndef SPINAL
            if(servo_[i].external_encoder_flag_) encoder_handler_.setOffset(servo_[i].joint_offset_);
#endif
          }
	}

        /*set angle scale values*/
        for (int i = 0; i < MAX_SERVO_NUM; i++){
          servo_[i].zero_point_offset_ = 2047;
          servo_[i].angle_scale_ = 3.1416 / 2047;
        }
}

void DynamixelSerial::pinReconfig()
{
  while(HAL_UART_DeInit(huart_) != HAL_OK);
  /*Change baud rate*/
  huart_->Init.BaudRate = 1000000;
  huart_->Init.WordLength = UART_WORDLENGTH_8B;
  huart_->Init.Parity = UART_PARITY_NONE;
  huart_->Init.Mode = UART_MODE_TX_RX;
  /*Initialize as halfduplex mode*/
#if DYNAMIXEL_BOARDLESS_CONTROL
  while(HAL_HalfDuplex_Init(huart_) != HAL_OK);  
#else 
  while(HAL_UART_Init(huart_) != HAL_OK);
#endif
}

void DynamixelSerial::ping()
{
	cmdPing(DX_BROADCAST_ID);
	for (int i = 0; i < PING_TRIAL_NUM; ++i) {
		readStatusPacket(INST_PING);
	}
}


void DynamixelSerial::reboot(uint8_t servo_index)
{
	cmdReboot(servo_[servo_index].id_);
}

void DynamixelSerial::setTorque(uint8_t servo_index)
{
  instruction_buffer_.push(std::make_pair(INST_SET_TORQUE, servo_index));
}

void DynamixelSerial::setTorqueFromPresetnPos(uint8_t servo_index)
{
  ServoData& s = servo_[servo_index];
  if(s.torque_enable_) s.goal_position_ = s.present_position_;
  instruction_buffer_.push(std::make_pair(INST_SET_TORQUE, servo_index));
}

void DynamixelSerial::setRoundOffset(uint8_t servo_index, int32_t ref_value)
{
  ServoData& s = servo_[servo_index];
  if (s.torque_enable_) return; // cannot process if torque is enable

  // workaround to update the internal offset for pulley type in the case that has round gap
  int32_t diff = ref_value - s.present_position_;
  if (diff > 2047) s.internal_offset_ += 4096;
  if (diff < -2047) s.internal_offset_ -= 4096;
}

void DynamixelSerial::setHomingOffset(uint8_t servo_index)
{
  if(servo_[servo_index].external_encoder_flag_){
	servo_[servo_index].joint_offset_ = servo_[servo_index].calib_value_ + servo_[servo_index].joint_offset_ - servo_[servo_index].present_position_;
#ifndef SPINAL
	servo_[servo_index].joint_offset_ = servo_[servo_index].calib_value_; // debug
	encoder_handler_.setOffset(servo_[servo_index].joint_offset_);
#endif
	servo_[servo_index].first_get_pos_flag_ = true;
	FlashMemory::erase();
	FlashMemory::write();
  }
  else{
    if (mutex_ != NULL)  osMutexWait(*mutex_, osWaitForever);

    // reset homing offset and internal offset
    servo_[servo_index].homing_offset_ = 0;
    servo_[servo_index].internal_offset_ = 0;
    cmdWriteHomingOffset(servo_index);
    cmdReadHomingOffset(servo_index);
    readStatusPacket(INST_GET_HOMING_OFFSET);

    // read updated position
    cmdReadPresentPosition(servo_index);
    readStatusPacket(INST_GET_PRESENT_POS);

    // set new homing offset
    servo_[servo_index].updateHomingOffset();
    cmdWriteHomingOffset(servo_index);
    cmdReadHomingOffset(servo_index);
    readStatusPacket(INST_GET_HOMING_OFFSET);

    // read updated position
    cmdReadPresentPosition(servo_index);
    readStatusPacket(INST_GET_PRESENT_POS);

    if (mutex_ != NULL) osMutexRelease(*mutex_);
  }
}

void DynamixelSerial::setPositionGains(uint8_t servo_index)
{
  if (mutex_ != NULL)  osMutexWait(*mutex_, osWaitForever);

  cmdWritePositionGains(servo_index);
  getPositionGains();

  if (mutex_ != NULL) osMutexRelease(*mutex_);
}

void DynamixelSerial::setProfileVelocity(uint8_t servo_index)
{
  if (mutex_ != NULL)  osMutexWait(*mutex_, osWaitForever);

  cmdWriteProfileVelocity(servo_index);
  getProfileVelocity();

  if (mutex_ != NULL) osMutexRelease(*mutex_);
}

void DynamixelSerial::setCurrentLimit(uint8_t servo_index)
{
  if (mutex_ != NULL)  osMutexWait(*mutex_, osWaitForever);

  cmdWriteCurrentLimit(servo_index);
  getCurrentLimit();

  if (mutex_ != NULL) osMutexRelease(*mutex_);
}

ServoData& DynamixelSerial::getOneServo(uint8_t id)
{
  for(int i = 0; i < servo_num_; i++)
    {
      if(servo_[i].id_ == id) return servo_[i];
    }
  ServoData non_servo;
  return non_servo;
}

uint8_t DynamixelSerial::getServoIndex(uint8_t id)
{
  for(int i = 0; i < servo_num_; i++)
    {
      if(servo_[i].id_ == id) return i;
    }
  return 0;
}

void DynamixelSerial::update()
{
  /* receive data process */
  /* For one round, change from "send -> receive" to " receive -> send" */
  /* This setting can accelerate the receiving process */
  if(read_status_packet_flag_) {
    for (unsigned int i = 0; i < servo_num_; i++) {
      if(!servo_[i].send_data_flag_ && !servo_[i].first_get_pos_flag_) continue;
      readStatusPacket(instruction_last_.first);
    }
    if (instruction_last_.first == INST_GET_PRESENT_POS)
    {
      setROSCommFlag(true);
    }
  }
  read_status_packet_flag_ = false;

  uint32_t current_time = HAL_GetTick();

  /* send position command to servo */
  if(current_time >= set_pos_tick_ + SET_POS_DU && SET_POS_DU > 0) {
    if (set_pos_tick_ == 0) set_pos_tick_ = current_time + SET_POS_OFFSET; // init
    else set_pos_tick_ = current_time;

    if (ttl_rs485_mixed_ != 0) {
      for (unsigned int i = 0; i < servo_num_; ++i) {
        instruction_buffer_.push(std::make_pair(INST_SET_GOAL_POS, i));
      }
    } else {
      instruction_buffer_.push(std::make_pair(INST_SET_GOAL_POS, 0));
    }
  }

  /* read servo position(angle) */
  if(current_time >= get_pos_tick_ + GET_POS_DU && GET_POS_DU > 0) {
    if (get_pos_tick_ == 0) get_pos_tick_ = current_time + GET_POS_OFFSET; // init
    else get_pos_tick_ = current_time;

    if (ttl_rs485_mixed_ != 0) {
      for (unsigned int i = 0; i < servo_num_; ++i) {
        instruction_buffer_.push(std::make_pair(INST_GET_PRESENT_POS, i));
      }
    } else {
      instruction_buffer_.push(std::make_pair(INST_GET_PRESENT_POS, 0));
    }
  }

  /* read servo load */

  if(current_time >= get_load_tick_ + GET_LOAD_DU && GET_LOAD_DU > 0) {
    if (get_load_tick_ == 0) get_load_tick_ = current_time + GET_LOAD_OFFSET; // init
    else get_load_tick_ = current_time;

    if (ttl_rs485_mixed_ != 0) {
      for (unsigned int i = 0; i < servo_num_; ++i) {
        instruction_buffer_.push(std::make_pair(INST_GET_PRESENT_CURRENT, i));
      }
    } else {
      instruction_buffer_.push(std::make_pair(INST_GET_PRESENT_CURRENT, 0));
    }
  }

  /* read servo temperature */

  if(current_time >= get_temp_tick_ + GET_TEMP_DU && GET_TEMP_DU > 0) {
    if (get_temp_tick_ == 0) get_temp_tick_ = current_time + GET_TEMP_OFFSET;  // init
    else get_temp_tick_ = current_time;

    if (ttl_rs485_mixed_ != 0) {
      for (unsigned int i = 0; i < servo_num_; ++i) {
        instruction_buffer_.push(std::make_pair(INST_GET_PRESENT_TEMPERATURE, i));
      }
    } else {
      instruction_buffer_.push(std::make_pair(INST_GET_PRESENT_TEMPERATURE, 0));
    }
  }

  /* read servo movement */
  if(current_time >= get_move_tick_ + GET_MOVE_DU && GET_MOVE_DU > 0) {
    if (get_move_tick_ == 0) get_move_tick_ = current_time + GET_MOVE_OFFSET; // init
    else get_move_tick_ = current_time;

    if (ttl_rs485_mixed_ != 0) {
      for (unsigned int i = 0; i < servo_num_; ++i) {
        instruction_buffer_.push(std::make_pair(INST_GET_PRESENT_MOVING, i));
      }
    } else {
      instruction_buffer_.push(std::make_pair(INST_GET_PRESENT_MOVING, 0));
    }
  }

  /* read hardware error status */
  if(current_time >= get_error_tick_ + GET_HARDWARE_ERROR_STATUS_DU && GET_HARDWARE_ERROR_STATUS_DU > 0) {
    if (get_error_tick_ == 0) get_error_tick_ = current_time + GET_HARDWARE_ERROR_STATUS_OFFSET; // init
    else get_error_tick_ = current_time;

    if (ttl_rs485_mixed_ != 0) {
      for (unsigned int i = 0; i < servo_num_; ++i) {
        instruction_buffer_.push(std::make_pair(INST_GET_HARDWARE_ERROR_STATUS, i));
      }
    } else {
      instruction_buffer_.push(std::make_pair(INST_GET_HARDWARE_ERROR_STATUS, 0));
    }
  }


  /* process the instruction from the instruction buffer */
  std::pair<uint8_t, uint8_t> instruction;
  while(instruction_buffer_.pop(instruction))
    {
      uint8_t servo_index = instruction.second;

      if (mutex_ != NULL)  osMutexWait(*mutex_, osWaitForever);

      /* set command */
      switch (instruction.first) {
      case INST_SET_GOAL_POS: /* send angle command to servo */
        cmdSyncWriteGoalPosition();
        break;
      case INST_SET_TORQUE: /* send torque enable flag */
        cmdWriteTorqueEnable(servo_index);
        break;
      case INST_SET_HOMING_OFFSET:
        cmdWriteHomingOffset(servo_index);
        break;
      case INST_SET_POSITION_GAINS:
        cmdWritePositionGains(servo_index);
        break;
      case INST_SET_CURRENT_LIMIT:
        cmdWriteCurrentLimit(servo_index);
        break;
      case INST_SET_PROFILE_VELOCITY:
        cmdWriteProfileVelocity(servo_index);
        break;
      default:
        break;
      }

      /* get command */
      if (ttl_rs485_mixed_ != 0) {
        switch (instruction.first) {
        case INST_GET_PRESENT_POS: /* read servo position(angle) */
          if(!servo_[servo_index].send_data_flag_ && !servo_[servo_index].first_get_pos_flag_) break;
          cmdReadPresentPosition(servo_index);
          readStatusPacket(instruction.first);
          break;
        case INST_GET_PRESENT_CURRENT: /* read servo load */
          if(!servo_[servo_index].send_data_flag_) break;
          cmdReadPresentCurrent(servo_index);
          readStatusPacket(instruction.first);
          break;
        case INST_GET_PRESENT_TEMPERATURE: /* read servo temp */
          if(!servo_[servo_index].send_data_flag_) break;
          cmdReadPresentTemperature(servo_index);
          readStatusPacket(instruction.first);
          break;
        case INST_GET_PRESENT_MOVING: /* read servo movement */
          if(!servo_[servo_index].send_data_flag_) break;
          cmdReadMoving(servo_index);
          readStatusPacket(instruction.first);
          break;
        case INST_GET_HARDWARE_ERROR_STATUS:
          if(!servo_[servo_index].send_data_flag_) break;
          cmdReadHardwareErrorStatus(servo_index);
          readStatusPacket(instruction.first);
          break;
        case INST_GET_HOMING_OFFSET:
          for (unsigned int i = 0; i < servo_num_; ++i) {
            cmdReadHomingOffset(i);
            readStatusPacket(instruction.first);
          }
          break;
        case INST_GET_POSITION_GAINS:
          for (unsigned int i = 0; i < servo_num_; ++i) {
            cmdReadPositionGains(i);
            readStatusPacket(instruction.first);
          }
          break;
        case INST_GET_PROFILE_VELOCITY:
          for (unsigned int i = 0; i < servo_num_; ++i) {
            cmdReadProfileVelocity(i);
            readStatusPacket(instruction.first);
          }
          break;
        case INST_GET_CURRENT_LIMIT:
          for (unsigned int i = 0; i < servo_num_; ++i) {
            cmdReadCurrentLimit(i);
            readStatusPacket(instruction.first);
          }
          break;
        default:
          break;
        }
      } else {
        switch (instruction.first) {
        case INST_GET_PRESENT_POS: /* read servo position(angle) */
          cmdSyncReadPresentPosition(false);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_PRESENT_CURRENT: /* read servo load */
          cmdSyncReadPresentCurrent(false);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_PRESENT_TEMPERATURE: /* read servo temp */
          cmdSyncReadPresentTemperature(false);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_PRESENT_MOVING: /* read servo movement */
          cmdSyncReadMoving(false);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_HARDWARE_ERROR_STATUS:
          cmdSyncReadHardwareErrorStatus(false);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_HOMING_OFFSET:
          cmdSyncReadHomingOffset(false);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_POSITION_GAINS:
          cmdSyncReadPositionGains(false);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_PROFILE_VELOCITY:
          cmdSyncReadProfileVelocity(false);
          read_status_packet_flag_ = true;
          break;
        case INST_GET_CURRENT_LIMIT:
          cmdSyncReadCurrentLimit(false);
          read_status_packet_flag_ = true;
          break;
        default:
          break;
        }

        if (read_status_packet_flag_) {
          instruction_last_ = instruction;
        }
      }

      if (mutex_ != NULL) osMutexRelease(*mutex_);
    }
}

/* Transmit instruction packet to Dynamixel */
void DynamixelSerial::transmitInstructionPacket(uint8_t id, uint16_t len, uint8_t instruction, uint8_t* parameters)
{
  uint8_t transmit_data[INSTRUCTION_PACKET_SIZE];
  /* headers */
  transmit_data[0] = HEADER0;
  transmit_data[1] = HEADER1;
  transmit_data[2] = HEADER2;
  transmit_data[3] = HEADER3;
  /* servo id */
  transmit_data[4] = id;
  /* instruction len */
  transmit_data[5] = len & 0xFF; //LEN_L
  transmit_data[6] = (len >> 8) & 0xFF; //LEN_H
  /* instruction */
  transmit_data[7] = instruction;
  /* parameters */
  //process for exception
  int header_match_count = 0;
  int transmit_data_index = 8;
  for (int i = 0; i < len - 3; i++) {
    uint8_t tx_data = parameters[i];
    if (header_match_count == 0) {
      if (tx_data == HEADER0) header_match_count++;
      else header_match_count = 0;
      transmit_data[transmit_data_index] = tx_data;
      transmit_data_index++;
    } else if (header_match_count == 1) {
      if (tx_data == HEADER1) header_match_count++;
      else header_match_count = 0;
      transmit_data[transmit_data_index] = tx_data;
      transmit_data_index++;
    } else if (header_match_count == 2) {
      if (tx_data == HEADER2) {
        //exception
        transmit_data[transmit_data_index] = tx_data;
        transmit_data_index++;
        transmit_data[transmit_data_index] = EXCEPTION_ADDITIONAL_BYTE;
        transmit_data_index++;
      } else {
        header_match_count = 0;
        transmit_data[transmit_data_index] = tx_data;
        transmit_data_index++;
      }
    }
  }

  /* checksum */
  uint16_t chksum = calcCRC16(0, transmit_data, transmit_data_index);

  transmit_data[transmit_data_index] = chksum & 0xFF; //CRC_L
  transmit_data_index++;
  transmit_data[transmit_data_index] = (chksum >> 8) & 0xFF; //CRC_H
  transmit_data_index++;

  /* send data */

#if DYNAMIXEL_BOARDLESS_CONTROL
  HAL_HalfDuplex_EnableTransmitter(huart_);
  uint8_t ret;
  ret = HAL_UART_Transmit(huart_, transmit_data, transmit_data_index, 10); //timeout: 10 ms. Although we found 2 ms is enough OK for our case by oscilloscope. Large value is better for UART async task in RTOS.
  if(ret == HAL_OK)
  {
    // After transmitting, enable the receiver
    HAL_HalfDuplex_EnableReceiver(huart_);
  }
#else
// WE; 
  HAL_UART_Transmit(huart_, transmit_data, transmit_data_index, 10); //timeout: 10 ms. Although we found 2 ms is enough OK for our case by oscilloscope. Large value is better for UART async task in RTOS.
// RE;
#endif
}
/* Receive status packet to Dynamixel */
int8_t DynamixelSerial::readStatusPacket(uint8_t status_packet_instruction)
{
	int status_stage = READ_HEADER0;
	uint8_t rx_data;
	int header_match_count = 0;
	uint16_t parameter_len = 0;
	uint8_t parameters[STATUS_PACKET_SIZE];
	int parameter_index = 0;
	int parameter_loop_count = 0;
	uint16_t checksum = 0;
	uint8_t receive_data[STATUS_PACKET_SIZE];
	bool read_end_flag = false;
	int loop_count = 0;
	uint8_t servo_id;

	while(!read_end_flag) {
		HAL_StatusTypeDef receive_status = read(&rx_data, 1);
		if(receive_status == HAL_TIMEOUT)
			{
				return -1;
			}

		switch (status_stage) {
		case READ_HEADER0:
			if (rx_data == HEADER0) {
				status_stage++;
			}
			break;
		case READ_HEADER1:
			if (rx_data == HEADER1) {
				status_stage++;
			} else {
				return -1;
			}
			break;
		case READ_HEADER2:
			if (rx_data == HEADER2) {
				status_stage++;
			} else {
				return -1;
			}
			break;
		case READ_HEADER3:
			if (rx_data == HEADER3) {
				status_stage++;
			} else {
				return -1;
			}
			break;
		case READ_SERVOID:
			servo_id = rx_data;
			status_stage++;
			break;
		case READ_LENL:
			parameter_len = rx_data;
			status_stage++;
			break;
		case READ_LENH:
			parameter_len |= ((rx_data << 8) & 0xFF00);
			status_stage++;
			break;
		case READ_INSTRUCTION:
			if (rx_data == STATUS_PACKET_INSTRUCTION) {
				status_stage++;
			} else {
				return -1;
			}
			break;
		case READ_ERROR:
			if ((rx_data & 0x7F) == ERROR_NO_ERROR) {
				status_stage++;
			} else {
				return -1;
			}
			break;
		case READ_PARAMETER:
			//process for exception
			if (header_match_count == 0) {
				if (rx_data == HEADER0) header_match_count++;
				else header_match_count = 0;
				parameters[parameter_index] = rx_data;
				parameter_index++;
			} else if (header_match_count == 1) {
				if (rx_data == HEADER1) header_match_count++;
				else header_match_count = 0;
				parameters[parameter_index] = rx_data;
				parameter_index++;
			} else if (header_match_count == 2) {
				if (rx_data == HEADER2) header_match_count++;
				else header_match_count = 0;
				parameters[parameter_index] = rx_data;
				parameter_index++;
			} else if (header_match_count == 3) {
				if (rx_data == EXCEPTION_ADDITIONAL_BYTE) {
					header_match_count = 0;
				} else {
					return -1;
				}
			}
			parameter_loop_count++;
			if (parameter_loop_count == parameter_len - 4) {
				status_stage++;
			}
			break;
		case READ_CHECKSUML:
			checksum = rx_data;
			status_stage++;
			break;
		case READ_CHECKSUMH:
			checksum |= ((rx_data << 8) & 0xFF00);
			if (checksum == calcCRC16(0, receive_data, loop_count)) {
				read_end_flag = true;
			} else {
				return -1;
			}
			break;
		default:
			return -1;
		}
		if (status_stage > READ_HEADER0 && status_stage != READ_CHECKSUMH) {
			receive_data[loop_count] = rx_data;
			loop_count++;
		}
	}

	auto s = std::find(servo_.begin(), servo_.end(), ServoData(servo_id));

        /* clear UART RX */
        __HAL_UART_CLEAR_FLAG(huart_, UART_FLAG_RXNE);
        __HAL_UART_CLEAR_PEFLAG(huart_);
        __HAL_UART_CLEAR_OREFLAG(huart_);
        __HAL_UART_CLEAR_FEFLAG(huart_);

	/* read success */
	switch (status_packet_instruction) {
	case INST_PING:
		servo_[servo_num_++].id_ = servo_id;
	    return 0;
	case INST_GET_PRESENT_POS:
	{
		int32_t present_position = ((parameters[3] << 24) & 0xFF000000) | ((parameters[2] << 16) & 0xFF0000) | ((parameters[1] << 8) & 0xFF00) | (parameters[0] & 0xFF);
		if (s != servo_.end()) {
                  s->hardware_error_status_ &= ((1 << ENCODER_CONNECT_ERROR) - 1); // &= 0b01111111
                  if(s->external_encoder_flag_) {
#ifndef SPINAL
                    encoder_handler_.update();
                    if(encoder_handler_.connected()) {
                      s->present_position_ = (int32_t)(encoder_handler_.getValue()); // use external encoder value instead of servo internal encoder value

                      if (s->first_get_pos_flag_) {
                        s->internal_offset_ = s->resolution_ratio_ * s->present_position_ - present_position;
                        s->first_get_pos_flag_ = false;
                      }

                      // TODO: check tooth jump
                    }
                    else {
                      s->hardware_error_status_ |= 1 << ENCODER_CONNECT_ERROR; // |= 0b10000000:  encoder is not connected
                    }
#else
                    return 0;
#endif
                  }
                  else {
                    if (s->first_get_pos_flag_) {
                      s->internal_offset_ = std::floor(present_position / 4096.0) * -4096; // to convert [0, 4096]
                      s->first_get_pos_flag_ = false;
                    }
                    s->setPresentPosition(present_position);
                  }
		}
                return 0;
	}
	case INST_GET_PRESENT_CURRENT:
		if (s != servo_.end()) {
			s->present_current_ = ((parameters[1] << 8) & 0xFF00) | (parameters[0] & 0xFF);
		}
		return 0;
	case INST_GET_PRESENT_TEMPERATURE:
		if (s != servo_.end()) {
			s->present_temp_ = parameters[0];
		}
		return 0;
	case INST_GET_PRESENT_MOVING:
		if (s != servo_.end()) {
			s->moving_ = parameters[0];
		}
		return 0;
	case INST_GET_HOMING_OFFSET:
		if (s != servo_.end()) {
			s->homing_offset_ = ((parameters[3] << 24) & 0xFF000000) | ((parameters[2] << 16) & 0xFF0000) | ((parameters[1] << 8) & 0xFF00) | (parameters[0] & 0xFF);
		}
		return 0;
	case INST_GET_HARDWARE_ERROR_STATUS:
		if (s != servo_.end()) {
                  s->hardware_error_status_ &=  ((1 << RESOLUTION_RATIO_ERROR) + (1 << ENCODER_CONNECT_ERROR));  //  &= 0b11000000;
                  s->hardware_error_status_ |= parameters[0];
		}
		return 0;
	case INST_GET_CURRENT_LIMIT:
		if (s != servo_.end()) {
			s->current_limit_ = ((parameters[1] << 8) & 0xFF00) | (parameters[0] & 0xFF);
		}
		return 0;
	case INST_GET_POSITION_GAINS:
		if (s != servo_.end()) {
			s->d_gain_ = ((parameters[1] << 8) & 0xFF00) | (parameters[0] & 0xFF);
			s->i_gain_ = ((parameters[3] << 8) & 0xFF00) | (parameters[2] & 0xFF);
			s->p_gain_ = ((parameters[5] << 8) & 0xFF00) | (parameters[4] & 0xFF);
		}
		return 0;
	case INST_GET_PROFILE_VELOCITY:
		if (s != servo_.end()) {
			s->profile_velocity_ = ((parameters[3] << 24) & 0xFF000000) | ((parameters[2] << 16) & 0xFF0000) | ((parameters[1] << 8) & 0xFF00) | (parameters[0] & 0xFF);
		}
		return 0;
	default:
		return -1;
	}
}

void DynamixelSerial::cmdPing(uint8_t id)
{
  transmitInstructionPacket(id, PING_LEN, COMMAND_PING, nullptr);
}

void DynamixelSerial::cmdReboot(uint8_t id)
{
	transmitInstructionPacket(id, REBOOT_LEN, COMMAND_REBOOT, nullptr);
}

void DynamixelSerial::cmdRead(uint8_t id, uint16_t address, uint16_t byte_size)
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];
	parameters[0] = address & 0xFF;
	parameters[1] = (address >> 8) & 0xFF;
	parameters[2] = byte_size & 0xFF;
	parameters[3] = (byte_size >> 8) & 0xFF;
	transmitInstructionPacket(id, READ_INST_LEN, COMMAND_READ, parameters);
}

void DynamixelSerial::cmdWrite(uint8_t id, uint16_t address, uint8_t* param, int param_len)
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];
	parameters[0] = address & 0xFF;
	parameters[1] = (address >> 8) & 0xFF;
	for (int i = 0; i < param_len; i++) {
		parameters[i + 2] = param[i];
	}
	transmitInstructionPacket(id, param_len + 5, COMMAND_WRITE, parameters);
}

void DynamixelSerial::cmdSyncRead(uint16_t address, uint16_t byte_size, bool send_all)
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];
	parameters[0] = address & 0xFF;
	parameters[1] = (address >> 8) & 0xFF;
	parameters[2] = byte_size & 0xFF;
	parameters[3] = (byte_size >> 8) & 0xFF;
	int param_idx = 4;
	for (unsigned int i = 0; i < servo_num_; i++) {
          if(!send_all && !servo_[i].send_data_flag_ && !servo_[i].first_get_pos_flag_) continue;
          parameters[param_idx++] = servo_[i].id_;
	}
	transmitInstructionPacket(DX_BROADCAST_ID, param_idx + 3, COMMAND_SYNC_READ, parameters);
}

void DynamixelSerial::cmdSyncWrite(uint16_t address, uint8_t* param, int param_len)
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];
	parameters[0] = address & 0xFF;
	parameters[1] = (address >> 8) & 0xFF;
	parameters[2] = param_len & 0xFF;
	parameters[3] = (param_len >> 8) & 0xFF;
	int all_param_idx = 4;
	int param_idx = 0;
	for (unsigned int i = 0; i < servo_num_; i++) {
		parameters[all_param_idx++] = servo_[i].id_;
		for (int j = 0; j < param_len; j++) {
			parameters[all_param_idx++] = param[param_idx++];
		}
	}
	transmitInstructionPacket(DX_BROADCAST_ID, all_param_idx + 3, COMMAND_SYNC_WRITE, parameters);
}

void DynamixelSerial::cmdReadCurrentLimit(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_CURRENT_LIMIT, CURRENT_LIMIT_BYTE_LEN);
}

void DynamixelSerial::cmdReadHardwareErrorStatus(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_HARDWARE_ERROR_STATUS, HARDWARE_ERROR_STATUS_BYTE_LEN);
}

void DynamixelSerial::cmdReadHomingOffset(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_HOMING_OFFSET, HOMING_OFFSET_BYTE_LEN);
}

void DynamixelSerial::cmdReadMoving(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_MOVING, MOVING_BYTE_LEN);
}

void DynamixelSerial::cmdReadPositionGains(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_POSITION_D_GAIN, POSITION_GAINS_BYTE_LEN);
}

void DynamixelSerial::cmdReadPresentCurrent(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_PRESENT_CURRENT, PRESENT_CURRENT_BYTE_LEN);
}

void DynamixelSerial::cmdReadPresentPosition(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_PRESENT_POSITION, PRESENT_POSITION_BYTE_LEN);
}

void DynamixelSerial::cmdReadPresentTemperature(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_PRESENT_TEMPERATURE, PRESENT_TEMPERATURE_BYTE_LEN);
}

void DynamixelSerial::cmdReadProfileVelocity(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_PROFILE_VELOCITY, PROFILE_VELOCITY_BYTE_LEN);
}

void DynamixelSerial::cmdWriteCurrentLimit(uint8_t servo_index)
{
	uint16_t current_limit = servo_[servo_index].current_limit_;
	uint8_t parameters[CURRENT_LIMIT_BYTE_LEN];
	parameters[0] = (uint8_t)(current_limit & 0xFF);
	parameters[1] = (uint8_t)((current_limit >> 8) & 0xFF);

	cmdWrite(servo_[servo_index].id_, CTRL_CURRENT_LIMIT, parameters, CURRENT_LIMIT_BYTE_LEN);
}

void DynamixelSerial::cmdWriteHomingOffset(uint8_t servo_index)
{
	int32_t homing_offset = servo_[servo_index].homing_offset_;
	uint8_t parameters[4];
	parameters[0] = (uint8_t)(homing_offset & 0xFF);
	parameters[1] = (uint8_t)((homing_offset >> 8) & 0xFF);
	parameters[2] = (uint8_t)((homing_offset >> 16) & 0xFF);
	parameters[3] = (uint8_t)((homing_offset >> 24) & 0xFF);
	cmdWrite(servo_[servo_index].id_, CTRL_HOMING_OFFSET, parameters, HOMING_OFFSET_BYTE_LEN);
}

void DynamixelSerial::cmdWritePositionGains(uint8_t servo_index)
{
	uint8_t parameters[POSITION_GAINS_BYTE_LEN];
	parameters[0] = servo_[servo_index].d_gain_ & 0xFF;
	parameters[1] = (servo_[servo_index].d_gain_ >> 8) & 0xFF;
	parameters[2] = servo_[servo_index].i_gain_ & 0xFF;
	parameters[3] = (servo_[servo_index].i_gain_ >> 8) & 0xFF;
	parameters[4] = servo_[servo_index].p_gain_ & 0xFF;
	parameters[5] = (servo_[servo_index].p_gain_ >> 8) & 0xFF;

	cmdWrite(servo_[servo_index].id_, CTRL_POSITION_D_GAIN, parameters, POSITION_GAINS_BYTE_LEN);
}

void DynamixelSerial::cmdWriteProfileVelocity(uint8_t servo_index)
{
	uint16_t vel = servo_[servo_index].profile_velocity_;
	uint8_t parameters[PROFILE_VELOCITY_BYTE_LEN];
	parameters[0] = (uint8_t)(vel & 0xFF);
	parameters[1] = (uint8_t)((vel >> 8) & 0xFF);
	parameters[2] = (uint8_t)((vel >> 16) & 0xFF);
	parameters[3] = (uint8_t)((vel >> 24) & 0xFF);
	cmdWrite(servo_[servo_index].id_, CTRL_PROFILE_VELOCITY, parameters, PROFILE_VELOCITY_BYTE_LEN);
}

void DynamixelSerial::cmdWriteStatusReturnLevel(uint8_t id, uint8_t set)
{
	uint8_t param = set;
	cmdWrite(id, CTRL_STATUS_RETURN_LEVEL, &param, STATUS_RETURN_LEVEL_BYTE_LEN);
}

void DynamixelSerial::cmdWriteTorqueEnable(uint8_t servo_index)
{

	uint8_t	parameter = (uint8_t)((servo_[servo_index].torque_enable_) ? 1 : 0);

	cmdWrite(servo_[servo_index].id_, CTRL_TORQUE_ENABLE, &parameter, TORQUE_ENABLE_BYTE_LEN);
}

void DynamixelSerial::cmdSyncReadCurrentLimit(bool send_all)
{
	cmdSyncRead(CTRL_CURRENT_LIMIT, CURRENT_LIMIT_BYTE_LEN, send_all);
}

void DynamixelSerial::cmdSyncReadHardwareErrorStatus(bool send_all)
{
	cmdSyncRead(CTRL_HARDWARE_ERROR_STATUS, HARDWARE_ERROR_STATUS_BYTE_LEN, send_all);
}

void DynamixelSerial::cmdSyncReadHomingOffset(bool send_all)
{
	cmdSyncRead(CTRL_HOMING_OFFSET, HOMING_OFFSET_BYTE_LEN, send_all);
}

void DynamixelSerial::cmdSyncReadMoving(bool send_all)
{
	cmdSyncRead(CTRL_MOVING, MOVING_BYTE_LEN, send_all);
}

void DynamixelSerial::cmdSyncReadPositionGains(bool send_all)
{
	cmdSyncRead(CTRL_POSITION_D_GAIN, POSITION_GAINS_BYTE_LEN, send_all);
}

void DynamixelSerial::cmdSyncReadPresentCurrent(bool send_all)
{
	cmdSyncRead(CTRL_PRESENT_CURRENT, PRESENT_CURRENT_BYTE_LEN, send_all);
}

void DynamixelSerial::cmdSyncReadPresentPosition(bool send_all)
{
	cmdSyncRead(CTRL_PRESENT_POSITION, PRESENT_POSITION_BYTE_LEN, send_all);
}

void DynamixelSerial::cmdSyncReadPresentTemperature(bool send_all)
{
	cmdSyncRead(CTRL_PRESENT_TEMPERATURE, PRESENT_TEMPERATURE_BYTE_LEN, send_all);
}

void DynamixelSerial::cmdSyncReadProfileVelocity(bool send_all)
{
	cmdSyncRead(CTRL_PROFILE_VELOCITY, PROFILE_VELOCITY_BYTE_LEN, send_all);
}

void DynamixelSerial::cmdSyncWriteGoalPosition()
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];

	for (unsigned int i = 0; i < servo_num_; i++) {
		int32_t goal_position = servo_[i].getGoalPosition();
		parameters[i * 4 + 0] = (uint8_t)((int32_t)(goal_position) & 0xFF);
		parameters[i * 4 + 1] = (uint8_t)(((int32_t)(goal_position) >> 8) & 0xFF);
		parameters[i * 4 + 2] = (uint8_t)(((int32_t)(goal_position) >> 16) & 0xFF);
		parameters[i * 4 + 3] = (uint8_t)(((int32_t)(goal_position) >> 24) & 0xFF);
	}

	cmdSyncWrite(CTRL_GOAL_POSITION, parameters, GOAL_POSITION_BYTE_LEN);
}

void DynamixelSerial::cmdSyncWriteLed()
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];
	for (unsigned int i = 0; i < servo_num_; i++) {
		parameters[i] = (uint8_t)((servo_[i].led_) ? 1 : 0);
	}

	cmdSyncWrite(CTRL_LED, parameters, LED_BYTE_LEN);
}

void DynamixelSerial::cmdSyncWritePositionGains()
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];
        /*p_gain*/
        for (unsigned int i = 0; i < servo_num_; i++) {
          parameters[i * 2 + 0] = servo_[i].p_gain_ & 0xFF;
          parameters[i * 2 + 1] = (servo_[i].p_gain_ >> 8) & 0xFF;
	}
	cmdSyncWrite(CTRL_POSITION_P_GAIN, parameters, 2);

        /*i_gain*/
        for (unsigned int i = 0; i < servo_num_; i++) {
          parameters[i * 2 + 0] = servo_[i].i_gain_ & 0xFF;
          parameters[i * 2 + 1] = (servo_[i].i_gain_ >> 8) & 0xFF;
	}
	cmdSyncWrite(CTRL_POSITION_I_GAIN, parameters, 2);

        /*d_gain*/
        for (unsigned int i = 0; i < servo_num_; i++) {
          parameters[i * 2 + 0] = servo_[i].d_gain_ & 0xFF;
          parameters[i * 2 + 1] = (servo_[i].d_gain_ >> 8) & 0xFF;
	}
	cmdSyncWrite(CTRL_POSITION_D_GAIN, parameters, 2);
}

void DynamixelSerial::cmdSyncWriteProfileVelocity()
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];

	for (unsigned int i = 0; i < servo_num_; i++) {
		parameters[i * 4 + 0] = (uint8_t)((int32_t)(servo_[i].profile_velocity_) & 0xFF);
		parameters[i * 4 + 1] = (uint8_t)(((int32_t)(servo_[i].profile_velocity_) >> 8) & 0xFF);
		parameters[i * 4 + 2] = (uint8_t)(((int32_t)(servo_[i].profile_velocity_) >> 16) & 0xFF);
		parameters[i * 4 + 3] = (uint8_t)(((int32_t)(servo_[i].profile_velocity_) >> 24) & 0xFF);
	}

	cmdSyncWrite(CTRL_PROFILE_VELOCITY, parameters, PROFILE_VELOCITY_BYTE_LEN);
}

void DynamixelSerial::cmdSyncWriteTorqueEnable()
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];

	for (unsigned int i = 0; i < servo_num_; i++) {
		parameters[i] = (uint8_t)((servo_[i].torque_enable_) ? 1 : 0);
	}

	cmdSyncWrite(CTRL_TORQUE_ENABLE, parameters, TORQUE_ENABLE_BYTE_LEN);
}


void DynamixelSerial::setStatusReturnLevel()
{
	cmdWriteStatusReturnLevel(DX_BROADCAST_ID, READ);
}

void DynamixelSerial::getHomingOffset()
{
	if (ttl_rs485_mixed_ != 0) {
		for (unsigned int i = 0; i < servo_num_; ++i) {
			cmdReadHomingOffset(i);
			readStatusPacket(INST_GET_HOMING_OFFSET);
		}
	} else {
		cmdSyncReadHomingOffset();
		for (unsigned int i = 0; i < servo_num_; i++) {
			readStatusPacket(INST_GET_HOMING_OFFSET);
		}
	}
}

void DynamixelSerial::getCurrentLimit()
{
	if (ttl_rs485_mixed_ != 0) {
		for (unsigned int i = 0; i < servo_num_; ++i) {
			cmdReadCurrentLimit(i);
			readStatusPacket(INST_GET_CURRENT_LIMIT);
		}
	} else {
		cmdSyncReadCurrentLimit();
		for (unsigned int i = 0; i < servo_num_; i++) {
			readStatusPacket(INST_GET_CURRENT_LIMIT);
		}
	}
}

void DynamixelSerial::getPositionGains()
{
	if (ttl_rs485_mixed_ != 0) {
		for (unsigned int i = 0; i < servo_num_; ++i) {
			cmdReadPositionGains(i);
			readStatusPacket(INST_GET_POSITION_GAINS);
		}
	} else {
		cmdSyncReadPositionGains();
		for (unsigned int i = 0; i < servo_num_; i++) {
			readStatusPacket(INST_GET_POSITION_GAINS);
		}
	}
}

void DynamixelSerial::getProfileVelocity()
{
	if (ttl_rs485_mixed_ != 0) {
		for (unsigned int i = 0; i < servo_num_; ++i) {
			cmdReadProfileVelocity(i);
			readStatusPacket(INST_GET_PROFILE_VELOCITY);
		}
	} else {
		cmdSyncReadProfileVelocity();
		for (unsigned int i = 0; i < servo_num_; i++) {
			readStatusPacket(INST_GET_PROFILE_VELOCITY);
		}
	}
}

static const uint16_t crc_table[256] = {
          0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
          0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
          0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
          0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
          0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
          0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
          0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
          0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
          0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
          0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
          0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
          0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
          0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
          0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
          0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
          0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
          0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
          0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
          0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
          0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
          0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
          0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
          0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
          0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
          0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
          0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
          0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
          0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
          0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
          0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
          0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
          0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
      	};

uint16_t DynamixelSerial::calcCRC16(uint16_t crc_accum, uint8_t *data_blk_ptr, int data_blk_size)
{
	for(int j = 0; j < data_blk_size; j++) {
		int i = ((int)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}
	return crc_accum;
}

HAL_StatusTypeDef DynamixelSerial::read(uint8_t* data,  uint32_t timeout)
{
  /* handle RX Overrun Error */
  if ( __HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE) )
    {
      __HAL_UART_CLEAR_FLAG(huart_, UART_FLAG_RXNE);
      __HAL_UART_CLEAR_PEFLAG(huart_);
      __HAL_UART_CLEAR_OREFLAG(huart_);
      __HAL_UART_CLEAR_FEFLAG(huart_);

      HAL_UART_Receive_DMA(huart_, rx_buf_, RX_BUFFER_SIZE);
    }

  uint32_t tick_start = HAL_GetTick();

  while (true)
    {
      // uint32_t dma_write_ptr =  (RX_BUFFER_SIZE - huart_->hdmarx->Instance->NDTR) % (RX_BUFFER_SIZE);
      uint32_t dma_write_ptr =  (RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (RX_BUFFER_SIZE);
      if(rd_ptr_ != dma_write_ptr)
        {
          *data = (int)rx_buf_[rd_ptr_++];
          rd_ptr_ %= RX_BUFFER_SIZE;
          return HAL_OK;
        }

      if (((HAL_GetTick() - tick_start) > timeout) || (timeout == 0U))
        {
          return HAL_TIMEOUT;
        }
    }
}
