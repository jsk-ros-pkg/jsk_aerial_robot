/*
 * initializer.cpp
 *
 *  Created on: 2017/10/14
 *      Author: anzai
 */

#include "initializer.h"


void Initializer::sendData()
{
	return;
}

void Initializer::sendBoardConfig()
{
	uint8_t data[8];
	data[0] = servo_.servo_handler_.getServoNum();
	data[1] = imu_.send_data_flag_ ? 1 : 0;
	data[2] = servo_.servo_handler_.getTTLRS485Mixed();
	sendMessage(CAN::MESSAGEID_SEND_INITIAL_CONFIG_0, m_slave_id, 3, data, 1);
	for (unsigned int i = 0; i < servo_.servo_handler_.getServoNum(); i++) {
		const ServoData& s = servo_.servo_handler_.getServo()[i];
		data[0] = i;
		data[1] = s.id_;
		data[2] = s.p_gain_ & 0xFF;
		data[3] = (s.p_gain_ >> 8) & 0xFF;
		data[4] = s.i_gain_ & 0xFF;
		data[5] = (s.i_gain_ >> 8) & 0xFF;
		data[6] = s.d_gain_ & 0xFF;
		data[7] = (s.d_gain_ >> 8) & 0xFF;
		sendMessage(CAN::MESSAGEID_SEND_INITIAL_CONFIG_1, m_slave_id, 8, data, 1);
		data[0] = i;
		data[1] = s.getPresentPosition() & 0xFF;
		data[2] = (s.getPresentPosition() >> 8) & 0xFF;
		data[3] = s.profile_velocity_ & 0xFF;
		data[4] = (s.profile_velocity_ >> 8) & 0xFF;
		data[5] = s.current_limit_ & 0xFF;
		data[6] = (s.current_limit_ >> 8) & 0xFF;
		data[7] = (s.send_data_flag_ ? 1 : 0);
		sendMessage(CAN::MESSAGEID_SEND_INITIAL_CONFIG_2, m_slave_id, 8, data, 1);
                data[0] = i;
                data[1] = s.hardware_error_status_;
                data[2] = (s.external_encoder_flag_ ? 1 : 0);
		data[3] = s.joint_resolution_ & 0xFF;
		data[4] = (s.joint_resolution_ >> 8) & 0xFF;
		data[5] = s.servo_resolution_ & 0xFF;
		data[6] = (s.servo_resolution_ >> 8) & 0xFF;
                data[7] = 0;
		sendMessage(CAN::MESSAGEID_SEND_INITIAL_CONFIG_3, m_slave_id, 8, data, 1);
	}
}

void Initializer::receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data)
{
	switch (message_id) {
	case CAN::MESSAGEID_RECEIVE_ENUM_REQUEST:
		sendMessage(CAN::MESSAGEID_SEND_ENUM_RESPONSE, m_slave_id, 0, nullptr, 1);
		CANDeviceManager::resetTick();
		break;
	case CAN::MESSAGEID_RECEIVE_INITIAL_CONFIG_REQUEST:
	{
		sendBoardConfig();
		servo_.setConnect(true);
	}
	break;
	case CAN::MESSAGEID_RECEIVE_BOARD_CONFIG_REQUEST:
		switch (data[0]) {
		case CAN::BOARD_CONFIG_SET_SLAVE_ID:
		{
			slave_id_ = data[1];
			Flashmemory::erase();
			Flashmemory::write();
			break;
		}
		case CAN::BOARD_CONFIG_SET_IMU_SEND_FLAG:
		{
			imu_.send_data_flag_ = data[1];
			Flashmemory::erase();
			Flashmemory::write();
			break;
		}
		case CAN::BOARD_CONFIG_SET_SERVO_HOMING_OFFSET:
		{
			uint8_t servo_index = data[1];
			ServoData& s = servo_.servo_handler_.getServo()[servo_index];
			int32_t calib_value = ((data[5] << 24) & 0xFF000000) | ((data[4] << 16) & 0xFF0000) | ((data[3] << 8) & 0xFF00) | ((data[2] << 0) & 0xFF);
                        s.calib_value_ = calib_value;
                        servo_.servo_handler_.setHomingOffset(servo_index);
			break;
		}
		case CAN::BOARD_CONFIG_SET_SERVO_PID_GAIN:
		{
			uint8_t servo_index = data[1];
			ServoData& s = servo_.servo_handler_.getServo()[servo_index];
			s.p_gain_ = ((data[3] << 8) & 0xFF00) | (data[2] & 0xFF);
			s.i_gain_ = ((data[5] << 8) & 0xFF00) | (data[4] & 0xFF);
			s.d_gain_ = ((data[7] << 8) & 0xFF00) | (data[6] & 0xFF);
			servo_.servo_handler_.setPositionGains(servo_index);
			Flashmemory::erase();
			Flashmemory::write();
			break;
		}
		case CAN::BOARD_CONFIG_SET_SERVO_PROFILE_VEL:
		{
			uint8_t servo_index = data[1];
			ServoData& s = servo_.servo_handler_.getServo()[servo_index];
			s.profile_velocity_ = ((data[3] << 8) & 0xFF00) | (data[2] & 0xFF);
			servo_.servo_handler_.setProfileVelocity(servo_index);
			Flashmemory::erase();
			Flashmemory::write();
			break;
		}
		case CAN::BOARD_CONFIG_SET_SEND_DATA_FLAG:
		{
			uint8_t servo_index = data[1];
			ServoData& s = servo_.servo_handler_.getServo()[servo_index];
			s.send_data_flag_ = data[2];
			Flashmemory::erase();
			Flashmemory::write();
			break;
		}
		case CAN::BOARD_CONFIG_SET_SERVO_CURRENT_LIMIT:
		{
			uint8_t servo_index = data[1];
			ServoData& s = servo_.servo_handler_.getServo()[servo_index];
			s.current_limit_ = ((data[3] << 8) & 0xFF00) | (data[2] & 0xFF);
			servo_.servo_handler_.setCurrentLimit(servo_index);
			break;
		}
		case CAN::BOARD_CONFIG_REBOOT:
		{
			NVIC_SystemReset();
			break;
		}
		case CAN::BOARD_CONFIG_SET_DYNAMIXEL_TTL_RS485_MIXED:
		{
			servo_.servo_handler_.setTTLRS485Mixed(data[1]);
			Flashmemory::erase();
			Flashmemory::write();
			break;
		}
		case CAN::BOARD_CONFIG_SET_EXTERNAL_ENCODER_FLAG:
                  {
                    uint8_t servo_index = data[1];
                    ServoData& s = servo_.servo_handler_.getServo()[servo_index];
                    if(!s.torque_enable_){
                      s.external_encoder_flag_ = data[2];
                      s.first_get_pos_flag_ = true;
                      if(!s.external_encoder_flag_)
                        { // if use the servo internal encoder, we directly output the encoder value without scaling by resolution_ratio.
                          s.servo_resolution_ = 1;
                          s.joint_resolution_ = 1;
                          s.resolution_ratio_ = 1;
                        }
                      Flashmemory::erase();
                      Flashmemory::write();
                    }
                    break;
		}
		case CAN::BOARD_CONFIG_SET_RESOLUTION_RATIO:
                  {
                    uint8_t servo_index = data[1];
                    ServoData& s = servo_.servo_handler_.getServo()[servo_index];
                    if(!s.torque_enable_){
                      s.joint_resolution_ = ((data[3] << 8) & 0xFF00) | (data[2] & 0xFF);
                      s.servo_resolution_ = ((data[5] << 8) & 0xFF00) | (data[4] & 0xFF);
                      s.hardware_error_status_ &= ((1 << RESOLUTION_RATIO_ERROR) - 1); // 0b00111111: reset

                      if(s.servo_resolution_ == 65535 || s.joint_resolution_ == 65535){
                        s.hardware_error_status_ |= (1 << RESOLUTION_RATIO_ERROR);  // 0b01000000;
                        s.resolution_ratio_ = 1;
                      }
                      else{
                        s.resolution_ratio_ = (float)s.servo_resolution_ / (float)s.joint_resolution_;
                        s.first_get_pos_flag_ = true;
                        Flashmemory::erase();
                        Flashmemory::write();
                      }
                    }
                    break;
		}
		case CAN::BOARD_CONFIG_SET_SERVO_ROUND_OFFSET:
		{
			uint8_t servo_index = data[1];
			int32_t ref_value = ((data[5] << 24) & 0xFF000000) | ((data[4] << 16) & 0xFF0000) | ((data[3] << 8) & 0xFF00) | ((data[2] << 0) & 0xFF);
			servo_.servo_handler_.setRoundOffset(servo_index, ref_value);
			break;
		}
		default:
			break;
		}
		sendBoardConfig();
		break;
	default:
		break;
	}
}
