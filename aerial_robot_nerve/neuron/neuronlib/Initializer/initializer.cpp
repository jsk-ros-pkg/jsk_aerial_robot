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
	setMessage(CAN::MESSAGEID_SEND_INITIAL_CONFIG_0, m_slave_id, 3, data);
	sendMessage(1);
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
		setMessage(CAN::MESSAGEID_SEND_INITIAL_CONFIG_1, m_slave_id, 8, data);
		sendMessage(1);
		data[0] = i;
		data[1] = s.getPresentPosition() & 0xFF;
		data[2] = (s.getPresentPosition() >> 8) & 0xFF;
		data[3] = s.profile_velocity_ & 0xFF;
		data[4] = (s.profile_velocity_ >> 8) & 0xFF;
		data[5] = s.current_limit_ & 0xFF;
		data[6] = (s.current_limit_ >> 8) & 0xFF;
		data[7] = (s.send_data_flag_ ? 1 : 0);
		setMessage(CAN::MESSAGEID_SEND_INITIAL_CONFIG_2, m_slave_id, 8, data);
		sendMessage(1);
	}
}

void Initializer::receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data)
{
	switch (message_id) {
	case CAN::MESSAGEID_RECEIVE_ENUM_REQUEST:
		setMessage(CAN::MESSAGEID_SEND_ENUM_RESPONSE, m_slave_id, 0, nullptr);
		sendMessage(1);
		break;
	case CAN::MESSAGEID_RECEIVE_INITIAL_CONFIG_REQUEST:
	{
		sendBoardConfig();
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
			int32_t offset_value = ((data[5] << 24) & 0xFF000000) | ((data[4] << 16) & 0xFF0000) | ((data[3] << 8) & 0xFF00) | ((data[2] << 0) & 0xFF);
			s.offset_value_ = offset_value;
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
		default:
			break;
		}
		sendBoardConfig();
		break;
	default:
		break;
	}
}
