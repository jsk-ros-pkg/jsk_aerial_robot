/*
 * initializer.cpp
 *
 *  Created on: 2017/10/14
 *      Author: anzai
 */

#include "initializer.h"

namespace
{
void writeUint16(uint8_t* data, uint16_t value)
{
  data[0] = value & 0xFF;
  data[1] = (value >> 8) & 0xFF;
}

void writeUint32(uint8_t* data, uint32_t value)
{
  data[0] = value & 0xFF;
  data[1] = (value >> 8) & 0xFF;
  data[2] = (value >> 16) & 0xFF;
  data[3] = (value >> 24) & 0xFF;
}

uint16_t clampCount(uint32_t value, uint16_t limit = 0xFFFF)
{
  return value > limit ? limit : static_cast<uint16_t>(value);
}
}


void Initializer::sendData()
{
#ifdef STM32G4
  const uint32_t now = HAL_GetTick();
  if (now - board_state_last_send_tick_ < 1000) return;
  board_state_last_send_tick_ = now;

  uint8_t data[CAN::CANFD_BOARD_STATE_DLC] = {};
  const uint8_t servo_count = std::min<unsigned int>(
    CAN::CANFD_MAX_SERVO_NUM, servo_.servo_handler_.getServoNum());
  data[0] = CAN::CANFD_PROTOCOL_VERSION;
  data[1] = board_state_sequence_++;
  data[2] = servo_count;
  writeUint32(data + 4, now);
  writeUint16(data + 8, clampCount(servo_.takeTargetCount()));
  writeUint16(data + 10, clampCount(motor_.takeTargetCount()));
  writeUint16(data + 12, clampCount(servo_.takeFeedbackCount()));
  writeUint16(data + 14, clampCount(motor_.takeFeedbackCount()));
  writeUint32(data + 16, CANDeviceManager::getRxDropCount());
  writeUint32(data + 20, CAN::getTxDropCount());

  for (uint8_t i = 0; i < servo_count; ++i) {
    const uint32_t current =
      servo_.servo_handler_.getServo()[i].pvc_update_count_;
    const uint32_t count = current - last_servo_control_count_[i];
    last_servo_control_count_[i] = current;
    writeUint16(data + 24 + i * 2, clampCount(count, 0x7FFF));
  }

  sendMessage(CAN::MESSAGEID_SEND_BOARD_STATE, m_slave_id,
              CAN::CANFD_BOARD_STATE_DLC, data, 1);
#endif
}

bool Initializer::sendBoardConfig()
{
		uint8_t data[16] = {};
	data[0] = servo_.servo_handler_.getServoNum();
	data[1] = imu_.send_data_flag_ ? 1 : 0;
	data[2] = servo_.servo_handler_.getTTLRS485Mixed();
	data[3] = servo_.servo_handler_.getPulleySkipThresh() & 0xFF;
	data[4] = (servo_.servo_handler_.getPulleySkipThresh() >> 8) & 0xFF;
#ifdef STM32G4
		data[5] = CAN::CANFD_PROTOCOL_VERSION;
		data[6] = servo_.servo_handler_.getServoFeedbackFrequency() & 0xFF;
		data[7] =
		  (servo_.servo_handler_.getServoFeedbackFrequency() >> 8) & 0xFF;
		const uint32_t watchdog =
		  servo_.servo_handler_.getTargetWatchdogTimeoutMs();
		data[8] = watchdog & 0xFF;
		data[9] = (watchdog >> 8) & 0xFF;
		data[10] = (watchdog >> 16) & 0xFF;
		data[11] = (watchdog >> 24) & 0xFF;
		if (!sendMessage(CAN::MESSAGEID_SEND_INITIAL_CONFIG_0,
		                 m_slave_id, 12, data, 1)) return false;
#else
		if (!sendMessage(CAN::MESSAGEID_SEND_INITIAL_CONFIG_0,
		                 m_slave_id, 5, data, 1)) return false;
#endif
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
		if (!sendMessage(CAN::MESSAGEID_SEND_INITIAL_CONFIG_1,
		                 m_slave_id, 8, data, 1)) return false;
		data[0] = i;
		data[1] = s.getPresentPosition() & 0xFF;
		data[2] = (s.getPresentPosition() >> 8) & 0xFF;
		data[3] = s.profile_velocity_ & 0xFF;
		data[4] = (s.profile_velocity_ >> 8) & 0xFF;
		data[5] = s.current_limit_ & 0xFF;
		data[6] = (s.current_limit_ >> 8) & 0xFF;
		data[7] = (s.send_data_flag_ ? 1 : 0);
		data[7] += ((s.external_encoder_flag_ ? 1 : 0) << 1);
		if (!sendMessage(CAN::MESSAGEID_SEND_INITIAL_CONFIG_2,
		                 m_slave_id, 8, data, 1)) return false;
		data[0] = i;
		data[1] = s.hardware_error_status_;
		data[2] = s.goal_current_ & 0xFF;
		data[3] = (s.goal_current_ >> 8) & 0xFF;
		data[4] = s.joint_resolution_ & 0xFF;
		data[5] = (s.joint_resolution_ >> 8) & 0xFF;
		data[6] = s.servo_resolution_ & 0xFF;
		data[7] = (s.servo_resolution_ >> 8) & 0xFF;
			if (!sendMessage(CAN::MESSAGEID_SEND_INITIAL_CONFIG_3,
			                 m_slave_id, 8, data, 1)) return false;
#ifdef STM32G4
			data[0] = i;
			data[1] = s.mode_;
			data[2] = s.bus_type_;
			data[3] = s.operating_mode_;
			data[4] = s.configured_ ? 1 : 0;
			data[5] = 0;
			data[6] = s.mit_kp_integer_ & 0xFF;
			data[7] = (s.mit_kp_integer_ >> 8) & 0xFF;
			data[8] = s.mit_kp_decimal_ & 0xFF;
			data[9] = (s.mit_kp_decimal_ >> 8) & 0xFF;
			data[10] = s.mit_kd_integer_ & 0xFF;
			data[11] = (s.mit_kd_integer_ >> 8) & 0xFF;
			data[12] = s.mit_kd_decimal_ & 0xFF;
			data[13] = (s.mit_kd_decimal_ >> 8) & 0xFF;
			data[14] = s.mit_current_limit_ & 0xFF;
			data[15] = (s.mit_current_limit_ >> 8) & 0xFF;
			if (!sendMessage(CAN::MESSAGEID_SEND_INITIAL_CONFIG_4,
			                 m_slave_id, 16, data, 1)) return false;
#endif
		}
	return true;
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
		if (sendBoardConfig()) servo_.setConnect(true);
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
			int32_t request[2] = {s.id_, calib_value};
				servo_.queueConfigRequest(
				  CAN::SERVO_CONFIG_SET_HOMING_OFFSET, request, 2);
			break;
		}
		case CAN::BOARD_CONFIG_SET_SERVO_PID_GAIN:
		{
			uint8_t servo_index = data[1];
			ServoData& s = servo_.servo_handler_.getServo()[servo_index];
			int32_t request[4] = {
			  s.id_,
			  ((data[3] << 8) & 0xFF00) | (data[2] & 0xFF),
			  ((data[5] << 8) & 0xFF00) | (data[4] & 0xFF),
			  ((data[7] << 8) & 0xFF00) | (data[6] & 0xFF)};
				servo_.queueConfigRequest(
				  CAN::SERVO_CONFIG_SET_PID_GAIN, request, 4);
			break;
		}
		case CAN::BOARD_CONFIG_SET_SERVO_PROFILE_VEL:
		{
			uint8_t servo_index = data[1];
			ServoData& s = servo_.servo_handler_.getServo()[servo_index];
			int32_t request[2] = {
			  s.id_, ((data[3] << 8) & 0xFF00) | (data[2] & 0xFF)};
				servo_.queueConfigRequest(
				  CAN::SERVO_CONFIG_SET_PROFILE_VELOCITY, request, 2);
			break;
		}
		case CAN::BOARD_CONFIG_SET_SEND_DATA_FLAG:
		{
			uint8_t servo_index = data[1];
			ServoData& s = servo_.servo_handler_.getServo()[servo_index];
			int32_t request[2] = {s.id_, data[2]};
				servo_.queueConfigRequest(
				  CAN::SERVO_CONFIG_SET_SEND_DATA_FLAG, request, 2);
			break;
		}
		case CAN::BOARD_CONFIG_SET_SERVO_CURRENT_LIMIT:
		{
			uint8_t servo_index = data[1];
			ServoData& s = servo_.servo_handler_.getServo()[servo_index];
			int32_t request[2] = {
			  s.id_, ((data[3] << 8) & 0xFF00) | (data[2] & 0xFF)};
				servo_.queueConfigRequest(
				  CAN::SERVO_CONFIG_SET_CURRENT_LIMIT, request, 2);
			break;
		}
		case CAN::BOARD_CONFIG_REBOOT:
		{
			NVIC_SystemReset();
			break;
		}
		case CAN::BOARD_CONFIG_SET_DYNAMIXEL_TTL_RS485_MIXED:
		{
			int32_t request[1] = {data[1]};
				servo_.queueConfigRequest(
				  CAN::SERVO_CONFIG_SET_TTL_RS485_MIXED, request, 1);
			break;
		}
		case CAN::BOARD_CONFIG_SET_EXTERNAL_ENCODER_FLAG:
                  {
                    uint8_t servo_index = data[1];
                    ServoData& s = servo_.servo_handler_.getServo()[servo_index];
	                    int32_t request[2] = {s.id_, data[2]};
				            servo_.queueConfigRequest(
				              CAN::SERVO_CONFIG_SET_EXTERNAL_ENCODER_FLAG,
				              request, 2);
                    break;
		}
		case CAN::BOARD_CONFIG_SET_RESOLUTION_RATIO:
                  {
                    uint8_t servo_index = data[1];
                    ServoData& s = servo_.servo_handler_.getServo()[servo_index];
	                    int32_t request[3] = {
	                      s.id_,
	                      ((data[3] << 8) & 0xFF00) | (data[2] & 0xFF),
	                      ((data[5] << 8) & 0xFF00) | (data[4] & 0xFF)};
				            servo_.queueConfigRequest(
				              CAN::SERVO_CONFIG_SET_RESOLUTION_RATIO,
				              request, 3);
                    break;
		}
		case CAN::BOARD_CONFIG_SET_SERVO_ROUND_OFFSET:
		{
			uint8_t servo_index = data[1];
			int32_t ref_value = ((data[5] << 24) & 0xFF000000) | ((data[4] << 16) & 0xFF0000) | ((data[3] << 8) & 0xFF00) | ((data[2] << 0) & 0xFF);
			servo_.servo_handler_.setRoundOffset(servo_index, ref_value);
			break;
		}
		case CAN::BOARD_CONFIG_SET_SERVO_PULLEY_SKIP_THRESH:
		{
			uint16_t thresh = ((data[2] << 8) & 0xFF00) | (data[1] & 0xFF);
			servo_.servo_handler_.setPulleySkipThresh(thresh);
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
