/*
 * can_configurator.cpp
 *
 *  Created on: 2017/10/14
 *      Author: anzai
 */

#include <CANDevice/initializer/can_initializer.h>

void CANInitializer::sendData()
{
  // reconnection with reboot neuron
  if (reboot_id_ > 0) {

    // wait for the neuron completely reboot, 5000ms
    if (HAL_GetTick() - reboot_time_ > 5000) {
      requestConfig(reboot_id_);
      reboot_time_ = 0;
      reboot_id_ = 0;
    }
  }
}

void CANInitializer::initDevices()
{
	sendMessage(CAN::MESSAGEID_RECEIVE_ENUM_REQUEST, CAN::BROADCAST_ID, 0, nullptr, 1);
	HAL_Delay(200);
	std::sort(neuron_.begin(), neuron_.end());
	for (unsigned int i = 0; i < neuron_.size(); i++) {
		uint8_t slave_id = neuron_.at(i).getSlaveId();
		requestConfig(slave_id);
		HAL_Delay(200);
	}
}

void CANInitializer::requestConfig(uint8_t slave_id)
{
	sendMessage(CAN::MESSAGEID_RECEIVE_INITIAL_CONFIG_REQUEST, slave_id, 0, nullptr, 1);
}

void CANInitializer::configDevice(const spinal::SetBoardConfig::Request& req)
{
	uint8_t slave_id = static_cast<uint8_t>(req.data[0]);
	switch (req.command) {
		case spinal::SetBoardConfig::Request::SET_SLAVE_ID:
		{
			uint8_t new_slave_id = static_cast<uint8_t>(req.data[1]);
			uint8_t send_data[2];
			send_data[0] = CAN::BOARD_CONFIG_SET_SLAVE_ID;
			send_data[1] = new_slave_id;
			sendMessage(CAN::MESSAGEID_RECEIVE_BOARD_CONFIG_REQUEST, slave_id, 2, send_data, 1);
                        // timeout of 1ms is OK for RTOS, since this function is called from nh_.spinOnce which is called in a lower priority task
			break;
		}
		case spinal::SetBoardConfig::Request::SET_IMU_SEND_FLAG:
		{
			uint8_t imu_send_data_flag = static_cast<uint8_t>(req.data[1]);
			uint8_t send_data[2];
			send_data[0] = CAN::BOARD_CONFIG_SET_IMU_SEND_FLAG;
			send_data[1] = imu_send_data_flag;
			sendMessage(CAN::MESSAGEID_RECEIVE_BOARD_CONFIG_REQUEST, slave_id, 2, send_data, 1);
			break;
		}
		case spinal::SetBoardConfig::Request::SET_SERVO_HOMING_OFFSET:
		{
			uint8_t servo_index = static_cast<uint8_t>(req.data[1]);
			int32_t joint_offset = req.data[2];
			uint8_t send_data[6];
			send_data[0] = CAN::BOARD_CONFIG_SET_SERVO_HOMING_OFFSET;
			send_data[1] = servo_index;
			send_data[2] = joint_offset & 0xFF;
			send_data[3] = (joint_offset >> 8) & 0xFF;
			send_data[4] = (joint_offset >> 16) & 0xFF;
			send_data[5] = (joint_offset >> 24) & 0xFF;
			sendMessage(CAN::MESSAGEID_RECEIVE_BOARD_CONFIG_REQUEST, slave_id, 6, send_data, 1);
			break;
		}
		case spinal::SetBoardConfig::Request::SET_SERVO_PID_GAIN:
		{
			uint8_t servo_index = static_cast<uint8_t>(req.data[1]);
			int16_t p_gain = static_cast<uint16_t>(req.data[2]);
			int16_t i_gain = static_cast<uint16_t>(req.data[3]);
			int16_t d_gain = static_cast<uint16_t>(req.data[4]);
			uint8_t send_data[8];
			send_data[0] = CAN::BOARD_CONFIG_SET_SERVO_PID_GAIN;
			send_data[1] = servo_index;
			send_data[2] = p_gain & 0xFF;
			send_data[3] = (p_gain >> 8) & 0xFF;
			send_data[4] = i_gain & 0xFF;
			send_data[5] = (i_gain >> 8) & 0xFF;
			send_data[6] = d_gain & 0xFF;
			send_data[7] = (d_gain >> 8) & 0xFF;
			sendMessage(CAN::MESSAGEID_RECEIVE_BOARD_CONFIG_REQUEST, slave_id, 8, send_data, 1);
			break;
		}
		case spinal::SetBoardConfig::Request::SET_SERVO_PROFILE_VEL:
		{
			uint8_t servo_index = static_cast<uint8_t>(req.data[1]);
			int16_t profile_vel = static_cast<uint16_t>(req.data[2]);
			uint8_t send_data[4];
			send_data[0] = CAN::BOARD_CONFIG_SET_SERVO_PROFILE_VEL;
			send_data[1] = servo_index;
			send_data[2] = profile_vel & 0xFF;
			send_data[3] = (profile_vel >> 8) & 0xFF;
			sendMessage(CAN::MESSAGEID_RECEIVE_BOARD_CONFIG_REQUEST, slave_id, 4, send_data, 1);
			break;
		}
		case spinal::SetBoardConfig::Request::SET_SERVO_SEND_DATA_FLAG:
		{
			uint8_t servo_index = static_cast<uint8_t>(req.data[1]);
			uint8_t servo_send_data_flag = static_cast<uint8_t>(req.data[2]);
			uint8_t send_data[3];
			send_data[0] = CAN::BOARD_CONFIG_SET_SEND_DATA_FLAG;
			send_data[1] = servo_index;
			send_data[2] = servo_send_data_flag & 0xFF;
			sendMessage(CAN::MESSAGEID_RECEIVE_BOARD_CONFIG_REQUEST, slave_id, 3, send_data, 1);
			break;
		}
		case spinal::SetBoardConfig::Request::SET_SERVO_CURRENT_LIMIT:
		{
			uint8_t servo_index = static_cast<uint8_t>(req.data[1]);
			int16_t current_limit = static_cast<uint16_t>(req.data[2]);
			uint8_t send_data[4];
			send_data[0] = CAN::BOARD_CONFIG_SET_SERVO_CURRENT_LIMIT;
			send_data[1] = servo_index;
			send_data[2] = current_limit & 0xFF;
			send_data[3] = (current_limit >> 8) & 0xFF;
			sendMessage(CAN::MESSAGEID_RECEIVE_BOARD_CONFIG_REQUEST, slave_id, 4, send_data, 1);
			break;
		}
                case spinal::SetBoardConfig::Request::SET_SERVO_EXTERNAL_ENCODER_FLAG:
                {
                  uint8_t servo_index = static_cast<uint8_t>(req.data[1]);
                  uint8_t servo_send_data_flag = static_cast<uint8_t>(req.data[2]);
                  uint8_t send_data[3];
                  send_data[0] = CAN::BOARD_CONFIG_SET_EXTERNAL_ENCODER_FLAG;
                  send_data[1] = servo_index;
                  send_data[2] = servo_send_data_flag & 0xFF;
                  sendMessage(CAN::MESSAGEID_RECEIVE_BOARD_CONFIG_REQUEST, slave_id, 3, send_data, 1);
                  break;
                }
                case spinal::SetBoardConfig::Request::SET_SERVO_RESOLUTION_RATIO:
                {
                  uint8_t servo_index = static_cast<uint8_t>(req.data[1]);
                  uint16_t joint_resolution = static_cast<uint16_t>(req.data[2]);
                  uint16_t servo_resolution = static_cast<uint16_t>(req.data[3]);
                  uint8_t send_data[6];
                  send_data[0] = CAN::BOARD_CONFIG_SET_RESOLUTION_RATIO;
                  send_data[1] = servo_index;
                  send_data[2] = joint_resolution & 0xFF;
                  send_data[3] = (joint_resolution >> 8) & 0xFF;
                  send_data[4] = servo_resolution & 0xFF;
                  send_data[5] = (servo_resolution >> 8) & 0xFF;
                  sendMessage(CAN::MESSAGEID_RECEIVE_BOARD_CONFIG_REQUEST, slave_id, 6, send_data, 1);
                  break;
                }
		case spinal::SetBoardConfig::Request::REBOOT:
		{
			uint8_t send_data[1];
			send_data[0] = CAN::BOARD_CONFIG_REBOOT;
			sendMessage(CAN::MESSAGEID_RECEIVE_BOARD_CONFIG_REQUEST, slave_id, 1, send_data, 1);
			reboot_id_ = slave_id;
			reboot_time_ = HAL_GetTick();
			break;
		}
		case spinal::SetBoardConfig::Request::SET_DYNAMIXEL_TTL_RS485_MIXED:
		{
			uint8_t dynamixel_ttl_rs485_mixed = static_cast<uint8_t>(req.data[1]);
			uint8_t send_data[2];
			send_data[0] = CAN::BOARD_CONFIG_SET_DYNAMIXEL_TTL_RS485_MIXED;
			send_data[1] = dynamixel_ttl_rs485_mixed;
			sendMessage(CAN::MESSAGEID_RECEIVE_BOARD_CONFIG_REQUEST, slave_id, 2, send_data, 1);
			break;
		}
		case CAN::BOARD_CONFIG_SET_SERVO_ROUND_OFFSET:
		{
			uint8_t servo_index = static_cast<uint8_t>(req.data[1]);
			int32_t ref_value = req.data[2];
			uint8_t send_data[6];
			send_data[0] = CAN::BOARD_CONFIG_SET_SERVO_ROUND_OFFSET;
			send_data[1] = servo_index;
			send_data[2] = ref_value & 0xFF;
			send_data[3] = (ref_value >> 8) & 0xFF;
			send_data[4] = (ref_value >> 16) & 0xFF;
			send_data[5] = (ref_value >> 24) & 0xFF;
			sendMessage(CAN::MESSAGEID_RECEIVE_BOARD_CONFIG_REQUEST, slave_id, 6, send_data, 1);
			break;
		}
		default:
			break;
	}
}


void CANInitializer::addDevice(uint8_t slave_id)
{
  neuron_.push_back(Neuron(slave_id));
}

void CANInitializer::receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data)
{
  if(message_id == CAN::MESSAGEID_SEND_ENUM_RESPONSE)
    {
      /*
        Important feature1:
         Directly call "push_back" will induce hard default in RTOS task, due to the wrong malloc process in RTOS.
         Thus, provide an another function "addDevice" to wrapper this function can avoid the direct calling (reserve in stack zone).
         Please refer to the register mechanism in ARM: https://qiita.com/edo_m18/items/a7c747c5bed600dca977
         https://developer.arm.com/documentation/dui0646/a/cortex-m7-peripherals/system-control-block
       */
      //neuron_.push_back(Neuron(slave_id));
      addDevice(slave_id);
      return;
    }

  /*
    Important feature2:
    std::find in RTOS also induces hard default.
    Thus, use the most trivial search method using for() as follows
  */
  int index = -1;
  for (int i = 0; i < neuron_.size(); i++)
    {
      if(neuron_[i].getSlaveId() == slave_id) index = i;
    }
  if (index == -1) return;

  switch (message_id)
    {
    case CAN::MESSAGEID_SEND_INITIAL_CONFIG_0:
      {
        if (neuron_[index].getInitialized()) {
          neuron_[index].can_imu_.setSendDataFlag((data[1] != 0) ? true : false);
          neuron_[index].can_servo_.setDynamixelTTLRS485Mixed((data[2] != 0) ? true : false);
          return;
        }
        neuron_[index].can_motor_ = CANMotor(slave_id);
        neuron_[index].can_servo_ = CANServo(slave_id, data[0], (data[2] != 0) ? true : false);
        neuron_[index].can_imu_ = CANIMU(slave_id, (data[1] != 0) ? true : false);
        neuron_[index].setInitialized();
        break;
      }
    case CAN::MESSAGEID_SEND_INITIAL_CONFIG_1:
      {
        uint8_t servo_index = data[0];
        neuron_[index].can_servo_.servo_[servo_index].id_ = data[1];
        neuron_[index].can_servo_.servo_[servo_index].p_gain_ = (data[3] << 8) | data[2];
        neuron_[index].can_servo_.servo_[servo_index].i_gain_ = (data[5] << 8) | data[4];
        neuron_[index].can_servo_.servo_[servo_index].d_gain_ = (data[7] << 8) | data[6];
        break;
      }
    case CAN::MESSAGEID_SEND_INITIAL_CONFIG_2:
      {
        uint8_t servo_index = data[0];
        neuron_[index].can_servo_.servo_[servo_index].present_position_ = (data[2] << 8) | data[1];
        neuron_[index].can_servo_.servo_[servo_index].goal_position_ = (data[2] << 8) | data[1];
        neuron_[index].can_servo_.servo_[servo_index].profile_velocity_ = (data[4] << 8) | data[3];
        neuron_[index].can_servo_.servo_[servo_index].current_limit_ = (data[6] << 8) | data[5];
        neuron_[index].can_servo_.servo_[servo_index].send_data_flag_ = data[7];
        break;
      }
    case CAN::MESSAGEID_SEND_INITIAL_CONFIG_3:
      {
        uint8_t servo_index = data[0];
        neuron_[index].can_servo_.servo_[servo_index].error_ = data[1];
        neuron_[index].can_servo_.servo_[servo_index].external_encoder_flag_ = data[2];
        neuron_[index].can_servo_.servo_[servo_index].joint_resolution_ = (data[4] << 8) | data[3];
        neuron_[index].can_servo_.servo_[servo_index].servo_resolution_ = (data[6] << 8) | data[5];
        break;
      }
    default:
      {
        break;
      }
    }
}
