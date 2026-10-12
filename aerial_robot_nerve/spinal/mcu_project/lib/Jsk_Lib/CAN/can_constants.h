/*
 * can_constants.h
 *
 *  Created on: 2017/10/16
 *      Author: anzai
 */

#ifndef APPLICATION_CAN_CAN_CONSTANTS_H_
#define APPLICATION_CAN_CAN_CONSTANTS_H_

/* Standard ID
 * 10~8bit device ID
 * 7~4bit message ID
 * 3~0bit slave ID
 */

namespace CAN {
	constexpr uint8_t DEVICE_ID_LEN = 3;
	constexpr uint8_t MESSAGE_ID_LEN = 4;
	constexpr uint8_t SLAVE_ID_LEN = 4;
	constexpr uint8_t MASTER_ID = 0;
	constexpr uint8_t BROADCAST_ID = 15;
	constexpr uint8_t DEVICEID_MOTOR = 0;
	constexpr uint8_t DEVICEID_IMU = 1;
	constexpr uint8_t DEVICEID_SERVO = 2;
	constexpr uint8_t DEVICEID_CANFD_TEST = 6;
	constexpr uint8_t DEVICEID_INITIALIZER = 7;
	constexpr uint8_t CANFD_PROTOCOL_VERSION = 2;
	constexpr uint8_t CANFD_MAX_NEURON_NUM = 5;
	constexpr uint8_t CANFD_MAX_SERVO_NUM = 4;
	constexpr uint8_t MESSAGEID_SEND_GYRO = 0;
	constexpr uint8_t MESSAGEID_SEND_ACC = 1;
	constexpr uint8_t MESSAGEID_SEND_MAG = 2;
	constexpr uint8_t MESSAGEID_SEND_IMU_STATE = 3;
	constexpr uint8_t MESSAGEID_RECEIVE_PWM_0_5 = 0;
	constexpr uint8_t MESSAGEID_RECEIVE_PWM_6_11 = 1;
	constexpr uint8_t MESSAGEID_RECEIVE_ROTOR_TARGET = 0;
	constexpr uint8_t MESSAGEID_SEND_ROTOR_STATE = 1;
	constexpr uint8_t MESSAGEID_RECEIVE_SERVO_ANGLE = 0;
	constexpr uint8_t MESSAGEID_RECEIVE_SERVO_CURRENT = 1;
	constexpr uint8_t MESSAGEID_RECEIVE_SERVO_TARGET = 4;
	constexpr uint8_t MESSAGEID_RECEIVE_SERVO_CONFIG = 15;
	constexpr uint8_t MESSAGEID_SEND_SERVO_LIST[4] = {0, 1, 2, 3};
	constexpr uint8_t CANFD_SERVO_TARGET_DLC = 64;
	constexpr uint8_t CANFD_SERVO_STATE_DLC = 16;
	constexpr uint8_t CANFD_SERVO_CONFIG_DLC = 48;
	constexpr uint8_t SERVO_CONFIG_SET_HOMING_OFFSET = 1;
	constexpr uint8_t SERVO_CONFIG_SET_PID_GAIN = 2;
	constexpr uint8_t SERVO_CONFIG_SET_PROFILE_VELOCITY = 3;
	constexpr uint8_t SERVO_CONFIG_SET_SEND_DATA_FLAG = 4;
	constexpr uint8_t SERVO_CONFIG_SET_CURRENT_LIMIT = 5;
	constexpr uint8_t SERVO_CONFIG_SET_TTL_RS485_MIXED = 6;
	constexpr uint8_t SERVO_CONFIG_SET_EXTERNAL_ENCODER_FLAG = 7;
	constexpr uint8_t SERVO_CONFIG_SET_RESOLUTION_RATIO = 8;
	constexpr uint8_t SERVO_CONFIG_SET_CONTROL_MODE = 9;
	constexpr uint8_t SERVO_CONFIG_SET_MIT_PD = 10;
	constexpr uint8_t SERVO_CONFIG_SET_BUS_TYPE = 12;
	constexpr uint8_t SERVO_CONFIG_SET_FEEDBACK_FREQUENCY = 13;
	constexpr uint8_t SERVO_CONFIG_SET_TARGET_WATCHDOG = 15;
	constexpr uint8_t SERVO_CONFIG_RESULT_OK = 0;
	constexpr uint8_t SERVO_CONFIG_RESULT_INVALID = 1;
	constexpr uint8_t SERVO_CONFIG_RESULT_BUSY = 2;
	constexpr uint8_t CANFD_ROTOR_TARGET_DLC = 32;
	constexpr uint8_t CANFD_ROTOR_STATE_DLC = 32;
	constexpr uint8_t CANFD_IMU_STATE_DLC = 24;
	constexpr uint8_t SERVO_TARGET_POSITION_VALID = 1 << 0;
	constexpr uint8_t SERVO_TARGET_VELOCITY_VALID = 1 << 1;
	constexpr uint8_t SERVO_TARGET_CURRENT_VALID = 1 << 2;
	constexpr uint8_t SERVO_TARGET_TORQUE_ENABLE = 1 << 3;
	constexpr uint8_t SERVO_TARGET_POSITION_ACK = 1 << 4;
	constexpr uint8_t SERVO_STATE_PVC_FRESH = 1 << 0;
	constexpr uint8_t SERVO_STATE_MOVING = 1 << 1;
	constexpr uint8_t SERVO_STATE_FORCE_OFF = 1 << 2;
	constexpr uint8_t SERVO_STATE_POSITION_SYNC_REQUEST = 1 << 3;
	constexpr uint8_t SERVO_STATE_TORQUE_ENABLE = 1 << 4;
	constexpr uint8_t SERVO_STATE_WATCHDOG_TIMEOUT = 1 << 5;
	constexpr uint8_t SERVO_STATE_CONFIGURED = 1 << 6;
	constexpr uint8_t ROTOR_STATE_VALID = 1 << 0;
	constexpr uint8_t ROTOR_STATE_FRESH = 1 << 1;
	constexpr uint8_t ROTOR_STATE_TIMEOUT = 1 << 2;
	constexpr uint8_t MESSAGEID_RECEIVE_ENUM_REQUEST = 0;
	constexpr uint8_t MESSAGEID_RECEIVE_INITIAL_CONFIG_REQUEST = 1;
	constexpr uint8_t MESSAGEID_RECEIVE_BOARD_CONFIG_REQUEST = 2;
	constexpr uint8_t MESSAGEID_SEND_ENUM_RESPONSE = 0;
	constexpr uint8_t MESSAGEID_SEND_INITIAL_CONFIG_0 = 1;
	constexpr uint8_t MESSAGEID_SEND_INITIAL_CONFIG_1 = 2;
	constexpr uint8_t MESSAGEID_SEND_INITIAL_CONFIG_2 = 3;
  	constexpr uint8_t MESSAGEID_SEND_INITIAL_CONFIG_3 = 4;
	constexpr uint8_t BOARD_CONFIG_SET_SLAVE_ID = 0;
	constexpr uint8_t MESSAGEID_SEND_INITIAL_CONFIG_4 = 5;
	constexpr uint8_t MESSAGEID_SEND_BOARD_STATE = 6;
	constexpr uint8_t CANFD_BOARD_STATE_DLC = 32;
	constexpr uint8_t BOARD_CONFIG_SET_IMU_SEND_FLAG = 1;
	constexpr uint8_t BOARD_CONFIG_SET_SERVO_HOMING_OFFSET = 2;
	constexpr uint8_t BOARD_CONFIG_SET_SERVO_PID_GAIN = 3;
	constexpr uint8_t BOARD_CONFIG_SET_SERVO_PROFILE_VEL = 4;
	constexpr uint8_t BOARD_CONFIG_SET_SEND_DATA_FLAG = 5;
	constexpr uint8_t BOARD_CONFIG_SET_SERVO_CURRENT_LIMIT = 6;
	constexpr uint8_t BOARD_CONFIG_REBOOT = 7;
	constexpr uint8_t BOARD_CONFIG_SET_DYNAMIXEL_TTL_RS485_MIXED = 8;
        constexpr uint8_t BOARD_CONFIG_SET_EXTERNAL_ENCODER_FLAG = 9;
        constexpr uint8_t BOARD_CONFIG_SET_RESOLUTION_RATIO = 10;
	constexpr uint8_t BOARD_CONFIG_SET_SERVO_ROUND_OFFSET = 11;
	constexpr uint8_t BOARD_CONFIG_SET_SERVO_PULLEY_SKIP_THRESH = 12;
}




#endif /* APPLICATION_CAN_CAN_CONSTANTS_H_ */
