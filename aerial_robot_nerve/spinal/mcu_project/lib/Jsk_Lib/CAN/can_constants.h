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
	constexpr uint8_t DEVICEID_INITIALIZER = 7;
	constexpr uint8_t MESSAGEID_SEND_GYRO = 0;
	constexpr uint8_t MESSAGEID_SEND_ACC = 1;
	constexpr uint8_t MESSAGEID_SEND_MAG = 2;
	constexpr uint8_t MESSAGEID_RECEIVE_PWM_0_5 = 0;
	constexpr uint8_t MESSAGEID_RECEIVE_PWM_6_11 = 1;
	constexpr uint8_t MESSAGEID_RECEIVE_SERVO_ANGLE = 0;
	constexpr uint8_t MESSAGEID_RECEIVE_SERVO_CONFIG = 15;
	constexpr uint8_t MESSAGEID_SEND_SERVO_LIST[4] = {0, 1, 2, 3};
	constexpr uint8_t MESSAGEID_RECEIVE_ENUM_REQUEST = 0;
	constexpr uint8_t MESSAGEID_RECEIVE_INITIAL_CONFIG_REQUEST = 1;
	constexpr uint8_t MESSAGEID_RECEIVE_BOARD_CONFIG_REQUEST = 2;
	constexpr uint8_t MESSAGEID_SEND_ENUM_RESPONSE = 0;
	constexpr uint8_t MESSAGEID_SEND_INITIAL_CONFIG_0 = 1;
	constexpr uint8_t MESSAGEID_SEND_INITIAL_CONFIG_1 = 2;
	constexpr uint8_t MESSAGEID_SEND_INITIAL_CONFIG_2 = 3;
  	constexpr uint8_t MESSAGEID_SEND_INITIAL_CONFIG_3 = 4;
	constexpr uint8_t BOARD_CONFIG_SET_SLAVE_ID = 0;
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
}




#endif /* APPLICATION_CAN_CAN_CONSTANTS_H_ */
