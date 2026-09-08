/*
 * servo.h
 *
 *  Created on: 2016/10/28
 *      Author: anzai
 */

#ifndef APPLICATION_SERVO_TEMP_SERVO_H_
#define APPLICATION_SERVO_TEMP_SERVO_H_

#include "CAN/can_device.h"
#include "Servo/Dynamixel/dynamixel_serial.h"
#include <algorithm>
#include <array>

class Initializer;

class Servo : public CANDevice
{
public:
		Servo():state_sequence_(0), last_target_sequence_(0),
		        last_sent_pvc_update_count_{}, config_request_busy_(false),
		        config_request_pending_(false), config_response_required_(false),
		        config_sequence_(0), config_command_(0), config_data_length_(0),
		        config_data_{} {}
		Servo(uint8_t slave_id):CANDevice(CAN::DEVICEID_SERVO, slave_id),
		        state_sequence_(0), last_target_sequence_(0),
		        last_sent_pvc_update_count_{}, config_request_busy_(false),
		        config_request_pending_(false), config_response_required_(false),
		        config_sequence_(0), config_command_(0), config_data_length_(0),
		        config_data_{} {}
		void init(UART_HandleTypeDef* huart, I2C_HandleTypeDef* hi2c, osMutexId* mutex);
		bool update();
		void sendData() override;
		void receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data) override;
		bool queueConfigRequest(uint8_t command, const int32_t* data,
		                        uint8_t data_length,
		                        bool response_required = false,
		                        uint8_t sequence = 0);

	bool getConnect() {return connect_;}
	void setConnect(bool connect) {connect_ = connect;}

private:
	struct CANServoData{
		int16_t angle;
		uint8_t temperature;
		uint8_t status;
		int16_t current;
		uint8_t error;
		CANServoData(uint16_t angle, uint8_t temperature, uint8_t moving, bool force_servo_disable, bool send_target_postion, int16_t current, uint8_t error)
		:angle(angle), temperature(temperature), current(current), error(error)
          {
            status = (force_servo_disable? 1 : 0) | moving << 1 | send_target_postion << 2;
          }
	};

	bool connect_;
	DynamixelSerial servo_handler_;
	friend class Initializer;
	uint8_t state_sequence_;
	uint8_t last_target_sequence_;
	std::array<uint32_t, CAN::CANFD_MAX_SERVO_NUM>
	  last_sent_pvc_update_count_;
	volatile bool config_request_busy_;
	volatile bool config_request_pending_;
	bool config_response_required_;
	uint8_t config_sequence_;
	uint8_t config_command_;
	uint8_t config_data_length_;
	std::array<int32_t, 5> config_data_;
	void processConfigRequest();
	void sendConfigResponse(uint8_t sequence, uint8_t command,
	                        uint8_t result, const int32_t* request_data,
	                        uint8_t data_length);
};


#endif /* APPLICATION_SERVO_SERVO_H_ */
