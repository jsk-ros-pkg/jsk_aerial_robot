/*
 * servo.h
 *
 *  Created on: 2016/10/28
 *      Author: anzai
 */

#ifndef APPLICATION_SERVO_TEMP_SERVO_H_
#define APPLICATION_SERVO_TEMP_SERVO_H_

#include "can_device.h"
#include "Dynamixel/dynamixel_serial.h"
#include <algorithm>

class Initializer;

class Servo : public CANDevice
{
public:
	Servo(){}
	Servo(uint8_t slave_id):CANDevice(CAN::DEVICEID_SERVO, slave_id){}
	void init(UART_HandleTypeDef* huart, I2C_HandleTypeDef* hi2c, osMutexId* mutex);
	void update();
	void sendData() override;
	void receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data) override;

	bool getConnect() {return connect_;}
	void setConnect(bool connect) {connect_ = connect;}

private:
	struct CANServoData{
		int16_t angle;
		uint8_t temperature;
		uint8_t status;
		int16_t current;
		uint8_t error;
		CANServoData(uint16_t angle, uint8_t temperature, uint8_t moving, bool force_servo_disable, int16_t current, uint8_t error)
		:angle(angle), temperature(temperature), current(current), error(error)
          {
            status = (force_servo_disable? 1 : 0) | moving << 1;
          }
	};

	bool connect_;
	DynamixelSerial servo_handler_;
	friend class Initializer;
};


#endif /* APPLICATION_SERVO_SERVO_H_ */
