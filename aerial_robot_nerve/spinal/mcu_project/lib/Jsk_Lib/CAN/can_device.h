/*
 * can_device.h
 *
 *  Created on: 2016/10/24
 *      Author: anzai
 */

#ifndef APPLICATION_CAN_CAN_DEVICE_H_
#define APPLICATION_CAN_CAN_DEVICE_H_

#include "can_core.h"

/* Standard ID
 * 10~8bit device ID
 * 7~4bit message ID
 * 3~0bit slave ID
 */

class CANDevice
{
protected:
	uint8_t m_device_id, m_slave_id;
public:
	CANDevice(){}
	CANDevice(uint8_t device_id, uint8_t slave_id):m_device_id(device_id), m_slave_id(slave_id){}
	uint8_t getDeviceId(){return m_device_id;}
	uint8_t getSlaveId(){return m_slave_id;}
	void sendMessage(uint8_t message_id, uint8_t slave_id, uint32_t dlc, uint8_t* data, uint32_t timeout){CAN::sendMessage(m_device_id, message_id, slave_id, dlc, data, timeout);}
	virtual void sendData() = 0;
	virtual void receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data) = 0;
};



class CANDirectDevice
{
protected:
	uint32_t m_identifier;
public:
	CANDirectDevice(){}
	CANDirectDevice(uint32_t identifier):m_identifier(identifier){}
	void sendMessage(uint32_t identifier, uint32_t dlc, uint8_t* data, uint32_t timeout, bool is_extended_id=false){CAN::sendMessage(identifier, dlc, data, timeout, is_extended_id);}
	virtual void sendData() = 0;
	virtual void receiveDataCallback(uint32_t identifier, uint32_t dlc, uint8_t* data) = 0;
};


#endif /* APPLICATION_CAN_CAN_DEVICE_H_ */
