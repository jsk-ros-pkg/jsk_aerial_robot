/*
 * can_device.h
 *
 *  Created on: 2016/10/24
 *      Author: anzai
 */

#ifndef APPLICATION_CAN_CAN_DEVICE_H_
#define APPLICATION_CAN_CAN_DEVICE_H_

#include "can_core.h"

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
	virtual void receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data) = 0;
};


#endif /* APPLICATION_CAN_CAN_DEVICE_H_ */
