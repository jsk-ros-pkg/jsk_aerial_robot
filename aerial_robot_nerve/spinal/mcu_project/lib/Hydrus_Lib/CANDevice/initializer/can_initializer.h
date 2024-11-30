/*
 * can_configurator.h
 *
 *  Created on: 2017/10/14
 *      Author: anzai
 */

#ifndef APPLICATION_HYDRUS_LIB_CANDEVICE_INITIALIZER_CAN_INITIALIZER_H_
#define APPLICATION_HYDRUS_LIB_CANDEVICE_INITIALIZER_CAN_INITIALIZER_H_

#include "CAN/can_device.h"
#include "Neuron/neuron.h"
#include <spinal/SetBoardConfig.h>
#include <vector>
#include <algorithm>

class CANInitializer : public CANDevice
{
private:
	std::vector<Neuron>& neuron_;
	uint8_t reboot_id_;
	uint32_t reboot_time_;

public:
	CANInitializer(std::vector<Neuron>& neuron):CANDevice(CAN::DEVICEID_INITIALIZER, CAN::MASTER_ID), neuron_(neuron), reboot_id_(0), reboot_time_(0){}
	void initDevices();
	void requestConfig(uint8_t slave_id);
	void configDevice(const spinal::SetBoardConfig::Request& req);
	void sendData() override;
	void receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data) override;
	void addDevice(uint8_t slave_id);
};



#endif /* APPLICATION_HYDRUS_LIB_CANDEVICE_INITIALIZER_CAN_INITIALIZER_H_ */
