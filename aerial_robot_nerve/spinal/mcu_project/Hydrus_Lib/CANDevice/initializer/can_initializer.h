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
#include <hydrus/BoardConfigCmd.h>
#include <vector>
#include <algorithm>

class CANInitializer : public CANDevice
{
private:
	std::vector<Neuron>& neuron_;
public:
	CANInitializer(std::vector<Neuron>& neuron):CANDevice(CAN::DEVICEID_INITIALIZER, CAN::MASTER_ID), neuron_(neuron){}
	void initDevices();
	void configDevice(const hydrus::BoardConfigCmd& config_msg);
	void sendData() override;
	void receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data) override;
};



#endif /* APPLICATION_HYDRUS_LIB_CANDEVICE_INITIALIZER_CAN_INITIALIZER_H_ */
