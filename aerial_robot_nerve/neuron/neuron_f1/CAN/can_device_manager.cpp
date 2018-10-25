/*
 * can_device_manager.cpp
 *
 *  Created on: 2016/10/26
 *      Author: anzai
 */

#include "can_device_manager.h"


namespace CANDeviceManager
{
	namespace {
		std::array<CANDevice*, (1 << CAN::DEVICE_ID_LEN)> can_device_list;
		int can_timeout_count = 0;
		constexpr int CAN_MAX_TIMEOUT_COUNT = 100;
		GPIO_TypeDef* m_GPIOx;
		uint16_t m_GPIO_Pin;
	}

	void init(CAN_HandleTypeDef* hcan, uint8_t slave_id, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
	{
		CAN::init(hcan, slave_id);
		m_GPIOx = GPIOx;
		m_GPIO_Pin = GPIO_Pin;
	}

	int makeCommunicationId(uint8_t device_id, uint8_t slave_id)
	{
		return static_cast<int>((device_id << 3) | slave_id);
	}

	void addDevice(CANDevice* device)
	{
		can_device_list[device->getDeviceId()] = device;
	}

	void tick(int cycle /* ms */)
	{
		can_timeout_count++;
		static int internal_count = 0;
		if (can_timeout_count <= CAN_MAX_TIMEOUT_COUNT) {
			internal_count++;
			if (internal_count == (1000 / cycle) - 1) {
				internal_count = 0;
			}
			if (internal_count == 0) {
				HAL_GPIO_TogglePin(m_GPIOx, m_GPIO_Pin);
			}
		}
	}

	void Receive_IT()
	{
		CAN::Receive_IT();
	}

	bool isTimeout()
	{
		return can_timeout_count > CAN_MAX_TIMEOUT_COUNT;
	}

	__weak void userSendMessages()
	{

	}

	__weak void userReceiveMessagesCallback(uint8_t slave_id, uint8_t device_id, uint8_t message_id, uint32_t DLC, uint8_t* data)
	{

	}
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	CANDeviceManager::can_timeout_count = 0;
	CANDeviceManager::Receive_IT();
	uint8_t device_id = CAN::getDeviceId(hcan);
	if (device_id >= CANDeviceManager::can_device_list.size()) return;
	CANDeviceManager::can_device_list[device_id]->receiveDataCallback(CAN::getMessageId(hcan), CAN::getDlc(hcan), CAN::getData(hcan));
	CANDeviceManager::userReceiveMessagesCallback(CAN::getSlaveId(hcan), CAN::getDeviceId(hcan), CAN::getMessageId(hcan), CAN::getDlc(hcan), CAN::getData(hcan));
}


