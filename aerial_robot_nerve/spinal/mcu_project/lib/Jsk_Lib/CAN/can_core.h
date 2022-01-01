/*
 * can_core.h
 *
 *  Created on: 2016/10/24
 *      Author: anzai
 */

#ifndef APPLICATION_CAN_CAN_CORE_H_
#define APPLICATION_CAN_CAN_CORE_H_

#include "stdint.h"
#include "can.h"
#include "string.h"
#include <vector>
#include "can_constants.h"

namespace CAN {
	void init(CAN_HandleTypeDef* hcan);
	CAN_HandleTypeDef* getHcanInstance();
	void sendMessage(uint8_t device_id, uint8_t message_id, uint8_t slave_id, uint32_t DLC, uint8_t* data, uint32_t timeout);

	inline void CAN_START()
	{
		HAL_CAN_Start(getHcanInstance());
		HAL_CAN_ActivateNotification(getHcanInstance(), CAN_IT_RX_FIFO1_MSG_PENDING);
	}

	inline uint8_t getDeviceId(CAN_RxHeaderTypeDef rx_header) {
		return static_cast<uint8_t>(((rx_header.StdId) >> (MESSAGE_ID_LEN + SLAVE_ID_LEN)) & ((1 << DEVICE_ID_LEN) - 1));
	}

	inline uint8_t getMessageId(CAN_RxHeaderTypeDef rx_header) {
		return static_cast<uint8_t>(((rx_header.StdId) >> SLAVE_ID_LEN) & ((1 << MESSAGE_ID_LEN) - 1));
	}

	inline uint8_t getSlaveId(CAN_RxHeaderTypeDef rx_header) {
		return static_cast<uint8_t>((rx_header.StdId) & ((1 << SLAVE_ID_LEN) - 1));
	}

	inline uint32_t getDlc(CAN_RxHeaderTypeDef rx_header) {
		return rx_header.DLC;
	}
}

#endif /* APPLICATION_CAN_CAN_CORE_H_ */
