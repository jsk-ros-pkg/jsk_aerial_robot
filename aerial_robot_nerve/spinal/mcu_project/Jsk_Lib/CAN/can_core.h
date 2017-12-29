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
	void setMessage(uint8_t device_id, uint8_t message_id, uint8_t slave_id, uint32_t DLC, uint8_t* data);
	inline void sendMessage(int timeout)
	{
		HAL_CAN_Transmit(getHcanInstance(), timeout);
	}

	inline void Receive_IT()
	{
		HAL_CAN_Receive_IT(getHcanInstance(), CAN_FIFO1);
	}

	inline uint8_t getDeviceId(CAN_HandleTypeDef* hcan) {
		return static_cast<uint8_t>(((hcan->pRxMsg->StdId) >> (MESSAGE_ID_LEN + SLAVE_ID_LEN)) & ((1 << DEVICE_ID_LEN) - 1));
	}

	inline uint8_t getMessageId(CAN_HandleTypeDef* hcan) {
		return static_cast<uint8_t>(((hcan->pRxMsg->StdId) >> SLAVE_ID_LEN) & ((1 << MESSAGE_ID_LEN) - 1));
	}

	inline uint8_t getSlaveId(CAN_HandleTypeDef* hcan) {
		return static_cast<uint8_t>((hcan->pRxMsg->StdId) & ((1 << SLAVE_ID_LEN) - 1));
	}

	inline uint32_t getDlc(CAN_HandleTypeDef* hcan) {
		return hcan->pRxMsg->DLC;
	}

	inline uint8_t* getData(CAN_HandleTypeDef* hcan) {
		return hcan->pRxMsg->Data;
	}
}

#endif /* APPLICATION_CAN_CAN_CORE_H_ */
