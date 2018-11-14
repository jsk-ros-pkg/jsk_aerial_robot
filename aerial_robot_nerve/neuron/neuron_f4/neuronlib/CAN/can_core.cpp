/*
 * can_core.cpp
 *
 *  Created on: 2016/10/24
 *      Author: anzai
 */

#include "can_core.h"
#include "can_device.h"

namespace CAN {

	namespace {
		//CAN bus registers
		CanTxMsgTypeDef Can_tx;
		CanRxMsgTypeDef Can_re;
		CAN_HandleTypeDef* hcan_;
	}

	void init(CAN_HandleTypeDef* hcan, uint8_t slave_id)
	{
		hcan_ = hcan;
		hcan_->pTxMsg = &Can_tx;
		hcan_->pRxMsg = &Can_re;
		//default IDE and RTR type,
		hcan_->pTxMsg->RTR = CAN_RTR_DATA;
		hcan_->pTxMsg->IDE = CAN_ID_STD;
		hcan_->pRxMsg->FIFONumber = CAN_FIFO1;

		CAN_FilterConfTypeDef sFilterConfig;

		/* broadcast message (SLAVE_ID = BROADCAST_ID) */
		sFilterConfig.FilterNumber = 0;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterIdHigh = CAN::BROADCAST_ID << 5;
		sFilterConfig.FilterIdLow = 0x0000;
		sFilterConfig.FilterMaskIdHigh = ((1 << SLAVE_ID_LEN) - 1) << 5;
		sFilterConfig.FilterMaskIdLow = 0x0000;
		sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
		sFilterConfig.FilterActivation = ENABLE;
		HAL_CAN_ConfigFilter(hcan_, &sFilterConfig);

		/* individual message (SLAVE_ID = CAN::SLAVE_ID) */
		sFilterConfig.FilterNumber = 1;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterIdHigh = (slave_id & ((1 << SLAVE_ID_LEN) - 1)) << 5;
		sFilterConfig.FilterIdLow = 0x0000;
		sFilterConfig.FilterMaskIdHigh = ((1 << SLAVE_ID_LEN) - 1) << 5;
		sFilterConfig.FilterMaskIdLow = 0x0000;
		sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
		sFilterConfig.FilterActivation = ENABLE;
		HAL_CAN_ConfigFilter(hcan_, &sFilterConfig);
	}

	CAN_HandleTypeDef* getHcanInstance()
	{
		return hcan_;
	}

	void setMessage(uint8_t device_id, uint8_t message_id, uint8_t slave_id, uint32_t dlc, uint8_t* data)
	{
		hcan_->pTxMsg->StdId = (((device_id & ((1 << DEVICE_ID_LEN) - 1))  << (MESSAGE_ID_LEN + SLAVE_ID_LEN))) | ((message_id & ((1 << MESSAGE_ID_LEN) - 1)) << SLAVE_ID_LEN) | (slave_id & ((1 << SLAVE_ID_LEN) - 1));
		hcan_->pTxMsg->DLC = dlc;
		memcpy(hcan_->pTxMsg->Data, data, sizeof(uint8_t) * dlc);
	}
}
