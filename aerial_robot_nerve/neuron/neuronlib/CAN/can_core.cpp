/*
 * can_core.cpp
 *
 *  Created on: 2016/10/24
 *      Author: anzai
 */

#include "can_core.h"

namespace CAN {

	namespace {
		//CAN bus registers
		CAN_HandleTypeDef* hcan_;

		CAN_TxHeaderTypeDef tx_header_;
		uint32_t tx_mailbox_;
		uint8_t tx_data_[8];
	}

	void init(CAN_HandleTypeDef* hcan, uint8_t slave_id)
	{
		hcan_ = hcan;
		CAN_FilterTypeDef sFilterConfig;

		/* broadcast message (SLAVE_ID = BROADCAST_ID) */
		sFilterConfig.FilterBank = 0;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterIdHigh = CAN::BROADCAST_ID << 5;
		sFilterConfig.FilterIdLow = 0x0000;
		sFilterConfig.FilterMaskIdHigh = ((1 << SLAVE_ID_LEN) - 1) << 5;
		sFilterConfig.FilterMaskIdLow = 0x0000;
		sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
		sFilterConfig.SlaveStartFilterBank = 14;
		sFilterConfig.FilterActivation = ENABLE;
		HAL_CAN_ConfigFilter(hcan_, &sFilterConfig);

		/* individual message (SLAVE_ID = CAN::SLAVE_ID) */
		sFilterConfig.FilterBank = 1;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterIdHigh = (slave_id & ((1 << SLAVE_ID_LEN) - 1)) << 5;
		sFilterConfig.FilterIdLow = 0x0000;
		sFilterConfig.FilterMaskIdHigh = ((1 << SLAVE_ID_LEN) - 1) << 5;
		sFilterConfig.FilterMaskIdLow = 0x0000;
		sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
		sFilterConfig.SlaveStartFilterBank = 14;
		sFilterConfig.FilterActivation = ENABLE;
		HAL_CAN_ConfigFilter(hcan_, &sFilterConfig);

		tx_header_.RTR = CAN_RTR_DATA;
		tx_header_.IDE = CAN_ID_STD;
	}

	CAN_HandleTypeDef* getHcanInstance()
	{
		return hcan_;
	}

  	void sendMessage(uint8_t device_id, uint8_t message_id, uint8_t slave_id, uint32_t dlc, uint8_t* data, uint32_t timeout)
	{
          tx_header_.StdId = (((device_id & ((1 << DEVICE_ID_LEN) - 1))  << (MESSAGE_ID_LEN + SLAVE_ID_LEN))) | ((message_id & ((1 << MESSAGE_ID_LEN) - 1)) << SLAVE_ID_LEN) | (slave_id & ((1 << SLAVE_ID_LEN) - 1));
          tx_header_.DLC = dlc;
          memcpy(tx_data_, data, sizeof(uint8_t) * dlc);

          uint32_t tickstart = HAL_GetTick();
          while (1) {

            if(HAL_CAN_GetTxMailboxesFreeLevel(hcan_) > 0){
              HAL_CAN_AddTxMessage(hcan_, &tx_header_, tx_data_, &tx_mailbox_);
              return;
            }

            if((timeout == 0U)||((HAL_GetTick() - tickstart ) > timeout)) return;
          }
	}
}
