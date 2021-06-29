/*
 * can_core.cpp
 *
 *  Created on: 2021/06/29
 *      Author: moju zhao
 */

#include "can_core.h"

namespace CAN {

  namespace {
    //CAN bus registers
    FDCAN_HandleTypeDef* hfdcan_;
    FDCAN_TxHeaderTypeDef tx_header_;
    uint8_t tx_data_[8]; // TODO: change to 64 data bytes
  }

  void init(FDCAN_HandleTypeDef* hfdcan)
  {
    hfdcan_ = hfdcan;

    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;
    HAL_FDCAN_ConfigFilter(hfdcan_, &sFilterConfig);
    /*
       Configure global filter:
       - Reject all remote frames with STD and EXT ID
       - Reject non matching frames with STD ID and EXT ID
    */
    HAL_FDCAN_ConfigGlobalFilter(hfdcan_, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    tx_header_.IdType = FDCAN_STANDARD_ID;
    tx_header_.TxFrameType = FDCAN_DATA_FRAME;
    tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header_.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header_.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header_.MessageMarker = 0;
  }

  FDCAN_HandleTypeDef* getHcanInstance()
  {
    return hfdcan_;
  }

  void sendMessage(uint8_t device_id, uint8_t message_id, uint8_t slave_id, uint32_t dlc, uint8_t* data, uint32_t timeout)
  {
    tx_header_.Identifier = (((device_id & ((1 << DEVICE_ID_LEN) - 1))  << (MESSAGE_ID_LEN + SLAVE_ID_LEN))) | ((message_id & ((1 << MESSAGE_ID_LEN) - 1)) << SLAVE_ID_LEN) | (slave_id & ((1 << SLAVE_ID_LEN) - 1));

    //tx_header_.DataLength = FDCAN_DLC_BYTES_8; // sample
    // TODO: we only support 8byte  data
    tx_header_.DataLength =  dlc << 16;
    
    memcpy(tx_data_, data, sizeof(uint8_t) * dlc);

    uint32_t tickstart = HAL_GetTick();
    while (1) {

      if(HAL_FDCAN_AddMessageToTxFifoQ(getHcanInstance(), &tx_header_, tx_data_) == HAL_OK)
        {
          return;
        }

      if((timeout == 0U)||((HAL_GetTick() - tickstart ) > timeout)) return;
    }
  }
}
