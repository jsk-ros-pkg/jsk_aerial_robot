/*
 * can_core.cpp
 *
 *  Created on: 2016/10/24
 *      Author: anzai
 */

#include "can_core.h"

#ifdef STM32F7
namespace CAN {

  namespace {
    //CAN bus registers
    CAN_HandleTypeDef* hcan_;
    CAN_TxHeaderTypeDef tx_header_;
    uint32_t tx_mailbox_;
    uint8_t tx_data_[8];
  }

  void init(CAN_HandleTypeDef* hcan)
  {
    hcan_ = hcan;

    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
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
#endif

#ifdef STM32H7
namespace CAN {

  namespace {
    //CAN bus registers
    FDCAN_HandleTypeDef* hfdcan_;
    FDCAN_TxHeaderTypeDef tx_header_;
    uint8_t tx_data_[64];
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

    tx_header_.BitRateSwitch = FDCAN_BRS_ON; // FDCAN_BRS_OFF;

    tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header_.MessageMarker = 0;

    // www.programmersought.com/article/43428259685/#TDC_568
    // The SSP location is defined as the sum of the measurement delay from the FDCAN_TX
    // pin to the FDCAN_RX pin, plus the transmitter delay compensation offset configured
    // by the TDCO [6: 0] field.
    // TdcOffset = DataTimeSeg1 * DataPrescaler.
    HAL_FDCAN_ConfigTxDelayCompensation(hfdcan_, 7, 0); //Configure hfdcan1
    HAL_FDCAN_EnableTxDelayCompensation(hfdcan_); //Enable TDC of hfdcan1
  }

  FDCAN_HandleTypeDef* getHcanInstance()
  {
    return hfdcan_;
  }

  void sendMessage(uint8_t device_id, uint8_t message_id, uint8_t slave_id, uint32_t dlc, uint8_t* data, uint32_t timeout)
  {
    tx_header_.Identifier = (((device_id & ((1 << DEVICE_ID_LEN) - 1))  << (MESSAGE_ID_LEN + SLAVE_ID_LEN))) | ((message_id & ((1 << MESSAGE_ID_LEN) - 1)) << SLAVE_ID_LEN) | (slave_id & ((1 << SLAVE_ID_LEN) - 1));

    if (dlc <= 8) { // calssic  model
      tx_header_.FDFormat = FDCAN_CLASSIC_CAN;
      tx_header_.DataLength =  dlc << 16;
    }
    else {
      tx_header_.FDFormat = FDCAN_FD_CAN;
      switch(dlc) {
      case 12:
        {
          tx_header_.DataLength = FDCAN_DLC_BYTES_12;
          break;
        }
      case 16:
        {
          tx_header_.DataLength = FDCAN_DLC_BYTES_16;
          break;
        }
      case 20:
        {
          tx_header_.DataLength = FDCAN_DLC_BYTES_20;
          break;
        }
      case 24:
        {
          tx_header_.DataLength = FDCAN_DLC_BYTES_24;
          break;
        }
      case 32:
        {
          tx_header_.DataLength = FDCAN_DLC_BYTES_32;
          break;
        }
      case 48:
        {
          tx_header_.DataLength = FDCAN_DLC_BYTES_48;
          break;
        }
      case 64:
        {
          tx_header_.DataLength = FDCAN_DLC_BYTES_64;
          break;
        }
      default:
        {
          // Not supported
          return;
        }
      }
    }

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
#endif
