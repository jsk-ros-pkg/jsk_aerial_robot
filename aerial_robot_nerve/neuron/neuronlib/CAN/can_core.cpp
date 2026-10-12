/*
 * can_core.cpp
 *
 *  Created on: 2016/10/24
 *      Author: anzai
 */

#include "can_core.h"

#if defined(STM32F1) || defined(STM32F4)

namespace CAN {

  namespace {
    //CAN bus registers
    CAN_HandleTypeDef* hcan_;
    volatile uint32_t tx_drop_count = 0;

    void recordTxDrop()
    {
      uint32_t primask = __get_PRIMASK();
      __disable_irq();
      tx_drop_count++;
      if (primask == 0U) __enable_irq();
    }
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
    sFilterConfig.SlaveStartFilterBank = 14; // should not be 0 that is used by CAN1, the slave can be pointed to CAN2
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

  }

  CAN_HandleTypeDef* getHcanInstance()
  {
    return hcan_;
  }

  uint32_t getTxDropCount()
  {
    return tx_drop_count;
  }

  bool sendMessage(uint8_t device_id, uint8_t message_id, uint8_t slave_id, uint32_t dlc, uint8_t* data, uint32_t timeout)
  {
    if (dlc > 8 || (dlc > 0 && data == nullptr)) {
      recordTxDrop();
      return false;
    }

    CAN_TxHeaderTypeDef tx_header = {};
    uint32_t tx_mailbox = 0;
    uint8_t tx_data[8] = {};
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.StdId = (((device_id & ((1 << DEVICE_ID_LEN) - 1))  << (MESSAGE_ID_LEN + SLAVE_ID_LEN))) | ((message_id & ((1 << MESSAGE_ID_LEN) - 1)) << SLAVE_ID_LEN) | (slave_id & ((1 << SLAVE_ID_LEN) - 1));
    tx_header.DLC = dlc;
    if (dlc > 0) memcpy(tx_data, data, sizeof(uint8_t) * dlc);

    uint32_t tickstart = HAL_GetTick();
    while (1) {
      HAL_StatusTypeDef status = HAL_BUSY;
      uint32_t primask = __get_PRIMASK();
      __disable_irq();
      if (HAL_CAN_GetTxMailboxesFreeLevel(hcan_) > 0) {
        status = HAL_CAN_AddTxMessage(hcan_, &tx_header, tx_data, &tx_mailbox);
      }
      if (primask == 0U) __enable_irq();
      if (status == HAL_OK) return true;

      if ((timeout == 0U) || ((HAL_GetTick() - tickstart) > timeout)) {
        recordTxDrop();
        return false;
      }
    }
  }
}
#endif

#ifdef STM32G4
namespace CAN {

  namespace {
    //CAN bus registers
    FDCAN_HandleTypeDef* hfdcan_;
    volatile uint32_t tx_drop_count = 0;

    void recordTxDrop()
    {
      uint32_t primask = __get_PRIMASK();
      __disable_irq();
      tx_drop_count++;
      if (primask == 0U) __enable_irq();
    }
  }

  void init(FDCAN_HandleTypeDef* hfdcan, uint8_t slave_id)
  {
    hfdcan_ = hfdcan;

    FDCAN_FilterTypeDef sFilterConfig;
    /* broadcast message (SLAVE_ID = BROADCAST_ID) */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig.FilterID1 = CAN::BROADCAST_ID; // Filter ID (11bit)
    sFilterConfig.FilterID2 = (1 << SLAVE_ID_LEN) - 1; // Mask ID (11bit)
    HAL_FDCAN_ConfigFilter(hfdcan_, &sFilterConfig);


    /* individual message (SLAVE_ID = CAN::SLAVE_ID) */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 1;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig.FilterID1 = slave_id & ((1 << SLAVE_ID_LEN) - 1); // Filter ID (11bit)
    sFilterConfig.FilterID2 = (1 << SLAVE_ID_LEN) - 1; // Mask ID (11bit)
    HAL_FDCAN_ConfigFilter(hfdcan_, &sFilterConfig);

    /*
       Configure global filter:
       - Reject all remote frames with STD and EXT ID
       - Reject non matching frames with STD ID and EXT ID
    */
    HAL_FDCAN_ConfigGlobalFilter(hfdcan_, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

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

  uint32_t getTxDropCount()
  {
    return tx_drop_count;
  }

  bool sendMessage(uint8_t device_id, uint8_t message_id, uint8_t slave_id, uint32_t dlc, uint8_t* data, uint32_t timeout)
  {
    if (dlc > 64 || (dlc > 0 && data == nullptr)) {
      recordTxDrop();
      return false;
    }

    FDCAN_TxHeaderTypeDef tx_header = {};
    uint8_t tx_data[64] = {};
    tx_header.Identifier = (((device_id & ((1 << DEVICE_ID_LEN) - 1))  << (MESSAGE_ID_LEN + SLAVE_ID_LEN))) | ((message_id & ((1 << MESSAGE_ID_LEN) - 1)) << SLAVE_ID_LEN) | (slave_id & ((1 << SLAVE_ID_LEN) - 1));
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (dlc <= 8) {
      tx_header.FDFormat = FDCAN_CLASSIC_CAN;
      tx_header.BitRateSwitch = FDCAN_BRS_OFF;
      tx_header.DataLength = dlc << 16;
    } else {
      tx_header.FDFormat = FDCAN_FD_CAN;
      tx_header.BitRateSwitch = FDCAN_BRS_ON;
      switch(dlc) {
      case 12:
        {
          tx_header.DataLength = FDCAN_DLC_BYTES_12;
          break;
        }
      case 16:
        {
          tx_header.DataLength = FDCAN_DLC_BYTES_16;
          break;
        }
      case 20:
        {
          tx_header.DataLength = FDCAN_DLC_BYTES_20;
          break;
        }
      case 24:
        {
          tx_header.DataLength = FDCAN_DLC_BYTES_24;
          break;
        }
      case 32:
        {
          tx_header.DataLength = FDCAN_DLC_BYTES_32;
          break;
        }
      case 48:
        {
          tx_header.DataLength = FDCAN_DLC_BYTES_48;
          break;
        }
      case 64:
        {
          tx_header.DataLength = FDCAN_DLC_BYTES_64;
          break;
        }
      default:
        {
          recordTxDrop();
          return false;
        }
      }
    }

    if (dlc > 0) memcpy(tx_data, data, sizeof(uint8_t) * dlc);

    uint32_t tickstart = HAL_GetTick();
    while (1) {
      uint32_t primask = __get_PRIMASK();
      __disable_irq();
      HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(
        getHcanInstance(), &tx_header, tx_data);
      if (primask == 0U) __enable_irq();
      if (status == HAL_OK) return true;

      if ((timeout == 0U) || ((HAL_GetTick() - tickstart) > timeout)) {
        recordTxDrop();
        return false;
      }
    }
  }
}

#endif
