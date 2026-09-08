/*
 * can_device_manager.cpp
 *
 *  Created on: 2016/10/26
 *      Author: anzai
 */

#include "can_device_manager.h"
#include <map>

namespace CANDeviceManager
{
#ifdef STM32F7
  using CAN_GeranlHandleTypeDef = CAN_HandleTypeDef;
#endif

#ifdef STM32H7
  using CAN_GeranlHandleTypeDef = FDCAN_HandleTypeDef;
#endif

  namespace {
    std::map<int, CANDevice&> can_device_list;
    int can_timeout_count = 0;
    constexpr int CAN_MAX_TIMEOUT_COUNT = 100;
    GPIO_TypeDef* m_GPIOx;
    uint16_t m_GPIO_Pin;
    osMailQId* canMsgMailHandle = NULL;
    volatile uint32_t rx_drop_count = 0;
  }

  void recordRxDrop()
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    rx_drop_count++;
    if (primask == 0U) __enable_irq();
  }

  uint32_t getRxDropCount()
  {
    return rx_drop_count;
  }

  void init(CAN_GeranlHandleTypeDef* hcan, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
  {
    CAN::init(hcan);
    m_GPIOx = GPIOx;
    m_GPIO_Pin = GPIO_Pin;
  }

  void useRTOS(osMailQId* handle)
  {
    canMsgMailHandle = handle;
  }

  int makeCommunicationId(uint8_t device_id, uint8_t slave_id)
  {
    return static_cast<int>((device_id << CAN::SLAVE_ID_LEN) | slave_id);
  }

  void addDevice(CANDevice& device)
  {
    can_device_list.insert(std::pair<int, CANDevice& >(makeCommunicationId(device.getDeviceId(), device.getSlaveId()), device));
  }

  void tick(int cycle /* ms */)
  {
    can_timeout_count++;
    static int internal_count = 0;
    if (can_timeout_count <= CAN_MAX_TIMEOUT_COUNT) {
      internal_count++;
      if (internal_count == 1000 / cycle) {
        internal_count = 0;
      }
      if (internal_count == 0) {
        HAL_GPIO_TogglePin(m_GPIOx, m_GPIO_Pin);
      }
    }
  }

  bool connected(void)
  {
    if(can_timeout_count > CAN_MAX_TIMEOUT_COUNT) return false;
    else return true;
  }

  void CAN_START()
  {
    CAN::CAN_START();
  }

  void receiveMessage(can_msg msg)
  {
#ifdef STM32H7
    const uint32_t dlc = CAN::getDlc(msg.rx_header);
    if (!CAN::validFrameFormat(msg.rx_header, dlc)) {
      recordRxDrop();
      return;
    }
    CANDeviceManager::can_timeout_count = 0;
#endif
    uint8_t slave_id = CAN::getSlaveId(msg.rx_header);
    uint8_t device_id = CAN::getDeviceId(msg.rx_header);
    uint8_t message_id = CAN::getMessageId(msg.rx_header);
#ifndef STM32H7
    uint32_t dlc = CAN::getDlc(msg.rx_header);
#endif

    int communication_id = CANDeviceManager::makeCommunicationId(device_id, slave_id);
    if (device_id == CAN::DEVICEID_INITIALIZER) { //special
      communication_id = CANDeviceManager::makeCommunicationId(device_id, CAN::MASTER_ID);
    }
    if (CANDeviceManager::can_device_list.count(communication_id) == 0) return;

    /* 
       CAUTION: canRxTask and coreTask (including spinal process) has same priotity (realtime)
       Therefore, no need to use mutex for following callback functions in canRXTask.
       However, if canRxTask has higher priority in the future, for example spinal process use an independent task wich lower priority,
       we will need to add mutex for set/get function for internal variable.
    */
    CANDeviceManager::can_device_list.at(communication_id).receiveDataCallback(slave_id, message_id, dlc, msg.rx_data);
  }
}

#ifdef STM32F7
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  CANDeviceManager::can_timeout_count = 0;

  if(CANDeviceManager::canMsgMailHandle == NULL)
    {
      can_msg msg;
      if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &msg.rx_header, msg.rx_data) == HAL_OK)
        {
          /* directly call receivemessage, since RTOS is not initialized */
          CANDeviceManager::receiveMessage(msg);
        }
      else
        {
          CANDeviceManager::recordRxDrop();
        }
    }
  else
    {
      /* add data to RTOS queue */
      can_msg *msg = (can_msg *)osMailAlloc(*CANDeviceManager::canMsgMailHandle, 0);

      if (msg == NULL) {
        CANDeviceManager::recordRxDrop();
        return;
      }

      if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &(msg->rx_header), msg->rx_data) == HAL_OK)
        {
          if (osMailPut(*CANDeviceManager::canMsgMailHandle, msg) == osOK) return;
        }
      CANDeviceManager::recordRxDrop();
      osMailFree(*CANDeviceManager::canMsgMailHandle, msg);
    }
}
#endif

#ifdef STM32H7
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                uint32_t RxFifo0ITs)
{
  (void)RxFifo0ITs;

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
    if (CANDeviceManager::canMsgMailHandle == NULL) {
      can_msg msg;
      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                 &msg.rx_header, msg.rx_data) == HAL_OK) {
        CANDeviceManager::receiveMessage(msg);
      }
      else {
        CANDeviceManager::recordRxDrop();
      }
      continue;
    }

    can_msg *msg = static_cast<can_msg*>(
      osMailAlloc(*CANDeviceManager::canMsgMailHandle, 0));
    if (msg == NULL) {
      can_msg dropped;
      HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                             &dropped.rx_header, dropped.rx_data);
      CANDeviceManager::recordRxDrop();
      continue;
    }

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                               &msg->rx_header, msg->rx_data) != HAL_OK ||
        osMailPut(*CANDeviceManager::canMsgMailHandle, msg) != osOK) {
      CANDeviceManager::recordRxDrop();
      osMailFree(*CANDeviceManager::canMsgMailHandle, msg);
    }
  }
}
#endif

#ifndef STMCUBEMX_NEW_VERSION
/*
   Workaround: we still have to add 'extern "C"' symbol for the project generated by old STM32CubeMX version. So we add a macro of STMCUBEMX_NEW_VERSION for new version project (e.g. spinal with STM32H7)
 */

extern "C"
{
#endif
  /* The callback function for CAN TX task in RTOS */
  void canRxTask(void const * argument)
  {
    for(;;)
      {
        if(CANDeviceManager::canMsgMailHandle == NULL)
          {
            osDelay(1);
            continue;
          }

        // Decode all frames accumulated during the preceding 1 ms interval.
        osEvent event;
        do
          {
            event = osMailGet(*CANDeviceManager::canMsgMailHandle, 0);
            if (event.status == osEventMail)
              {
                can_msg *msg = static_cast<can_msg*>(event.value.p);
                CANDeviceManager::receiveMessage(*msg);
                osMailFree(*CANDeviceManager::canMsgMailHandle, msg);
              }
          }
        while (event.status == osEventMail);
        osDelay(1);
      }
  }
#ifndef STMCUBEMX_NEW_VERSION
}
#endif
