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
    int internal_count = 0;
    constexpr int CAN_MAX_TIMEOUT_COUNT = 100;
    GPIO_TypeDef* m_GPIOx;
    uint16_t m_GPIO_Pin;
    osMailQId* canMsgMailHandle = NULL;
  }

  void CAN_START()
  {
    CAN::CAN_START();
  }

  void init(CAN_GeranlHandleTypeDef* hcan, uint8_t slave_id, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
  {
    CAN::init(hcan, slave_id);
    m_GPIOx = GPIOx;
    m_GPIO_Pin = GPIO_Pin;
    CAN_START();
  }

  void useRTOS(osMailQId* handle)
  {
    canMsgMailHandle = handle;
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

  void resetTick()
  {
    internal_count = 0;
    HAL_GPIO_WritePin(m_GPIOx, m_GPIO_Pin, GPIO_PIN_RESET);
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

  void receiveMessage(can_msg msg)
  {
    uint8_t slave_id = CAN::getSlaveId(msg.rx_header);
    uint8_t device_id = CAN::getDeviceId(msg.rx_header);
    uint8_t message_id = CAN::getMessageId(msg.rx_header);
    uint32_t dlc = CAN::getDlc(msg.rx_header);

    if (device_id >= CANDeviceManager::can_device_list.size()) return;
    CANDeviceManager::can_device_list[device_id]->receiveDataCallback(message_id, dlc, msg.rx_data);
    CANDeviceManager::userReceiveMessagesCallback(slave_id, device_id, message_id, dlc, msg.rx_data);
  }
}

#if defined(STM32F1) || defined(STM32F4)
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
    }
  else
    {
      /* add data to RTOS queue */
      can_msg *msg = (can_msg *)osMailAlloc(*CANDeviceManager::canMsgMailHandle, 0);

      if (msg == NULL) return; //allocation is not ready, exit

      if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &(msg->rx_header), msg->rx_data) == HAL_OK)
        {
          osMailPut(*CANDeviceManager::canMsgMailHandle, msg);
        }
    }
}
#endif

#ifdef STM32G4
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

  CANDeviceManager::can_timeout_count = 0;

  if(CANDeviceManager::canMsgMailHandle == NULL)
    {
      can_msg msg;
      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &msg.rx_header, msg.rx_data) == HAL_OK)
        {
          /* directly call receivemessage, since RTOS is not initialized */
          CANDeviceManager::receiveMessage(msg);
        }
    }
  else
    {
      /* add data to RTOS queue */
      can_msg *msg = (can_msg *)osMailAlloc(*CANDeviceManager::canMsgMailHandle, 0);

      if (msg == NULL) return; //allocation is not ready, exit

      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &(msg->rx_header), msg->rx_data) == HAL_OK)
        {
          osMailPut(*CANDeviceManager::canMsgMailHandle, msg);
        }
    }
}
#endif

extern "C"
{
  /* The callback function for CAN TX task in RTOS */
  void canRxCallback(void const * argument)
  {
    for(;;)
      {
        if(CANDeviceManager::canMsgMailHandle == NULL)
          {
            osDelay(1);
            continue;
          }

        // get data from RTOS queue if available
        osEvent event = osMailGet(*CANDeviceManager::canMsgMailHandle, osWaitForever);

        if (event.status == osEventMail)
          {
            can_msg *msg = (can_msg*)event.value.p;

            CANDeviceManager::receiveMessage(*msg);
            osMailFree(*CANDeviceManager::canMsgMailHandle, msg);
          }
      }
  }
}

