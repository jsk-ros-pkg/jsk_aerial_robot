/*
 * can_device_manager.h
 *
 *  Created on: 2016/10/26
 *      Author: anzai
 */

#ifndef APPLICATION_CAN_CAN_DEVICE_MANAGER_H_
#define APPLICATION_CAN_CAN_DEVICE_MANAGER_H_

#include "can_device.h"
#include "cmsis_os.h"

#if defined(STM32F1) || defined(STM32F4)
typedef struct
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
} can_msg;
typedef CAN_HandleTypeDef CAN_GeranlHandleTypeDef;
#endif

#ifdef STM32G4
typedef struct
{
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[64];
} can_msg;
typedef FDCAN_HandleTypeDef CAN_GeranlHandleTypeDef;
#endif


namespace CANDeviceManager
{
  void init(CAN_GeranlHandleTypeDef* hcan, uint8_t slave_id, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
  void addDevice(CANDevice* device);
  void sendMessages();
  void useRTOS(osMailQId* handle);
  void tick(int cycle /* ms */);
  void resetTick();
  bool isTimeout();
  void userSendMessages();
  void userReceiveMessagesCallback(uint8_t slave_id, uint8_t device_id, uint8_t message_id, uint32_t DLC, uint8_t* data);
  void CAN_START();
}

#endif /* APPLICATION_CAN_CAN_DEVICE_MANAGER_H_ */
