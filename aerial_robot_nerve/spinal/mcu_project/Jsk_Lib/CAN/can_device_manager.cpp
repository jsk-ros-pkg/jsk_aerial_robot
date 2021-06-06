/*
 * can_device_manager.cpp
 *
 *  Created on: 2016/10/26
 *      Author: anzai
 */

#include "can_device_manager.h"
#include <map>
#include "cmsis_os.h"

extern osMailQId canMsgMailHandle;
extern bool start_processing_flag_;

namespace CANDeviceManager
{
	namespace {
		std::map<int, CANDevice&> can_device_list;
		int can_timeout_count = 0;
		constexpr int CAN_MAX_TIMEOUT_COUNT = 100;
		GPIO_TypeDef* m_GPIOx;
		uint16_t m_GPIO_Pin;
	}

	void init(CAN_HandleTypeDef* hcan, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
	{
		CAN::init(hcan);
		m_GPIOx = GPIOx;
		m_GPIO_Pin = GPIO_Pin;
	}

	int makeCommunicationId(uint8_t device_id, uint8_t slave_id)
	{
		return static_cast<int>((device_id << CAN::DEVICE_ID_LEN) | slave_id);
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
			if (internal_count == (1000 / cycle) - 1) {
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
    uint8_t slave_id = CAN::getSlaveId(msg.rx_header);
    uint8_t device_id = CAN::getDeviceId(msg.rx_header);
    uint8_t message_id = CAN::getMessageId(msg.rx_header);
    uint32_t dlc = CAN::getDlc(msg.rx_header);

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

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  CANDeviceManager::can_timeout_count = 0;

  if(start_processing_flag_)
    {
      /* add data to RTOS queue */
      can_msg *msg = (can_msg *)osMailAlloc(canMsgMailHandle, 0);

      if (msg == NULL) return; //allocation is not ready, exit

      if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &(msg->rx_header), msg->rx_data) == HAL_OK)
        {
          osMailPut(canMsgMailHandle, msg);
        }
    }
  else
    {
      can_msg msg;
      if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &msg.rx_header, msg.rx_data) == HAL_OK)
        {
          /* directly call receivemessage, since RTOS is not initialized */
          CANDeviceManager::receiveMessage(msg);
        }
    }
}

extern "C"
{
  /* The callback function for CAN TX task in RTOS */
  void canRxTask(void const * argument)
  {
    for(;;)
      {
        // get data from RTOS queue if available
        osEvent event = osMailGet(canMsgMailHandle, osWaitForever);

        if (event.status == osEventMail)
          {
            can_msg *msg = (can_msg*)event.value.p;

            CANDeviceManager::receiveMessage(*msg);
            osMailFree(canMsgMailHandle, msg);
          }
      }
  }
}
