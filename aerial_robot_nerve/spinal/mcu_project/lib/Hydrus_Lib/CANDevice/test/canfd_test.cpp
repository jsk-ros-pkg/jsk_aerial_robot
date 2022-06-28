/*
 * canfd_test.cpp
 *
 *  Created on: 2021/06/30
 *      Author: chou
 */

#include "canfd_test.h"
#include <string.h>

CANFDTest::CANFDTest() : CANDevice(CAN::DEVICEID_CANFD_TEST, CAN::MASTER_ID)
{
  for (uint8_t i = 0; i < 64; i++)
    tx_data[i] = i;
}

void CANFDTest::sendData()
{
  sendMessage(REQ, CAN::MASTER_ID, 64, tx_data, 0);
  return;
}

void CANFDTest::receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data)
{
  if(message_id == REQ)
    {
      // send back data:
      sendMessage(ACK, CAN::MASTER_ID, 64, tx_data, 0);
    }
}
