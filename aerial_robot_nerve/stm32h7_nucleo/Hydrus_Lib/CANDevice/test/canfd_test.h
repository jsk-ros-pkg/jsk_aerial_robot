/*
 * canfd_test.h
 *
 *  Created on: 2021/06/30
 *      Author: chou
 */

#ifndef APPLICATION_HYDRUS_LIB_CANDEVICE_TEST_CANFD_TEST_H_
#define APPLICATION_HYDRUS_LIB_CANDEVICE_TEST_CANFD_TEST_H_

#include "CAN/can_device.h"

class CANFDTest : public CANDevice{
private:
  uint8_t tx_data[64];
  const uint8_t REQ = 0;
  const uint8_t ACK = 1;

public:
  CANFDTest();
  void sendData() override;
  void receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data) override;
};




#endif /* APPLICATION_HYDRUS_LIB_CANDEVICE_TEST_CANFD_TEST_H_ */
