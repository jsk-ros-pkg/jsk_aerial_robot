/*
 * can_encoder.h
 *
 *  Created on: 2025/12/11
 *      Author: K.Sugihara
 */

#ifndef APPLICATION_HYDRUS_LIB_CANDEVICE_ENCODER_CAN_ENCODER_H_
#define APPLICATION_HYDRUS_LIB_CANDEVICE_ENCODER_CAN_ENCODER_H_

#include "CAN/can_device.h"

class CANEncoder : public CANDevice
{
private:
  uint16_t m_raw_value;
  uint16_t m_value;

public:
  CANEncoder(){}
  CANEncoder(uint8_t slave_id) : CANDevice(CAN::DEVICEID_ENCODER, slave_id), m_raw_value(0), m_value(0){}
  void sendData() override;
  void receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data) override;
  uint16_t getRawValue() { return m_raw_value; }
  uint16_t getValue() { return m_value; }

};

#endif /* APPLICATION_HYDRUS_LIB_CANDEVICE_ENCODER_CAN_ENCODER_H_ */
