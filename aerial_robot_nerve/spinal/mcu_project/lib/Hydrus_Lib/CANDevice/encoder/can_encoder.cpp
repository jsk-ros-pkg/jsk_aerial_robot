/*
 * can_encoder.h
 *
 *  Created on: 2025/12/11
 *      Author: K.Sugihara
 */

#include "can_encoder.h"

void CANEncoder::sendData()
{
  return;
}

void CANEncoder::receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data)
{
  if (message_id == CAN::MESSAGEID_SEND_ENCODER_DATA) {
    m_raw_value = static_cast<uint16_t>((data[1] << 8) | data[0]);
    m_value = static_cast<int16_t>((data[3] << 8) | data[2]);
  }
}
