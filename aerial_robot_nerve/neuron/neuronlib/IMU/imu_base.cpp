/*
 * imu_base.cpp
 *
 *  Created on: 2016/10/25
 *      Author: anzai
 */

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "imu_base.h"

void IMU::init(SPI_HandleTypeDef* hspi)
{
  acc_.fill(0);
  gyro_.fill(0);
  mag_.fill(0);

  hspi_ = hspi;
  gyroInit();
  accInit();
  magInit();

  Flashmemory::addValue(&send_data_flag_, 2);
  Flashmemory::read();
}

void IMU::update()
{
  pollingRead(); //read from SPI
}

void IMU::sendData()
{
	if (send_data_flag_ == 0) return;
	sendMessage(CAN::MESSAGEID_SEND_GYRO, m_slave_id, 6, reinterpret_cast<uint8_t*>(gyro_.data()), 1);
	sendMessage(CAN::MESSAGEID_SEND_ACC, m_slave_id, 6, reinterpret_cast<uint8_t*>(acc_.data()), 1);
	sendMessage(CAN::MESSAGEID_SEND_MAG, m_slave_id, 6, reinterpret_cast<uint8_t*>(mag_.data()), 1);
}

void IMU::receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data)
{
	return;
}
