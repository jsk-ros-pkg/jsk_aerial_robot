/*
 * can_imu_mpu9250.cpp
 *
 *  Created on: 2016/11/07
 *      Author: anzai
 */

#include "can_imu.h"
#include <string.h>

void CANIMU::init()
{
  IMU::init();

  if(send_data_flag_)
    {
      // get the calibration data from the neruon that send imu data
      uint8_t send_data[1];
      send_data[0] = CAN::IMU_CALIBRATE_REQUEST_DATA;
      setMessage(CAN::MESSAGEID_CALIBRATE, m_slave_id, 1, send_data);
      sendMessage(0);
    }
}

void CANIMU::update()
{
  /* Temporary transform becuase of the limited bandwidth of CAN */
  gyro_[0] = r_gyro_data[0] * MPU9250_GYRO_SCALE;
  gyro_[1] = r_gyro_data[1] * MPU9250_GYRO_SCALE;
  gyro_[2] = r_gyro_data[2] * MPU9250_GYRO_SCALE;
  acc_[0] = r_acc_data[0] * MPU9250_ACC_SCALE;
  acc_[1] = r_acc_data[1] * MPU9250_ACC_SCALE;
  acc_[2] = r_acc_data[2] * MPU9250_ACC_SCALE;

  /* temporary ignore
     mag_[0] = r_mag_data[0] * MPU9250_MAG_SCALE;
     mag_[1] = r_mag_data[1] * MPU9250_MAG_SCALE;
     mag_[2] = r_mag_data[2] * MPU9250_MAG_SCALE;
  */

  rpy_[0] = r_rpy_data[0] / 1000.0f;
  rpy_[1] = r_rpy_data[1] / 1000.0f;
  rpy_[2] = r_rpy_data[2] / 1000.0f;

  gyro_bias_[0] = r_gyro_calib[0] * MPU9250_GYRO_SCALE;
  gyro_bias_[1] = r_gyro_calib[1] * MPU9250_GYRO_SCALE;
  gyro_bias_[2] = r_gyro_calib[2] * MPU9250_GYRO_SCALE;

  acc_bias_[0] = r_acc_calib[0] * MPU9250_ACC_SCALE;
  acc_bias_[1] = r_acc_calib[1] * MPU9250_ACC_SCALE;
  acc_bias_[2] = r_acc_calib[2] * MPU9250_ACC_SCALE;
}

void CANIMU::sendData()
{
  return;
}

void CANIMU::receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data)
{
  switch (message_id) {
  case CAN::MESSAGEID_SEND_GYRO:
    memcpy(r_gyro_data, data, sizeof(uint8_t) * 6);
    break;
  case CAN::MESSAGEID_SEND_ACC:
    memcpy(r_acc_data, data, sizeof(uint8_t) * 6);
    break;
  case CAN::MESSAGEID_SEND_MAG:
    memcpy(r_mag_data, data, sizeof(uint8_t) * 6);
    break;
  case CAN::MESSAGEID_SEND_RPY:
    memcpy(r_rpy_data, data, sizeof(uint8_t) * 6);
    break;
  case CAN::MESSAGEID_SEND_GYRO_CALIBRATE_INFO:
    memcpy(r_gyro_calib, data, sizeof(uint8_t) * 6);
    break;
  case CAN::MESSAGEID_SEND_ACC_CALIBRATE_INFO:
    memcpy(r_acc_calib, data, sizeof(uint8_t) * 6);
    break;
  }
}

void CANIMU::gyroCalib(bool flag, float duration)
{
  uint8_t send_data[2];
  send_data[0] = CAN::IMU_CALIBRATE_GYRO;
  send_data[1] = flag?1:0;
  setMessage(CAN::MESSAGEID_CALIBRATE, m_slave_id, 2, send_data);
  sendMessage(0);
}

void CANIMU::accCalib(bool flag, float duration)
{
  uint8_t send_data[2];
  send_data[0] = CAN::IMU_CALIBRATE_ACC;
  send_data[1] = flag?1:0;
  setMessage(CAN::MESSAGEID_CALIBRATE, m_slave_id, 2, send_data);
  sendMessage(0);
}

void CANIMU::magCalib(bool flag, float duration)
{
  uint8_t send_data[2];
  send_data[0] = CAN::IMU_CALIBRATE_MAG;
  send_data[1] = flag?1:0;
  setMessage(CAN::MESSAGEID_CALIBRATE, m_slave_id, 2, send_data);
  sendMessage(0);
}

void CANIMU::resetCalib()
{
  uint8_t send_data[1];
  send_data[0] = CAN::IMU_CALIBRATE_RESET;
  setMessage(CAN::MESSAGEID_CALIBRATE, m_slave_id, 1, send_data);
  sendMessage(0);
}

void CANIMU::writeCalibData()
{
  uint8_t send_data[1];
  send_data[0] = CAN::IMU_CALIBRATE_SAVE;
  setMessage(CAN::MESSAGEID_CALIBRATE, m_slave_id, 1, send_data);
  sendMessage(0);
}
