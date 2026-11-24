/*
 * icm42686.cpp
 *
 *  Created on: 2026/7/29
 *      Author: Miyamichi
 */

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "icm42686.h"

#define IMU_SPI_CS_H HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define IMU_SPI_CS_L HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)

void ICM42686::init(SPI_HandleTypeDef* hspi)
{
  for (uint8_t i = 0; i < SENSOR_DATA_LENGTH; ++i) adc_[i] = 0;
  update_ = false;

  IMU_SPI_CS_H;
  HAL_Delay(1);

  IMU::init(hspi);
}

void ICM42686::icmWrite(uint8_t address, uint8_t value)
{
  uint8_t data[2] = {static_cast<uint8_t>(address & 0x7F), value};

  IMU_SPI_CS_L;
  HAL_SPI_Transmit(hspi_, data, sizeof(data), 1000); // ignore transmit return
  IMU_SPI_CS_H;
}

uint8_t ICM42686::icmRead(uint8_t address)
{
  uint8_t value = 0;
  uint8_t command = address | 0x80;

  IMU_SPI_CS_L;
  HAL_SPI_Transmit(hspi_, &command, 1, 1000);
  HAL_SPI_Receive(hspi_, &value, 1, 1000);
  IMU_SPI_CS_H;

  return value;
}

void ICM42686::gyroInit()
{
  icmWrite(REG_BANK_SEL, 0x00);
  icmWrite(REG_DEVICE_CONFIG, SOFT_RESET_CONFIG);
  HAL_Delay(2);

  icmWrite(REG_BANK_SEL, 0x00);
  who_am_i_ = icmRead(REG_WHO_AM_I);
  detected_ = (who_am_i_ == WHO_AM_I_VALUE);
  if (!detected_) return;

  icmWrite(REG_GYRO_CONFIG0, GYRO_2000DPS_1KHZ);
}

void ICM42686::accInit()
{
  if (!detected_) return;
  icmWrite(REG_ACCEL_CONFIG0, ACCEL_8G_1KHZ);
  icmWrite (REG_PWR_MGMT0, ACCEL_GYRO_LOW_NOISE);

  HAL_Delay(70);
}

void ICM42686::pollingRead()
{
  if (!detected_) return;

  uint8_t command = REG_ACCEL_DATA_X1 | 0x80;

  IMU_SPI_CS_L;
  HAL_SPI_Transmit(hspi_, &command, 1, 1000);
  HAL_SPI_Receive(hspi_, adc_, SENSOR_DATA_LENGTH, 1000);
  IMU_SPI_CS_H;

  acc_[0] = static_cast<int16_t>((adc_[0] << 8) | adc_[1]);
  acc_[1] = static_cast<int16_t>((adc_[2] << 8) | adc_[3]);
  acc_[2] = static_cast<int16_t>((adc_[4] << 8) | adc_[5]);

  gyro_[0] = static_cast<int16_t>((adc_[6] << 8) | adc_[7]);
  gyro_[1] = static_cast<int16_t>((adc_[8] << 8) | adc_[9]);
  gyro_[2] = static_cast<int16_t>((adc_[10] << 8) | adc_[11]);

  update_ = true;
}
