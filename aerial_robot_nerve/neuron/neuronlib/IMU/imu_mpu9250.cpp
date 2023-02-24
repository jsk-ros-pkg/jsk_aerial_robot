/*
 * imu_mpu9250.cpp
 *
 *  Created on: 2016/10/25
 *      Author: anzai
 */

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "imu_mpu9250.h"

uint8_t IMU::adc_[SENSOR_DATA_LENGTH];

void IMU::init(SPI_HandleTypeDef* hspi)
{
  acc_.fill(0);
  gyro_.fill(0);
  mag_.fill(0);

  ahb_tx_suspend_flag_ = false;

  for(int i =0; i < SENSOR_DATA_LENGTH; i++)
    {
      dummy_[i] = 0;
      adc_[i] = 0;
    }

  hspi_ = hspi;
  gyroInit();
  accInit();
  magInit();

  /* change to 13.5Mhz for polling sensor data from acc, gyro and mag */
  hspi_->Instance->CR1 &= (uint32_t)(~SPI_BAUDRATEPRESCALER_256); //reset
  hspi_->Instance->CR1 |= (uint32_t)(SPI_BAUDRATEPRESCALER_8); //8 = 13.5Mhz

  Flashmemory::addValue(&send_data_flag_, 2);
  Flashmemory::read();
}

void IMU::update()
{
  pollingRead(); //read from SPI
}

void IMU::mpuWrite(uint8_t address, uint8_t value)
{
  IMU_SPI_CS_L;
  HAL_SPI_Transmit(hspi_, &address, 1, 1000);
  HAL_SPI_Transmit(hspi_, &value, 1, 1000);
  IMU_SPI_CS_H;
}

uint8_t IMU::mpuRead(uint8_t address)
{
  uint8_t t_data[1] = {0};
  t_data[0] = address | 0x80;
  uint8_t temp;
  IMU_SPI_CS_L;
  HAL_SPI_Transmit(hspi_, t_data, 1, 1000);
  HAL_SPI_Receive(hspi_, &temp, 1, 1000);
  IMU_SPI_CS_H;
  return temp;
}

void IMU::gyroInit(void)
{
  HAL_Delay(100);
  //  mpuWrite( 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  HAL_Delay(10);
  //mpuWrite( 0x6B, 0x01);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  HAL_Delay(1); //very important!, some duration for process the setting
  mpuWrite( 0x6A, 0x10);             //disable i2c communication
  HAL_Delay(1); //very importnat! between gyro and acc
  mpuWrite( 0x1A, GYRO_DLPF_CFG); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  HAL_Delay(1); //very importnat! between gyro and acc
  mpuWrite( 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
  HAL_Delay(10); //very importnat! between gyro and acc
}

void IMU::accInit (void) {
  mpuWrite( 0x1C, 0x10); //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
  HAL_Delay(1);
  //old: acceleration bandwidth is 460Hz
  mpuWrite( 0x1D, ACC_DLPF_CFG);
  HAL_Delay(10);
}


void IMU::magInit(void)
{
  HAL_Delay(10);
  //at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
  //now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
  mpuWrite( 0x6A, 0x20); //USER_CTRL -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
  HAL_Delay(10);
  mpuWrite( 0x37, 0x00); //INT_PIN_CFG -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
  HAL_Delay(1);
  mpuWrite( 0x24, 0x0D); //I2C_MST_CTRL -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
  HAL_Delay(1);

  //write mode
  mpuWrite( 0x25, MAG_ADDRESS);
  HAL_Delay(1);
  mpuWrite( 0x26, 0x0B);
  HAL_Delay(1);
  mpuWrite( 0x63, 0x01);
  HAL_Delay(1);
  mpuWrite( 0x27, 0x81);
  HAL_Delay(1);

  mpuWrite( 0x26, 0x0A);
  HAL_Delay(1);
  mpuWrite( 0x63, 0x16);
  HAL_Delay(1);
  mpuWrite( 0x27, 0x81);
  HAL_Delay(1);

  //read mode
  mpuWrite( 0x25, 0x80|MAG_ADDRESS);//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
  HAL_Delay(1);
  mpuWrite( 0x26, MAG_DATA_REGISTER);//I2C_SLV0_REG -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
  HAL_Delay(1);
  mpuWrite( 0x27, 0x87);
  HAL_Delay(1);
}

void IMU::pollingRead()
{
  static int i = 0;
  uint8_t t_data[1];

  t_data[0] = GYRO_ADDRESS | 0x80;

  IMU_SPI_CS_L;
  HAL_SPI_Transmit(hspi_, t_data, 1, 1000);
  HAL_SPI_Receive(hspi_, adc_, 6, 1000);
  IMU_SPI_CS_H;

  /* we need add some delay between each sensor reading */
  /* previous code */
  /*gyro_[0] = (int16_t)(adc_[0] << 8 | adc_[1]) * 2000.0f / 32767.0f * M_PI / 180.0f  ; */
  gyro_[0] = (int16_t)(adc_[0] << 8 | adc_[1]);
  gyro_[1] = (int16_t)(adc_[2] << 8 | adc_[3]);
  gyro_[2] = (int16_t)(adc_[4] << 8 | adc_[5]);

  t_data[0] = ACC_ADDRESS | 0x80;
  IMU_SPI_CS_L;
  HAL_SPI_Transmit(hspi_, t_data, 1, 1000);
  HAL_SPI_Receive(hspi_, adc_, 6, 1000);
  IMU_SPI_CS_H;

  /* we need add some delay between each sensor reading */
  /* previous code */
   /*acc_[0] = (int16_t)(adc_[0] << 8 | adc_[1]) / 4096.0f * GRAVITY_MSS;*/
  acc_[0] = (int16_t)(adc_[0] << 8 | adc_[1]);
  acc_[1] = (int16_t)(adc_[2] << 8 | adc_[3]);
  acc_[2] = (int16_t)(adc_[4] << 8 | adc_[5]);


  if(i == MAG_PRESCALER)
    {
      //mag is in low speed
      hspi_->Instance->CR1 &= (uint32_t)(~SPI_BAUDRATEPRESCALER_256); //reset
      hspi_->Instance->CR1 |= (uint32_t)(SPI_BAUDRATEPRESCALER_64); //128 = 0.8Mhz
      t_data[0] = MAG_SPI_ADDRESS | 0x80;
      IMU_SPI_CS_L;
      HAL_SPI_Transmit(hspi_, t_data, 1, 1000);
      HAL_SPI_Receive(hspi_, adc_, 7, 1000);
      IMU_SPI_CS_H;

      hspi_->Instance->CR1 &= (uint32_t)(~SPI_BAUDRATEPRESCALER_256); //reset
      hspi_->Instance->CR1 |= (uint32_t)(SPI_BAUDRATEPRESCALER_8); //8 = 13.5Mhz

      //uT(10e-6 T)
      /* previous code */
      /*mag_[0] = (int16_t)(adc_[1] << 8 | adc_[0]) * 4912.0f / 32760.0f;*/
      mag_[0] = (int16_t)(adc_[1] << 8 | adc_[0]);
      mag_[1] = (int16_t)(adc_[3] << 8 | adc_[2]);
      mag_[2] = (int16_t)(adc_[5] << 8 | adc_[4]);
    }
  if(i == MAG_PRESCALER) i = 0;
  else i++;

  update_ = true;
}

void IMU::sendData()
{
	if (!(send_data_flag_ != 0)) return;
	sendMessage(CAN::MESSAGEID_SEND_GYRO, m_slave_id, 6, reinterpret_cast<uint8_t*>(gyro_.data()), 1);
	sendMessage(CAN::MESSAGEID_SEND_ACC, m_slave_id, 6, reinterpret_cast<uint8_t*>(acc_.data()), 1);
	sendMessage(CAN::MESSAGEID_SEND_MAG, m_slave_id, 6, reinterpret_cast<uint8_t*>(mag_.data()), 1);
}

void IMU::receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data)
{
	return;
}
