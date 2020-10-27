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
  acc_adc_.zero();
  raw_acc_adc_.zero();
  gyro_adc_.zero();
  mag_adc_.zero();

  acc_.zero();
  gyro_.zero();
  mag_.zero();
  prev_mag_.zero();

  acc_bias_sum_.zero();
  gyro_bias_sum_.zero();
  gyro_bias_.zero();
  rpy_.zero();
  gyro_calib_time_ = HAL_GetTick();
  acc_calib_time_ = HAL_GetTick();
  gyro_calib_duration_ = GYRO_DEFAULT_CALIB_DURATION;
  acc_calib_duration_ = ACC_DEFAULT_CALIB_DURATION;
  gyro_calib_cnt_ = 0;
  acc_calib_cnt_ = 0;

  ahb_tx_suspend_flag_ = false;

  acc_filter_cnt_ = 0; // complementary filter for attitude estimate

  send_calib_data_ = false; // send calib data via CAN
  calib_gyro_ = true;
  calib_acc_ = false;
  calib_mag_ = false;

  for(int i =0; i < SENSOR_DATA_LENGTH; i++)
    adc_[i] = 0;

  hspi_ = hspi;
  gyroInit();
  accInit();
  magInit();

  /* change to 13.5Mhz for polling sensor data from acc, gyro and mag */
  hspi_->Instance->CR1 &= (uint32_t)(~SPI_BAUDRATEPRESCALER_256); //reset
  hspi_->Instance->CR1 |= (uint32_t)(SPI_BAUDRATEPRESCALER_8); //8 = 13.5Mhz

  Flashmemory::addValue(&send_data_flag_, 2);

  for (int i = 0; i < 3; i++) {
    Flashmemory::addValue(&(acc_bias_[i]), sizeof(int16_t));
    Flashmemory::addValue(&(mag_bias_[i]), sizeof(int16_t));
  }
  Flashmemory::read();
}

void IMU::update()
{
  pollingRead(); //read from SPI

  calibrate();

  attitude_estimate(); // estimate RPY attitude
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
  raw_gyro_adc_[0] = (int16_t)(adc_[0] << 8 | adc_[1]);
  raw_gyro_adc_[1] = (int16_t)(adc_[2] << 8 | adc_[3]);
  raw_gyro_adc_[2] = (int16_t)(adc_[4] << 8 | adc_[5]);


  t_data[0] = ACC_ADDRESS | 0x80;
  IMU_SPI_CS_L;
  HAL_SPI_Transmit(hspi_, t_data, 1, 1000);
  HAL_SPI_Receive(hspi_, adc_, 6, 1000);
  IMU_SPI_CS_H;

  /* we need add some delay between each sensor reading */
  /* previous code */
  /*acc_[0] = (int16_t)(adc_[0] << 8 | adc_[1]) / 4096.0f * GRAVITY_MSS;*/
  raw_acc_adc_[0] = (int16_t)(adc_[0] << 8 | adc_[1]);
  raw_acc_adc_[1] = (int16_t)(adc_[2] << 8 | adc_[3]);
  raw_acc_adc_[2] = (int16_t)(adc_[4] << 8 | adc_[5]);

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
      raw_mag_adc_[0] = (int16_t)(adc_[1] << 8 | adc_[0]);
      raw_mag_adc_[1] = (int16_t)(adc_[3] << 8 | adc_[2]);
      raw_mag_adc_[2] = (int16_t)(adc_[5] << 8 | adc_[4]);
    }
  if(i == MAG_PRESCALER) i = 0;
  else i++;

  update_ = true;
}

void IMU::calibrate()
{
  /* gyro part */
  gyro_adc_ = raw_gyro_adc_;
  if (calib_gyro_) {
    gyro_bias_sum_ += Vector3l((int32_t)raw_gyro_adc_.x, (int32_t)raw_gyro_adc_.y, (int32_t)raw_gyro_adc_.z);
    gyro_calib_cnt_++;

    if (gyro_calib_duration_ > 0 && HAL_GetTick() - gyro_calib_time_ >= gyro_calib_duration_)
      {
        gyro_bias_sum_ /= gyro_calib_cnt_;
        gyro_bias_((int16_t)gyro_bias_sum_.x, (int16_t)gyro_bias_sum_.y, (int16_t)gyro_bias_sum_.z);
        calib_gyro_ = false;
      }

    update_ = false;
  }
  else
    gyro_adc_ -= gyro_bias_;

  gyro_ =  Vector3f(gyro_adc_.x * GYRO_SCALE, gyro_adc_.y * GYRO_SCALE, gyro_adc_.z * GYRO_SCALE);


  /* acc part */
  acc_adc_ = raw_acc_adc_;
  if (calib_acc_) {
    acc_bias_sum_ += Vector3l((int32_t)raw_acc_adc_.x, (int32_t)raw_acc_adc_.y, (int32_t)(raw_acc_adc_.z - ACC_GRAVITY_RESOLUTION));
    acc_calib_cnt_++;

    if (acc_calib_duration_ > 0 && HAL_GetTick() - acc_calib_time_ >= acc_calib_duration_)
      {
        acc_bias_sum_ /= acc_calib_cnt_;
        acc_bias_((int16_t)acc_bias_sum_.x, (int16_t)acc_bias_sum_.y, (int16_t)acc_bias_sum_.z);

        calib_acc_ = false;
      }

    update_ = false;
  }
  else
    acc_adc_ -= acc_bias_;

  acc_ =  Vector3f(acc_adc_.x * ACC_SCALE, acc_adc_.y * ACC_SCALE, acc_adc_.z * ACC_SCALE);

  /* mag part */
  mag_adc_ = raw_mag_adc_ - mag_bias_; //bias
  mag_ =  Vector3f(mag_adc_.x * MAG_SCALE, mag_adc_.y * MAG_SCALE, mag_adc_.z * MAG_SCALE);
}

void IMU::attitude_estimate()
{
  if(!update_)
    {
      attitdue_est_timestamp_ = HAL_GetTick();
      return;
    }

  // complementary filter
  int  valid_acc = 0;

  float acc_magnitude = acc_ * acc_;
  Vector3f est_g_b_tmp = est_g_;
  Vector3f est_m_b_tmp = est_m_;

  Vector3f gyro_rotate = gyro_ * (HAL_GetTick() - attitdue_est_timestamp_)  * 0.001f;

  est_m_ += (est_m_b_tmp % gyro_rotate  ); //rotation by gyro
  est_g_ += (est_g_b_tmp % gyro_rotate ); //rotation by gyro

  if( G_MIN < acc_magnitude && acc_magnitude < G_MAX) valid_acc = 1;
  else valid_acc = 0;

  est_g_b_tmp = est_g_;
  est_m_b_tmp = est_m_;

  /* acc correction */
  if ( valid_acc == 1 && acc_filter_cnt_ == 0)
    est_g_ = (est_g_b_tmp * GYR_CMPF_FACTOR + acc_) * INV_GYR_CMPF_FACTOR;

  /* mag correction */
  if ( prev_mag_ != mag_ )
    {
      prev_mag_ = mag_;
      est_m_ = (est_m_b_tmp * GYR_CMPFM_FACTOR  + mag_) * INV_GYR_CMPFM_FACTOR;
    }

  // Attitude of the estimated vector
  float sq_g_x_sq_g_z = est_g_.x * est_g_.x + est_g_.z * est_g_.z;
  float sq_g_y_sq_g_z = est_g_.y * est_g_.y + est_g_.z * est_g_.z;
  float invG = ap::inv_sqrt(sq_g_x_sq_g_z + est_g_.y * est_g_.y);
  rpy_.x = atan2f(est_g_.y , est_g_.z);
  rpy_.y = atan2f(-est_g_.x , ap::inv_sqrt(sq_g_y_sq_g_z)* sq_g_y_sq_g_z);
  rpy_.z = atan2f( est_m_.z * est_g_.y - est_m_.y * est_g_.z,
                   est_m_.x * invG * sq_g_y_sq_g_z  - (est_m_.y * est_g_.y + est_m_.z * est_g_.z) * invG * est_g_.x );

  /* update */
  if(valid_acc) acc_filter_cnt_++;
  if(acc_filter_cnt_ == PRESCLAER_ACC) acc_filter_cnt_ = 0;

  attitdue_est_timestamp_ = HAL_GetTick();
}

void IMU::sendData()
{
  if (send_data_flag_ == 0) return;

  int16_t data[3];
  data[0] = gyro_adc_.x;
  data[1] = gyro_adc_.y;
  data[2] = gyro_adc_.z;
  setMessage(CAN::MESSAGEID_SEND_GYRO, m_slave_id, 6, reinterpret_cast<uint8_t*>(data));
  sendMessage(1);
  data[0] = acc_adc_.x;
  data[1] = acc_adc_.y;
  data[2] = acc_adc_.z;
  setMessage(CAN::MESSAGEID_SEND_ACC, m_slave_id, 6, reinterpret_cast<uint8_t*>(data));
  sendMessage(1);
  //setMessage(CAN::MESSAGEID_SEND_MAG, m_slave_id, 6, reinterpret_cast<uint8_t*>(mag_.data()));
  // hack: radian => x 1000 => int16
  data[0] = (int16_t)(rpy_.x * 1000);
  data[1] = (int16_t)(rpy_.y * 1000);
  data[2] = (int16_t)(rpy_.z * 1000);
  setMessage(CAN::MESSAGEID_SEND_RPY, m_slave_id, 6, reinterpret_cast<uint8_t*>(data));
  sendMessage(1);

  if(send_calib_data_)
    {
      send_calib_data_ = false;
      data[0] = gyro_bias_.x;
      data[1] = gyro_bias_.y;
      data[2] = gyro_bias_.z;
      setMessage(CAN::MESSAGEID_SEND_GYRO_CALIBRATE_INFO, m_slave_id, 6, reinterpret_cast<uint8_t*>(data));
      sendMessage(1);

      data[0] = acc_bias_.x;
      data[1] = acc_bias_.y;
      data[2] = acc_bias_.z;
      setMessage(CAN::MESSAGEID_SEND_ACC_CALIBRATE_INFO, m_slave_id, 6, reinterpret_cast<uint8_t*>(data));
      sendMessage(1);
    }
}

void IMU::receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data)
{
  switch (message_id) {
  case CAN::MESSAGEID_CALIBRATE:
    {
      uint8_t calibrate_command_id = data[0];
      switch (calibrate_command_id) {
      case CAN::IMU_CALIBRATE_REQUEST_DATA:
        {
          send_calib_data_ = true;
          break;
        }
      case CAN::IMU_CALIBRATE_GYRO:
        {
          uint8_t calib_flag = data[1];
          if(calib_flag)
            {
              // re-calib with infinite time
              gyro_bias_.zero();
              calib_gyro_ = true;
              gyro_calib_duration_ = 0;
              gyro_calib_time_ = HAL_GetTick();
              gyro_calib_cnt_ = 0;
            }
          else
            {
              gyro_calib_duration_ = 1; // stop re-calib
            }
          break;
        }
      case CAN::IMU_CALIBRATE_ACC:
        {
          uint8_t calib_flag = data[1];
          if(calib_flag)
            {
              // re-calib with infinite time
              acc_bias_.zero();
              calib_acc_ = true;
              acc_calib_duration_ = 0;
              acc_calib_time_ = HAL_GetTick();
              acc_calib_cnt_ = 0;
            }
          else
            {
              acc_calib_duration_ = 1; // stop re-calib
            }
          break;
        }
      case CAN::IMU_CALIBRATE_MAG:
        {
          // TODO for implementation
          break;
        }
      case CAN::IMU_CALIBRATE_RESET:
        {
          gyro_bias_.zero();
          acc_bias_.zero();
          mag_bias_.zero();
          break;
        }
      case CAN::IMU_CALIBRATE_SAVE:
        {
          Flashmemory::erase();
          Flashmemory::write();

          send_calib_data_ = true; // tell the latest data to spinal via CAN
          break;
        }
      }
      break;
    }
  }
}

