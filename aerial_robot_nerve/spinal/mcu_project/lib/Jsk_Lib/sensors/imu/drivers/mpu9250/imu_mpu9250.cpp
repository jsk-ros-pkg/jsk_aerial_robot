/*
******************************************************************************
* File Name          : imu_mpu9250.cpp
* Description        : IMU(MPU9250) Interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif
#include "sensors/imu/drivers/mpu9250/imu_mpu9250.h"

uint8_t IMUOnboard::adc_[SENSOR_DATA_LENGTH];
uint32_t IMUOnboard::last_mag_time_;

void IMUOnboard::init(SPI_HandleTypeDef* hspi, I2C_HandleTypeDef* hi2c, ros::NodeHandle* nh,
                      GPIO_TypeDef* spi_cs_port, uint16_t spi_cs_pin,
                      GPIO_TypeDef* led_port, uint16_t led_pin)
{
  nh_ = nh;
  IMU::init();

  spi_cs_port_ = spi_cs_port;
  spi_cs_pin_ = spi_cs_pin;
  led_port_ = led_port;
  led_pin_ = led_pin;

  use_external_mag_flag_ = false;

  for(int i =0; i < SENSOR_DATA_LENGTH; i++)
    {
      dummy_[i] = 0;
      adc_[i] = 0;
    }

  hspi_ = hspi;
  hi2c_ = hi2c;

  calib_indicator_time_ = HAL_GetTick();

  gyroInit();
  accInit();
  magInit();

  /* change to higher for polling sensor data from acc, gyro and mag */
  hspi_->Instance->CR1 &= (uint32_t)(~SPI_BAUDRATEPRESCALER_256); //reset
#ifdef STM32F7
  hspi_->Instance->CR1 |= (uint32_t)(SPI_BAUDRATEPRESCALER_8); //8 = 13.5Mhz, 108MHz
#endif

#ifdef STM32H7
  hspi_->Instance->CR1 |= (uint32_t)(SPI_BAUDRATEPRESCALER_8); //16 = 12.5Mhz, 200MHz
#endif

}


void IMUOnboard::mpuWrite(uint8_t address, uint8_t value)
{
  GPIO_L(spi_cs_port_, spi_cs_pin_);
  HAL_SPI_Transmit(hspi_, &address, 1, 1000);
  HAL_SPI_Transmit(hspi_, &value, 1, 1000);
  GPIO_H(spi_cs_port_, spi_cs_pin_);
}

uint8_t IMUOnboard::mpuRead(uint8_t address)
{
  uint8_t t_data[1] = {0};
  t_data[0] = address | 0x80;
  uint8_t temp;
  GPIO_L(spi_cs_port_, spi_cs_pin_);
  HAL_SPI_Transmit(hspi_, t_data, 1, 1000);
  HAL_SPI_Receive(hspi_, &temp, 1, 1000);
  GPIO_H(spi_cs_port_, spi_cs_pin_);
  return temp;
}

void IMUOnboard::gyroInit(void)
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
  //calib in the first time
}

void IMUOnboard::accInit (void)
{
  mpuWrite( 0x1C, 0x10); //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
  HAL_Delay(1); 
  //old: acceleration bandwidth is 460Hz
  mpuWrite( 0x1D, ACC_DLPF_CFG);
  HAL_Delay(10);
}


void IMUOnboard::magInit(void)
{
  /* check whether use external magnetometer */
  mag_id_ = checkExternalMag();

  if(mag_id_ == INTERNAL_MAG)
    {
      /* use internal mag */
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
  last_mag_time_ = HAL_GetTick();
}

uint8_t IMUOnboard::checkExternalMag(void)
{
  HAL_Delay(4000); // wait for the setup of the Blox M8N module, especially for winter
  uint8_t val[2];
  int i2c_status = 1;

  /*
   * we need check the i2c connectio with external mag(HMC5883L) several times initially,
   * since the module UBlox M8N has some problem with the gps/mag simultaneous polling.
   */

  for(int i = 0; i < EXTERNAL_MAG_CHECK_COUNT; i ++)
    {
      /* check the existence of external magnetometer */

      /* 1. LIS3MDL */
      val[0] = LIS3MDL_PING;
      i2c_status = HAL_I2C_Master_Transmit(hi2c_, LIS3MDL_MAG_REGISTER, val, 1, 100);
      HAL_I2C_Master_Receive(hi2c_, LIS3MDL_MAG_REGISTER, val, 1, 100);

      if(i2c_status == HAL_OK && val[0] == 61) //0b111101
        {
          HAL_Delay(10);
          lis3mdlInit();
          return LIS3MDL;
        }

      /* 2. HMC58X3 */
      val[0] = HMC58X3_R_CONFB;
      val[1] = 0x20;
      i2c_status = HAL_I2C_Master_Transmit(hi2c_, HMC58X3_MAG_REGISTER, val, 2, 100);

      if(i2c_status == HAL_OK)
        {
          HAL_Delay(10);
          hmc58x3Init();
          return HMC58X3;
        }
      HAL_Delay(10);
    }
  return INTERNAL_MAG;
}

void IMUOnboard::lis3mdlInit(void)
{
  uint8_t val[2];
  val[0] = LIS3MDL_CTRL_REG1;
  val[1] = LIS3MDL_OM_XY_HIGH | LIS3MDL_FAST_ODR;
  HAL_I2C_Master_Transmit(hi2c_, LIS3MDL_MAG_REGISTER, val, 2, 100);

  val[0] = LIS3MDL_CTRL_REG3;
  val[1] = LIS3MDL_CONTINUOUS_MODE;
  HAL_I2C_Master_Transmit(hi2c_, LIS3MDL_MAG_REGISTER, val, 2, 100);

  val[0] = LIS3MDL_CTRL_REG4;
  val[1] = LIS3MDL_OM_Z_HIGH;
  HAL_I2C_Master_Transmit(hi2c_, LIS3MDL_MAG_REGISTER, val, 2, 100);

  val[0] = LIS3MDL_CTRL_REG5;
  val[1] = LIS3MDL_BDU_MSBLSB;
  HAL_I2C_Master_Transmit(hi2c_, LIS3MDL_MAG_REGISTER, val, 2, 100);
}

void IMUOnboard::hmc58x3Init(void)
{
  uint8_t val[2];

  val[0] = HMC58X3_R_CONFA;
  val[1] = 0x18;//Configuration Register A  -- 0 00 110 00  num samples: 1 ; output rate: 75Hz ; normal measurement mode
  HAL_I2C_Master_Transmit(hi2c_, HMC58X3_MAG_REGISTER, val, 2, 100);
  HAL_Delay(10);
  val[0] = HMC58X3_R_MODE;
  val[1] =  0x00; //Mode register             -- 000000 00    continuous Conversion Mode
  HAL_I2C_Master_Transmit(hi2c_, HMC58X3_MAG_REGISTER, val, 2, 100);
  HAL_Delay(1);
}

void IMUOnboard::ledOutput()
{
  /* calibration pattern */
  if(calib_acc_ || calib_gyro_ || calib_mag_)
    {
      if(HAL_GetTick() - calib_indicator_time_ < 100) //ms
        GPIO_L(led_port_, led_pin_);
      else if(HAL_GetTick() - calib_indicator_time_ < 200) //ms
        GPIO_H(led_port_, led_pin_);
      else if(HAL_GetTick() - calib_indicator_time_ < 300) //ms
        GPIO_L(led_port_, led_pin_);
      else if(HAL_GetTick() - calib_indicator_time_ < 400) //ms
        GPIO_H(led_port_, led_pin_);

      if(HAL_GetTick() - calib_indicator_time_ > 1000) //ms
        calib_indicator_time_ = HAL_GetTick();
    }
  else GPIO_L(led_port_, led_pin_);

}

void IMUOnboard::updateRawData()
{
  static int mag_cnt = 0;
  uint8_t t_data[1];

  t_data[0] = GYRO_ADDRESS | 0x80;
  GPIO_L(spi_cs_port_, spi_cs_pin_);
  HAL_SPI_Transmit(hspi_, t_data, 1, 1000);
  HAL_SPI_Receive(hspi_, adc_, 6, 1000);
  GPIO_H(spi_cs_port_, spi_cs_pin_);

  /* we need add some delay between each sensor reading */
  raw_gyro_adc_[0] = (int16_t)(adc_[0] << 8 | adc_[1]) * 2000.0f / 32767.0f * M_PI / 180.0f  ;
  raw_gyro_adc_[1] = (int16_t)(adc_[2] << 8 | adc_[3]) * 2000.0f / 32767.0f * M_PI / 180.0f  ;
  raw_gyro_adc_[2] = (int16_t)(adc_[4] << 8 | adc_[5]) * 2000.0f / 32767.0f * M_PI / 180.0f  ;

  t_data[0] = ACC_ADDRESS | 0x80;
  GPIO_L(spi_cs_port_, spi_cs_pin_);
  HAL_SPI_Transmit(hspi_, t_data, 1, 1000);
  HAL_SPI_Receive(hspi_, adc_, 6, 1000);
  GPIO_H(spi_cs_port_, spi_cs_pin_);

  /* we need add some delay between each sensor reading */
  raw_acc_adc_[0] = (int16_t)(adc_[0] << 8 | adc_[1]) / 4096.0f * GRAVITY_MSS;
  raw_acc_adc_[1] = (int16_t)(adc_[2] << 8 | adc_[3]) / 4096.0f * GRAVITY_MSS;
  raw_acc_adc_[2] = (int16_t)(adc_[4] << 8 | adc_[5]) / 4096.0f * GRAVITY_MSS;

  switch(mag_id_)
    {
    case INTERNAL_MAG:
      {
        if(mag_cnt == MAG_READ_PRESCALER)
          {
            mag_cnt = 0;

            hspi_->Instance->CR1 &= (uint32_t)(~SPI_BAUDRATEPRESCALER_256); //reset
            hspi_->Instance->CR1 |= (uint32_t)(SPI_BAUDRATEPRESCALER_64); //128 = 0.8Mhz
            t_data[0] = MAG_SPI_ADDRESS | 0x80;
            GPIO_L(spi_cs_port_, spi_cs_pin_);
            HAL_SPI_Transmit(hspi_, t_data, 1, 1000);
            HAL_SPI_Receive(hspi_, adc_, 7, 1000);
            GPIO_H(spi_cs_port_, spi_cs_pin_);

            hspi_->Instance->CR1 &= (uint32_t)(~SPI_BAUDRATEPRESCALER_256); //reset
            hspi_->Instance->CR1 |= (uint32_t)(SPI_BAUDRATEPRESCALER_8); //8 = 13.5Mhz

            //uT(10e-6 T) + transform
            raw_mag_adc_[0] = (int16_t)(adc_[3] << 8 | adc_[2]) * 4912.0f / 32760.0f;
            raw_mag_adc_[1] = (int16_t)(adc_[1] << 8 | adc_[0]) * 4912.0f / 32760.0f;
            raw_mag_adc_[2] = (int16_t)(adc_[5] << 8 | adc_[4]) * 4912.0f / -32760.0f;
          }
        else mag_cnt++;

        break;
      }
    case LIS3MDL:
      {
        if(mag_cnt == LIS3MDL_READ_PRESCALER)
          {
            mag_cnt = 0;

            uint8_t val[2];
            val[0] = (LIS3MDL_OUTX_L | 0x80);
            HAL_I2C_Master_Transmit(hi2c_, LIS3MDL_MAG_REGISTER , val, 1, 100);
            HAL_I2C_Master_Receive(hi2c_, LIS3MDL_MAG_REGISTER , adc_, 6, 100);

            //uT(10e-6 T) + transform
            raw_mag_adc_[0] = (int16_t)(adc_[1] << 8 | adc_[0]) * LIS3MDL_MAG_RATE;
            raw_mag_adc_[1] = (int16_t)(adc_[3] << 8 | adc_[2]) * -LIS3MDL_MAG_RATE;
            raw_mag_adc_[2] = (int16_t)(adc_[5] << 8 | adc_[4]) * -LIS3MDL_MAG_RATE;
          }
        else mag_cnt++;

        break;
      }
    case HMC58X3:
      {
        if(mag_cnt == HMC58X3_READ_PRESCALER)
          {
            mag_cnt = 0;

            uint8_t val[2];
            val[0] = HMC58X3_DATA_REGISTER;
            HAL_I2C_Master_Transmit(hi2c_, HMC58X3_MAG_REGISTER , val, 1, 100);
            HAL_I2C_Master_Receive(hi2c_, HMC58X3_MAG_REGISTER , adc_, 6, 100);

            //uT(10e-6 T) + transform
            raw_mag_adc_[0] = (int16_t)(adc_[4] << 8 | adc_[5]) * -HMC58X3_RATE;
            raw_mag_adc_[1] = (int16_t)(adc_[0] << 8 | adc_[1]) * -HMC58X3_RATE;
            raw_mag_adc_[2] = (int16_t)(adc_[2] << 8 | adc_[3]) * -HMC58X3_RATE;
          }
        else mag_cnt++;

        break;
      }
    }

  setUpdate(true); //no need?

}
