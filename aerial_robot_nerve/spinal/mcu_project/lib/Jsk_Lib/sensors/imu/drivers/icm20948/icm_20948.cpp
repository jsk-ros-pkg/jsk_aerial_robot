/*
******************************************************************************
* File Name          : icm_20948.cpp
* Description        : IMU(ICM20948) Interface
* Author             : J.Sugihara 2024/3/11
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "sensors/imu/drivers/icm20948/icm_20948.h"

void ICM20948::init(SPI_HandleTypeDef* hspi, I2C_HandleTypeDef* hi2c, ros::NodeHandle* nh,
                      GPIO_TypeDef* spi_cs_port, uint16_t spi_cs_pin,
                      GPIO_TypeDef* led_port, uint16_t led_pin)
{
  nh_ = nh;
  IMU::init();

  spi_cs_port_ = spi_cs_port;
  spi_cs_pin_ = spi_cs_pin;
  led_port_ = led_port;
  led_pin_ = led_pin;

  hspi_ = hspi;
  hi2c_ = hi2c;

  calib_indicator_time_ = HAL_GetTick();

#ifdef STM32H7
  hspi_->Instance->CR1 |= (uint32_t)(SPI_BAUDRATEPRESCALER_64);
#endif

  gyroInit();
  accInit();
  magInit();

}

void ICM20948::gyroInit(void)
{
  HAL_Delay(100);

  /* Waiting for finding Imu */

  while(!getIcm20948WhoAmI());
  
  /* 1.Clear all bits in ub0-ub3 register */
  deviceReset();

  /* 2.Wakeup icm20948 and set fundamental properties */
  setClockSource(1); // Clock source is automatically selected.
  odrAlignEnable(); // Synchronize odr with sampling rates.
  spiModeEnable(); // Use only SPI mode
  
  /* 3.Gyro initialization (Do not change following order) */
  setGyroLpf(3);
  setGyroSampleRate(0);
  setGyroFullScale(_2000dps);  
}

void ICM20948::accInit (void)
{
  HAL_Delay(100);
  /* 5.Acc initialization (Do not change following order) */
  setAccelLpf(6);
  setAccelSampleRate(0);
  setAccelFullScale(_8g);
}

void ICM20948::magInit(void)
{
  HAL_Delay(100);
  /* 6.1 Check whether use external magnetometer */
  mag_id_ = IMUOnboard::checkExternalMag();

  if(mag_id_ == INTERNAL_MAG)
    {
      /* 6.2 Enable communication between ICM20948 and AK09916*/
      // ICM20948 can get magnetometer value from AK09916 via I2C using AUX_CL and AUX_DA. Then, we configure ICM20948 as a master device and AK09916 as a slave device.

      // intPinBpEnable();
      /* 6.2.1 Reset I2C bus*/
      i2cMasterReset();
      /* 6.2.2 Enable I2C bus*/
      i2cMasterEnable();
      /* 6.2.3 Set communication freq of I2C */
      i2cMasterClkFrq(7); // 345.6 kHz / 46.67% dyty cycle (recoomended)

      /* Waiting for finding Magnetometer */
      while(!getAk09916WhoAmI());

      /* 6.3 Clear all bits in mag register */
      magSoftReset();
      /* 6.4 Wakeup and set operation mode to magnetometer */
      setMagOperationMode(continuous_measurement_100hz);
    }
}

void ICM20948::updateRawData()
{
  gyroReadRad(&raw_gyro_adc_);
  accelReadG(&raw_acc_adc_);
  magReadUT(&raw_mag_adc_);
  tempReadC(&raw_temp_adc_);
}

void ICM20948::gyroRead(Vector3f* data)
{
  uint8_t read_reg = READ | B0_GYRO_XOUT_H;
  selectUserBank(ub_0);
  readMultipleIcm20948(ub_0, B0_GYRO_XOUT_H);

  data->x = (int16_t)(multi_adc_[0] << 8 | multi_adc_[1]);
  data->y = (int16_t)(multi_adc_[2] << 8 | multi_adc_[3]);
  data->z = (int16_t)(multi_adc_[4] << 8 | multi_adc_[5]);
}

void ICM20948::accelRead(Vector3f* data)
{
  readMultipleIcm20948(ub_0, B0_ACCEL_XOUT_H);

  data->x = (int16_t)(multi_adc_[0] << 8 | multi_adc_[1]);
  data->y = (int16_t)(multi_adc_[2] << 8 | multi_adc_[3]);
  data->z = (int16_t)(multi_adc_[4] << 8 | multi_adc_[5]); 
}

bool ICM20948::magRead(Vector3f* data)
{
  static uint32_t pre_mag_read;

  /* following AK09916's update frequency*/
  if(HAL_GetTick() - pre_mag_read > MAG_READ_INTERVAL)
    pre_mag_read = HAL_GetTick();
  else
    return false;

  uint8_t* temp;
  uint8_t drdy, hofl;	// data ready, overflow

  readSingleAk09916(MAG_ST1);
  drdy = single_adc_ & 0x01;
  if(!drdy) return false;

  readMultipleAk09916(MAG_HXL);

  readSingleAk09916(MAG_ST2);  
  hofl = single_adc_ & 0x08;
  if(hofl) return false;

  data->x = (int16_t)(multi_adc_[1] << 8 | multi_adc_[0]);
  data->y = (int16_t)(multi_adc_[3] << 8 | multi_adc_[2]);
  data->z = (int16_t)(multi_adc_[5] << 8 | multi_adc_[4]);
  return true;
}

void ICM20948::tempRead(float* data)
{
  readSingleIcm20948(ub_0, B0_TEMP_OUT_H);
  uint8_t temp_high = single_adc_;
  readSingleIcm20948(ub_0, B0_TEMP_OUT_L);
  uint8_t temp_low = single_adc_;
  *data = (float)(temp_high << 8 | temp_low);
}

void ICM20948::gyroReadRad(Vector3f* data)
{
  gyroRead(data);

  data->x = data->x / gyro_scale_factor_ * M_PI / 180.0f;
  data->y = data->y / gyro_scale_factor_ * M_PI / 180.0f;
  data->z = data->z / gyro_scale_factor_ * M_PI / 180.0f;
}

void ICM20948::accelReadG(Vector3f* data)
{
  accelRead(data);
  /*Temperature correction*/
  float corrected_scale_factor = accel_scale_factor_+ accel_scale_factor_ * (0.026f * raw_temp_adc_ )/100.0f;
  data->x = data->x / corrected_scale_factor * GRAVITY_MSS;
  data->y = data->y / corrected_scale_factor * GRAVITY_MSS;
  data->z = data->z / corrected_scale_factor * GRAVITY_MSS;
}

bool ICM20948::magReadUT(Vector3f* data)
{
  Vector3f temp;
  bool new_data = magRead(&temp);
  if(!new_data) return false;

  data->x = (float)(temp.x * 0.15);
  data->y = (float)(temp.y * 0.15);
  data->z = (float)(temp.z * -0.15);

  return true;
}	

void ICM20948::tempReadC(float* data)
{
  tempRead(data);
  *data = *data/333.87f + 21.0f;
}

/* Sub Functions */
bool ICM20948::getIcm20948WhoAmI()
{
  readSingleIcm20948(ub_0, B0_WHO_AM_I);

  if(single_adc_ == ICM20948_ID)
    return true;
  else
    return false;
}

bool ICM20948::getAk09916WhoAmI()
{
  readSingleAk09916(MAG_WIA2);
  if(single_adc_ == AK09916_ID)
    return true;
  else
    return false;
}

void ICM20948::deviceReset()
{
  writeSingleIcm20948(ub_0, B0_PWR_MGMT_1, 0x80 | 0x41);
  HAL_Delay(100);
}

void ICM20948::magSoftReset()
{
  writeSingleAk09916(MAG_CNTL3, 0x01);
  HAL_Delay(100);
}

void ICM20948::wakeUp()
{
  readSingleIcm20948(ub_0, B0_PWR_MGMT_1);
  uint8_t new_val = single_adc_ & 0xBF;

  writeSingleIcm20948(ub_0, B0_PWR_MGMT_1, new_val);
  HAL_Delay(100);
}

void ICM20948::sleep()
{
  readSingleIcm20948(ub_0, B0_PWR_MGMT_1);
  uint8_t new_val = single_adc_ | 0x40;

  writeSingleIcm20948(ub_0, B0_PWR_MGMT_1, new_val);
  HAL_Delay(100);
}

void ICM20948::spiModeEnable()
{
  readSingleIcm20948(ub_0, B0_USER_CTRL);
  uint8_t new_val = single_adc_ | 0x10;

  HAL_Delay(100);

  writeSingleIcm20948(ub_0, B0_USER_CTRL, new_val);
  HAL_Delay(100);
}

void ICM20948::intPinBpEnable()
{
  uint8_t new_val = 0x01;
  writeSingleIcm20948(ub_0, B0_INT_PIN_CFG, new_val);
  HAL_Delay(100);
}

void ICM20948::i2cMasterReset()
{
  readSingleIcm20948(ub_0, B0_USER_CTRL);
  uint8_t new_val = single_adc_ | 0x02;

  HAL_Delay(100);

  writeSingleIcm20948(ub_0, B0_USER_CTRL, new_val);
  HAL_Delay(100);
}

void ICM20948::i2cMasterEnable()
{
  readSingleIcm20948(ub_0, B0_USER_CTRL);
  uint8_t new_val = single_adc_ | 0x20;

  HAL_Delay(100);

  writeSingleIcm20948(ub_0, B0_USER_CTRL, new_val);
  HAL_Delay(100);
}

void ICM20948::i2cMasterClkFrq(uint8_t config)
{
  readSingleIcm20948(ub_3, B3_I2C_MST_CTRL);
  uint8_t new_val = single_adc_ | config;

  HAL_Delay(100);

  writeSingleIcm20948(ub_3, B3_I2C_MST_CTRL, new_val);
  HAL_Delay(100);
}

void ICM20948::i2cOdrCfg(uint8_t config)
{
  /* ODR = 1.1 kHz/(2^config) */
  uint8_t new_val = config;
  writeSingleIcm20948(ub_0, B3_I2C_MST_ODR_CONFIG, new_val);
  HAL_Delay(100);
}

void ICM20948::setClockSource(uint8_t source)
{
  uint8_t new_val = 0x01;

  writeSingleIcm20948(ub_0, B0_PWR_MGMT_1, new_val);
  HAL_Delay(100);
}


void ICM20948::odrAlignEnable()
{
  writeSingleIcm20948(ub_2, B2_ODR_ALIGN_EN, 0x01);
  HAL_Delay(100);
}

void ICM20948::setGyroLpf(uint8_t config)
{
  readSingleIcm20948(ub_2, B2_GYRO_CONFIG_1);
  uint8_t new_val =  single_adc_ | (config << 3);

  writeSingleIcm20948(ub_2, B2_GYRO_CONFIG_1, new_val);
}

void ICM20948::setAccelLpf(uint8_t config)
{
  readSingleIcm20948(ub_2, B2_ACCEL_CONFIG);
  uint8_t new_val =  single_adc_ | (config << 3);

  writeSingleIcm20948(ub_2, B2_ACCEL_CONFIG, new_val);
}

void ICM20948::setGyroSampleRate(uint8_t divider)
{
  writeSingleIcm20948(ub_2, B2_GYRO_SMPLRT_DIV, divider);
  HAL_Delay(100);
}

void ICM20948::setAccelSampleRate(uint16_t divider)
{
  uint8_t divider_1 = (uint8_t)(divider >> 8);
  uint8_t divider_2 = (uint8_t)(0x0F & divider);

  writeSingleIcm20948(ub_2, B2_ACCEL_SMPLRT_DIV_1, divider_1);
  writeSingleIcm20948(ub_2, B2_ACCEL_SMPLRT_DIV_2, divider_2);
  HAL_Delay(100);
}

void ICM20948::setMagOperationMode(operation_mode mode)
{
  writeSingleAk09916(MAG_CNTL2, mode);
  HAL_Delay(100);
}


void ICM20948::setGyroFullScale(gyro_full_scale full_scale)
{
  readSingleIcm20948(ub_2, B2_GYRO_CONFIG_1);
  uint8_t new_val = single_adc_;
	
  switch(full_scale)
    {
    case _250dps :
      new_val |= 0x00;
      gyro_scale_factor_ = 131.0;
      break;
    case _500dps :
      new_val |= 0x02;
      gyro_scale_factor_ = 65.5;
      break;
    case _1000dps :
      new_val |= 0x04;
      gyro_scale_factor_ = 32.8;
      break;
    case _2000dps :
      new_val |= 0x06;
      gyro_scale_factor_ = 16.4;
      break;
    }

  writeSingleIcm20948(ub_2, B2_GYRO_CONFIG_1, new_val);
  HAL_Delay(100);
}

void ICM20948::setAccelFullScale(accel_full_scale full_scale)
{
  readSingleIcm20948(ub_2, B2_ACCEL_CONFIG);
  uint8_t new_val = single_adc_;

  HAL_Delay(100);
	
  switch(full_scale)
    {
    case _2g :
      new_val |= 0x00;
      accel_scale_factor_ = 16384;
      break;
    case _4g :
      new_val |= 0x02;
      accel_scale_factor_ = 8192;
      break;
    case _8g :
      new_val |= 0x04;
      accel_scale_factor_ = 4096;
      break;
    case _16g :
      new_val |= 0x06;
      accel_scale_factor_ = 2048;
      break;
    }

  writeSingleIcm20948(ub_2, B2_ACCEL_CONFIG, new_val);
  HAL_Delay(100);
}

void ICM20948::selectUserBank(userbank ub)
{
  uint8_t write_reg[2];
  write_reg[0] = WRITE | REG_BANK_SEL;
  write_reg[1] = ub;

  GPIO_L(spi_cs_port_, spi_cs_pin_);
  HAL_SPI_Transmit(hspi_, write_reg, 2, 10);
  GPIO_H(spi_cs_port_, spi_cs_pin_);
}

void ICM20948::readSingleIcm20948(userbank ub, uint8_t reg)
{
  uint8_t read_reg = READ | reg;
  selectUserBank(ub);

  GPIO_L(spi_cs_port_, spi_cs_pin_);
  HAL_SPI_Transmit(hspi_, &read_reg, 1, 1000);
  HAL_SPI_Receive(hspi_, &single_adc_, 1, 1000);
  GPIO_H(spi_cs_port_, spi_cs_pin_);
}

void ICM20948::writeSingleIcm20948(userbank ub, uint8_t reg, uint8_t val)
{
  uint8_t write_reg[2];
  write_reg[0] = WRITE | reg;
  write_reg[1] = val;
  selectUserBank(ub);
  GPIO_L(spi_cs_port_, spi_cs_pin_);
  HAL_SPI_Transmit(hspi_, write_reg, 2, 1000);
  GPIO_H(spi_cs_port_, spi_cs_pin_);
}

void ICM20948::readMultipleIcm20948(userbank ub, uint8_t reg)
{
  uint8_t read_reg = READ | reg;
  uint8_t len = (uint8_t)(sizeof(multi_adc_)/sizeof(multi_adc_[0]));
  selectUserBank(ub);

  GPIO_L(spi_cs_port_, spi_cs_pin_);
  HAL_SPI_Transmit(hspi_, &read_reg, 1, 1000);
  HAL_SPI_Receive(hspi_, multi_adc_, len, 1000);
  GPIO_H(spi_cs_port_, spi_cs_pin_);

}

void ICM20948::writeMultipleIcm20948(userbank ub, uint8_t reg, uint8_t* val, uint8_t len)
{
  uint8_t write_reg = WRITE | reg;
  selectUserBank(ub);

  GPIO_L(spi_cs_port_, spi_cs_pin_);
  HAL_SPI_Transmit(hspi_, &write_reg, 1, 1000);
  HAL_SPI_Transmit(hspi_, val, len, 1000);
  GPIO_H(spi_cs_port_, spi_cs_pin_);
}

void ICM20948::readSingleAk09916(uint8_t reg)
{
  writeSingleIcm20948(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
  writeSingleIcm20948(ub_3, B3_I2C_SLV0_REG, reg);
  writeSingleIcm20948(ub_3, B3_I2C_SLV0_CTRL, 0x81);
  readSingleIcm20948(ub_0, B0_EXT_SLV_SENS_DATA_00);
}

void ICM20948::writeSingleAk09916(uint8_t reg, uint8_t val)
{
  writeSingleIcm20948(ub_3, B3_I2C_SLV0_ADDR, WRITE | MAG_SLAVE_ADDR);
  writeSingleIcm20948(ub_3, B3_I2C_SLV0_REG, reg);
  writeSingleIcm20948(ub_3, B3_I2C_SLV0_DO, val);
  HAL_Delay(1);
  writeSingleIcm20948(ub_3, B3_I2C_SLV0_CTRL, 0x81);
}

void ICM20948::readMultipleAk09916(uint8_t reg)
{
  uint8_t len = (uint8_t)(sizeof(multi_adc_)/sizeof(multi_adc_[0]));
  writeSingleIcm20948(ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
  writeSingleIcm20948(ub_3, B3_I2C_SLV0_REG, reg);
  writeSingleIcm20948(ub_3, B3_I2C_SLV0_CTRL, 0x80 | len);
  HAL_Delay(1);
  readMultipleIcm20948(ub_0, B0_EXT_SLV_SENS_DATA_00);
}
