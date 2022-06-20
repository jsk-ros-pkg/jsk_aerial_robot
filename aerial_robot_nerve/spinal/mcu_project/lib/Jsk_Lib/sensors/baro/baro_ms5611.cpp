/*
******************************************************************************
* File Name          : baro_ms5611.h
* Description        : Baro MS5611 Interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif


#include "sensors/baro/baro_ms5611.h"

Baro::Baro(): BaroBackend(), baro_config_sub_("baro_config_cmd", &Baro::baroConfigCallback, this )
{
  state_ = 0;
  tp_updated_ = false;
}

void Baro::init(I2C_HandleTypeDef* hi2c, ros::NodeHandle* nh,
                GPIO_TypeDef* baro_ctrl_port, uint16_t baro_ctrl_pin)
{
  i2c_handle_ = hi2c;
  nh_ = nh;
  nh_->subscribe(baro_config_sub_);

  baro_ctrl_port_ = baro_ctrl_port;
  baro_ctrl_pin_ = baro_ctrl_pin;

  GPIO_H(baro_ctrl_port_, baro_ctrl_pin_);
  //reset
  uint8_t reg[1];
  reg[0] = CMD_MS56XX_RESET;
  HAL_I2C_Master_Transmit(i2c_handle_, MS561101BA_ADDRESS, reg, 1, 100);

  HAL_Delay(10);
  uint16_t prom[8];
  if (!readCalib(prom)) return;
  calibrated_ = true;

  // Save factory calibration coefficients
  c1_ = prom[1];
  c2_ = prom[2];
  c3_ = prom[3];
  c4_ = prom[4];
  c5_ = prom[5];
  c6_ = prom[6];

  // Send a command to read temperature first
  reg[0] = ADDR_CMD_CONVERT_TEMPERATURE;
  //reg[0] = ADDR_CMD_CONVERT_PRESSURE;
  HAL_I2C_Master_Transmit(i2c_handle_, MS561101BA_ADDRESS, reg, 1, 100);
  last_timer_ = HAL_GetTick();
  state_ = 0;

  s_d1_ = 0;
  s_d2_ = 0;
  d1_count_ = 0;
  d2_count_ = 0;

}

/**
 * MS56XX crc4 method from datasheet for 16 bytes (8 short values)
 */
static uint16_t crc4(uint16_t *data)
{
  uint16_t n_rem = 0;
  uint8_t n_bit;

  for (uint8_t cnt = 0; cnt < 16; cnt++) {
    /* uneven bytes */
    if (cnt & 1) {
      n_rem ^= (uint8_t)((data[cnt >> 1]) & 0x00FF);
    } else {
      n_rem ^= (uint8_t)(data[cnt >> 1] >> 8);
    }

    for (n_bit = 8; n_bit > 0; n_bit--) {
      if (n_rem & 0x8000) {
        n_rem = (n_rem << 1) ^ 0x3000;
      } else {
        n_rem = (n_rem << 1);
      }
    }
  }

  return (n_rem >> 12) & 0xF;
}

bool Baro::readCalib(uint16_t prom[8])
{
  /*
   * MS5611-01BA datasheet, CYCLIC REDUNDANCY CHECK (CRC): "MS5611-01BA
   * contains a PROM memory with 128-Bit. A 4-bit CRC has been implemented
   * to check the data validity in memory."
   *
   * CRC field must me removed for CRC-4 calculation.
   */
  for (uint8_t i = 0; i < 8; i++) {
    prom[i] = readCalibWord(i);
  }

  /* save the read crc */
  const uint16_t crc_read = prom[7] & 0xf;

  /* remove CRC byte */
  prom[7] &= 0xff00;

  return crc_read == crc4(prom);
}

uint16_t Baro::readCalibWord(uint8_t word)
{
  uint8_t val[2];
  uint8_t reg[1];
  reg[0] = CMD_MS56XX_PROM + (word << 1);

  HAL_I2C_Master_Transmit(i2c_handle_, MS561101BA_ADDRESS, reg, 1, 100);
  HAL_I2C_Master_Receive(i2c_handle_, MS561101BA_ADDRESS, val, 2, 100);

  return (val[0] << 8) | val[1];
}

uint32_t Baro::readAdc()
{
  uint8_t val[3];

  if(HAL_I2C_Master_Receive(i2c_handle_, MS561101BA_ADDRESS, val, 3, 100) != HAL_OK){
    return 0;
  }
  return (val[0] << 16) | (val[1] << 8) | val[2];
}

/*
  Read the sensor. This is a state machine
  We read one time Temperature (state=1) and then 4 times Pressure (states 2-5)
  temperature does not change so quickly...
*/
void Baro::accumulate(void)
{
  // Throttle read rate to 100hz maximum.
  if (HAL_GetTick() - last_timer_ < 10) return;

  uint8_t reg[1];

  reg[0] = 0;
  HAL_I2C_Master_Transmit(i2c_handle_, MS561101BA_ADDRESS, reg, 1, 100);

  if (state_ == 0) {
    // On state 0 we read temp
    uint32_t d2 = readAdc();
    test_value = d2;
    if (d2 != 0) {
      s_d2_ += d2;
      d2_count_++;
      if (d2_count_ == 32) {
        // we have summed 32 values. This only happens
        // when we stop reading the barometer for a long time
        // (more than 1.2 seconds)
        s_d2_ >>= 1;
        d2_count_ = 16;
      }
      reg[0] = ADDR_CMD_CONVERT_PRESSURE;
      if(HAL_I2C_Master_Transmit(i2c_handle_, MS561101BA_ADDRESS, reg, 1, 100)  == HAL_OK){
        state_ ++;
      }
    } else {
      reg[0] = ADDR_CMD_CONVERT_TEMPERATURE;
      HAL_I2C_Master_Transmit(i2c_handle_, MS561101BA_ADDRESS, reg, 1, 100);
    }
  } else {
    uint32_t d1 = readAdc();
    if (d1 != 0) {
      // occasional zero values have been seen on the PXF
      // board. These may be SPI errors, but safest to ignore
      s_d1_ += d1;
      d1_count_ ++;
      if (d1_count_ == 128) {
        // we have summed 128 values. This only happens
        // when we stop reading the barometer for a long time
        // (more than 1.2 seconds)
        s_d1_ >>= 1;
        d1_count_ = 64;
      }

      // Now a new reading exists
      tp_updated_ = true;

      //if (state_ == 1) {
      if (state_ == 4) {
        reg[0] = ADDR_CMD_CONVERT_TEMPERATURE;
        if(HAL_I2C_Master_Transmit(i2c_handle_, MS561101BA_ADDRESS, reg, 1, 100)  == HAL_OK){
          state_ = 0;
        }
      } else {
        reg[0] = ADDR_CMD_CONVERT_PRESSURE;

        if(HAL_I2C_Master_Transmit(i2c_handle_, MS561101BA_ADDRESS, reg, 1, 100)  == HAL_OK){
          state_ ++;
        }
      }
					
    } else {
      /* if read fails, re-initiate a pressure read command or we are
       * stuck */
      reg[0] = ADDR_CMD_CONVERT_PRESSURE;
      HAL_I2C_Master_Transmit(i2c_handle_, MS561101BA_ADDRESS, reg, 1, 100);
    }
  }

  last_timer_ = HAL_GetTick();
}

void Baro::update(bool calibrate)
{
  accumulate();

  if (!tp_updated_) return;

  uint32_t sD1, sD2;
  uint8_t d1count, d2count;

  // Suspend timer procs because these variables are written to
  // in "_update".
  sD1 = s_d1_; s_d1_ = 0;
  sD2 = s_d2_; s_d2_ = 0;
  d1count = d1_count_; d1_count_ = 0;
  d2count = d2_count_; d2_count_ = 0;
  tp_updated_ = false;

  if (d1count != 0) {
    d1_ = ((float)sD1) / d1count;
  }
  if (d2count != 0) {
    d2_ = ((float)sD2) / d2count;
  }
  calculate();

  if(calibrate_count_ > 0)
    {
      base_pressure_ += pressure_;
      calibrate_count_ --;
      //alt_offset_ = 0;
      if(calibrate_count_ == 0)
        base_pressure_ /= CALIBRATE_COUNT;
      return;
    }

  if(calibrate)
    {
      base_pressure_ = pressure_;
      alt_offset_ = 0;
    }

  baseUpdate();
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void Baro::calculate()
{
  float dT;
  float TEMP;
  float OFF;
  float SENS;

  // Formulas from manufacturer datasheet
  // sub -15c temperature compensation is not included

  // we do the calculations using floating point allows us to take advantage
  // of the averaging of D1 and D1 over multiple samples, giving us more
  // precision
  dT = d2_-(((uint32_t)c5_)<<8);
  TEMP = (dT * c6_)/8388608;
  OFF = c2_ * 65536.0f + (c4_ * dT) / 128;
  SENS = c1_ * 32768.0f + (c3_ * dT) / 256;

  if (TEMP < 0) {
    // second order temperature compensation when under 20 degrees C
    float T2 = (dT*dT) / 0x80000000;
    float Aux = TEMP*TEMP;
    float OFF2 = 2.5f*Aux;
    float SENS2 = 1.25f*Aux;
    TEMP = TEMP - T2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;
  }

  pressure_ = (d1_*SENS/2097152 - OFF)/32768;
  temperature_ = (TEMP + 2000) * 0.01f;
}

void Baro::baroConfigCallback(const std_msgs::UInt8& config_msg)
{
  //TODO
  //temp
  if(config_msg.data == 1) {}
}

