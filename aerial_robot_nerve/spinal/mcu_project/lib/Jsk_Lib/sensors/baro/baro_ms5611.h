/*
******************************************************************************
* File Name          : baro_ms5611.h
* Description        : Baro MS5611 Interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __BARO_MS5611_H
#define __BARO_MS5611_H


#include "sensors/baro/baro.h"

// #define BARO_H      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_SET)
// #define BARO_L      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET)

class Baro :public BaroBackend
{
public:
  Baro();

  void update(bool calibrate = false);
  void accumulate();
  void init(I2C_HandleTypeDef* hi2c, ros::NodeHandle* nh,
            GPIO_TypeDef* baro_ctrl_port, uint16_t baro_ctrl_pin);

  static const uint8_t MS561101BA_ADDRESS =  0xEC;  // 0x76<<1;

  static const uint8_t CMD_MS56XX_RESET = 0x1E;
  static const uint8_t CMD_MS56XX_READ_ADC = 0x00;

  /* PROM start address */
  static const uint8_t CMD_MS56XX_PROM = 0xA0;

  /* write to one of these addresses to start pressure conversion */
  static const uint8_t ADDR_CMD_CONVERT_D1_OSR256  = 0x40;
  static const uint8_t ADDR_CMD_CONVERT_D1_OSR512  = 0x42;
  static const uint8_t ADDR_CMD_CONVERT_D1_OSR1024 = 0x44;
  static const uint8_t ADDR_CMD_CONVERT_D1_OSR2048 = 0x46;
  static const uint8_t ADDR_CMD_CONVERT_D1_OSR4096 = 0x48;

  /* write to one of these addresses to start temperature conversion */
  static const uint8_t ADDR_CMD_CONVERT_D2_OSR256  = 0x50;
  static const uint8_t ADDR_CMD_CONVERT_D2_OSR512  = 0x52;
  static const uint8_t ADDR_CMD_CONVERT_D2_OSR1024 = 0x54;
  static const uint8_t ADDR_CMD_CONVERT_D2_OSR2048 = 0x56;
  static const uint8_t ADDR_CMD_CONVERT_D2_OSR4096 = 0x58;

  /*
    use an OSR of 1024 to reduce the self-heating effect of the
    sensor. Information from MS tells us that some individual sensors
    are quite sensitive to this effect and that reducing the OSR can
    make a big difference
  */
  static const uint8_t ADDR_CMD_CONVERT_PRESSURE = ADDR_CMD_CONVERT_D1_OSR4096; //ADDR_CMD_CONVERT_D1_OSR1024;
  static const uint8_t ADDR_CMD_CONVERT_TEMPERATURE = ADDR_CMD_CONVERT_D2_OSR4096;//ADDR_CMD_CONVERT_D2_OSR1024;

  /*

    #define MS561101BA_PRESSURE    0x40
    #define MS561101BA_TEMPERATURE 0x50
    #define OSR 0x08
  */

private:
  ros::NodeHandle* nh_;
  ros::Subscriber<std_msgs::UInt8, Baro> baro_config_sub_;

  void calculate();
  bool readCalib(uint16_t prom[8]);

  uint16_t readCalibWord(uint8_t word);
  uint32_t readAdc();

  GPIO_TypeDef* baro_ctrl_port_;
  uint16_t baro_ctrl_pin_;

  uint32_t test_value;

  /* Asynchronous state: */
  volatile bool            tp_updated_; //Temperature and Pressure are updated
  volatile uint8_t         d1_count_;
  volatile uint8_t         d2_count_;
  volatile uint32_t        s_d1_, s_d2_;
  uint8_t                  state_;

  // Internal calibration registers
  uint16_t                 c1_, c2_ , c3_, c4_, c5_, c6_;
  float                    d1_, d2_;

  void baroConfigCallback(const std_msgs::UInt8& config_msg);

};
#endif


