#ifndef APPLICATION_IMU_DRIVERS_MPU9250_MPU9250_H_
#define APPLICATION_IMU_DRIVERS_MPU9250_MPU9250_H_

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "IMU/imu_base.h"

#define SPI_POLLING_MODE 0
#define SPI_DMA_MODE 1
#define SPI_MODE SPI_POLLING_MODE
#define SENSOR_DATA_LENGTH 7

#define IMU_SPI_CS_H       HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET)
#define IMU_SPI_CS_L      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET)

/* magenetometer update at about 100Hz, but the entire sensor data polling process is at 1KHz, should add prescaler for mag */
#define MAG_PRESCALER 4
/* magnetometer has bad noise bacause of the polling process, I think. So, we use a threshold filter method to remove the outlier */

class MPU9250 : public IMU {
public:
  MPU9250(){}
  MPU9250(uint8_t slave_id):IMU(slave_id){}
  ~MPU9250(){}
  void init(SPI_HandleTypeDef* hspi) override;

  static  uint8_t adc_[SENSOR_DATA_LENGTH];

private:
  uint8_t dummy_[SENSOR_DATA_LENGTH];
  bool ahb_tx_suspend_flag_; // to avoid the confliction between SPI1 and USART1TX(ros)

  void gyroInit(void) override;
  void accInit(void) override;
  void magInit(void) override;
  void pollingRead() override;

  void mpuWrite(uint8_t address, uint8_t value);
  uint8_t mpuRead(uint8_t address);
};

#endif /* APPLICATION_IMU_DRIVERS_MPU9250_MPU9250_H_ */
