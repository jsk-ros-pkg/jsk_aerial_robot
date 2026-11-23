#ifndef APPLICATION_IMU_DRIVERS_ICM42686_ICM42686_H_
#define APPLICATION_IMU_DRIVERS_ICM42686_ICM42686_H_

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "IMU/imu_base.h"

class ICM42686 : public IMU {
public:
  ICM42686() : detected_(false), who_am_i_(0) {}
  ICM42686(uint8_t slave_id) : IMU(slave_id), detected_(false), who_am_i_(0) {}
  ~ICM42686() {}

  void init(SPI_HandleTypeDef* hspi) override;

  bool detected() const { return detected_; }
  uint8_t whoAmI() const { return who_am_i_; }

private:
  static const uint8_t SENSOR_DATA_LENGTH = 12;

  static const uint8_t REG_DEVICE_CONFIG = 0x11;
  static const uint8_t REG_ACCEL_DATA_X1 = 0x1F;
  static const uint8_t REG_PWR_MGMT0 = 0x4E;
  static const uint8_t REG_GYRO_CONFIG0 = 0x4F;
  static const uint8_t REG_ACCEL_CONFIG0 = 0x50;
  static const uint8_t REG_WHO_AM_I = 0x75;
  static const uint8_t REG_BANK_SEL = 0x76;

  static const uint8_t WHO_AM_I_VALUE = 0x44;
  static const uint8_t SOFT_RESET_CONFIG = 0x01;
  static const uint8_t GYRO_2000DPS_1KHZ = 0x26;
  static const uint8_t ACCEL_8G_1KHZ = 0x46;
  static const uint8_t ACCEL_GYRO_LOW_NOISE = 0x0F;

  uint8_t adc_[SENSOR_DATA_LENGTH];
  bool detected_;
  uint8_t who_am_i_;

  void gyroInit() override;
  void accInit() override;
  void pollingRead() override;

  void icmWrite(uint8_t address, uint8_t value);
  uint8_t icmRead(uint8_t address);
};

#endif /* APPLICATION_IMU_DRIVERS_ICM42686_ICM42686_H_ */
