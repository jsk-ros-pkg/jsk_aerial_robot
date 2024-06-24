/*
******************************************************************************
* File Name          : icm_mpu9250.h
* Description        : IMU(ICM20948) Interface
* Author             : J.Sugihara 2024/3/11
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __IMU_ICM_H
#define __IMU_ICM_H

#include "config.h"
#include "sensors/imu/drivers/mpu9250/imu_mpu9250.h"


#define SENSOR_DATA_LENGTH 7
#define IMU_FIND_TIMEOUT 10000
#define MAG_FIND_TIMEOUT 10000

#define MAG_READ_INTERVAL 10

/* ICM-20948 Registers */
#define ICM20948_ID						0xEA
#define REG_BANK_SEL					0x7F

// USER BANK 0
#define B0_WHO_AM_I						0x00		
#define B0_USER_CTRL					0x03
#define B0_LP_CONFIG					0x05
#define B0_PWR_MGMT_1					0x06
#define B0_PWR_MGMT_2					0x07
#define B0_INT_PIN_CFG					0x0F		
#define B0_INT_ENABLE					0x10
#define B0_INT_ENABLE_1					0x11
#define B0_INT_ENABLE_2					0x12
#define B0_INT_ENABLE_3					0x13
#define B0_I2C_MST_STATUS				0x17		
#define B0_INT_STATUS					0x19		
#define B0_INT_STATUS_1					0x1A
#define B0_INT_STATUS_2					0x1B
#define B0_INT_STATUS_3					0x1C
#define B0_DELAY_TIMEH					0x28
#define B0_DELAY_TIMEL					0x29
#define B0_ACCEL_XOUT_H					0x2D		
#define B0_ACCEL_XOUT_L					0x2E		
#define B0_ACCEL_YOUT_H					0x2F		
#define B0_ACCEL_YOUT_L					0x30		
#define B0_ACCEL_ZOUT_H					0x31		
#define B0_ACCEL_ZOUT_L					0x32	
#define B0_GYRO_XOUT_H					0x33	
#define B0_GYRO_XOUT_L					0x34
#define B0_GYRO_YOUT_H					0x35
#define B0_GYRO_YOUT_L					0x36
#define B0_GYRO_ZOUT_H					0x37
#define B0_GYRO_ZOUT_L					0x38
#define B0_TEMP_OUT_H					0x39		
#define B0_TEMP_OUT_L					0x3A
#define B0_EXT_SLV_SENS_DATA_00			0x3B
#define B0_EXT_SLV_SENS_DATA_01			0x3C
#define B0_EXT_SLV_SENS_DATA_02			0x3D
#define B0_EXT_SLV_SENS_DATA_03			0x3E
#define B0_EXT_SLV_SENS_DATA_04			0x3F
#define B0_EXT_SLV_SENS_DATA_05			0x40
#define B0_EXT_SLV_SENS_DATA_06			0x41
#define B0_EXT_SLV_SENS_DATA_07			0x42
#define B0_EXT_SLV_SENS_DATA_08			0x43
#define B0_EXT_SLV_SENS_DATA_09			0x44
#define B0_EXT_SLV_SENS_DATA_10			0x45
#define B0_EXT_SLV_SENS_DATA_11			0x46
#define B0_EXT_SLV_SENS_DATA_12			0x47
#define B0_EXT_SLV_SENS_DATA_13			0x48
#define B0_EXT_SLV_SENS_DATA_14			0x49
#define B0_EXT_SLV_SENS_DATA_15			0x4A
#define B0_EXT_SLV_SENS_DATA_16			0x4B
#define B0_EXT_SLV_SENS_DATA_17			0x4C
#define B0_EXT_SLV_SENS_DATA_18			0x4D
#define B0_EXT_SLV_SENS_DATA_19			0x4E
#define B0_EXT_SLV_SENS_DATA_20			0x4F
#define B0_EXT_SLV_SENS_DATA_21			0x50
#define B0_EXT_SLV_SENS_DATA_22			0x51
#define B0_EXT_SLV_SENS_DATA_23			0x52
#define B0_FIFO_EN_1					0x66	
#define B0_FIFO_EN_2					0x67
#define B0_FIFO_RST						0x68
#define B0_FIFO_MODE					0x69
#define B0_FIFO_COUNTH					0X70
#define B0_FIFO_COUNTL					0X71
#define B0_FIFO_R_W						0x72
#define B0_DATA_RDY_STATUS				0x74
#define B0_FIFO_CFG						0x76	

// USER BANK 1
#define B1_SELF_TEST_X_GYRO				0x02	
#define B1_SELF_TEST_Y_GYRO				0x03
#define B1_SELF_TEST_Z_GYRO				0x04
#define B1_SELF_TEST_X_ACCEL			0x0E	
#define B1_SELF_TEST_Y_ACCEL			0x0F
#define B1_SELF_TEST_Z_ACCEL			0x10
#define B1_XA_OFFS_H					0x14	
#define B1_XA_OFFS_L					0x15
#define B1_YA_OFFS_H					0x17
#define B1_YA_OFFS_L					0x18
#define B1_ZA_OFFS_H					0x1A
#define B1_ZA_OFFS_L					0x1B
#define B1_TIMEBASE_CORRECTION_PLL		0x28	

// USER BANK 2
#define B2_GYRO_SMPLRT_DIV				0x00	
#define B2_GYRO_CONFIG_1				0x01	
#define B2_GYRO_CONFIG_2				0x02
#define B2_XG_OFFS_USRH					0x03	
#define B2_XG_OFFS_USRL 				0x04
#define B2_YG_OFFS_USRH					0x05
#define B2_YG_OFFS_USRL					0x06
#define B2_ZG_OFFS_USRH					0x07
#define B2_ZG_OFFS_USRL					0x08
#define B2_ODR_ALIGN_EN					0x09	
#define B2_ACCEL_SMPLRT_DIV_1			0x10	
#define B2_ACCEL_SMPLRT_DIV_2			0x11		
#define B2_ACCEL_INTEL_CTRL				0x12		
#define B2_ACCEL_WOM_THR				0x13
#define B2_ACCEL_CONFIG					0x14
#define B2_ACCEL_CONFIG_2				0x15
#define B2_FSYNC_CONFIG					0x52
#define B2_TEMP_CONFIG					0x53
#define B2_MOD_CTRL_USR					0X54

// USER BANK 3#
#define B3_I2C_MST_ODR_CONFIG			0x00
#define B3_I2C_MST_CTRL					0x01
#define B3_I2C_MST_DELAY_CTRL			0x02	
#define B3_I2C_SLV0_ADDR				0x03
#define B3_I2C_SLV0_REG					0x04		
#define B3_I2C_SLV0_CTRL				0x05
#define B3_I2C_SLV0_DO					0x06
#define B3_I2C_SLV1_ADDR				0x07		
#define B3_I2C_SLV1_REG					0x08		
#define B3_I2C_SLV1_CTRL				0x09
#define B3_I2C_SLV1_DO					0x0A
#define B3_I2C_SLV2_ADDR				0x0B		
#define B3_I2C_SLV2_REG					0x0C		
#define B3_I2C_SLV2_CTRL				0x0D
#define B3_I2C_SLV2_DO					0x0E
#define B3_I2C_SLV3_ADDR				0x0F		
#define B3_I2C_SLV3_REG					0x10		
#define B3_I2C_SLV3_CTRL				0x11
#define B3_I2C_SLV3_DO					0x12
#define B3_I2C_SLV4_ADDR				0x13	
#define B3_I2C_SLV4_REG					0x14		
#define B3_I2C_SLV4_CTRL				0x15
#define B3_I2C_SLV4_DO					0x16
#define B3_I2C_SLV4_DI					0x17
	

/* AK09916 Registers */
#define AK09916_ID						0x09
#define MAG_SLAVE_ADDR                  0x0C

#define MAG_WIA2						0x01
#define MAG_ST1							0x10

#define MAG_HXL							0x11
#define MAG_HXH							0x12
#define MAG_HYL							0x13
#define MAG_HYH							0x14
#define MAG_HZL							0x15
#define MAG_HZH							0x16

#define MAG_ST2							0x18
#define MAG_CNTL2						0x31
#define MAG_CNTL3						0x32
#define MAG_TS1							0x33
#define MAG_TS2							0x34

/* Intructions */
#define READ							0x80
#define WRITE							0x00

/* Typedefs */
typedef enum
  {
   ub_0 = 0 << 4,
   ub_1 = 1 << 4,
   ub_2 = 2 << 4,
   ub_3 = 3 << 4
  } userbank;

typedef enum
  {
   _250dps,
   _500dps,
   _1000dps,
   _2000dps
  } gyro_full_scale;

typedef enum
  {
   _2g,
   _4g,
   _8g,
   _16g
  } accel_full_scale;

typedef enum
  {
   /*
     You can find this informatoin at https://www.y-ic.es/datasheet/78/SMDSW.020-2OZ.pdf
   */
   power_down_mode = 0,
   single_measurement_mode = 1,
   continuous_measurement_10hz = 2,
   continuous_measurement_20hz = 4,
   continuous_measurement_50hz = 6,
   continuous_measurement_100hz = 8
  } operation_mode;

class ICM20948 : public IMUOnboard {
public:
  ICM20948():IMUOnboard(){}
  ~ICM20948(){}
  void init(SPI_HandleTypeDef* hspi, I2C_HandleTypeDef* hi2c, ros::NodeHandle* nh,
            GPIO_TypeDef* spi_cs_port, uint16_t spi_cs_pin,
            GPIO_TypeDef* led_port, uint16_t led_pin);

  enum{INTERNAL_MAG = 0, LIS3MDL = 1, HMC58X3 = 2,};

private:
  float gyro_scale_factor_;
  float accel_scale_factor_;

  /* raw adc data */
  uint8_t single_adc_;
  uint8_t multi_adc_[6];
  float raw_temp_adc_;

  /* Initialization */
  void gyroInit(void) override;
  void accInit(void) override;
  void magInit(void) override;

  /* Update */
  void updateRawData (void) override;

  /* Raw 16 bits adc data reading function */
  void gyroRead(Vector3f* data);	
  void accelRead(Vector3f* data);
  bool magRead(Vector3f* data);
  void tempRead(float* data);

  /* Conversion from raw adc data to proper units */
  void gyroReadRad(Vector3f* data);
  void accelReadG(Vector3f* data);
  bool magReadUT(Vector3f* data);
  void tempReadC(float* data);

  /* Sub functions */
  bool getIcm20948WhoAmI();
  bool getAk09916WhoAmI();

  void deviceReset();
  void magSoftReset();

  void wakeUp();
  void sleep();

  void spiModeEnable();

  void intPinBpEnable();
  void i2cMasterReset();
  void i2cMasterEnable();
  void i2cMasterClkFrq(uint8_t config); // 0 - 15
  void i2cOdrCfg(uint8_t config);

  void setClockSource(uint8_t source);
  void odrAlignEnable();

  void setGyroLpf(uint8_t config); // 0 - 7
  void setAccelLpf(uint8_t config); // 0 - 7

  void setGyroSampleRate(uint8_t divider);
  void setAccelSampleRate(uint16_t divider);
  void setMagOperationMode(operation_mode mode);

  void setGyroFullScale(gyro_full_scale full_scale);
  void setAccelFullScale(accel_full_scale full_scale);

  /* Fundamental functions */
  void selectUserBank(userbank ub);
    
  void readSingleIcm20948(userbank ub, uint8_t reg);
  void writeSingleIcm20948(userbank ub, uint8_t reg, uint8_t val);

  void readMultipleIcm20948(userbank ub, uint8_t reg);
  void writeMultipleIcm20948(userbank ub, uint8_t reg, uint8_t* val, uint8_t len);

  void readSingleAk09916(uint8_t reg);
  void writeSingleAk09916(uint8_t reg, uint8_t val);

  void readMultipleAk09916(uint8_t reg);

};
#endif
