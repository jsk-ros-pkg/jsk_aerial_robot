/*
 * can_imu_mpu9250.h
 *
 *  Created on: 2016/11/07
 *      Author: anzai
 */

#ifndef APPLICATION_HYDRUS_LIB_CANDEVICE_IMU_CAN_IMU_MPU9250_H_
#define APPLICATION_HYDRUS_LIB_CANDEVICE_IMU_CAN_IMU_MPU9250_H_

#include "CAN/can_device.h"
#include "sensors/imu/imu_basic.h"

class CANIMU : public CANDevice, public IMU {
private:
	void updateRawData() override;
	int16_t r_gyro_data[3], r_acc_data[3], r_mag_data[3];
	bool send_data_flag_;
public:
	CANIMU(){}
	CANIMU(uint8_t slave_id, bool send_data_flag) : CANDevice(CAN::DEVICEID_IMU, slave_id), IMU(), send_data_flag_(send_data_flag){}
	void sendData() override;
	void receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data) override;
	bool getSendDataFlag() const { return send_data_flag_;}
	void setSendDataFlag(bool send_data_flag) {send_data_flag_ = send_data_flag;}

	static constexpr float GYRO_SCALE = 2000.0f / 32767.0f * M_PI / 180.0f;
	static constexpr float ACC_SCALE = GRAVITY_MSS / 4096.0f;
	static constexpr float MAG_SCALE = 4912.0f / 32760.0f;
};




#endif /* APPLICATION_HYDRUS_LIB_CANDEVICE_IMU_CAN_IMU_MPU9250_H_ */
