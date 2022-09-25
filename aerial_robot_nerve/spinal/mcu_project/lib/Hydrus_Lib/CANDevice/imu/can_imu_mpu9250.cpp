/*
 * can_imu_mpu9250.cpp
 *
 *  Created on: 2016/11/07
 *      Author: anzai
 */

#include "can_imu_mpu9250.h"
#include <string.h>

void CANIMU::updateRawData()
{
	/* transform */
	raw_gyro_adc_[0] = -r_gyro_data[1] * GYRO_SCALE;
	raw_gyro_adc_[1] = r_gyro_data[0] * GYRO_SCALE;
	raw_gyro_adc_[2] = r_gyro_data[2] * GYRO_SCALE;
	raw_acc_adc_[0] = -r_acc_data[1] * ACC_SCALE;
	raw_acc_adc_[1] = r_acc_data[0] * ACC_SCALE;
	raw_acc_adc_[2] = r_acc_data[2] * ACC_SCALE;

	raw_mag_adc_[0] = -r_mag_data[0] * MAG_SCALE;
	raw_mag_adc_[1] = r_mag_data[1] * MAG_SCALE;
	raw_mag_adc_[2] = -r_mag_data[2] * MAG_SCALE;
}

void CANIMU::sendData()
{
	return;
}

void CANIMU::receiveDataCallback(uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data)
{
	switch (message_id) {
	case CAN::MESSAGEID_SEND_GYRO:
		memcpy(r_gyro_data, data, sizeof(uint8_t) * 6);
		break;
	case CAN::MESSAGEID_SEND_ACC:
		memcpy(r_acc_data, data, sizeof(uint8_t) * 6);
		break;
	case CAN::MESSAGEID_SEND_MAG:
		memcpy(r_mag_data, data, sizeof(uint8_t) * 6);
		break;
	}
}
