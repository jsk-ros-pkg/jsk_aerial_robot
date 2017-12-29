/*
 * imu_basic.h
 *
 *  Created on: 2016/11/01
 *      Author: anzai
 */

#ifndef APPLICATION_JSK_LIB_SENSORS_IMU_IMU_BASIC_H_
#define APPLICATION_JSK_LIB_SENSORS_IMU_IMU_BASIC_H_

#include "math/AP_Math.h"
#include "stm32f7xx_hal.h"
#include <ros.h>
#include <std_msgs/UInt8.h>
#include "config.h"
#include "math/definitions.h"

class IMU {
private:
	bool update_;
	static constexpr uint8_t ACC_LPF_FACTOR = 42; // old: 16
	static constexpr uint8_t GYRO_LPF_FACTOR = 12; //old: 8
	static constexpr int MAG_OUTLIER_MAX_COUNT = 500; //= 500ms, 1count = 1ms
	Vector3f gyro_, acc_, mag_; //board frame
	Vector3f gyro_v_, acc_v_, mag_v_; //virtual frame
	bool virtual_frame_; /* flag to decide wether use virtual frame */
	int calibrate_acc_, calibrate_gyro_, calibrate_mag_;
	Vector3f acc_offset_, gyro_offset_, mag_offset_;
	uint16_t mag_outlier_counter_;
	Vector3f raw_acc_, raw_gyro_, raw_mag_;
	Vector3f raw_gyro_p_, raw_acc_p_;
	Vector3f mag_max_, mag_min_;
	bool mag_filtering_flag_;
	/* calibration cmd form ROS */
	static constexpr uint32_t CALIBRATING_STEP =  1000;
	static constexpr uint32_t CALIBRATING_MAG_STEP =  30000; //about 30s (0.01s * 3000; 0.001s * 30000)
	static constexpr float MAG_GENERAL_THRESH = 20.0f;
	void readCalibData(void);
	void writeCalibData(void);
	void process (void);
	void ledOutput();

protected:

	virtual void updateRawData() = 0;
	Vector3f raw_gyro_adc_, raw_acc_adc_, raw_mag_adc_;

public:
	IMU();
	void init();
	Vector3f getGyro(bool force_board_frame = false)
	{
		if(force_board_frame) return gyro_;

		if(virtual_frame_) return gyro_v_;
		else return gyro_;
	}

	Vector3f getAcc(bool force_board_frame = false)
	{
		if(force_board_frame) return acc_;

		if(virtual_frame_) return acc_v_;
		else return acc_;
	}

	Vector3f getMag(bool force_board_frame = false)
	{
		if(force_board_frame) return mag_;

		if(virtual_frame_) return mag_v_;
		else return mag_;
	}

	inline void setGyroV(Vector3f gyro_v) { gyro_v_ = gyro_v; }
	inline void setAccV(Vector3f acc_v) { acc_v_ = acc_v; }
	inline void setMagV(Vector3f mag_v) { mag_v_ = mag_v; }

	bool getUpdate() {return update_;}
	void setUpdate(bool update) {update_ = update;}
	bool getCalibrated() {
	  if(!calibrate_acc_ && !calibrate_gyro_ && !calibrate_mag_) return true;
	  else return false;
	}
	void update();
	void startGyroCalib(){calibrate_gyro_ = CALIBRATING_STEP;}
	void startAccCalib(){calibrate_acc_ = CALIBRATING_STEP;}
	void startMagCalib(){calibrate_mag_ = CALIBRATING_MAG_STEP;}
	void resetCalib(){acc_offset_.zero();mag_offset_.zero();}

	inline bool getVirtualFrame() {return virtual_frame_;}
	inline void setVirtualFrame(bool virtual_frame) { virtual_frame_ = virtual_frame;}
};


#endif /* APPLICATION_JSK_LIB_SENSORS_IMU_IMU_BASIC_H_ */
