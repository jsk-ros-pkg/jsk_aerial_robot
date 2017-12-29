/*
 * imu_ros_cmd.cpp
 *
 *  Created on: 2016/12/02
 *      Author: anzai
 */

#include "imu_ros_cmd.h"
#include <vector>
#include "flashmemory/flashmemory.h"

namespace IMU_ROS_CMD {
	namespace {
		constexpr uint8_t RESET_CALIB_CMD = 0x00;
		constexpr uint8_t MPU_ACC_GYRO_CALIB_CMD = 0x01;
		constexpr uint8_t MPU_MAG_CALIB_CMD = 0x02;
		constexpr uint8_t FLASH_WRITE_CMD = 0x03;
		std::vector<IMU*> imu_;
	}

	void imuConfigCallback(const std_msgs::UInt8& config_msg)
	{
		switch(config_msg.data) {
	  		case RESET_CALIB_CMD:
	  			for(unsigned int i = 0; i < imu_.size(); i++) {
	  				imu_[i]->resetCalib();
	  			}
	  			FlashMemory::erase();
	  			FlashMemory::write();
	  			break;
	  		case MPU_ACC_GYRO_CALIB_CMD:
	  			for(unsigned int i = 0; i < imu_.size(); i++) {
	  				imu_[i]->startGyroCalib();
	  				imu_[i]->startAccCalib();
	  			}
	  			break;
	  		case MPU_MAG_CALIB_CMD:
	  			for(unsigned int i = 0; i < imu_.size(); i++) {
	  				imu_[i]->startMagCalib();
	  			}
	  			break;
	  		case FLASH_WRITE_CMD:
	  			FlashMemory::erase();
	  			FlashMemory::write();
	  			break;
		}
	}

	namespace {
		ros::Subscriber<std_msgs::UInt8> imu_config_sub_("/imu_config_cmd", imuConfigCallback);
		ros::NodeHandle* nh_;
	}

	void init(ros::NodeHandle* nh) {
		nh_ = nh;
		nh_->subscribe< ros::Subscriber<std_msgs::UInt8> >(imu_config_sub_);
	}

	void addImu(IMU* imu) {
		imu_.push_back(imu);
	}
}


