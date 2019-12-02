/*
 * imu_ros_cmd.h
 *
 *  Created on: 2016/12/02
 *      Author: anzai
 */

#ifndef APPLICATION_JSK_LIB_SENSORS_IMU_IMU_ROS_CMD_H_
#define APPLICATION_JSK_LIB_SENSORS_IMU_IMU_ROS_CMD_H_

#include <ros.h>
#include <spinal/ImuCalib.h>
#include "sensors/imu/imu_basic.h"

namespace IMU_ROS_CMD {
	void init(ros::NodeHandle* nh);
	void addImu(IMU* imu);
};



#endif /* APPLICATION_JSK_LIB_SENSORS_IMU_IMU_ROS_CMD_H_ */
