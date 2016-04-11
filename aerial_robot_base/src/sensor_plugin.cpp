#include <pluginlib/class_list_macros.h>
#include <aerial_robot_base/sensor/sensor_base_plugin.h>
#include <aerial_robot_base/sensor/imu.h>
#include <aerial_robot_base/sensor/slam2d.h>
#include <aerial_robot_base/sensor/optical_flow.h>
#include <aerial_robot_base/sensor/mirror_height.h>
#include <aerial_robot_base/sensor/mocap.h>

PLUGINLIB_EXPORT_CLASS(sensor_plugin::Imu, sensor_base_plugin::SensorBase);
PLUGINLIB_EXPORT_CLASS(sensor_plugin::OpticalFlow, sensor_base_plugin::SensorBase);
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Mocap, sensor_base_plugin::SensorBase);
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Mirror, sensor_base_plugin::SensorBase);
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Slam2D, sensor_base_plugin::SensorBase);

