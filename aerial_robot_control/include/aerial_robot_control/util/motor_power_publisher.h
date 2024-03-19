#pragma once

#include <ros/ros.h>
#include <spinal/PwmInfo.h>
#include <spinal/MotorInfo.h>
#include <spinal/Pwms.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

class motorPowerPublisher
{
public:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher motor_currencies_pub_;
  ros::Publisher motor_power_pub_;
  ros::Subscriber motor_pwms_sub_;
  ros::Subscriber battery_voltage_sub_;
  ros::Timer timer_;

  std::vector<float> motor_currencies_;
  std::vector<double> voltages_;
  std::vector<std::vector<double>> motor_infos_;
  std::vector<int> motor_pwms_;
  double battery_voltage_;

  motorPowerPublisher(ros::NodeHandle nh, ros::NodeHandle nhp);
  virtual ~motorPowerPublisher() = default;

  void motorPwmsCallback(const spinal::PwmsPtr& msg);
  void batteryVoltageStatusCallback(const std_msgs::Float32Ptr& msg);
  void timerCallback(const ros::TimerEvent& e);
  void rosParamInit();

  template<class T> void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
  {
    nh.param<T>(param_name, param, default_value);
  }

};
