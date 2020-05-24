#pragma once

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <aerial_robot_base/control/flight_control.h>
#include <aerial_robot_base/flight_navigation.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <aerial_robot_model/transformable_aerial_robot_model_ros.h>

using namespace std;

class AerialRobotBase
{
 public:
  AerialRobotBase(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~AerialRobotBase();

  void mainFunc(const ros::TimerEvent & e);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Timer main_timer_;

  boost::shared_ptr<aerial_robot_model::RobotModelRos> robot_model_ros_;
  boost::shared_ptr<aerial_robot_estimation::StateEstimator>  estimator_;

  Navigator* navigator_;

  pluginlib::ClassLoader<control_plugin::ControlBase> controller_loader_;
  boost::shared_ptr<control_plugin::ControlBase> controller_;
};
