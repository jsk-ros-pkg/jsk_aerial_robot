#ifndef AERIAL_ROBOT_BASE_H
#define AERIAL_ROBOT_BASE_H

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <aerial_robot_base/control/flight_control.h>
#include <aerial_robot_base/flight_navigation.h>
#include <aerial_robot_base/state_estimation.h>
#include <aerial_robot_model/transformable_aerial_robot_model_ros.h>
#include <boost/thread.hpp>
#include <iostream>

using namespace std;

class AerialRobotBase
{
 public:
  AerialRobotBase(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~ AerialRobotBase();

  void mainFunc(const ros::TimerEvent & e);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Timer  main_timer_;
  double main_rate_;

  boost::shared_ptr<aerial_robot_model::RobotModelRos> robot_model_ros_;

  StateEstimator*  estimator_;
  Navigator* navigator_;
  pluginlib::ClassLoader<control_plugin::ControlBase> controller_loader_;
  boost::shared_ptr<control_plugin::ControlBase> controller_;


};


#endif
