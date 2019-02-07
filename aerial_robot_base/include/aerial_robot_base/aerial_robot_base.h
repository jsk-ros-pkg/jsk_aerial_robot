#ifndef AERIAL_ROBOT_BASE_H
#define AERIAL_ROBOT_BASE_H

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <aerial_robot_base/control/flight_control.h>
#include <aerial_robot_base/flight_navigation.h>
#include <aerial_robot_base/state_estimation.h>
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

  StateEstimator*  estimator_;
  Navigator* navigator_;
  boost::shared_ptr< pluginlib::ClassLoader<control_plugin::ControlBase> > controller_loader_ptr_;
  boost::shared_ptr<control_plugin::ControlBase> controller_;

};


#endif
