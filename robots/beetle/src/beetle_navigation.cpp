// -*- mode: c++ -*-

#include <beetle/beetle_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

BeetleNavigator::BeetleNavigator():
  GimbalrotorNavigator()
{
}

void BeetleNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                   boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator)
{
  /* initialize the flight control */
  GimbalrotorNavigator::initialize(nh, nhp, robot_model, estimator);
}

void BeetleNavigator::update()
{
  GimbalrotorNavigator::update();
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::BeetleNavigator, aerial_robot_navigation::BaseNavigator);
