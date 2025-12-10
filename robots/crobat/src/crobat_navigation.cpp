// -*- mode: c++ -*-

#include <crobat/crobat_navigation.h>

using namespace aerial_robot_navigation;

CrobatNavigator::CrobatNavigator():
  GimbalrotorNavigator()
{
}

void CrobatNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                      double loop_du)
{
  GimbalrotorNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);
}

void CrobatNavigator::update()
{
  GimbalrotorNavigator::update();
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::CrobatNavigator, aerial_robot_navigation::BaseNavigator);
