// -*- mode: c++ -*-

#include <ninja/ninja_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

NinjaNavigator::NinjaNavigator():
  BeetleNavigator()
{
}

void NinjaNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                double loop_du)
{
  BeetleNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::NinjaNavigator, aerial_robot_navigation::BaseNavigator);
