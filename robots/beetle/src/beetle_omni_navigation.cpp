// -*- mode: c++ -*-

#include <beetle/beetle_omni_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

void BeetleOmniNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator)
{
  GimbalrotorNavigator::initialize(nh, nhp, robot_model, estimator);
  nh_ = nh;
  nhp_ = nhp;
  beetle_robot_model_ = boost::dynamic_pointer_cast<BeetleRobotModel>(robot_model);
}

void BeetleOmniNavigator::update()
{
  beetle_robot_model_->calcCenterOfMoving();
  GimbalrotorNavigator::update();
  beetle_robot_model_->setHoveringFlag((getNaviState() == HOVER_STATE));
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::BeetleOmniNavigator, aerial_robot_navigation::BaseNavigator);