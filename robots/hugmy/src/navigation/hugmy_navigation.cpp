#include <hugmy/navigation/hugmy_navigation.h>

using namespace aerial_robot_navigation;

HugmyNavigator::HugmyNavigator():
  BaseNavigator()
{
}


void HugmyNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                double loop_du)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);
}

void HugmyNavigator::update()
{
  BaseNavigator::update();
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::HugmyNavigator, aerial_robot_navigation::BaseNavigator);
