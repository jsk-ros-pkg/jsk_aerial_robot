#include <hugmy/navigation/hugmy_navigation.h>

using namespace aerial_robot_navigation;


HugmyNavigator::HugmyNavigator():
  BaseNavigator(),
  perching_flag_(false)
{
}


void HugmyNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                double loop_du)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  perching_flag_sub_ = nh.subscribe("perching_flag", 1, &HugmyNavigator::perchingFlagCallback, this);
}

void HugmyNavigator::update()
{
  BaseNavigator::update();
  if(perching_flag_){
    takeoff_height_ = 0.5;
    perching_flag_ = false;
  }
}

void HugmyNavigator::perchingFlagCallback(const std_msgs::UInt8& msg)
{
  if (msg->data == 1){
    perching_flag_ = true;
  }else{
    perching_flag_ = false;
  }
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::HugmyNavigator, aerial_robot_navigation::BaseNavigator);
