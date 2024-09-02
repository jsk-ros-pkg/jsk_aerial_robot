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

  perching_flag_sub_ = nh_.subscribe("/perching_state", 1, &HugmyNavigator::perchingFlagCallback, this);
}

void HugmyNavigator::update()
{
  BaseNavigator::update();
  if(perching_flag_){
    takeoff_height_ = init_height_ + 0.4;
    ROS_ERROR("takeoff_height is %f", takeoff_height_);
    perching_flag_ = false;
  }
}

void HugmyNavigator::perchingFlagCallback(const std_msgs::UInt8& msg)
{
  ROS_ERROR("perching is %o", perching_flag_);
  if (msg.data == 1){
    perching_flag_ = true;
  }
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::HugmyNavigator, aerial_robot_navigation::BaseNavigator);
