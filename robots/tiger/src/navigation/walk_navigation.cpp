#include <tiger/navigation/walk_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation::Tiger;

WalkNavigator::WalkNavigator():
  BaseNavigator(),
  target_pos_(0,0,0),
  target_vel_(0,0,0),
  target_rpy_(0,0,0)
{
}

void WalkNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                 boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator);
}

void WalkNavigator::update()
{
  BaseNavigator::update();

  // TODO: add new states for walk/stand.
  // e.g., idle stand state, joint move state, fee joint end state

  tf::Vector3 baselink_pos = estimator_->getPos(Frame::BASELINK, estimate_mode_);
  tf::Vector3 baselink_vel = estimator_->getVel(Frame::BASELINK, estimate_mode_);
  tf::Vector3 baselink_rpy = estimator_->getEuler(Frame::BASELINK, estimate_mode_);

  if (getNaviState() == aerial_robot_navigation::START_STATE) {

    // set the target position for baselink
    ROS_INFO("[Walk][Navigator] set initial position as target position");
    target_pos_ = baselink_pos;
    target_rpy_ = baselink_rpy;
  }

}




void WalkNavigator::reset()
{
}


void WalkNavigator::halt()
{
  ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("joints/torque_enable");
  std_srvs::SetBool srv;
  srv.request.data = false;
  if (client.call(srv))
    ROS_INFO("dragon control halt process: disable the joint torque");
  else
    ROS_ERROR("Failed to call service joints/torque_enable");

  client = nh_.serviceClient<std_srvs::SetBool>("gimbals/torque_enable");

  srv.request.data = false;
  if (client.call(srv))
    ROS_INFO("dragon control halt process: disable the gimbal torque");
  else
    ROS_ERROR("Failed to call service gimbals/torque_enable");
}

void WalkNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();

  ros::NodeHandle navi_nh(nh_, "navigation");
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::Tiger::WalkNavigator, aerial_robot_navigation::BaseNavigator);
