#include <rolling/rolling_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

RollingNavigator::RollingNavigator():
  BaseNavigator(),
  curr_target_baselink_rot_(0, 0, 0),
  final_target_baselink_rot_(0, 0, 0)
{
}

void RollingNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator);

  curr_target_baselink_rot_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);
  final_target_baselink_rot_sub_ = nh_.subscribe("final_target_baselink_rot", 1, &RollingNavigator::setFinalTargetBaselinkRotCallback, this);
  prev_rotation_stamp_ = ros::Time::now().toSec();
}

void RollingNavigator::reset()
{
  BaseNavigator::reset();

  curr_target_baselink_rot_.setValue(0, 0, 0);
  final_target_baselink_rot_.setValue(0, 0, 0);

  spinal::DesireCoord target_baselink_rot_msg;
  target_baselink_rot_msg.roll = curr_target_baselink_rot_.x();
  target_baselink_rot_msg.pitch = curr_target_baselink_rot_.y();
  curr_target_baselink_rot_pub_.publish(target_baselink_rot_msg);
}

void RollingNavigator::update()
{
  BaseNavigator::update();

  baselinkRotationProcess();
}

void RollingNavigator::setFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg)
{
  final_target_baselink_rot_.setValue(msg->roll, msg->pitch, msg->yaw);
}

void RollingNavigator::baselinkRotationProcess()
{
  if(curr_target_baselink_rot_ == final_target_baselink_rot_) return;

  if(ros::Time::now().toSec() - prev_rotation_stamp_ > baselink_rot_pub_interval_)
    {
      if((final_target_baselink_rot_- curr_target_baselink_rot_).length() > baselink_rot_change_thresh_)
        curr_target_baselink_rot_ += ((final_target_baselink_rot_ - curr_target_baselink_rot_).normalize() * baselink_rot_change_thresh_);
      else
        curr_target_baselink_rot_ = final_target_baselink_rot_;

      spinal::DesireCoord target_baselink_rot_msg;
      target_baselink_rot_msg.roll = curr_target_baselink_rot_.x();
      target_baselink_rot_msg.pitch = curr_target_baselink_rot_.y();
      curr_target_baselink_rot_pub_.publish(target_baselink_rot_msg);

      prev_rotation_stamp_ = ros::Time::now().toSec();
    }
}

void RollingNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();

  ros::NodeHandle navi_nh(nh_, "navigation");

  getParam<double>(navi_nh, "baselink_rot_change_thresh", baselink_rot_change_thresh_, 0.02);  // the threshold to change the baselink rotation
  getParam<double>(navi_nh, "baselink_rot_pub_interval", baselink_rot_pub_interval_, 0.1); // the rate to pub baselink rotation command
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::RollingNavigator, aerial_robot_navigation::BaseNavigator);
