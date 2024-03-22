// -*- mode: c++ -*-

#include <gimbalrotor/gimbalrotor_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

GimbalrotorNavigator::GimbalrotorNavigator():
  BaseNavigator(),
  curr_target_baselink_rot_(0, 0, 0),
  final_target_baselink_rot_(0, 0, 0)
{
}

void GimbalrotorNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                      double loop_du)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  curr_target_baselink_rot_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);
  final_target_baselink_rot_sub_ = nh_.subscribe("final_target_baselink_rot", 1, &GimbalrotorNavigator::setFinalTargetBaselinkRotCallback, this);
  prev_rotation_stamp_ = ros::Time::now().toSec();

}

void GimbalrotorNavigator::update()
{
  BaseNavigator::update();
  baselinkRotationProcess();
}

void GimbalrotorNavigator::setFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg)
{
  final_target_baselink_rot_.setValue(msg->roll, msg->pitch, msg->yaw);
}

void GimbalrotorNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr & msg)
{
  BaseNavigator::naviCallback(msg);
  if(msg->roll_nav_mode == 2) setTargetRoll(msg->target_roll);
  if(msg->pitch_nav_mode == 2) setTargetPitch(msg->target_pitch);
}

void GimbalrotorNavigator::baselinkRotationProcess()
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

void GimbalrotorNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();

  ros::NodeHandle navi_nh(nh_, "navigation");

  getParam<double>(navi_nh, "baselink_rot_change_thresh", baselink_rot_change_thresh_, 0.02);  // the threshold to change the baselink rotation
  getParam<double>(navi_nh, "baselink_rot_pub_interval", baselink_rot_pub_interval_, 0.1); // the rate to pub baselink rotation command
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::GimbalrotorNavigator, aerial_robot_navigation::BaseNavigator);
