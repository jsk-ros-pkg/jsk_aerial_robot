#include <rolling/rolling_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

RollingNavigator::RollingNavigator():
  BaseNavigator(),
  curr_target_baselink_rot_(0, 0, 0),
  final_target_baselink_rot_(0, 0, 0),
  landing_flag_(false)
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

  landing_flag_ = false;
}

void RollingNavigator::update()
{
  BaseNavigator::update();

  baselinkRotationProcess();

  landingProcess();
}

void RollingNavigator::setFinalTargetBaselinkRot(tf::Vector3 rot)
{
  final_target_baselink_rot_.setValue(rot.x(), rot.y(), rot.z());
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

void RollingNavigator::landingProcess()
{
  if(getForceLandingFlag() || getNaviState() == LAND_STATE)
    {
      if(curr_target_baselink_rot_.length())
        {
          ROS_WARN_ONCE("[navigation][landing] set final desired baselink rotation to (0, 0, 0)");
          final_target_baselink_rot_.setValue(0, 0, 0);

          if(getNaviState() == LAND_STATE && !landing_flag_)
            {
              ROS_WARN("[navigation][landing] set to hovering mode");
              landing_flag_ = true;
              setTeleopFlag(false);
              setTargetPosZ(estimator_->getState(State::Z_COG, estimate_mode_)[0]);
              setNaviState(HOVER_STATE);
            }
        }
    }

  /* back to landing process */
  if(landing_flag_)
    {
      bool already_level = true;

      if(curr_target_baselink_rot_.length()) already_level = false;

      if(!already_level)
        {
          ROS_WARN_ONCE("[navigation][landing] waiting for baselink rotation conversion");
        }

      if(already_level && getNaviState() == HOVER_STATE)
        {
          ROS_WARN("[navigation][landing] back to land state");
          setNaviState(LAND_STATE);
          setTargetPosZ(estimator_->getLandingHeight());
          setTeleopFlag(true);
        }
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
