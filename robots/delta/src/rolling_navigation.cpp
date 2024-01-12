#include <delta/rolling_navigation.h>

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
                                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                  double loop_du)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  rolling_robot_model_ = boost::dynamic_pointer_cast<RollingRobotModel>(robot_model_);

  curr_target_baselink_rot_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);
  final_target_baselink_rot_sub_ = nh_.subscribe("final_target_baselink_rot", 1, &RollingNavigator::setFinalTargetBaselinkRotCallback, this);
  joy_sub_ = nh_.subscribe("joy", 1, &RollingNavigator::joyCallback, this);
  ground_navigation_mode_sub_ = nh_.subscribe("ground_navigation_command", 1, &RollingNavigator::groundNavigationModeCallback, this);
  ground_navigation_mode_pub_ = nh_.advertise<std_msgs::Int16>("ground_navigation_ack", 1);
  prev_rotation_stamp_ = ros::Time::now().toSec();

  prev_ground_navigation_mode_ = -1;
  current_ground_navigation_mode_ = 0;
  baselink_rot_force_update_mode_ = false;

  target_pitch_ang_vel_ = 0.0;
  target_yaw_ang_vel_ = 0.0;
  pitch_ang_vel_updating_ = false;
  yaw_ang_vel_updating_ = false;
}

void RollingNavigator::update()
{
  BaseNavigator::update();

  baselinkRotationProcess();

  landingProcess();

  groundModeProcess();
}

void RollingNavigator::reset()
{
  BaseNavigator::reset();

  curr_target_baselink_rot_.setValue(0, 0, 0);
  final_target_baselink_rot_.setValue(0, 0, 0);

  prev_ground_navigation_mode_ = 0;
  current_ground_navigation_mode_ = 0;

  spinal::DesireCoord target_baselink_rot_msg;
  target_baselink_rot_msg.roll = curr_target_baselink_rot_.x();
  target_baselink_rot_msg.pitch = curr_target_baselink_rot_.y();
  curr_target_baselink_rot_pub_.publish(target_baselink_rot_msg);

  landing_flag_ = false;
  baselink_rot_force_update_mode_ = false;

  target_pitch_ang_vel_ = 0.0;
  target_yaw_ang_vel_ = 0.0;
  pitch_ang_vel_updating_ = false;
  yaw_ang_vel_updating_ = false;

  ROS_INFO_STREAM("[navigation] reset navigator");

}

void RollingNavigator::baselinkRotationProcess()
{
  // if(curr_target_baselink_rot_ == final_target_baselink_rot_) return;

  spinal::DesireCoord target_baselink_rot_msg;

  /* publish desire coord with constant timestep */
  if(ros::Time::now().toSec() - prev_rotation_stamp_ > baselink_rot_pub_interval_)
    {
      /* force update mode */
      if(baselink_rot_force_update_mode_)
        {
          target_baselink_rot_msg.roll = curr_target_baselink_rot_.x();
          target_baselink_rot_msg.pitch = curr_target_baselink_rot_.y();
          final_target_baselink_rot_.setValue(curr_target_baselink_rot_.x(), curr_target_baselink_rot_.y(), curr_target_baselink_rot_.z());
        }
      /* linear interpolation */
      else
        {
          if((final_target_baselink_rot_- curr_target_baselink_rot_).length() > baselink_rot_change_thresh_)
            {
              double curr_cog_roll = estimator_->getEuler(Frame::COG, estimate_mode_).x();
              double curr_cog_pitch = estimator_->getEuler(Frame::COG, estimate_mode_).y();
              if(fabs(curr_cog_roll) < baselink_rotation_stop_error_ && fabs(curr_cog_pitch) < baselink_rotation_stop_error_)
                {
                  curr_target_baselink_rot_ += ((final_target_baselink_rot_ - curr_target_baselink_rot_).normalize() * baselink_rot_change_thresh_);
                }
              else
                {
                  ROS_WARN_STREAM("[navigation] do not update desire coord because rp error [" << curr_cog_roll << ", " << curr_cog_pitch << "] is larger than " << baselink_rotation_stop_error_);
                }
            }
          else
            {
              curr_target_baselink_rot_ = final_target_baselink_rot_;
            }
          target_baselink_rot_msg.roll = curr_target_baselink_rot_.x();
          target_baselink_rot_msg.pitch = curr_target_baselink_rot_.y();
        }

      curr_target_baselink_rot_pub_.publish(target_baselink_rot_msg);
      prev_rotation_stamp_ = ros::Time::now().toSec();
    }

  curr_target_baselink_rot_roll_ = curr_target_baselink_rot_.x();
  curr_target_baselink_rot_pitch_ = curr_target_baselink_rot_.y();
  final_target_baselink_rot_roll_ = final_target_baselink_rot_.x();
  final_target_baselink_rot_pitch_ = final_target_baselink_rot_.y();

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
          // setTargetPosZ(estimator_->getLandingHeight());
          setTeleopFlag(true);
        }
    }
}

void RollingNavigator::groundModeProcess()
{
  std_msgs::Int16 msg;
  msg.data = current_ground_navigation_mode_;
  ground_navigation_mode_pub_.publish(msg);
}


void RollingNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();

  ros::NodeHandle navi_nh(nh_, "navigation");

  getParam<double>(navi_nh, "baselink_rot_change_thresh", baselink_rot_change_thresh_, 0.02);  // the threshold to change the baselink rotation
  getParam<double>(navi_nh, "baselink_rot_pub_interval", baselink_rot_pub_interval_, 0.1); // the rate to pub baselink rotation command
  getParam<double>(navi_nh, "baselink_rotation_stop_error", baselink_rotation_stop_error_, 3.14);
  getParam<double>(navi_nh, "joy_stick_deadzone", joy_stick_deadzone_, 0.2);
  getParam<double>(navi_nh, "rolling_max_pitch_ang_vel", rolling_max_pitch_ang_vel_, 0.0);
  getParam<double>(navi_nh, "rolling_max_yaw_ang_vel", rolling_max_yaw_ang_vel_, 0.0);

}

void RollingNavigator::setGroundNavigationMode(int state)
{
  if(state == aerial_robot_navigation::FLYING_STATE && current_ground_navigation_mode_ != aerial_robot_navigation::FLYING_STATE)
    {
      ROS_WARN_STREAM("[navigation] switch to flying state");
    }

  if(state == aerial_robot_navigation::STANDING_STATE && current_ground_navigation_mode_ != aerial_robot_navigation::STANDING_STATE)
    {
      standing_initial_pos_ = estimator_->getPos(Frame::COG, estimate_mode_);
      standing_initial_euler_ = estimator_->getEuler(Frame::COG, estimate_mode_);
      ROS_WARN_STREAM("[navigation] switch to staning mode");
      ROS_WARN_STREAM("[navigation] standing inital pos = [" << standing_initial_pos_.x() << " " << standing_initial_pos_.y() << " " << standing_initial_pos_.z() << "]");
      ROS_WARN_STREAM("[navigation] standing inital euler = [" << standing_initial_euler_.x() << " " << standing_initial_euler_.y() << " " << standing_initial_euler_.z() << "]");
      current_ground_navigation_mode_ = state;
    }

  if(state == aerial_robot_navigation::STEERING_STATE && current_ground_navigation_mode_ != aerial_robot_navigation::STEERING_STATE)
    {
      ROS_WARN_STREAM("[navigation] switch to steering mode");

      steering_initial_pos_ = estimator_->getPos(Frame::COG, estimate_mode_);
      steering_initial_euler_ = estimator_->getEuler(Frame::COG, estimate_mode_);
      current_ground_navigation_mode_ = state;
    }

  if(state == aerial_robot_navigation::ROLLING_STATE && current_ground_navigation_mode_ != aerial_robot_navigation::ROLLING_STATE)
    {
      ROS_WARN_STREAM("[navigation] switch to rolling mode");
      current_ground_navigation_mode_ = state;
      rolling_initial_pos_ = estimator_->getPos(Frame::COG, estimate_mode_);
      rolling_initial_euler_ = estimator_->getEuler(Frame::COG, estimate_mode_);
    }

  if(state == aerial_robot_navigation::RECOVERING_STATE && current_ground_navigation_mode_ != aerial_robot_navigation::RECOVERING_STATE)
    {
      current_ground_navigation_mode_ = state;
      ROS_ERROR_STREAM("[navigation] recovery state");
    }
}

void RollingNavigator::groundNavigationModeCallback(const std_msgs::Int16Ptr & msg)
{
  ROS_WARN_STREAM("[navigation] set ground navigation mode to " << msg->data);
  setGroundNavigationMode(msg->data);
}

void RollingNavigator::setFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg)
{
  ROS_WARN_STREAM("[navigation] baselink rotation callback. RPY: " << msg->roll << " " << msg->pitch << " " << msg->yaw);
  setFinalTargetBaselinkRot(tf::Vector3(msg->roll, msg->pitch, msg->yaw));
}

void RollingNavigator::joyCallback(const sensor_msgs::JoyConstPtr & joy_msg)
{
  sensor_msgs::Joy joy_cmd = (*joy_msg);

  if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_1] && joy_cmd.axes[PS4_AXIS_BUTTON_CROSS_UP_DOWN] == 1.0 && current_ground_navigation_mode_ != STANDING_STATE)
    {
      ROS_INFO("[joy] change to standing state");
      setGroundNavigationMode(STANDING_STATE);
    }

  if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_1] && joy_cmd.axes[PS4_AXIS_BUTTON_CROSS_UP_DOWN] == -1.0 && current_ground_navigation_mode_ != STEERING_STATE)
    {
      ROS_INFO("[joy] change to steering state");
      setGroundNavigationMode(STEERING_STATE);
    }

  // if(joy_cmd.axes[PS4_AXIS_BUTTON_CROSS_UP_DOWN] == -1.0)
  //   {
  //     final_target_baselink_rot_.setX(curr_target_baselink_rot_.x() - 0.1);
  //     ROS_WARN_STREAM("[joy] set target final baselink roll: " << final_target_baselink_rot_.x());
  //   }
  // if(joy_cmd.axes[PS4_AXIS_BUTTON_CROSS_UP_DOWN] == 1.0)
  //   {
  //     final_target_baselink_rot_.setX(curr_target_baselink_rot_.x() + 0.1);
  //     ROS_WARN_STREAM("[joy] set target final baselink roll: " << final_target_baselink_rot_.x());
  //   }

  /* set target angular velocity around yaw based on R-stick hilizontal */
  if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_1] && fabs(joy_cmd.axes[PS4_AXIS_STICK_RIGHT_LEFTWARDS]) > joy_stick_deadzone_)
    {
      target_yaw_ang_vel_ = rolling_max_yaw_ang_vel_ * joy_cmd.axes[PS4_AXIS_STICK_RIGHT_LEFTWARDS];
      yaw_ang_vel_updating_ = true;
      // ROS_WARN_STREAM("[joy] set target yaw ang vel to " << target_yaw_ang_vel_ << " rad/s");
    }
  else
    {
      if(yaw_ang_vel_updating_)
        {
          target_yaw_ang_vel_ = 0.0;
          // ROS_WARN_STREAM("[joy] set target yaw ang vel to  0");
          yaw_ang_vel_updating_ = false;
        }
    }

  /* set target angular velocity around pitch based on L-stick vertical */
  if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_1] && fabs(joy_cmd.axes[PS4_AXIS_STICK_LEFT_UPWARDS]) > joy_stick_deadzone_)
    {
      target_pitch_ang_vel_ = rolling_max_pitch_ang_vel_ * joy_cmd.axes[PS4_AXIS_STICK_LEFT_UPWARDS];
      pitch_ang_vel_updating_ = true;
      // ROS_WARN_STREAM("[joy] set target pitch ang vel to " << target_pitch_ang_vel_ << " rad/s");
    }
  else
    {
      if(pitch_ang_vel_updating_)
        {
          target_pitch_ang_vel_ = 0.0;
          // ROS_WARN_STREAM("[joy] set target pitch ang vel to 0");
          pitch_ang_vel_updating_ = false;
        }
    }

}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::RollingNavigator, aerial_robot_navigation::BaseNavigator);
