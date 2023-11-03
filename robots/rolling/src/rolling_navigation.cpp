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

  rolling_robot_model_ = boost::dynamic_pointer_cast<RollingRobotModel>(robot_model_);

  curr_target_baselink_rot_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);
  final_target_baselink_rot_sub_ = nh_.subscribe("final_target_baselink_rot", 1, &RollingNavigator::setFinalTargetBaselinkRotCallback, this);
  joy_sub_ = nh_.subscribe("joy", 1, &RollingNavigator::joyCallback, this);
  ground_navigation_mode_sub_ = nh_.subscribe("ground_navigation_command", 1, &RollingNavigator::groundNavigationModeCallback, this);
  ground_navigation_mode_pub_ = nh_.advertise<std_msgs::Int16>("ground_navigation_ack", 1);
  prev_rotation_stamp_ = ros::Time::now().toSec();

  prev_ground_navigation_mode_ = 0;
  current_ground_navigation_mode_ = 0;
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

  ROS_INFO_STREAM("[navigation] reset navigator");

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
  getParam<double>(navi_nh, "rolling_joy_stick_deadzone", rolling_joy_sitck_deadzone_, 0.2);
  getParam<double>(navi_nh, "steering_joy_stick_deadzone", steering_joy_stick_deadzone_, 0.2);

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

}

void RollingNavigator::setFinalTargetBaselinkRot(tf::Vector3 rot)
{
  final_target_baselink_rot_.setValue(rot.x(), rot.y(), rot.z());
}

void RollingNavigator::groundNavigationModeCallback(const std_msgs::Int16Ptr & msg)
{
  ROS_WARN_STREAM("[navigation] set ground navigation mode to " << msg->data);
  setGroundNavigationMode(msg->data);
}

void RollingNavigator::setFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg)
{
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

  if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_1] && fabs(joy_cmd.axes[PS4_AXIS_STICK_RIGHT_LEFTWARDS]) > steering_joy_stick_deadzone_)
    {
      ROS_WARN_STREAM("[joy] set target yaw angle to ");
    }

  if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_1] && fabs(joy_cmd.axes[PS4_AXIS_STICK_LEFT_UPWARDS]) > rolling_joy_sitck_deadzone_)
    {
      ROS_WARN_STREAM("[joy] set rolling speed");
    }

  if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_1] && joy_cmd.buttons[PS4_BUTTON_REAR_RIGHT_1])
    {
      final_target_baselink_rot_.setX(0);
      final_target_baselink_rot_.setY(0);
      ROS_WARN("[joy] set target final baselink to horizon");
    }

}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::RollingNavigator, aerial_robot_navigation::BaseNavigator);
