#include <delta/navigation/delta_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

RollingNavigator::RollingNavigator():
  BaseNavigator(),
  poly_(11, agi::Vector<3>(0, 0, 1), 2)
{
  curr_target_baselink_quat_.setRPY(0, 0, 0);
  final_target_baselink_quat_.setRPY(0, 0, 0);
}

void RollingNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                  double loop_du)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  rolling_robot_model_ = boost::dynamic_pointer_cast<RollingRobotModel>(robot_model_);
  robot_model_for_plan_ = boost::make_shared<RollingRobotModel>();

  desire_coord_pub_ = nh_.advertise<geometry_msgs::Quaternion>("desire_coordinate", 1);
  final_target_baselink_quat_sub_ = nh_.subscribe("final_target_baselink_quat", 1, &RollingNavigator::setFinalTargetBaselinkQuatCallback, this);
  final_target_baselink_rpy_sub_ = nh_.subscribe("final_target_baselink_rpy", 1, &RollingNavigator::setFinalTargetBaselinkRpyCallback, this);
  joints_control_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  ik_target_rel_ee_pos_sub_ = nh_.subscribe("full_body_ik_target", 1, &RollingNavigator::fullBodyIKTargetRelPosCallback, this);
  joy_sub_ = nh_.subscribe("joy", 1, &RollingNavigator::joyCallback, this);
  ground_navigation_mode_sub_ = nh_.subscribe("ground_navigation_command", 1, &RollingNavigator::groundNavigationModeCallback, this);
  ground_motion_mode_sub_ = nh_.subscribe("ground_motion_command", 1, &RollingNavigator::groundMotionModeCallback, this);
  ground_navigation_mode_pub_ = nh_.advertise<std_msgs::Int16>("ground_navigation_ack", 1);
  ground_motion_mode_pub_ = nh_.advertise<std_msgs::Int16>("ground_motion_ack", 1);
  prev_rotation_stamp_ = ros::Time::now().toSec();

  setPrevGroundNavigationMode(aerial_robot_navigation::NONE);
  setGroundNavigationMode(aerial_robot_navigation::FLYING_STATE);
  motion_mode_ = aerial_robot_navigation::LOCOMOTION_MODE;

  transforming_flag_ = false;

  rotation_control_link_name_ = robot_model_->getBaselinkName();

  full_body_ik_initial_cp_p_ee_target_  = Eigen::Vector3d(0, 0, 0);

  controllers_reset_flag_ = false;

  target_pitch_ang_vel_ = 0.0;
  target_yaw_ang_vel_ = 0.0;
  pitch_ang_vel_updating_ = false;
  yaw_ang_vel_updating_ = false;
}

void RollingNavigator::update()
{
  BaseNavigator::update();


  if(motion_mode_ == aerial_robot_navigation::MANIPULATION_MODE)
    {
      calcEndEffetorJacobian();
      fullBodyIKSolve();
    }
  else if(motion_mode_ == aerial_robot_navigation::LOCOMOTION_MODE)
    {
      rollingPlanner();
    }

  baselinkRotationProcess();

  rosPublishProcess();
}

void RollingNavigator::reset()
{
  BaseNavigator::reset();

  curr_target_baselink_quat_.setRPY(0, 0, 0);
  final_target_baselink_quat_.setRPY(0, 0, 0);

  setPrevGroundNavigationMode(aerial_robot_navigation::NONE);
  setGroundNavigationMode(aerial_robot_navigation::FLYING_STATE);
  motion_mode_ = aerial_robot_navigation::LOCOMOTION_MODE;

  full_body_ik_initial_cp_p_ee_target_ = Eigen::Vector3d(0, 0, 0);

  setRotationControlLink(robot_model_->getBaselinkName());

  controllers_reset_flag_ = false;
  ground_trajectory_mode_ = false;
  poly_.reset();

  transforming_flag_ = false;

  target_pitch_ang_vel_ = 0.0;
  target_yaw_ang_vel_ = 0.0;
  pitch_ang_vel_updating_ = false;
  yaw_ang_vel_updating_ = false;

  ROS_INFO_STREAM("[navigation] reset navigator");

}

void RollingNavigator::startTakeoff()
{
  BaseNavigator::startTakeoff();

  setRotationControlLink(robot_model_->getBaselinkName());

  switch(current_ground_navigation_mode_)
    {
    case aerial_robot_navigation::STANDING_STATE:
      { /* if standing mode, generate new trajectory when starting */
        ground_trajectory_start_time_ = ros::Time::now().toSec();
        poly_.reset();
        poly_.scale(ground_trajectory_start_time_, ground_trajectory_duration_);
        poly_.addConstraint(ground_trajectory_start_time_, agi::Vector<3>(0.0, 0.0, 0.0));
        poly_.addConstraint(ground_trajectory_start_time_ + ground_trajectory_duration_, agi::Vector<3>(M_PI / 2.0, 0.0, 0.0));
        poly_.solve();
        ground_trajectory_mode_ = true;
        ROS_INFO_STREAM("[navigation] generate new standing trajectory from " << std::setprecision(12) << ground_trajectory_start_time_ << " to " << ground_trajectory_start_time_ + ground_trajectory_duration_);
        break;
      }
    case aerial_robot_navigation::ROLLING_STATE:
      { /* if rolling mode, set current baselink rotation as target baselink roation when starting */
        ground_trajectory_mode_ = false;
        Eigen::Vector3d b1 = Eigen::Vector3d(1.0, 0.0, 0.0), b2 = Eigen::Vector3d(0.0, 1.0, 0.0);

        tf::Vector3 baselink_euler = estimator_->getEuler(Frame::BASELINK, estimate_mode_);
        double target_roll, target_pitch;
        target_roll = M_PI / 2.0;
        if(baselink_euler.x() > 0)
          target_pitch = baselink_euler.y();
        else
          target_pitch = M_PI - baselink_euler.y();

        ROS_INFO_STREAM("[navigation] current baselink roll: " << baselink_euler.x() << " pitch: " << baselink_euler.y());
        ROS_INFO_STREAM("[navigation] set desire coordinate same as baselink roll: " << target_roll << " pitch: " << target_pitch << "\n");

        /* calculate desire orientation */
        Eigen::Matrix3d rot_mat;
        rot_mat = Eigen::AngleAxisd(target_pitch, b2) * Eigen::AngleAxisd(target_roll, b1);

        /* set desire coordinate */
        KDL::Rotation rot_mat_kdl = eigenToKdl(rot_mat);
        double qx, qy, qz, qw;
        rot_mat_kdl.GetQuaternion(qx, qy, qz, qw);
        setCurrentTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
        setFinalTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
        break;
      }
    default:
      break;
    }

  /* set iniital baselink rotation to robot_model and set current yaw angle as target */
  baselinkRotationProcess();
  setTargetYawFromCurrentState();
}

void RollingNavigator::rosPublishProcess()
{
  groundModeProcess();
}

void RollingNavigator::setRotationControlLink(std::string link_name)
{
  /* update controlled link's current rotation from cog_R_base */
  const auto& seg_tf_map = robot_model_->getSegmentsTf();
  KDL::Rotation curr_cog_R_base; tf::quaternionTFToKDL(curr_target_baselink_quat_, curr_cog_R_base);
  KDL::Rotation final_cog_R_base; tf::quaternionTFToKDL(final_target_baselink_quat_, final_cog_R_base);

  KDL::Frame base_f_link =  (seg_tf_map.at(robot_model_->getBaselinkName())).Inverse() * seg_tf_map.at(link_name);
  curr_target_cog_R_link_ = curr_cog_R_base * base_f_link.M;
  final_target_cog_R_link_ = final_cog_R_base * base_f_link.M;

  if(rotation_control_link_name_ != link_name)
    ROS_INFO_STREAM("[navigation] set rotation control link from "<< rotation_control_link_name_ << " to " << link_name);

  rotation_control_link_name_ = link_name;
}

void RollingNavigator::baselinkRotationProcess()
{
  const auto& seg_tf_map = robot_model_->getSegmentsTf();
  if(rotation_control_link_name_ == robot_model_->getBaselinkName())
    {
      tf::quaternionTFToKDL(curr_target_baselink_quat_, curr_target_cog_R_link_);
      tf::quaternionTFToKDL(final_target_baselink_quat_, final_target_cog_R_link_);
    }
  else
    {
      KDL::Rotation link_R_base = (seg_tf_map.at(rotation_control_link_name_).Inverse() * seg_tf_map.at(robot_model_->getBaselinkName())).M;
      tf::quaternionKDLToTF(curr_target_cog_R_link_ * link_R_base, curr_target_baselink_quat_);
      tf::quaternionKDLToTF(final_target_cog_R_link_ * link_R_base, final_target_baselink_quat_);
      curr_target_baselink_quat_.normalize(); // important
      final_target_baselink_quat_.normalize(); // important
    }

  /* interpolation between final target and current command of cog_R_base */
  if(ros::Time::now().toSec() - prev_rotation_stamp_ > baselink_rot_pub_interval_)
    {
      tf::Quaternion delta_q = curr_target_baselink_quat_.inverse() * final_target_baselink_quat_;
      double angle = delta_q.getAngle();
      if (angle > M_PI) angle -= 2 * M_PI;

      if(fabs(angle) > baselink_rot_change_thresh_)
        {
          curr_target_baselink_quat_ *= tf::Quaternion(delta_q.getAxis(), fabs(angle) / angle * baselink_rot_change_thresh_);
        }
      else
        curr_target_baselink_quat_ = final_target_baselink_quat_;

      KDL::Rotation rot;
      tf::quaternionTFToKDL(curr_target_baselink_quat_, rot);

      /* set to robot model */
      robot_model_->setCogDesireOrientation(rot);

      /* send to spinal */
      geometry_msgs::Quaternion msg;
      msg.x = curr_target_baselink_quat_.x();
      msg.y = curr_target_baselink_quat_.y();
      msg.z = curr_target_baselink_quat_.z();
      msg.w = curr_target_baselink_quat_.w();
      desire_coord_pub_.publish(msg);

      prev_rotation_stamp_ = ros::Time::now().toSec();
    }

  /* update rotation matrix of controlling frame */
  if(rotation_control_link_name_ != robot_model_->getBaselinkName())
    {
      KDL::Rotation cog_R_base; tf::quaternionTFToKDL(curr_target_baselink_quat_, cog_R_base);
      curr_target_cog_R_link_ = cog_R_base * seg_tf_map.at(robot_model_->getBaselinkName()).Inverse().M * seg_tf_map.at(rotation_control_link_name_).M;
    }
}

void RollingNavigator::groundModeProcess()
{
  std_msgs::Int16 msg;
  msg.data = current_ground_navigation_mode_;
  ground_navigation_mode_pub_.publish(msg);

  msg.data = motion_mode_;
  ground_motion_mode_pub_.publish(msg);
}

void RollingNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();

  ros::NodeHandle navi_nh(nh_, "navigation");

  getParam<double>(navi_nh, "baselink_rot_change_thresh", baselink_rot_change_thresh_, 0.02);  // the threshold to change the baselink rotation
  getParam<double>(navi_nh, "baselink_rot_pub_interval", baselink_rot_pub_interval_, 0.1); // the rate to pub baselink rotation command
  getParam<double>(navi_nh, "joint_angvel", joint_angvel_, 1.0);
  getParam<double>(navi_nh, "joy_stick_deadzone", joy_stick_deadzone_, 0.2);
  getParam<double>(navi_nh, "rolling_max_pitch_ang_vel", rolling_max_pitch_ang_vel_, 0.0);
  getParam<double>(navi_nh, "rolling_max_yaw_ang_vel", rolling_max_yaw_ang_vel_, 0.0);
  getParam<double>(navi_nh, "down_mode_roll_angvel", down_mode_roll_anglvel_, 0.0);
  getParam<double>(navi_nh, "standing_baselink_roll_converged_thresh", standing_baselink_roll_converged_thresh_, 0.0);
  getParam<double>(navi_nh, "rolling_pitch_update_thresh", rolling_pitch_update_thresh_, 0.0);
  getParam<double>(navi_nh, "ground_trajectory_duration", ground_trajectory_duration_, 0.0);
  getParam<std::string>(nhp_, "tf_prefix", tf_prefix_, std::string(""));
}

void RollingNavigator::groundNavigationModeCallback(const std_msgs::Int16Ptr & msg)
{
  ROS_INFO_STREAM("[navigation] ground navigation command callback: switch mode to " << indexToGroundNavigationModeString(msg->data));
  setGroundNavigationMode(msg->data);
}

void RollingNavigator::groundMotionModeCallback(const std_msgs::Int16Ptr & msg)
{
  ROS_INFO_STREAM("[navigation] ground motion mode callback: switch mode to " << indexToGroundMotionModeString(msg->data));
  setGroundMotionMode(msg->data);
}

void RollingNavigator::setFinalTargetBaselinkQuatCallback(const geometry_msgs::QuaternionStampedConstPtr & msg)
{
  tf::Quaternion final_target_baselink_quat;
  tf::quaternionMsgToTF(msg->quaternion, final_target_baselink_quat);
}

void RollingNavigator::setFinalTargetBaselinkRpyCallback(const geometry_msgs::Vector3StampedConstPtr & msg)
{
  ROS_WARN_STREAM("[navigation] baselink rotation callback. RPY: " << msg->vector.x << " " << msg->vector.y << " " << msg->vector.z);
  final_target_baselink_quat_.setRPY(msg->vector.x, msg->vector.y, msg->vector.z);
}

void RollingNavigator::joyCallback(const sensor_msgs::JoyConstPtr & joy_msg)
{
  sensor_msgs::Joy joy_cmd = (*joy_msg);

  /* change to locomotion mode */
  if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_2] && motion_mode_ != aerial_robot_navigation::LOCOMOTION_MODE)
    {
      ROS_INFO_STREAM("[joy] change to " << indexToGroundMotionModeString(aerial_robot_navigation::LOCOMOTION_MODE));
      setGroundMotionMode(aerial_robot_navigation::LOCOMOTION_MODE);
    }

  /* change to manipulation mode when rolling state */
  if(joy_cmd.buttons[PS4_BUTTON_REAR_RIGHT_2] && motion_mode_ != aerial_robot_navigation::MANIPULATION_MODE)
    {
      if(current_ground_navigation_mode_ == aerial_robot_navigation::ROLLING_STATE)
        {
          ROS_INFO_STREAM("[joy] change to " << indexToGroundMotionModeString(aerial_robot_navigation::MANIPULATION_MODE));
          setGroundMotionMode(aerial_robot_navigation::MANIPULATION_MODE);
        }
      else
        {
          ROS_WARN_STREAM_THROTTLE(0.5, "[joy] do not change " << indexToGroundMotionModeString(aerial_robot_navigation::MANIPULATION_MODE) << " because current ground navigation mode is not " << indexToGroundNavigationModeString(aerial_robot_navigation::ROLLING_STATE));
        }
    }

  locomotionJoyCallback(joy_msg);
  transformJoyCallback(joy_msg);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::RollingNavigator, aerial_robot_navigation::BaseNavigator);
