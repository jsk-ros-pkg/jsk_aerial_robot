#include <dragon/dragon_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

DragonNavigator::DragonNavigator():
  BaseNavigator(),
  servo_torque_(false),
  level_flag_(false),
  landing_flag_(false),
  curr_target_baselink_rot_(0, 0, 0),
  final_target_baselink_rot_(0, 0, 0)
{
}

void DragonNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                 boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                 double loop_du)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  curr_target_baselink_rot_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);
  joint_control_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  final_target_baselink_rot_sub_ = nh_.subscribe("final_target_baselink_rot", 1, &DragonNavigator::setFinalTargetBaselinkRotCallback, this);

  prev_rotation_stamp_ = ros::Time::now().toSec();
}

void DragonNavigator::update()
{
  BaseNavigator::update();

  baselinkRotationProcess();

  landingProcess();
  // servoTorqueProcess();

}

void DragonNavigator::setFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg)
{
  final_target_baselink_rot_.setValue(msg->roll, msg->pitch, msg->yaw);
}

void DragonNavigator::baselinkRotationProcess()
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

void DragonNavigator::landingProcess()
{
  if(getForceLandingFlag() || getNaviState() == LAND_STATE)
    {
      if(!level_flag_)
        {
          const auto joint_state = robot_model_->kdlJointToMsg(robot_model_->getJointPositions());
          sensor_msgs::JointState joint_control_msg;
          for(int i = 0; i < joint_state.position.size(); i++)
            {
              if(joint_state.name[i].find("joint") != std::string::npos)
                {
                  double target_cmd;
                  if(joint_state.name[i].find("pitch") != std::string::npos)
                    target_cmd = 0;
                  else target_cmd = joint_state.position[i];

                  joint_control_msg.position.push_back(target_cmd);

                }
            }


          joint_control_pub_.publish(joint_control_msg);
          final_target_baselink_rot_.setValue(0, 0, 0);

          double curr_roll = estimator_->getState(State::ROLL_BASE, estimate_mode_)[0];
          double curr_pitch = estimator_->getState(State::PITCH_BASE, estimate_mode_)[0];

          if(fabs(fabs(curr_pitch) - M_PI/2) < 0.05 && fabs(curr_roll) > M_PI/2) // singularity of XYZ Euler
            curr_roll = 0;

          /* force set the current deisre tilt to current estimated tilt */
          curr_target_baselink_rot_.setValue(curr_roll, curr_pitch, 0);
        }

      level_flag_ = true;

      if(getNaviState() == LAND_STATE && !landing_flag_)
        {
          landing_flag_ = true;
          setTeleopFlag(false);
          setTargetPosZ(estimator_->getState(State::Z_COG, estimate_mode_)[0]);
          setTargetVelZ(0);
          land_height_ = 0; // reset the land height, since it is updated in the first land_state which is forced to change to hover state to level the orientation. Thus, it is possible to have the same land height just after switching back to land state and thus stop in midair
          setNaviState(HOVER_STATE);
        }
    }

  /* back to landing process */
  if(landing_flag_)
    {
      const auto joint_state = robot_model_->kdlJointToMsg(robot_model_->getJointPositions());
      bool already_level = true;
      for(int i = 0; i < joint_state.position.size(); i++)
        {
          if(joint_state.name[i].find("joint") != std::string::npos
             && joint_state.name[i].find("pitch") != std::string::npos)
            {
              if(fabs(joint_state.position[i]) > 0.085) already_level = false;
            }
        }

      if(curr_target_baselink_rot_.length()) already_level = false;

      if(already_level && getNaviState() == HOVER_STATE)
        {
          ROS_WARN("gimbal control: back to land state");
          setNaviState(LAND_STATE);
          setTeleopFlag(true);
        }
    }
}

void DragonNavigator::servoTorqueProcess()
{
  ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>(joints_torque_control_srv_name_);
  std_srvs::SetBool srv;

  if(servo_torque_)
    {
      if(getNaviState() == LAND_STATE)
        {
          if(estimator_->getState(State::Z_COG, estimate_mode_)[0] < height_thresh_)
            {

              srv.request.data = false;

              if (client.call(srv))
                {
                  ROS_INFO("dragon control: disable the joint torque");
                  servo_torque_ = false;
                }
              else
                {
                  ROS_ERROR("Failed to call service %s", joints_torque_control_srv_name_.c_str());
                }
            }
        }
    }
  else
    {
      if(getNaviState() == ARM_ON_STATE)
        {
          srv.request.data = true;

          if (client.call(srv))
            {
              ROS_INFO("dragon control: enable the joint torque");
              servo_torque_ = true;
            }
          else
            {
              ROS_ERROR("Failed to call service %s", joints_torque_control_srv_name_.c_str());
            }
        }
    }
}

void DragonNavigator::reset()
{
  BaseNavigator::reset();

  level_flag_ = false;
  landing_flag_ = false;
}


void DragonNavigator::halt()
{
  ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>(joints_torque_control_srv_name_);
  std_srvs::SetBool srv;
  srv.request.data = false;
  if (client.call(srv))
    ROS_INFO("dragon control halt process: disable the joint torque");
  else
    ROS_ERROR("Failed to call service %s", joints_torque_control_srv_name_.c_str());

  client = nh_.serviceClient<std_srvs::SetBool>(gimbals_torque_control_srv_name_);

  srv.request.data = false;
  if (client.call(srv))
    ROS_INFO("dragon control halt process: disable the gimbal torque");
  else
    ROS_ERROR("Failed to call service %s", gimbals_torque_control_srv_name_.c_str());
}

void DragonNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();

  ros::NodeHandle navi_nh(nh_, "navigation");

  getParam<std::string>(navi_nh, "joints_torque_control_srv_name", joints_torque_control_srv_name_, std::string("joints/torque_enable"));
  getParam<std::string>(navi_nh, "gimbals_torque_control_srv_name", gimbals_torque_control_srv_name_, std::string("gimbals/torque_enable"));
  getParam<double>(navi_nh, "height_thresh", height_thresh_, 0.1); // height threshold to disable the joint servo when landing
  getParam<double>(navi_nh, "baselink_rot_change_thresh", baselink_rot_change_thresh_, 0.02);  // the threshold to change the baselink rotation
  getParam<double>(navi_nh, "baselink_rot_pub_interval", baselink_rot_pub_interval_, 0.1); // the rate to pub baselink rotation command
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::DragonNavigator, aerial_robot_navigation::BaseNavigator);
