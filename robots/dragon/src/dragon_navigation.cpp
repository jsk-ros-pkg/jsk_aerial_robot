#include <dragon/dragon_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

DragonNavigator::DragonNavigator():
  BaseNavigator(),
  servo_torque_(false),
  level_flag_(false),
  eq_cog_world_(false)
{
  curr_target_baselink_rot_.setRPY(0, 0, 0);
  final_target_baselink_rot_.setRPY(0, 0, 0);
}

void DragonNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                 boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                 double loop_du)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  target_baselink_rpy_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1); // to spinal
  joint_control_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  flight_nav_pub_ = nh_.advertise<aerial_robot_msgs::FlightNav>("uav/nav", 1);
  target_rotation_motion_pub_ = nh_.advertise<nav_msgs::Odometry>("target_rotation_motion", 1);
  final_target_baselink_rot_sub_ = nh_.subscribe("final_target_baselink_rot", 1, &DragonNavigator::targetBaselinkRotCallback, this);
  final_target_baselink_rpy_sub_ = nh_.subscribe("final_target_baselink_rpy", 1, &DragonNavigator::targetBaselinkRPYCallback, this);
  target_rotation_motion_sub_ = nh_.subscribe("target_rotation_motion", 1, &DragonNavigator::targetRotationMotionCallback, this);
  full_state_target_sub_ = nh_.subscribe("full_state_target", 1, &DragonNavigator::fullStateTargetCallback, this);

  prev_rotation_stamp_ = ros::Time::now().toSec();

  // Initialize link joint information
  auto transformable_model = boost::dynamic_pointer_cast<aerial_robot_model::transformable::RobotModel>(robot_model_);
  if (transformable_model)
  {
    link_joint_indices_ = transformable_model->getLinkJointIndices();
    link_joint_num_ = link_joint_indices_.size();
    ROS_INFO("[DragonNavigator] Link joint indices initialized: %d joints", link_joint_num_);
  }
  else
  {
    link_joint_num_ = 0;
    ROS_WARN("[DragonNavigator] Failed to cast to transformable robot model");
  }
}

void DragonNavigator::update()
{
  BaseNavigator::update();

  baselinkRotationProcess();

  landingProcess();
  // servoTorqueProcess();

}

void DragonNavigator::targetBaselinkRotCallback(const geometry_msgs::QuaternionStampedConstPtr & msg)
{
  tf::quaternionMsgToTF(msg->quaternion, final_target_baselink_rot_);
  target_omega_.setValue(0,0,0); // for sure to reset the target angular velocity

  // special process
  if(getTargetRPY().z() != 0)
    {
      curr_target_baselink_rot_.setRPY(0, 0, getTargetRPY().z());
      eq_cog_world_ = true;
    }
}

void DragonNavigator::targetBaselinkRPYCallback(const geometry_msgs::Vector3StampedConstPtr & msg)
{
  final_target_baselink_rot_.setRPY(msg->vector.x, msg->vector.y, msg->vector.z);
  target_omega_.setValue(0,0,0); // for sure to reset the target angular velocity
}


void DragonNavigator::targetRotationMotionCallback(const nav_msgs::OdometryConstPtr& msg)
{
  std::string frame = msg->header.frame_id;

  if(frame != std::string("cog") && frame != std::string("baselink"))
    {
      ROS_ERROR("frame %s is not support in target rotation motion",frame.c_str());
      return;
    }

  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);

  tf::Vector3 w;
  tf::vector3MsgToTF(msg->twist.twist.angular, w);

  // special process for current special definition of tranformation between CoG and Baselink
  if(frame == std::string("baselink"))
    {
      // and directly send to aerial model
      KDL::Rotation rot;
      tf::quaternionMsgToKDL(msg->pose.pose.orientation, rot);

      double qx,qy,qz,qw;
      rot.GetQuaternion(qx,qy,qz,qw);
      robot_model_->setCogDesireOrientation(rot);
      tf::quaternionMsgToTF(msg->pose.pose.orientation, curr_target_baselink_rot_);
      final_target_baselink_rot_ = curr_target_baselink_rot_;

      // convert to target CoG frame from Baselink frame
      eq_cog_world_ = true;
      setTargetOmega(tf::Matrix3x3(q) * w);

      // send to spinal
      spinal::DesireCoord msg;
      double r,p,y;
      rot.GetRPY(r, p, y);
      msg.roll = r;
      msg.pitch = p;
      msg.yaw = y;
      target_baselink_rpy_pub_.publish(msg);
    }
  else
    { // CoG frame

      // This should be the standard process to receive desired rotation motion (orientation + angular velocity)
      double r,p,y; tf::Matrix3x3(q).getRPY(r, p, y);
      setTargetRPY(tf::Vector3(r, p, y));
      setTargetOmega(w);
    }
}

void DragonNavigator::baselinkRotationProcess()
{
  if(curr_target_baselink_rot_ == final_target_baselink_rot_) return;

  if(ros::Time::now().toSec() - prev_rotation_stamp_ > baselink_rot_pub_interval_)
    {
      tf::Quaternion delta_q = curr_target_baselink_rot_.inverse() * final_target_baselink_rot_;
      double angle = delta_q.getAngle();
      if (angle > M_PI) angle -= 2 * M_PI;

      if(fabs(angle) > baselink_rot_change_thresh_)
        {
          curr_target_baselink_rot_ *= tf::Quaternion(delta_q.getAxis(), fabs(angle) / angle * baselink_rot_change_thresh_);
        }
      else
        curr_target_baselink_rot_ = final_target_baselink_rot_;

      KDL::Rotation rot;
      tf::quaternionTFToKDL(curr_target_baselink_rot_, rot);
      robot_model_->setCogDesireOrientation(rot);

      // send to spinal
      spinal::DesireCoord msg;
      double r,p,y;
      tf::Matrix3x3(curr_target_baselink_rot_).getRPY(r, p, y);
      msg.roll = r;
      msg.pitch = p;
      msg.yaw = y;
      target_baselink_rpy_pub_.publish(msg);

      prev_rotation_stamp_ = ros::Time::now().toSec();
    }
}

void DragonNavigator::landingProcess()
{
  if(getForceLandingFlag() || getNaviState() == LAND_STATE)
    {
      target_omega_.setValue(0,0,0); // for sure to  target angular velocity is zero

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

          if (!eq_cog_world_)
            {
              final_target_baselink_rot_.setRPY(0, 0, 0);

              tf::Vector3 base_euler = estimator_->getEuler(Frame::BASELINK, estimate_mode_);

              if(fabs(fabs(base_euler.y()) - M_PI/2) > 0.2)
                {
                  /* force set the current deisre tilt to current estimated tilt */
                  curr_target_baselink_rot_.setRPY(base_euler.x(), base_euler.y(), 0); // case1
                }
              else
                { // singularity of XYZ Euler

                  double r,p,y;
                  tf::Matrix3x3(curr_target_baselink_rot_).getRPY(r,p,y);
                  if(fabs(fabs(p) - M_PI/2) > 0.2)
                    curr_target_baselink_rot_.setRPY(0, base_euler.y(), 0); // case2

                  // case3: use the last curr_target_baselink_rot_
                }
            }
          else
            {
              tf::Quaternion base_rot;
              estimator_->getOrientation(Frame::BASELINK, estimate_mode_).getRotation(base_rot);

              double delta_angle = (curr_target_baselink_rot_.inverse() * base_rot).getAngle();
              if (delta_angle > M_PI) delta_angle -= 2 * M_PI;
              if(fabs(delta_angle) > 0.2)
                {
                  curr_target_baselink_rot_ = base_rot;
                }


              tf::Vector3 cross_v =  tf::Vector3(0,0,1).cross(tf::Matrix3x3(curr_target_baselink_rot_.inverse()) * tf::Vector3(0,0,1));
              if(cross_v.length() < 0.001) // sine of delta_angle to world z frame
                {
                  final_target_baselink_rot_ = curr_target_baselink_rot_;
                }
              else
                {
                  // IMPORTANT: to avoid NaN because of asin(angle) when angle > 1, thus use tfAsin which has clamping process before do asin
                  tf::Quaternion delta_q(cross_v, tfAsin(cross_v.length()));
                  final_target_baselink_rot_ = curr_target_baselink_rot_ * delta_q;
                  // Note: normalize quaterinon just in case that curr_target_baselink_rot is not a perfect quaternion (e.g., manual rostopic pub)
                  double r,p,y;
                  tf::Matrix3x3(final_target_baselink_rot_).getRPY(r,p,y);
                  final_target_baselink_rot_.setRPY(0, 0, y);
                  ROS_DEBUG("refine final_target_baselink_rot_: [%f, %f, %f, %f]", final_target_baselink_rot_.x(), final_target_baselink_rot_.y(),
                            final_target_baselink_rot_.z(), final_target_baselink_rot_.w());
                }
            }

          if(getNaviState() == LAND_STATE)
            {
              setNaviState(PRE_LAND_STATE);
              setTeleopFlag(false);
              setTargetPosZ(estimator_->getState(State::Z_COG, estimate_mode_)[0]);
              setTargetVelZ(0);
              land_height_ = 0; // reset the land height, since it is updated in the first land_state which is forced to change to hover state to level the orientation. Thus, it is possible to have the same land height just after switching back to land state and thus stop in midair
              ROS_INFO("[Navigation] shift to pre_land state to make the robot level");
            }

          level_flag_ = true;
        }
    }

  /* back to landing process */
  if(getNaviState() == PRE_LAND_STATE)
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

      double r,p,y;
      tf::Matrix3x3(curr_target_baselink_rot_).getRPY(r,p,y);
      if(fabs(r) > 0.01 || fabs(p) > 0.01) already_level = false;

      if(already_level)
        {
          ROS_INFO("[Navigation] back to land state");
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

  // reset SO3
  eq_cog_world_ = false;
  curr_target_baselink_rot_.setRPY(0, 0, 0);
  final_target_baselink_rot_.setRPY(0, 0, 0);
  KDL::Rotation rot;
  tf::quaternionTFToKDL(curr_target_baselink_rot_, rot);
  robot_model_->setCogDesireOrientation(rot);
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

void DragonNavigator::fullStateTargetCallback(const aerial_robot_msgs::FullStateTargetConstPtr& msg)
{
  // Validate joint state size
  if (link_joint_num_ > 0 && msg->joint_state.position.size() != link_joint_num_)
  {
    ROS_ERROR("[DragonNavigator] Joint state size mismatch: expected %d, got %zu", link_joint_num_,
              msg->joint_state.position.size());
    return;
  }

  KDL::JntArray current_joint_positions = robot_model_->getJointPositions();

  KDL::JntArray updated_joint_positions = current_joint_positions;
  for (size_t i = 0; i < link_joint_num_; ++i)
  {
    int joint_index = link_joint_indices_[i];
    updated_joint_positions(joint_index) = msg->joint_state.position[i];
  }

  // Use mutex to protect temporary robot model update and restore
  // This ensures thread-safety if other callbacks or update() access robot_model_ simultaneously
  KDL::Frame cog_frame;
  {
    std::lock_guard<std::mutex> lock(cog_calculation_mutex_);
    robot_model_->updateRobotModel(updated_joint_positions);
    cog_frame = robot_model_->getCog<KDL::Frame>();
    robot_model_->updateRobotModel(current_joint_positions);
  }

  tf::Vector3 root_pos;
  tf::pointMsgToTF(msg->root_state.pose.pose.position, root_pos);
  tf::Quaternion root_rot;
  tf::quaternionMsgToTF(msg->root_state.pose.pose.orientation, root_rot);

  tf::Vector3 root_vel;
  tf::vector3MsgToTF(msg->root_state.twist.twist.linear, root_vel);
  tf::Vector3 root_omega;
  tf::vector3MsgToTF(msg->root_state.twist.twist.angular, root_omega);

  KDL::Frame root_frame_kdl;
  root_frame_kdl.p = KDL::Vector(root_pos.x(), root_pos.y(), root_pos.z());
  root_frame_kdl.M = KDL::Rotation::Quaternion(root_rot.x(), root_rot.y(), root_rot.z(), root_rot.w());

  KDL::Frame world_cog_frame = root_frame_kdl * cog_frame;

  tf::Vector3 cog_pos(world_cog_frame.p.x(), world_cog_frame.p.y(), world_cog_frame.p.z());

  double qx, qy, qz, qw;
  world_cog_frame.M.GetQuaternion(qx, qy, qz, qw);
  tf::Quaternion cog_rot(qx, qy, qz, qw);

  // v_cog = v_root + omega_root × (p_cog - p_root)
  tf::Vector3 cog_rel_pos = cog_pos - root_pos;
  tf::Vector3 cog_vel = root_vel + root_omega.cross(cog_rel_pos);
  tf::Vector3 cog_omega = root_omega;

  double r, p, y;
  tf::Matrix3x3(cog_rot).getRPY(r, p, y);

  // CoG position + CoG orientation + joint control commands fully define the full state target
  // CoG position
  aerial_robot_msgs::FlightNav cog_pos_msg;
  cog_pos_msg.header.stamp = ros::Time::now();
  cog_pos_msg.header.frame_id = "world";
  cog_pos_msg.target = aerial_robot_msgs::FlightNav::COG;
  cog_pos_msg.control_frame = aerial_robot_msgs::FlightNav::WORLD_FRAME;
  cog_pos_msg.pos_xy_nav_mode = aerial_robot_msgs::FlightNav::POS_VEL_MODE;
  cog_pos_msg.pos_z_nav_mode = aerial_robot_msgs::FlightNav::POS_VEL_MODE;
  cog_pos_msg.yaw_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION; // Yaw is controlled by target_rotation_motion  

  cog_pos_msg.target_pos_x = cog_pos.x();
  cog_pos_msg.target_pos_y = cog_pos.y();
  cog_pos_msg.target_pos_z = cog_pos.z();

  cog_pos_msg.target_vel_x = cog_vel.x();
  cog_pos_msg.target_vel_y = cog_vel.y();
  cog_pos_msg.target_vel_z = cog_vel.z();

  flight_nav_pub_.publish(cog_pos_msg);

  // CoG orientation
  nav_msgs::Odometry cog_rotation_msg;
  cog_rotation_msg.header.stamp = ros::Time::now();
  cog_rotation_msg.header.frame_id = "cog"; // Using CoG frame
  
  cog_rotation_msg.pose.pose.orientation.x = cog_rot.x();
  cog_rotation_msg.pose.pose.orientation.y = cog_rot.y();
  cog_rotation_msg.pose.pose.orientation.z = cog_rot.z();
  cog_rotation_msg.pose.pose.orientation.w = cog_rot.w();
  
  cog_rotation_msg.twist.twist.angular.x = cog_omega.x();
  cog_rotation_msg.twist.twist.angular.y = cog_omega.y();
  cog_rotation_msg.twist.twist.angular.z = cog_omega.z();
  
  target_rotation_motion_pub_.publish(cog_rotation_msg);

  // Publish joint control commands
  sensor_msgs::JointState joint_control_msg;
  joint_control_msg.position = msg->joint_state.position;
  joint_control_pub_.publish(joint_control_msg);
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
