/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Moju Zhao
   Desc:   Hardware Interface for aerial robot(JSK) in Gazebo
*/


#include <aerial_robot_simulation/aerial_robot_hw_sim.h>
#include <urdf/model.h>


namespace
{

double clamp(const double val, const double min_val, const double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

}

namespace gazebo_ros_control
{


bool AerialRobotHWSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
  // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
  const ros::NodeHandle joint_limit_nh(model_nh);

  // Resize vectors to our DOF
  n_dof_ = transmissions.size();
  joint_names_.resize(n_dof_);
  joint_types_.resize(n_dof_);
  joint_lower_limits_.resize(n_dof_);
  joint_upper_limits_.resize(n_dof_);
  joint_effort_limits_.resize(n_dof_);
  joint_control_methods_.resize(n_dof_);
  pid_controllers_.resize(n_dof_);
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);
  joint_position_command_.resize(n_dof_);
  joint_velocity_command_.resize(n_dof_);

  parent_model_ = parent_model;

  // Initialize values
  for(unsigned int j=0; j < n_dof_; j++)
  {
    // Check that this transmission has one joint
    if(transmissions[j].joints_.size() == 0)
    {
      ROS_WARN_STREAM_NAMED("aerial_robot_hw_sim","Transmission " << transmissions[j].name_
        << " has no associated joints.");
      continue;
    }
    else if(transmissions[j].joints_.size() > 1)
    {
      ROS_WARN_STREAM_NAMED("aerial_robot_hw_sim","Transmission " << transmissions[j].name_
        << " has more than one joint. Currently the aerial robot hardware simulation "
        << " interface only supports one.");
      continue;
    }

    std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
    if (joint_interfaces.empty() &&
        !(transmissions[j].actuators_.empty()) &&
        !(transmissions[j].actuators_[0].hardware_interfaces_.empty()))
    {
      // TODO: Deprecate HW interface specification in actuators in ROS J
      joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
      ROS_WARN_STREAM_NAMED("aerial_robot_hw_sim", "The <hardware_interface> element of tranmission " <<
        transmissions[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
        "The transmission will be properly loaded, but please update " <<
        "your robot model to remain compatible with future versions of the plugin.");
    }
    if (joint_interfaces.empty())
    {
      ROS_WARN_STREAM_NAMED("aerial_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
        " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
        "Not adding it to the robot hardware simulation.");
      continue;
    }
    else if (joint_interfaces.size() > 1)
    {
      ROS_WARN_STREAM_NAMED("aerial_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
        " of transmission " << transmissions[j].name_ << " specifies multiple hardware interfaces. " <<
        "Currently the aerial robot hardware simulation interface only supports one. Using the first entry!");
      //continue;
    }

    // Add data from transmission
    joint_names_[j] = transmissions[j].joints_[0].name_;
    joint_position_[j] = 1.0;
    joint_velocity_[j] = 0.0;
    joint_effort_[j] = 1.0;  // N/m for continuous joints
    joint_effort_command_[j] = 0.0;
    joint_position_command_[j] = 0.0;
    joint_velocity_command_[j] = 0.0;

    const std::string& hardware_interface = joint_interfaces.front();

    // Debug
    std::cout << "Loading joint '" << joint_names_[j]
              << "' of type '" << hardware_interface << "'" << std::endl;

    /* Aerial Robot */
    /* create joint state only for general joint interface */
    if(hardware_interface.find("JointInterface") != std::string::npos)
      {
        js_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[j],
                                                                          &joint_position_[j],
                                                                          &joint_velocity_[j],
                                                                          &joint_effort_[j]));
      }

    // Decide what kind of command interface this actuator/joint has
    hardware_interface::JointHandle joint_handle;
    /* Aerial Robot */
    /* interface for aerial robot */
    hardware_interface::RotorHandle rotor_handle;
    if(hardware_interface == "EffortJointInterface" || hardware_interface == "hardware_interface/EffortJointInterface")
      {
        // Create effort joint interface
        joint_control_methods_[j] = EFFORT;
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                       &joint_effort_command_[j]);
        ej_interface_.registerHandle(joint_handle);
      }
    else if(hardware_interface == "PositionJointInterface" || hardware_interface == "hardware_interface/PositionJointInterface")
      {
        // Create position joint interface
        joint_control_methods_[j] = POSITION;
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                       &joint_position_command_[j]);
        pj_interface_.registerHandle(joint_handle);
      }
    else if(hardware_interface == "VelocityJointInterface" || hardware_interface == "hardware_interface/VelocityJointInterface")
      {
        // Create velocity joint interface
        joint_control_methods_[j] = VELOCITY;
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                       &joint_velocity_command_[j]);
        vj_interface_.registerHandle(joint_handle);
      }
    else if(hardware_interface == "RotorInterface")
      {    /* rotor interface */
        //TODO
        joint_control_methods_[j] = ROTOR;
        rotor_handle = hardware_interface::RotorHandle(model_nh, joint_names_[j]);
        rotor_interface_.registerHandle(rotor_handle);
      }
    else
      {
        ROS_FATAL_STREAM_NAMED("aerial_robot_hw_sim","No matching hardware interface found for '"
                               << hardware_interface << "' while loading interfaces for " << joint_names_[j] );
        return false;
      }

    if(hardware_interface == "EffortJointInterface" || hardware_interface == "PositionJointInterface" || hardware_interface == "VelocityJointInterface") {
      ROS_WARN_STREAM("Deprecated syntax, please prepend 'hardware_interface/' to '" << hardware_interface << "' within the <hardwareInterface> tag in joint '" << joint_names_[j] << "'.");
    }

    // Get the gazebo joint that corresponds to the robot joint.
    gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
    if (!joint)
    {
      ROS_ERROR_STREAM_NAMED("aerial_robot_hw", "This robot has a joint named \"" << joint_names_[j]
        << "\" which is not in the gazebo model.");
      return false;
    }
    sim_joints_.push_back(joint);

    /* Aerial Robot */
    /* TODO: rotor limit interface */
    if(hardware_interface == "RotorInterface")
      {
        const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(joint_names_[j]);
        const rotor_limits_interface::EffortRotorSaturationHandle limits_handle(rotor_handle, urdf_joint);
        er_sat_interface_.registerHandle(limits_handle);
      }
    else /* general joint limit interface */
      registerJointLimits(joint_names_[j], joint_handle, joint_control_methods_[j],
                          joint_limit_nh, urdf_model,
                          &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
                          &joint_effort_limits_[j]);

    if (joint_control_methods_[j] != EFFORT)
    {
      // Initialize the PID controller. If no PID gain values are found, use joint->SetAngle() or
      // joint->SetParam("vel") to control the joint.
      const ros::NodeHandle nh(model_nh, "/gazebo_ros_control/pid_gains/" +
                               joint_names_[j]);
      if (pid_controllers_[j].init(nh, true))
      {
        switch (joint_control_methods_[j])
        {
          case POSITION:
            joint_control_methods_[j] = POSITION_PID;
            break;
          case VELOCITY:
            joint_control_methods_[j] = VELOCITY_PID;
            break;
        }
      }
      else
      {
        // joint->SetParam("fmax") must be called if joint->SetAngle() or joint->SetParam("vel") are
        // going to be called. joint->SetParam("fmax") must *not* be called if joint->SetForce() is
        // going to be called.
#if GAZEBO_MAJOR_VERSION > 2
        joint->SetParam("fmax", 0, joint_effort_limits_[j]);
#else
        joint->SetMaxForce(0, joint_effort_limits_[j]);
#endif
      }
    }
  }

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);

  /* Aerial Robot */
  /* Temporary: get the motor attribute from rosparam */
  /* We need a better location and method to do this */
  int motor_num = 0;
  if (!model_nh.getParam("/motor_info/motor_num", motor_num))
    ROS_ERROR("Cannot get motor_num from ros nodehandle: %s", model_nh.getNamespace().c_str());

  registerInterface(&rotor_interface_);
  rotor_interface_.setJointNum(js_interface_.getNames().size());
  if(rotor_interface_.getJointNum() > 0)
    {
      std::string root_link_name;
      if(model_nh.getParam("root_link_name", root_link_name))
        rotor_interface_.setRootLinkName(root_link_name);
      else
        ROS_FATAL("aerial_robot_hw_sim: can not get the root link name for transformable robot");

      if(motor_num > 1 && motor_num != rotor_interface_.getNames().size())
        ROS_FATAL("the motor number in urdf model and control model are different, control: %d, urdf: %d", motor_num, rotor_interface_.getNames().size());
    }
  flight_mode_ = CONTROL_MODE;
  cmd_vel_sub_ = model_nh.subscribe("/cmd_vel", 1, &AerialRobotHWSim::cmdVelCallback, this);
  cmd_pos_sub_ = model_nh.subscribe("/cmd_pos", 1, &AerialRobotHWSim::cmdPosCallback, this);
  model_nh.param("ground_truth_pub_rate", ground_truth_pub_rate_, 0.01); // [sec]
  //ground_truth_pub_ = model_nh.advertise<nav_msgs::Odometry>("ground_truth", 1);
  ground_truth_pub_ = model_nh.advertise<geometry_msgs::PoseStamped>("ground_truth", 1);

  // Initialize the emergency stop code.
  e_stop_active_ = false;
  last_e_stop_active_ = false;

  return true;
}

void AerialRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  static ros::Time last_time = time;
  for(unsigned int j=0; j < n_dof_; j++)
  {
    // Gazebo has an interesting API...
    if (joint_types_[j] == urdf::Joint::PRISMATIC)
    {
      joint_position_[j] = sim_joints_[j]->GetAngle(0).Radian();
    }
    else
    {
      joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
                            sim_joints_[j]->GetAngle(0).Radian());
    }
    joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
    joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
  }

  /* Aerial Robot */
  /* [Ground Truth] set the orientation of root link for attitude control */
  const gazebo::physics::LinkPtr root_link = parent_model_->GetLink(rotor_interface_.getRootLinkName());
  if(root_link != NULL)
    {
      if(!rotor_interface_.foundRootLink()) rotor_interface_.setRootLink();

      /* please check whether this is same with IMU process */
      /* orientation should calculate the rpy in world frame,
         and angular velocity should show the value in board(body) frame */
      /* set orientation and angular of root link */
      gazebo::math::Quaternion q = root_link->GetWorldPose().rot;
      rotor_interface_.setRootLinkOrientation(q.x, q.y, q.z, q.w);
      gazebo::math::Vector3 w = root_link->GetRelativeAngularVel();
      rotor_interface_.setRootLinkAngular(w.x, w.y, w.z);

      if(time.toSec() - last_time.toSec() > ground_truth_pub_rate_)
        {
#if 0 //odometry
          nav_msgs::Odometry pose_msg;
          pose_msg.header.stamp = time;
          pose_msg.pose.pose.position.x = root_link->GetWorldPose().pos.x;
          pose_msg.pose.pose.position.y = root_link->GetWorldPose().pos.y;
          pose_msg.pose.pose.position.z = root_link->GetWorldPose().pos.z;
          pose_msg.pose.pose.orientation.x = q.x;
          pose_msg.pose.pose.orientation.y = q.y;
          pose_msg.pose.pose.orientation.z = q.z;
          pose_msg.pose.pose.orientation.w = q.w;
          pose_msg.twist.twist.linear.x = root_link->GetWorldLinearVel().x;
          pose_msg.twist.twist.linear.y = root_link->GetWorldLinearVel().y;
          pose_msg.twist.twist.linear.z = root_link->GetWorldLinearVel().z;
#else
          geometry_msgs::PoseStamped pose_msg;
          pose_msg.header.stamp = time;
          pose_msg.pose.position.x = root_link->GetWorldPose().pos.x;
          pose_msg.pose.position.y = root_link->GetWorldPose().pos.y;
          pose_msg.pose.position.z = root_link->GetWorldPose().pos.z;
          pose_msg.pose.orientation.x = q.x;
          pose_msg.pose.orientation.y = q.y;
          pose_msg.pose.orientation.z = q.z;
          pose_msg.pose.orientation.w = q.w;
#endif
          ground_truth_pub_.publish(pose_msg);
          last_time = time;
        }
    }
  else
    {
      //ROS_WARN("the root link does not exist");
    }

}

void AerialRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
  // If the E-stop is active, joints controlled by position commands will maintain their positions.
  if (e_stop_active_)
  {
    if (!last_e_stop_active_)
    {
      last_joint_position_command_ = joint_position_;
      last_e_stop_active_ = true;
    }
    joint_position_command_ = last_joint_position_command_;
  }
  else
  {
    last_e_stop_active_ = false;
  }

  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);
  pj_sat_interface_.enforceLimits(period);
  pj_limits_interface_.enforceLimits(period);
  vj_sat_interface_.enforceLimits(period);
  vj_limits_interface_.enforceLimits(period);

  /* Aerial Robot */
  er_sat_interface_.enforceLimits(period);
  const gazebo::physics::LinkPtr root_link = parent_model_->GetLink(rotor_interface_.getRootLinkName());
  if(root_link != NULL)
    {

      if (flight_mode_ == CMD_VEL_MODE)
        {
          root_link->SetLinearVel(gazebo::math::Vector3(cmd_vel_.twist.linear.x,
                                                        cmd_vel_.twist.linear.y,
                                                        cmd_vel_.twist.linear.z));
          root_link->SetAngularVel(gazebo::math::Vector3(cmd_vel_.twist.angular.x,
                                                         cmd_vel_.twist.angular.y,
                                                         cmd_vel_.twist.angular.z));
          //flight_mode_ = CONTROL_MODE;
        }

      if (flight_mode_ == CMD_POS_MODE)
        {
          root_link->SetWorldPose(gazebo::math::Pose(
                                                       gazebo::math::Vector3(cmd_pos_.pose.position.x,
                                                                             cmd_pos_.pose.position.y,
                                                                             cmd_pos_.pose.position.z),
                                                       gazebo::math::Quaternion(cmd_pos_.pose.orientation.w,
                                                                                cmd_pos_.pose.orientation.x,
                                                                                cmd_pos_.pose.orientation.y,
                                                                                cmd_pos_.pose.orientation.z)));
          flight_mode_ = CONTROL_MODE;
        }
    }

  for(unsigned int j=0; j < n_dof_; j++)
  {
    switch (joint_control_methods_[j])
      {
      case EFFORT:
        {
          const double effort = e_stop_active_ ? 0 : joint_effort_command_[j];
          sim_joints_[j]->SetForce(0, effort);
        }
        break;

      case POSITION:
#if GAZEBO_MAJOR_VERSION >= 4
        sim_joints_[j]->SetPosition(0, joint_position_command_[j]);
#else
        sim_joints_[j]->SetAngle(0, joint_position_command_[j]);
#endif
        break;

      case POSITION_PID:
        {
          double error;
          switch (joint_types_[j])
            {
            case urdf::Joint::REVOLUTE:
              angles::shortest_angular_distance_with_limits(joint_position_[j],
                                                            joint_position_command_[j],
                                                            joint_lower_limits_[j],
                                                            joint_upper_limits_[j],
                                                            error);
              break;
            case urdf::Joint::CONTINUOUS:
              error = angles::shortest_angular_distance(joint_position_[j],
                                                        joint_position_command_[j]);
              break;
            default:
              error = joint_position_command_[j] - joint_position_[j];
            }

          const double effort_limit = joint_effort_limits_[j];
          const double effort = clamp(pid_controllers_[j].computeCommand(error, period),
                                      -effort_limit, effort_limit);
          sim_joints_[j]->SetForce(0, effort);
        }
        break;

      case VELOCITY:
#if GAZEBO_MAJOR_VERSION > 2
        sim_joints_[j]->SetParam("vel", 0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
#else
        sim_joints_[j]->SetVelocity(0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
#endif
        break;

      case VELOCITY_PID:
        {
        double error;
        if (e_stop_active_)
          error = -joint_velocity_[j];
        else
          error = joint_velocity_command_[j] - joint_velocity_[j];
        const double effort_limit = joint_effort_limits_[j];
        const double effort = clamp(pid_controllers_[j].computeCommand(error, period),
                                    -effort_limit, effort_limit);
        sim_joints_[j]->SetForce(0, effort);
        }
        break;

      case ROTOR:
        {
          /* Aerial Robot */
          /* implement as rotor simulators: force + torque */
          if (flight_mode_ ==  CONTROL_MODE)
            {
              hardware_interface::RotorHandle rotor = rotor_interface_.getHandle(joint_names_[j]);
              gazebo::physics::LinkPtr parent_link  = sim_joints_[j]->GetParent();
              gazebo::physics::LinkPtr child_link  = sim_joints_[j]->GetChild();

              //std::cout << child_link->GetName() << ": force: " << rotor.getForce() << std::endl;
              /* add force and torque, need to check */
              child_link->AddRelativeForce(gazebo::math::Vector3(0, 0, rotor.getForce()));
              parent_link->AddRelativeTorque(gazebo::math::Vector3(0, 0, rotor.getTorque()));
            }
        }
        break;
    }
  }
}

void AerialRobotHWSim::eStopActive(const bool active)
{
  e_stop_active_ = active;
}

// Register the limits of the joint specified by joint_name and joint_handle. The limits are
// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
// Return the joint's type, lower position limit, upper position limit, and effort limit.
void AerialRobotHWSim::registerJointLimits(const std::string& joint_name,
                         const hardware_interface::JointHandle& joint_handle,
                         const ControlMethod ctrl_method,
                         const ros::NodeHandle& joint_limit_nh,
                         const urdf::Model *const urdf_model,
                         int *const joint_type, double *const lower_limit,
                         double *const upper_limit, double *const effort_limit)
{
  *joint_type = urdf::Joint::UNKNOWN;
  *lower_limit = -std::numeric_limits<double>::max();
  *upper_limit = std::numeric_limits<double>::max();
  *effort_limit = std::numeric_limits<double>::max();

  joint_limits_interface::JointLimits limits;
  bool has_limits = false;
  joint_limits_interface::SoftJointLimits soft_limits;
  bool has_soft_limits = false;

  if (urdf_model != NULL)
  {
    const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(joint_name);
    if (urdf_joint != NULL)
    {
      *joint_type = urdf_joint->type;
      // Get limits from the URDF file.
      if (joint_limits_interface::getJointLimits(urdf_joint, limits))
        has_limits = true;
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
        has_soft_limits = true;
    }
  }
  // Get limits from the parameter server.
  if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
    has_limits = true;

  if (!has_limits)
    return;

  if (*joint_type == urdf::Joint::UNKNOWN)
  {
    // Infer the joint type.

    if (limits.has_position_limits)
    {
      *joint_type = urdf::Joint::REVOLUTE;
    }
    else
    {
      if (limits.angle_wraparound)
        *joint_type = urdf::Joint::CONTINUOUS;
      else
        *joint_type = urdf::Joint::PRISMATIC;
    }
  }

  if (limits.has_position_limits)
  {
    *lower_limit = limits.min_position;
    *upper_limit = limits.max_position;
  }
  if (limits.has_effort_limits)
    *effort_limit = limits.max_effort;

  if (has_soft_limits)
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          const joint_limits_interface::EffortJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          ej_limits_interface_.registerHandle(limits_handle);
        }
        break;
      case POSITION:
        {
          const joint_limits_interface::PositionJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          pj_limits_interface_.registerHandle(limits_handle);
        }
        break;
      case VELOCITY:
        {
          const joint_limits_interface::VelocityJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          vj_limits_interface_.registerHandle(limits_handle);
        }
        break;
    }
  }
  else
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          const joint_limits_interface::EffortJointSaturationHandle
            sat_handle(joint_handle, limits);
          ej_sat_interface_.registerHandle(sat_handle);
        }
        break;
      case POSITION:
        {
          const joint_limits_interface::PositionJointSaturationHandle
            sat_handle(joint_handle, limits);
          pj_sat_interface_.registerHandle(sat_handle);
        }
        break;
      case VELOCITY:
        {
          const joint_limits_interface::VelocityJointSaturationHandle
            sat_handle(joint_handle, limits);
          vj_sat_interface_.registerHandle(sat_handle);
        }
        break;
    }
  }
}


}

PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::AerialRobotHWSim, gazebo_ros_control::RobotHWSim)
