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

namespace gazebo_ros_control
{

  bool AerialRobotHWSim::initSim(const std::string& robot_namespace,
                                 ros::NodeHandle model_nh,
                                 gazebo::physics::ModelPtr parent_model,
                                 const urdf::Model *const urdf_model,
                                 std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    parent_model_ = parent_model;


    if(!spinal_interface_.init(*urdf_model))
      {
        ROS_ERROR_STREAM_NAMED("aerial_robot_hw", "Failed to initialize spinal interface");
        return false;
      }
    baselink_parent_ = spinal_interface_.getBaseLinkParentName();
    auto baselink_offset = spinal_interface_.getBaselinkOffset();
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Quaterniond q;
    baselink_offset.M.GetQuaternion(q.X(), q.Y(), q.Z(), q.W());
    baselink_offset_.Set(ignition::math::Vector3d(baselink_offset.p.x(), baselink_offset.p.y(), baselink_offset.p.z()), q);
#else
    gazebo::math::Quaternion q;
    baselink_offset.M.GetQuaternion(q.x, q.y, q.z, q.w);
    baselink_offset_.Set(gazebo::math::Vector3(baselink_offset.p.x(), baselink_offset.p.y(), baselink_offset.p.z()), q);
#endif

    std::vector<transmission_interface::TransmissionInfo> joint_transmissions;
    for(const auto& transmission: transmissions)
      {
        auto hardware_interface = transmission.joints_[0].hardware_interfaces_.front();
        if(hardware_interface.find("JointInterface") != std::string::npos)
          {
            joint_transmissions.push_back(transmission);
          }
      }
    if(!DefaultRobotHWSim::initSim(robot_namespace, model_nh,  parent_model, urdf_model, joint_transmissions))
      return false;

    // Initialize values
    rotor_n_dof_ = transmissions.size() - n_dof_;

    for(const auto& transmission: transmissions)
      {
        const std::string& hardware_interface = transmission.joints_[0].hardware_interfaces_.front();

        hardware_interface::RotorHandle rotor_handle;
        if(hardware_interface == "RotorInterface")
          {
            /* TODO: change to debug */
            std::string rotor_name = transmission.joints_[0].name_;
            ROS_INFO_STREAM_NAMED("aerial_robot_hw_sim","Loading joint '"
                                  << rotor_name
                                  << "' of type '" << hardware_interface << "'");

            /* create handle for each rotor */
            const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(rotor_name);
            rotor_handle = hardware_interface::RotorHandle(model_nh, urdf_joint);

            /* register rotor handle to spinal interface */
            spinal_interface_.registerHandle(rotor_handle);

            /* register rotor handle to limitation (saturation) interface */
            const rotor_limits_interface::EffortRotorSaturationHandle limits_handle(rotor_handle, urdf_joint);
            er_sat_interface_.registerHandle(limits_handle);

            /* get the gazebo joint model that corresponds to the rotor */
            gazebo::physics::JointPtr rotor = parent_model->GetJoint(rotor_name);
            if (!rotor)
              {
                ROS_ERROR_STREAM_NAMED("aerial_robot_hw", "This robot has a rotor named \""
                                       << rotor_name
                                       << "\" which is not in the gazebo model.");
                return false;
              }
            sim_rotors_.push_back(rotor);
          }
      }

    registerInterface(&spinal_interface_);
    spinal_interface_.setJointNum(n_dof_);


    control_mode_ = FORCE_CONTROL_MODE;
    sim_vel_sub_ = model_nh.subscribe("sim_cmd_vel", 1, &AerialRobotHWSim::cmdVelCallback, this);
    sim_pos_sub_ = model_nh.subscribe("sim_cmd_pos", 1, &AerialRobotHWSim::cmdPosCallback, this);
    model_nh.param("ground_truth_pub_rate", ground_truth_pub_rate_, 0.01); // [sec]
    ground_truth_pub_ = model_nh.advertise<nav_msgs::Odometry>("ground_truth", 1);
    return true;
  }

  void AerialRobotHWSim::readSim(ros::Time time, ros::Duration period)
  {
    DefaultRobotHWSim::readSim(time, period);

    /* Aerial Robot */
    const gazebo::physics::LinkPtr baselink_parent = parent_model_->GetLink(baselink_parent_);

    /* TODO: switch to spinal interface */
    /* please check whether this is same with IMU process */
    /* orientation should calculate the rpy in world frame,
       and angular velocity should show the value in board(body) frame */
    /* set orientation and angular of baselink */
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Quaterniond q = baselink_parent->WorldPose().Rot() * baselink_offset_.Rot();
    spinal_interface_.setBaseLinkOrientation(q.X(), q.Y(), q.Z(), q.W());
    ignition::math::Vector3d w = baselink_offset_.Rot().Inverse() * baselink_parent->RelativeAngularVel();
    spinal_interface_.setBaseLinkAngular(w.X(), w.Y(), w.Z());
#else
    gazebo::math::Quaternion q = baselink_parent->GetWorldPose().rot * baselink_offset_.rot;
    spinal_interface_.setBaseLinkOrientation(q.x, q.y, q.z, q.w);
    gazebo::math::Vector3 w = baselink_offset_.rot.GetInverse() * baselink_parent->GetRelativeAngularVel();
    spinal_interface_.setBaseLinkAngular(w.x, w.y, w.z);
#endif

    if(time.toSec() - last_time_.toSec() > ground_truth_pub_rate_)
      {
        nav_msgs::Odometry pose_msg;
        pose_msg.header.stamp = time;
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Vector3d baselink_pos = baselink_parent->WorldPose().Pos() + baselink_parent->WorldPose().Rot() * baselink_offset_.Pos();
        pose_msg.pose.pose.position.x = baselink_pos.X();
        pose_msg.pose.pose.position.y = baselink_pos.Y();
        pose_msg.pose.pose.position.z = baselink_pos.Z();
        pose_msg.pose.pose.orientation.x = q.X();
        pose_msg.pose.pose.orientation.y = q.Y();
        pose_msg.pose.pose.orientation.z = q.Z();
        pose_msg.pose.pose.orientation.w = q.W();

        ignition::math::Vector3d baselink_vel = baselink_parent->WorldLinearVel(baselink_offset_.Pos());
        pose_msg.twist.twist.linear.x = baselink_vel.X();
        pose_msg.twist.twist.linear.y = baselink_vel.Y();
        pose_msg.twist.twist.linear.z = baselink_vel.Z();

        /* CAUTION! the angular is describe in the fc frame */
        pose_msg.twist.twist.angular.x = w.X();
        pose_msg.twist.twist.angular.y = w.Y();
        pose_msg.twist.twist.angular.z = w.Z();
#else
        gazebo::math::Vector3 baselink_pos = baselink_parent->GetWorldPose().pos + baselink_parent->GetWorldPose().rot * baselink_offset_.pos;
        pose_msg.pose.pose.position.x = baselink_pos.x;
        pose_msg.pose.pose.position.y = baselink_pos.y;
        pose_msg.pose.pose.position.z = baselink_pos.z;
        pose_msg.pose.pose.orientation.x = q.x;
        pose_msg.pose.pose.orientation.y = q.y;
        pose_msg.pose.pose.orientation.z = q.z;
        pose_msg.pose.pose.orientation.w = q.w;

        gazebo::math::Vector3 baselink_vel = baselink_parent->GetWorldLinearVel(baselink_offset_.pos);
        pose_msg.twist.twist.linear.x = baselink_vel.x;
        pose_msg.twist.twist.linear.y = baselink_vel.y;
        pose_msg.twist.twist.linear.z = baselink_vel.z;

        /* CAUTION! the angular is described in the fc frame */
        pose_msg.twist.twist.angular.x = w.x;
        pose_msg.twist.twist.angular.y = w.y;
        pose_msg.twist.twist.angular.z = w.z;
#endif
        ground_truth_pub_.publish(pose_msg);
        last_time_ = time;
      }

    /* TODO : mocap and imu */
    /* attitude estimation from spinal interface */
  }

  void AerialRobotHWSim::writeSim(ros::Time time, ros::Duration period)
  {
    DefaultRobotHWSim::writeSim(time, period);

    er_sat_interface_.enforceLimits(period);
    const gazebo::physics::LinkPtr baselink = parent_model_->GetLink(spinal_interface_.getBaseLinkName());
    if (control_mode_ == SIM_VEL_MODE)
      {
#if GAZEBO_MAJOR_VERSION >= 8
        baselink->SetLinearVel(ignition::math::Vector3d(cmd_vel_.twist.linear.x,
                                                        cmd_vel_.twist.linear.y,
                                                        cmd_vel_.twist.linear.z));
        baselink->SetAngularVel(ignition::math::Vector3d(cmd_vel_.twist.angular.x,
                                                         cmd_vel_.twist.angular.y,
                                                         cmd_vel_.twist.angular.z));
#else
        baselink->SetLinearVel(gazebo::math::Vector3(cmd_vel_.twist.linear.x,
                                                     cmd_vel_.twist.linear.y,
                                                     cmd_vel_.twist.linear.z));
        baselink->SetAngularVel(gazebo::math::Vector3(cmd_vel_.twist.angular.x,
                                                      cmd_vel_.twist.angular.y,
                                                      cmd_vel_.twist.angular.z));
#endif
      }
    else if (control_mode_ == SIM_POS_MODE)
      {
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d pose(ignition::math::Vector3d(cmd_pos_.pose.position.x,
                                                             cmd_pos_.pose.position.y,
                                                             cmd_pos_.pose.position.z),
                                    ignition::math::Quaterniond(cmd_pos_.pose.orientation.w,
                                                                cmd_pos_.pose.orientation.x,
                                                                cmd_pos_.pose.orientation.y,
                                                                cmd_pos_.pose.orientation.z));
        baselink->SetWorldPose(pose);
#else
        gazebo::math::Pose pose(gazebo::math::Vector3(cmd_pos_.pose.position.x,
                                                      cmd_pos_.pose.position.y,
                                                      cmd_pos_.pose.position.z),
                                gazebo::math::Quaternion(cmd_pos_.pose.orientation.w,
                                                         cmd_pos_.pose.orientation.x,
                                                         cmd_pos_.pose.orientation.y,
                                                         cmd_pos_.pose.orientation.z));
        baselink->SetWorldPose(pose);
#endif
        control_mode_ = FORCE_CONTROL_MODE;
      }
    else if (control_mode_ ==  FORCE_CONTROL_MODE)
      {
        for (int j = 0; j < rotor_n_dof_; j++)
          {
            hardware_interface::RotorHandle rotor = spinal_interface_.getHandle(sim_rotors_.at(j)->GetName());
            gazebo::physics::LinkPtr parent_link  = sim_rotors_.at(j)->GetParent();
            gazebo::physics::LinkPtr child_link  = sim_rotors_.at(j)->GetChild();

#if GAZEBO_MAJOR_VERSION >= 8
            child_link->AddRelativeForce(ignition::math::Vector3d(0, 0, rotor.getForce()));
            parent_link->AddRelativeTorque(ignition::math::Vector3d(0, 0, rotor.getTorque()));
#else
            child_link->AddRelativeForce(gazebo::math::Vector3(0, 0, rotor.getForce()));
            parent_link->AddRelativeTorque(gazebo::math::Vector3(0, 0, rotor.getTorque()));
#endif
          }
      }
  }

}

PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::AerialRobotHWSim, gazebo_ros_control::RobotHWSim)
