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

    /* get baselink from robot model */
    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(*urdf_model, tree);

    auto robot_model_xml = aerial_robot_model::RobotModel::getRobotModelXml("robot_description", model_nh);
    TiXmlElement* baselink_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("baselink");
    if(!baselink_attr)
      {
        ROS_ERROR_STREAM_NAMED("spianl interface", "Failed to find baselink attribute from urdf model, please add '<baselink name=\"fc\" />' to your urdf file");
        return false;
      }
    baselink_ = std::string(baselink_attr->Attribute("name"));

    KDL::SegmentMap::const_iterator it = tree.getSegment(baselink_);
    if(it == tree.getSegments().end())
      {
        ROS_ERROR_STREAM_NAMED("spianl interface", "Failed to find baselink '" << baselink_ << "' in urdf from robot_description");
        return false;
      }

    std::function<KDL::Frame (const KDL::SegmentMap::const_iterator& ) > recursiveFindParent = [&recursiveFindParent, this, &tree](const KDL::SegmentMap::const_iterator& it)
      {
        const KDL::TreeElementType& currentElement = it->second;
        KDL::Frame currentFrame = GetTreeElementSegment(currentElement).pose(0);

        KDL::SegmentMap::const_iterator parentIt = GetTreeElementParent(currentElement);

        if(GetTreeElementSegment(parentIt->second).getJoint().getType() != KDL::Joint::None ||
           parentIt == tree.getRootSegment())
          {
            baselink_parent_ = parentIt->first;
            return currentFrame;
          }
        else
          {
            return recursiveFindParent(parentIt) * currentFrame;
          }
      };

    auto baselink_offset = recursiveFindParent(it);
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Quaterniond q;
    baselink_offset.M.GetQuaternion(q.X(), q.Y(), q.Z(), q.W());
    baselink_offset_.Set(ignition::math::Vector3d(baselink_offset.p.x(), baselink_offset.p.y(), baselink_offset.p.z()), q);
#else
    gazebo::math::Quaternion q;
    baselink_offset.M.GetQuaternion(q.x, q.y, q.z, q.w);
    baselink_offset_.Set(gazebo::math::Vector3(baselink_offset.p.x(), baselink_offset.p.y(), baselink_offset.p.z()), q);
#endif

    if(baselink_parent_ == std::string("none"))
      {
        ROS_ERROR_STREAM_NAMED("spianl interface", "Can not find the parent of the baselink '" << baselink_);
        return false;
      }

    /* Initialize joint handlers */
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

    /* Initialize rotor handlers */
    rotor_n_dof_ = transmissions.size() - n_dof_;
    for(const auto& transmission: transmissions)
      {
        const std::string& hardware_interface = transmission.joints_[0].hardware_interfaces_.front();

        hardware_interface::RotorHandle rotor_handle;
        if(hardware_interface == "RotorInterface")
          {
            /* TODO: change to debug */
            std::string rotor_name = transmission.joints_[0].name_;
            ROS_INFO_STREAM_NAMED("aerial_robot_hw_sim","Loading rotor '"
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

    /* Initialize spinal interface */
    spinal_interface_.init(model_nh, n_dof_);
    registerInterface(&spinal_interface_);

    /* Initialize sensor handlers */
    for(auto sensor: gazebo::sensors::SensorManager::Instance()->GetSensors())
      {
        if(sensor->Name() == std::string("spinal_imu"))
          imu_handler_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(sensor);
        if(sensor->Name() == std::string("magnetometer"))
          mag_handler_ = std::dynamic_pointer_cast<gazebo::sensors::MagnetometerSensor>(sensor);
        ROS_DEBUG_STREAM_NAMED("aerial_robot_hw", "get gazebo sensor named: " << sensor->Name());
      }

    control_mode_ = FORCE_CONTROL_MODE;
    sim_vel_sub_ = model_nh.subscribe("sim_cmd_vel", 1, &AerialRobotHWSim::cmdVelCallback, this);
    sim_pos_sub_ = model_nh.subscribe("sim_cmd_pos", 1, &AerialRobotHWSim::cmdPosCallback, this);

    ros::NodeHandle simulation_nh = ros::NodeHandle(model_nh, "simulation");
    simulation_nh.param("ground_truth_pub_rate", ground_truth_pub_rate_, 0.01); // [sec]
    simulation_nh.param("ground_truth_pos_noise", ground_truth_pos_noise_, 0.0); // m
    simulation_nh.param("ground_truth_vel_noise", ground_truth_vel_noise_, 0.0); // m/s
    simulation_nh.param("ground_truth_rot_noise", ground_truth_rot_noise_, 0.0); // rad
    simulation_nh.param("ground_truth_angular_noise", ground_truth_angular_noise_, 0.0); // rad/s
    simulation_nh.param("ground_truth_rot_drift", ground_truth_rot_drift_, 0.0); // rad
    simulation_nh.param("ground_truth_vel_drift", ground_truth_vel_drift_, 0.0); // m/s
    simulation_nh.param("ground_truth_angular_drift", ground_truth_angular_drift_, 0.0); // rad/s
    simulation_nh.param("ground_truth_rot_drift_frequency", ground_truth_rot_drift_frequency_, 0.0); // 1/s
    simulation_nh.param("ground_truth_vel_drift_frequency", ground_truth_vel_drift_frequency_, 0.0); // 1/s
    simulation_nh.param("ground_truth_angular_drift_frequency", ground_truth_angular_drift_frequency_, 0.0); // 1/s

    simulation_nh.param("mocap_pub_rate", mocap_pub_rate_, 0.01); // [sec]
    simulation_nh.param("mocap_pos_noise", mocap_pos_noise_, 0.001); // m
    simulation_nh.param("mocap_rot_noise", mocap_rot_noise_, 0.001); // rad
    ground_truth_pub_ = model_nh.advertise<nav_msgs::Odometry>("ground_truth", 1);
    mocap_pub_ = model_nh.advertise<geometry_msgs::PoseStamped>("mocap/pose", 1);

    return true;
  }

  void AerialRobotHWSim::readSim(ros::Time time, ros::Duration period)
  {
    DefaultRobotHWSim::readSim(time, period);

    /* set ground truth value to spinal interface */
    const gazebo::physics::LinkPtr baselink_parent = parent_model_->GetLink(baselink_parent_);
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Vector3d baselink_pos = baselink_parent->WorldPose().Pos() + baselink_parent->WorldPose().Rot() * baselink_offset_.Pos();
    ignition::math::Vector3d baselink_vel = baselink_parent->WorldLinearVel(baselink_offset_.Pos());
    ignition::math::Quaterniond q = baselink_parent->WorldPose().Rot() * baselink_offset_.Rot();
    ignition::math::Vector3d w = baselink_offset_.Rot().Inverse() * baselink_parent->RelativeAngularVel();
#else
    gazebo::math::Vector3 gazebo_pos = baselink_parent->GetWorldPose().pos + baselink_parent->GetWorldPose().rot * baselink_offset_.pos;
    gazebo::math::Vector3 gazebo_vel = baselink_parent->GetWorldLinearVel(baselink_offset_.pos);
    ignition::math::Vector3d baselink_pos(gazebo_pos.x, gazebo_pos.y, gazebo_pos.z);
    ignition::math::Vector3d baselink_vel(gazebo_vel.x, gazebo_vel.y, gazebo_vel.z);

    gazebo::math::Quaternion gazebo_q = baselink_parent->GetWorldPose().rot * baselink_offset_.rot;
    gazebo::math::Vector3 gazebo_w = baselink_offset_.rot.GetInverse() * baselink_parent->GetRelativeAngularVel();
    ignition::math::Quaterniond q(gazebo_q.w, gazebo_q.x, gazebo_q.y, gazebo_q.z);
    ignition::math::Vector3d w(gazebo_w.x, gazebo_w.y, gazebo_w.z);
#endif

    /* get sensor values and do atittude estimation */
    if(imu_handler_)
      {
        auto acc = imu_handler_->LinearAcceleration();
        auto gyro = imu_handler_->AngularVelocity();
        spinal_interface_.setImuValue(acc.X(), acc.Y(), acc.Z(), gyro.X(), gyro.Y(), gyro.Z());
      }
    else
      ROS_DEBUG_THROTTLE(1.0, "No imu sensor handler to read sensor data");

    if(mag_handler_)
      {
        auto mag = mag_handler_->MagneticField();
        spinal_interface_.setMagValue(mag.X(), mag.Y(), mag.Z());
      }
    else
      ROS_DEBUG_THROTTLE(1.0, "No magnetometer sensor handler to read sensor data");

    spinal_interface_.stateEstimate();

    /* publish ground truth value */
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = time;
    odom_msg.pose.pose.position.x = baselink_pos.X() + gazebo::gaussianKernel(ground_truth_pos_noise_);
    odom_msg.pose.pose.position.y = baselink_pos.Y() + gazebo::gaussianKernel(ground_truth_pos_noise_);
    odom_msg.pose.pose.position.z = baselink_pos.Z() + gazebo::gaussianKernel(ground_truth_pos_noise_);

    ignition::math::Vector3d euler = q.Euler();
    euler += ignition::math::Vector3d(gazebo::addNoise(ground_truth_rot_curr_drift_.X(), ground_truth_rot_drift_, ground_truth_rot_drift_frequency_, 0, ground_truth_rot_noise_, period.toSec()),
                                      gazebo::addNoise(ground_truth_rot_curr_drift_.Y(), ground_truth_rot_drift_, ground_truth_rot_drift_frequency_, 0, ground_truth_rot_noise_, period.toSec()),
                                      gazebo::addNoise(ground_truth_rot_curr_drift_.Z(), ground_truth_rot_drift_, ground_truth_rot_drift_frequency_, 0, ground_truth_rot_noise_, period.toSec()));

    ignition::math::Quaterniond q_noise(euler);
    odom_msg.pose.pose.orientation.x = q_noise.X();
    odom_msg.pose.pose.orientation.y = q_noise.Y();
    odom_msg.pose.pose.orientation.z = q_noise.Z();
    odom_msg.pose.pose.orientation.w = q_noise.W();

    odom_msg.twist.twist.linear.x = baselink_vel.X() + gazebo::addNoise(ground_truth_vel_curr_drift_.X(), ground_truth_vel_drift_, ground_truth_vel_drift_frequency_, 0, ground_truth_vel_noise_, period.toSec());
    odom_msg.twist.twist.linear.y = baselink_vel.Y() + gazebo::addNoise(ground_truth_vel_curr_drift_.Y(), ground_truth_vel_drift_, ground_truth_vel_drift_frequency_, 0, ground_truth_vel_noise_, period.toSec());
    odom_msg.twist.twist.linear.z = baselink_vel.Z() + gazebo::addNoise(ground_truth_vel_curr_drift_.Z(), ground_truth_vel_drift_, ground_truth_vel_drift_frequency_, 0, ground_truth_vel_noise_, period.toSec());

    /* CAUTION! the angular is describe in the fc frame */

    odom_msg.twist.twist.angular.x = w.X() + gazebo::addNoise(ground_truth_angular_curr_drift_.X(), ground_truth_angular_drift_, ground_truth_angular_drift_frequency_, 0, ground_truth_angular_noise_, period.toSec());
    odom_msg.twist.twist.angular.y = w.Y() + gazebo::addNoise(ground_truth_angular_curr_drift_.Y(), ground_truth_angular_drift_, ground_truth_angular_drift_frequency_, 0, ground_truth_angular_noise_, period.toSec());
    odom_msg.twist.twist.angular.z = w.Z() + gazebo::addNoise(ground_truth_angular_curr_drift_.Z(), ground_truth_angular_drift_, ground_truth_angular_drift_frequency_, 0, ground_truth_angular_noise_, period.toSec());

    /* set ground truth for controller: use the value with noise */
    spinal_interface_.setTrueBaselinkOrientation(q_noise.X(),
                                                 q_noise.Y(),
                                                 q_noise.Z(),
                                                 q_noise.W());
    spinal_interface_.setTrueBaselinkAngular(odom_msg.twist.twist.angular.x,
                                             odom_msg.twist.twist.angular.y,
                                             odom_msg.twist.twist.angular.z);


    if(time.toSec() - last_ground_truth_time_.toSec() > ground_truth_pub_rate_)
      {
        ground_truth_pub_.publish(odom_msg);
        last_ground_truth_time_ = time;
      }

    if(time.toSec() - last_mocap_time_.toSec() > mocap_pub_rate_)
      {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = odom_msg.header;

        pose_msg.pose.position.x = odom_msg.pose.pose.position.x + gazebo::gaussianKernel(mocap_pos_noise_);
        pose_msg.pose.position.y = odom_msg.pose.pose.position.y + gazebo::gaussianKernel(mocap_pos_noise_);
        pose_msg.pose.position.z = odom_msg.pose.pose.position.z + gazebo::gaussianKernel(mocap_pos_noise_);

        // pose.pose.orientation = odom_msg.pose.pose.orientation;
        euler = q.Euler();
        euler += ignition::math::Vector3d(gazebo::gaussianKernel(mocap_rot_noise_),
                                          gazebo::gaussianKernel(mocap_rot_noise_),
                                          gazebo::gaussianKernel(mocap_rot_noise_));
        q_noise = ignition::math::Quaterniond(euler);
        pose_msg.pose.orientation.x = q_noise.X();
        pose_msg.pose.orientation.y = q_noise.Y();
        pose_msg.pose.orientation.z = q_noise.Z();
        pose_msg.pose.orientation.w = q_noise.W();


        mocap_pub_.publish(pose_msg);
        last_mocap_time_ = time;
      }
  }

  void AerialRobotHWSim::writeSim(ros::Time time, ros::Duration period)
  {
    DefaultRobotHWSim::writeSim(time, period);

    er_sat_interface_.enforceLimits(period);
    const gazebo::physics::LinkPtr baselink = parent_model_->GetLink(baselink_);
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
            auto  torque = rotor.getTorque();
            parent_link->AddRelativeTorque(ignition::math::Vector3d(torque.x(), torque.y(), torque.z()));
#else
            child_link->AddRelativeForce(gazebo::math::Vector3(0, 0, rotor.getForce()));
            auto  torque = rotor.getTorque();
            parent_link->AddRelativeTorque(gazebo::math::Vector3(torque.x(), torque.y(), torque.z()));
#endif
            sim_rotors_.at(j)->SetVelocity(0, rotor.getSpeed());
          }
      }
  }
}

PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::AerialRobotHWSim, gazebo_ros_control::RobotHWSim)
