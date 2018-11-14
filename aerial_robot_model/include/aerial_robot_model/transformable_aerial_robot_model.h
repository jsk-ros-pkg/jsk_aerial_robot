// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
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

#pragma once

#include <aerial_robot_model/kdl_utils.h>
#include <cmath>
#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/Transform.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <vector>

namespace aerial_robot_model {

  //Transformable Aerial Robot Model
  class RobotModel {
  public:
    RobotModel(bool init_with_rosparam,
               bool verbose = false,
               std::string baselink = std::string(""),
               std::string thrust_link = std::string(""));
    virtual ~RobotModel() = default;

    //public functions
    bool addExtraModule(std::string module_name, std::string parent_link_name, KDL::Frame transform, KDL::RigidBodyInertia inertia);
    template<class T> T forwardKinematics(std::string link, const KDL::JntArray& joint_positions) const;
    template<class T> T forwardKinematics(std::string link, const sensor_msgs::JointState& state) const;
    void fullForwardKinematics(const KDL::JntArray& joint_positions, std::map<std::string, KDL::Frame>& seg_tf_map) { fullForwardKinematicsImpl(joint_positions, seg_tf_map); }
    void fullForwardKinematics(const sensor_msgs::JointState& state,  std::map<std::string, KDL::Frame>& seg_tf_map) { fullForwardKinematicsImpl(jointMsgToKdl(state), seg_tf_map); }
    void fullForwardKinematicsImpl(const KDL::JntArray& joint_positions, std::map<std::string, KDL::Frame>& seg_tf_map);

    std::vector<int> getActuatorJointMap() const { return actuator_joint_map_; }
    std::map<std::string, uint32_t> getActuatorMap() const { return actuator_map_; }
    std::map<std::string, KDL::Frame> getSegmentsTf() const { return seg_tf_map_; }
    std::string getBaselinkName() const { return baselink_; }
    template<class T> T getCog() const;
    template<class T> T getCogDesireOrientation() const;
    template<class T> T getCog2Baselink() const;
    template<class T> T getInertia() const;
    double getLinkLength() const { return link_length_; }
    double getMass() const { return mass_; }
    std::map<int, int> getRotorDirection() { return rotor_direction_; }
    std::string getRootFrameName() const { return GetTreeElementSegment(tree_.getRootSegment()->second).getName(); }
    int getRotorNum() const { return rotor_num_; }
    template<class T> std::vector<T> getRotorsNormalFromCog() const;
    template<class T> std::vector<T> getRotorsOriginFromCog() const;
    double getVerbose() const { return verbose_; }
    void setActuatorJointMap(const sensor_msgs::JointState& actuator_state);
    void setCogDesireOrientation(double roll, double pitch, double yaw) { cog_desire_orientation_ = KDL::Rotation::RPY(roll, pitch, yaw); }
    bool removeExtraModule(std::string module_name);
    virtual void updateRobotModelImpl(const KDL::JntArray& joint_positions);
    void updateRobotModel(const KDL::JntArray& joint_positions) { updateRobotModelImpl(joint_positions); }
    void updateRobotModel(const sensor_msgs::JointState& state) { updateRobotModel(jointMsgToKdl(state)); }

  protected:
    //protected functions
    KDL::JntArray jointMsgToKdl(const sensor_msgs::JointState& state) const;
    sensor_msgs::JointState kdlJointToMsg(const KDL::JntArray& joint_positions) const;

  private:
    //private attributes
    std::vector<int> actuator_joint_map_; //the real joint (other than rotor or gimbal)
    std::map<std::string, uint32_t> actuator_map_; // regarding to KDL tree
    std::string baselink_;
    KDL::Frame cog_;
    KDL::Rotation cog_desire_orientation_;
    KDL::Frame cog2baselink_transform_;
    std::map<std::string, KDL::Frame> seg_tf_map_;
    std::map<std::string, KDL::Segment> extra_module_map_;
    std::map<std::string, KDL::RigidBodyInertia> inertia_map_;
    KDL::RotationalInertia link_inertia_cog_;
    double link_length_;
    double mass_;
    urdf::Model model_;
    std::map<int, int> rotor_direction_;
    int rotor_num_;
    std::vector<KDL::Vector> rotors_origin_from_cog_;
    std::vector<KDL::Vector> rotors_normal_from_cog_;
    KDL::Tree tree_;
    std::string thrust_link_;
    bool verbose_;

    //private functions
    KDL::Frame forwardKinematicsImpl(std::string link, const KDL::JntArray& joint_positions) const
    {
      KDL::TreeFkSolverPos_recursive fk_solver(tree_);
      KDL::Frame f;
      int status = fk_solver.JntToCart(joint_positions, f, link);
      if(status < 0) ROS_ERROR("can not solve FK to link: %s", link.c_str());

      return f;
    }
    void getParamFromRos();
    KDL::RigidBodyInertia inertialSetup(const KDL::TreeElement& tree_element);
    void resolveLinkLength();
  };

  template<> inline Eigen::Affine3d RobotModel::forwardKinematics(std::string link, const KDL::JntArray& joint_positions) const
  {
    return aerial_robot_model::kdlToEigen(forwardKinematicsImpl(link, joint_positions));
  }

  template<> inline geometry_msgs::TransformStamped RobotModel::forwardKinematics(std::string link, const KDL::JntArray& joint_positions) const
  {
    return aerial_robot_model::kdlToMsg(forwardKinematicsImpl(link, joint_positions));
  }

  template<> inline KDL::Frame RobotModel::forwardKinematics(std::string link, const KDL::JntArray& joint_positions) const
  {
    return forwardKinematicsImpl(link, joint_positions);
  }

  template<> inline tf2::Transform RobotModel::forwardKinematics(std::string link, const KDL::JntArray& joint_positions) const
  {
    return aerial_robot_model::kdlToTf2(forwardKinematicsImpl(link, joint_positions));
  }

  template<> inline Eigen::Affine3d RobotModel::forwardKinematics(std::string link, const sensor_msgs::JointState& state) const
  {
    return aerial_robot_model::kdlToEigen(forwardKinematicsImpl(link, jointMsgToKdl(state)));
  }

  template<> inline geometry_msgs::TransformStamped RobotModel::forwardKinematics(std::string link, const sensor_msgs::JointState& state) const
  {
    return aerial_robot_model::kdlToMsg(forwardKinematicsImpl(link, jointMsgToKdl(state)));
  }

  template<> inline KDL::Frame RobotModel::forwardKinematics(std::string link, const sensor_msgs::JointState& state) const
  {
    return forwardKinematicsImpl(link, jointMsgToKdl(state));
  }

  template<> inline tf2::Transform RobotModel::forwardKinematics(std::string link, const sensor_msgs::JointState& state) const
  {
    return aerial_robot_model::kdlToTf2(forwardKinematicsImpl(link, jointMsgToKdl(state)));
  }


  template<> inline Eigen::Affine3d RobotModel::getCog() const
  {
    return aerial_robot_model::kdlToEigen(cog_);
  }

  template<> inline geometry_msgs::TransformStamped RobotModel::getCog() const
  {
    return aerial_robot_model::kdlToMsg(cog_);
  }

  template<> inline KDL::Frame RobotModel::getCog() const
  {
    return cog_;
  }

  template<> inline tf2::Transform RobotModel::getCog() const
  {
    return aerial_robot_model::kdlToTf2(cog_);
  }

  template<> inline Eigen::Matrix3d RobotModel::getCogDesireOrientation() const
  {
    return aerial_robot_model::kdlToEigen(cog_desire_orientation_);
  }

  template<> inline KDL::Rotation RobotModel::getCogDesireOrientation() const
  {
    return cog_desire_orientation_;
  }

  template<> inline Eigen::Affine3d RobotModel::getCog2Baselink() const
  {
    return aerial_robot_model::kdlToEigen(cog2baselink_transform_);
  }

  template<> inline geometry_msgs::TransformStamped RobotModel::getCog2Baselink() const
  {
    return aerial_robot_model::kdlToMsg(cog2baselink_transform_);
  }

  template<> inline KDL::Frame RobotModel::getCog2Baselink() const
  {
    return cog2baselink_transform_;
  }

  template<> inline tf2::Transform RobotModel::getCog2Baselink() const
  {
    return aerial_robot_model::kdlToTf2(cog2baselink_transform_);
  }

  template<> inline Eigen::Matrix3d RobotModel::getInertia() const
  {
    return aerial_robot_model::kdlToEigen(link_inertia_cog_);
  }

  template<> inline KDL::RotationalInertia RobotModel::getInertia() const
  {
    return link_inertia_cog_;
  }

  template<> inline std::vector<Eigen::Vector3d> RobotModel::getRotorsNormalFromCog() const
  {
    return aerial_robot_model::kdlToEigen(rotors_normal_from_cog_);
  }

  template<> inline std::vector<geometry_msgs::PointStamped> RobotModel::getRotorsNormalFromCog() const
  {
    return aerial_robot_model::kdlToMsg(rotors_normal_from_cog_);
  }

  template<> inline std::vector<KDL::Vector> RobotModel::getRotorsNormalFromCog() const
  {
    return rotors_normal_from_cog_;
  }

  template<> inline std::vector<tf2::Vector3> RobotModel::getRotorsNormalFromCog() const
  {
    return aerial_robot_model::kdlToTf2(rotors_normal_from_cog_);
  }

  template<> inline std::vector<Eigen::Vector3d> RobotModel::getRotorsOriginFromCog() const
  {
    return aerial_robot_model::kdlToEigen(rotors_origin_from_cog_);
  }

  template<> inline std::vector<geometry_msgs::PointStamped> RobotModel::getRotorsOriginFromCog() const
  {
    return aerial_robot_model::kdlToMsg(rotors_origin_from_cog_);
  }

  template<> inline std::vector<KDL::Vector> RobotModel::getRotorsOriginFromCog() const
  {
    return rotors_origin_from_cog_;
  }

  template<> inline std::vector<tf2::Vector3> RobotModel::getRotorsOriginFromCog() const
  {
    return aerial_robot_model::kdlToTf2(rotors_origin_from_cog_);
  }
} //namespace aerial_robot_model
