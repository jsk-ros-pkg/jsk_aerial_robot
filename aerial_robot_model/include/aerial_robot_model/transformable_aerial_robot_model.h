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

#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include <aerial_robot_model/AddExtraModule.h>
#include <aerial_robot_model/kdl_utils.h>
#include <Eigen/Geometry>
#include <cmath>
#include <eigen_conversions/eigen_kdl.h>
#include <vector>

namespace aerial_robot_model {

  //Transformable Aerial Robot Model
  class RobotModel {
  public:
    RobotModel() = default;
    RobotModel(std::string baselink, std::string thrust_link, bool verbose);

    bool addExtraModule(std::string module_name, std::string parent_link_name, KDL::Frame transform, KDL::RigidBodyInertia inertia);
    bool removeExtraModule(std::string module_name);
    void forwardKinematics(const sensor_msgs::JointState& state);
    void forwardKinematics(const KDL::JntArray& joint_positions);
    KDL::Frame getRoot2Link(std::string link, const sensor_msgs::JointState& state) const; //TODO need?
    void setActuatorJointMap(const sensor_msgs::JointState& actuator_state); //TODO need?

    //API declaration
    template<class T> T getCog() const;
    template<class T> std::vector<T> getRotorsOriginFromCog() const;
    template<class T> T getCog2Baselink() const;
    template<class T> T getInertia() const;

    double getMass() const
    {
      return mass_;
    }
    urdf::Model getRobotModel() const
    {
      return model_;
    }
    KDL::Tree getModelTree() const
    {
      return tree_;
    }
    std::string getRootFrameName() const
    {
      return GetTreeElementSegment(tree_.getRootSegment()->second).getName();
    }
    int getRotorNum() const
    {
      return rotor_num_;
    }
    double getLinkLength() const
    {
      return link_length_;
    }
    std::map<std::string, uint32_t> getActuatorMap() const
    {
      return actuator_map_;
    }
    std::vector<int> getActuatorJointMap() const
    {
      return actuator_joint_map_;
    }

  private:
    urdf::Model model_;
    KDL::Tree tree_;
    std::map<std::string, KDL::RigidBodyInertia> inertia_map_;
    std::map<std::string, uint32_t> actuator_map_; // regarding to KDL tree
    std::map<std::string, KDL::Segment> extra_module_map_; //string: module_name
    int rotor_num_;
    double link_length_; //TODO need?
    double mass_;
    KDL::RotationalInertia link_inertia_cog_;
    std::vector<KDL::Vector> rotors_origin_from_cog_;
    KDL::Frame cog2baselink_transform_;
    KDL::Frame cog_;
    KDL::Rotation cog_desire_orientation_;
    std::string baselink_;
    std::string thrust_link_;
    bool verbose_;
    std::map<int, int> rotor_direction_;

    std::vector<int> actuator_joint_map_; //the real joint (other than rotor or gimbal) //TODO need?

    KDL::RigidBodyInertia inertialSetup(const KDL::TreeElement& tree_element);
    void resolveLinkLength();

    bool addExtraModuleCallback(const aerial_robot_model::AddExtraModule::Request &req, aerial_robot_model::AddExtraModule::Response &res);
  };

  template<> inline KDL::Frame RobotModel::getCog() const
  {
    return cog_;
  }

  template<> inline geometry_msgs::TransformStamped RobotModel::getCog() const
  {
    return aerial_robot_model::kdlToMsg(cog_);
  }

  template<> inline Eigen::Affine3d RobotModel::getCog() const
  {
    return aerial_robot_model::kdlToEigen(cog_);
  }

  template<> inline tf2::Transform RobotModel::getCog() const
  {
    return aerial_robot_model::kdlToTf2(cog_);
  }

  template<>
  inline std::vector<KDL::Vector> RobotModel::getRotorsOriginFromCog() const
  {
    return rotors_origin_from_cog_;
  }

  template<>
  inline std::vector<geometry_msgs::PointStamped> RobotModel::getRotorsOriginFromCog() const
  {
    std::vector<geometry_msgs::PointStamped> vec;
    vec.reserve(rotors_origin_from_cog_.size());
    for(const auto& elem : rotors_origin_from_cog_)
      vec.push_back(aerial_robot_model::kdlToMsg(elem));
    return vec;
  }

  template<>
  inline std::vector<Eigen::Vector3d> RobotModel::getRotorsOriginFromCog() const
  {
    std::vector<Eigen::Vector3d> vec;
    vec.reserve(rotors_origin_from_cog_.size());
    for(const auto& elem : rotors_origin_from_cog_)
      vec.push_back(aerial_robot_model::kdlToEigen(elem));
    return vec;
  }

  template<>
  inline std::vector<tf2::Vector3> RobotModel::getRotorsOriginFromCog() const
  {
    std::vector<tf2::Vector3> vec;
    vec.reserve(rotors_origin_from_cog_.size());
    for(const auto& elem : rotors_origin_from_cog_)
      vec.push_back(aerial_robot_model::kdlToTf2(elem));
    return vec;
  }

  template<> inline KDL::Frame RobotModel::getCog2Baselink() const
  {
    return cog2baselink_transform_;
  }

  template<> inline geometry_msgs::TransformStamped RobotModel::getCog2Baselink() const
  {
    return aerial_robot_model::kdlToMsg(cog2baselink_transform_);
  }

  template<> inline Eigen::Affine3d RobotModel::getCog2Baselink() const
  {
    return aerial_robot_model::kdlToEigen(cog2baselink_transform_);
  }

  template<> inline tf2::Transform RobotModel::getCog2Baselink() const
  {
    return aerial_robot_model::kdlToTf2(cog2baselink_transform_);
  }

  template<> inline KDL::RotationalInertia RobotModel::getInertia() const
  {
    return link_inertia_cog_;
  }

  template<> inline Eigen::Matrix3d RobotModel::getInertia() const
  {
    return aerial_robot_model::kdlToEigen(link_inertia_cog_);
  }

} //namespace aerial_robot_model
