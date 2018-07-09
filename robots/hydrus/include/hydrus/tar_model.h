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

//Transformable Aerial Robot Model
class TARModel {
public:
  TARModel();
  TARModel(std::string baselink, std::string thrust_link): TARModel(), baselink_(baselink), thrust_link_(thrust_link) {}
  bool addExtraModule(int action, std::string module_name, std::string parent_link_name, geometry_msgs::Transform transform, geometry_msgs::Inertia inertia);
  tf::Transform getRoot2Link(std::string link, sensor_msgs::JointState state) const;
  void setActuatorJointMap(const sensor_msgs::JointState& actuator_state);
  const tf::Transform& getCog() const
  {
    return cog_;
  }
  void setCog(const tf::Transform& cog)
  {
    cog_ = cog;
  }
  const Eigen::Vector3d& getRotorOirginFromCog(const int index) const
  {
    return rotors_origin_from_cog_.at(index);
  }
  const std::vector<Eigen::Vector3d> getRotorsOriginFromCog() const
  {
    return rotors_origin_from_cog_;
  }
  void setRotorsOriginFromCog(const std::vector<Eigen::Vector3d>& rotors_origin_from_cog)
  {
    assert(rotors_origin_from_cog_.size() == rotors_origin_from_cog.size());
    rotors_origin_from_cog_ = rotors_origin_from_cog;
  }
  void setCog2Baselink(const tf::Transform& transform)
  {
    cog2baselink_transform_ = transform;
  }
  const tf::Transform& getCog2Baselink() const
  {
    return cog2baselink_transform_;
  }
  void setBaselink(const std::string& baselink)
  {
    baselink_ = baselink;
  }
  void setCogDesireOrientation(double roll, double pitch, double yaw)
  {
    cog_desire_orientation_ = KDL::Rotation::RPY(roll, pitch, yaw);
  }
  const Eigen::Matrix3d& getInertia() const
  {
    return links_inertia_;
  }
  void setInertia(const Eigen::Matrix3d& link_inertia)
  {
    links_inertia_ = link_inertia;
  }
  double getMass() const
  {
    return mass_;
  }
  void setMass(const double& mass)
  {
    mass_ = mass;
  }
  const urdf::Model& getRobotModel() const
  {
    return model_;
  }
  const KDL::Tree& getModelTree() const
  {
    return tree_;
  }
  int getRotorNum() const
  {
    return rotor_num_;
  }
  double getLinkLength() const
  {
    return link_length_;
  }
  const std::map<std::string, uint32_t>& getActuatorMap() const
  {
    return actuator_map_;
  }
  const std::vector<int>& getActuatorJointMap() const
  {
    return actuator_joint_map_;
  }

private:
  urdf::Model model_;
  KDL::Tree tree_;
  std::map<std::string, KDL::RigidBodyInertia> inertia_map_;
  std::map<std::string, uint32_t> actuator_map_; // regarding to KDL tree
  std::vector<int> actuator_joint_map_; //the real joint (other than rotor or gimbal)
  std::map<std::string /* module_name */, KDL::Segment> extra_module_map_;
  sensor_msgs::JointState current_actuator_state_;
  int rotor_num_;
  double link_length_;
  std::string thrust_link_;

  /* kinematics */
  double mass_;
  Eigen::Matrix3d links_inertia_;
  std::vector<Eigen::Vector3d> rotors_origin_from_cog_;
  tf2::Transform cog2baselink_transform_;
  std::string baselink_;
  tf2::Transform cog_;
  KDL::Rotation cog_desire_orientation_;

  KDL::RigidBodyInertia inertialSetup(const KDL::TreeElement tree_element);
  void resolveLinkLength();
};
