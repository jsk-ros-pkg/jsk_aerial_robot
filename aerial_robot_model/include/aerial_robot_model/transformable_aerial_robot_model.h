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
#include <aerial_robot_model/math_utils.h>
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
#include <stdexcept>
#include <ros/ros.h>
#include <urdf/model.h>
#include <vector>

namespace aerial_robot_model {


 //Transformable Aerial Robot Model
  class RobotModel {
  public:
    RobotModel(bool init_with_rosparam, bool verbose = false);
    virtual ~RobotModel() = default;

    //public functions
    bool addExtraModule(std::string module_name, std::string parent_link_name, KDL::Frame transform, KDL::RigidBodyInertia inertia);
    template<class T> T forwardKinematics(std::string link, const KDL::JntArray& joint_positions) const;
    template<class T> T forwardKinematics(std::string link, const sensor_msgs::JointState& state) const;
    std::map<std::string, KDL::Frame> fullForwardKinematics(const KDL::JntArray& joint_positions) {return fullForwardKinematicsImpl(joint_positions); }
    std::map<std::string, KDL::Frame> fullForwardKinematics(const sensor_msgs::JointState& state) {return fullForwardKinematics(jointMsgToKdl(state)); }
    const std::map<std::string, uint32_t>& getJointIndexMap() const { return joint_index_map_; }
    const std::map<std::string, std::vector<std::string> >& getJointSegmentMap() const { return joint_segment_map_; }
    const std::map<std::string, int>& getJointHierachy() const {return joint_hierachy_;}
    const std::vector<std::string>& getJointNames() const { return joint_names_; }
    const std::vector<int>& getJointIndices() const { return joint_indices_; }
    const std::vector<std::string>& getLinkJointNames() const { return link_joint_names_; }
    const std::vector<int>& getLinkJointIndices() const { return link_joint_indices_; }
    const std::vector<double>& getLinkJointLowerLimits() const { return link_joint_lower_limits_; }
    const std::vector<double>& getLinkJointUpperLimits() const { return link_joint_upper_limits_; }
    const std::map<std::string, KDL::Frame>& getSegmentsTf() const { return seg_tf_map_; }
    std::string getBaselinkName() const { return baselink_; }
    void setBaselinkName(const std::string baselink) { baselink_ = baselink; }
    template<class T> T getCog() const;
    template<class T> T getCogDesireOrientation() const;
    template<class T> T getCog2Baselink() const;
    template<class T> T getInertia() const;
    std::map<std::string, KDL::RigidBodyInertia> getInertiaMap() const { return inertia_map_; }
    double getLinkLength() const { return link_length_; }
    double getMass() const { return mass_; }
    const std::map<int, int>& getRotorDirection() { return rotor_direction_; }
    std::string getRootFrameName() const { return GetTreeElementSegment(tree_.getRootSegment()->second).getName(); }
    int getJointNum() const { return joint_num_; }
    int getRotorNum() const { return rotor_num_; }
    template<class T> std::vector<T> getRotorsNormalFromCog() const;
    template<class T> std::vector<T> getRotorsOriginFromCog() const;
    const urdf::Model& getUrdfModel() const { return model_; }
    const KDL::Tree& getTree() const { return tree_; }
    double getVerbose() const { return verbose_; }
    void setCogDesireOrientation(double roll, double pitch, double yaw) { setCogDesireOrientation(KDL::Rotation::RPY(roll, pitch, yaw)); }
    void setCogDesireOrientation(KDL::Rotation cog_desire_orientation) { cog_desire_orientation_  = cog_desire_orientation;}
    bool removeExtraModule(std::string module_name);
    virtual void updateRobotModelImpl(const KDL::JntArray& joint_positions);
    void updateRobotModel(const KDL::JntArray& joint_positions) { updateRobotModelImpl(joint_positions); }
    void updateRobotModel(const sensor_msgs::JointState& state) { updateRobotModel(jointMsgToKdl(state)); }
    KDL::JntArray jointMsgToKdl(const sensor_msgs::JointState& state) const;
    sensor_msgs::JointState kdlJointToMsg(const KDL::JntArray& joint_positions) const;

    virtual void updateStatics(bool verbose = false) {}
    virtual bool stabilityCheck(bool verbose = false) {return true;}
    inline const double getThrustUpperLimit() const {return thrust_max_;}
    inline const double getThrustLowerLimit() const {return thrust_min_;}
    inline const double getMFRate() const  {return m_f_rate_;}

    static TiXmlDocument getRobotModelXml(const std::string& param);

  private:
    //private attributes
    std::map<std::string, uint32_t> joint_index_map_; // index in KDL::JntArray
    std::map<std::string, std::vector<std::string> > joint_segment_map_;
    std::map<std::string, int> joint_hierachy_;

    std::vector<std::string> joint_names_;
    std::vector<int> joint_indices_; // index in KDL::JntArray
    std::vector<std::string> link_joint_names_;
    std::vector<int> link_joint_indices_; // index in KDL::JntArray
    std::vector<double> link_joint_lower_limits_, link_joint_upper_limits_;


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
    int joint_num_;
    int rotor_num_;
    std::vector<KDL::Vector> rotors_origin_from_cog_;
    std::vector<KDL::Vector> rotors_normal_from_cog_;
    KDL::Tree tree_;
    std::string thrust_link_;
    double thrust_max_;
    double thrust_min_;
    double m_f_rate_; //moment / force rate
    bool verbose_;

    //private functions
    KDL::Frame forwardKinematicsImpl(std::string link, const KDL::JntArray& joint_positions) const
    {
      if (joint_positions.rows() != tree_.getNrOfJoints())
        throw std::runtime_error("joint num is invalid");

      KDL::TreeFkSolverPos_recursive fk_solver(tree_);
      KDL::Frame f;
      int status = fk_solver.JntToCart(joint_positions, f, link);
      if(status < 0) ROS_ERROR("can not solve FK to link: %s", link.c_str());

      return f;
    }
    std::map<std::string, KDL::Frame> fullForwardKinematicsImpl(const KDL::JntArray& joint_positions);
    void getParamFromRos();
    KDL::RigidBodyInertia inertialSetup(const KDL::TreeElement& tree_element);
    void makeJointSegmentMap();
    void jointSegmentSetupRecursive(const KDL::TreeElement& tree_element, std::vector<std::string> current_joints);
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
