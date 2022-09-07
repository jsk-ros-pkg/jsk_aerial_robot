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
#include <mutex>
#include <sensor_msgs/JointState.h>
#include <stdexcept>
#include <ros/ros.h>
#include <urdf/model.h>
#include <vector>

namespace aerial_robot_model {

  //Basic Aerial Robot Model
  class RobotModel {
  public:
    RobotModel(bool init_with_rosparam = true, bool verbose = false, double fc_f_min_thre = 0, double fc_t_min_thre = 0, double epsilon = 10.0);
    virtual ~RobotModel() = default;

    void updateRobotModel();
    void updateRobotModel(const KDL::JntArray& joint_positions);
    void updateRobotModel(const sensor_msgs::JointState& state);

    // kinematics
    const bool initialized() const { return initialized_; }
    const std::string getBaselinkName() const { return baselink_; }
    const std::map<std::string, KDL::RigidBodyInertia>& getInertiaMap() const { return inertia_map_; }
    const double getMass() const { return mass_; }
    const int getRotorNum() const { return rotor_num_; }
    const std::map<int, int>& getRotorDirection() { return rotor_direction_; }
    const std::string getRootFrameName() const { return GetTreeElementSegment(tree_.getRootSegment()->second).getName(); }
    const int getJointNum() const { return joint_num_;}
    const KDL::JntArray& getJointPositions() const { return joint_positions_; }
    const std::map<std::string, uint32_t>& getJointIndexMap() const { return joint_index_map_; }
    const std::map<std::string, std::vector<std::string> >& getJointSegmentMap() const { return joint_segment_map_; }
    const std::map<std::string, int>& getJointHierachy() const {return joint_hierachy_;}
    const std::vector<std::string>& getJointNames() const { return joint_names_; }
    const std::vector<int>& getJointIndices() const { return joint_indices_; }
    const std::vector<std::string>& getJointParentLinkNames() const { return joint_parent_link_names_; }

    const std::map<std::string, KDL::Frame> getSegmentsTf() {
      std::lock_guard<std::mutex> lock(mutex_seg_tf_);
      return seg_tf_map_;
    }

    const KDL::Frame getSegmentTf(const std::string seg_name) {
      std::lock_guard<std::mutex> lock(mutex_seg_tf_);
      return seg_tf_map_.at(seg_name);
    }

    template<class T> T forwardKinematics(std::string link, const KDL::JntArray& joint_positions) const;
    template<class T> T forwardKinematics(std::string link, const sensor_msgs::JointState& state) const;
    std::map<std::string, KDL::Frame> fullForwardKinematics(const KDL::JntArray& joint_positions) {return fullForwardKinematicsImpl(joint_positions); }
    std::map<std::string, KDL::Frame> fullForwardKinematics(const sensor_msgs::JointState& state) {return fullForwardKinematics(jointMsgToKdl(state)); }

    const KDL::Tree& getTree() const { return tree_; }
    const urdf::Model& getUrdfModel() const { return model_; }
    const double getVerbose() const { return verbose_; }

    template<class T> T getCog();
    template<class T> T getCogDesireOrientation();
    template<class T> T getCog2Baselink();
    template<class T> T getInertia();
    template<class T> std::vector<T> getRotorsNormalFromCog();
    template<class T> std::vector<T> getRotorsOriginFromCog();
    static TiXmlDocument getRobotModelXml(const std::string param, ros::NodeHandle nh = ros::NodeHandle());

    KDL::JntArray jointMsgToKdl(const sensor_msgs::JointState& state) const;
    sensor_msgs::JointState kdlJointToMsg(const KDL::JntArray& joint_positions) const;

    void setBaselinkName(const std::string baselink) { baselink_ = baselink; }
    void setCogDesireOrientation(double roll, double pitch, double yaw)
    {
      setCogDesireOrientation(KDL::Rotation::RPY(roll, pitch, yaw));
    }
    void setCogDesireOrientation(const KDL::Rotation cog_desire_orientation)
    {
      std::lock_guard<std::mutex> lock(mutex_desired_baselink_rot_);
      cog_desire_orientation_  = cog_desire_orientation;
    }

    bool addExtraModule(std::string module_name, std::string parent_link_name, KDL::Frame transform, KDL::RigidBodyInertia inertia);
    bool removeExtraModule(std::string module_name);

    // statics (static thrust, joint torque)
    Eigen::VectorXd calcGravityWrenchOnRoot();
    virtual void calcStaticThrust();
    Eigen::MatrixXd calcWrenchMatrixOnCoG();
    virtual void calcWrenchMatrixOnRoot();

    const Eigen::VectorXd& getGravity() const {return gravity_;}
    const Eigen::VectorXd& getGravity3d() const {return gravity_3d_;}
    const double getMFRate() const  {return m_f_rate_;}
    const Eigen::VectorXd& getStaticThrust() const {return static_thrust_;}
    const std::vector<Eigen::MatrixXd>& getThrustWrenchAllocations() const {return thrust_wrench_allocations_;}
    const Eigen::MatrixXd& getThrustWrenchMatrix() const {return q_mat_;}
    const std::vector<Eigen::VectorXd>& getThrustWrenchUnits() const {return thrust_wrench_units_;}
    const double getThrustUpperLimit() const {return thrust_max_;}
    const double getThrustLowerLimit() const {return thrust_min_;}

    // control stability
    virtual void calcFeasibleControlFDists();
    virtual void calcFeasibleControlTDists();
    double calcTripleProduct(const Eigen::Vector3d& ui, const Eigen::Vector3d& uj, const Eigen::Vector3d& uk);
    std::vector<Eigen::Vector3d> calcV();
    const double getEpsilon() const {return epsilon_;}
    const Eigen::VectorXd& getFeasibleControlFDists() const {return fc_f_dists_;}
    const double& getFeasibleControlFMin()  const {return fc_f_min_;}
    const double& getFeasibleControlFMinThre()  const {return fc_f_min_thre_;}
    const Eigen::VectorXd& getFeasibleControlTDists() const {return fc_t_dists_;}
    const double& getFeasibleControlTMin()  const {return fc_t_min_;}
    const double& getFeasibleControlTMinThre()  const {return fc_t_min_thre_;}

    const void setFeasibleControlFMinThre(const double fc_f_min_thre)  { fc_f_min_thre_ = fc_f_min_thre;}
    const void setFeasibleControlTMinThre(const double fc_t_min_thre)  { fc_t_min_thre_ = fc_t_min_thre;}

    virtual bool stabilityCheck(bool verbose = true);

    KDL::JntArray convertEigenToKDL(const Eigen::VectorXd& joint_vector);

  private:

    // kinematics
    bool initialized_;
    double mass_;
    urdf::Model model_;
    std::string baselink_;
    KDL::Frame cog_;
    KDL::Rotation cog_desire_orientation_;
    KDL::Frame cog2baselink_transform_;

    std::vector<std::string> joint_names_; // index in KDL::JntArray
    std::vector<int> joint_indices_; // index in KDL::JntArray
    std::vector<std::string> joint_parent_link_names_; // index in KDL::JntArray
    KDL::JntArray joint_positions_;
    KDL::RotationalInertia link_inertia_cog_;
    std::map<std::string, KDL::Segment> extra_module_map_;
    std::map<std::string, KDL::RigidBodyInertia> inertia_map_;
    std::map<std::string, uint32_t> joint_index_map_; // index in KDL::JntArray
    std::map<std::string, std::vector<std::string> > joint_segment_map_;
    std::map<std::string, int> joint_hierachy_;
    std::map<std::string, KDL::Frame> seg_tf_map_;
    int joint_num_;
    int rotor_num_;
    std::vector<KDL::Vector> rotors_origin_from_cog_;
    std::vector<KDL::Vector> rotors_normal_from_cog_;
    KDL::Tree tree_;
    std::string thrust_link_;
    bool verbose_;


    // statics (static thrust, joint torque)
    Eigen::VectorXd gravity_;
    Eigen::VectorXd gravity_3d_;
    double m_f_rate_; //moment / force rate
    Eigen::MatrixXd q_mat_;
    std::map<int, int> rotor_direction_;
    Eigen::VectorXd static_thrust_;
    double thrust_max_;
    double thrust_min_;
    std::vector<Eigen::VectorXd> thrust_wrench_units_;
    std::vector<Eigen::MatrixXd> thrust_wrench_allocations_;

    // control stability
    double epsilon_;
    Eigen::VectorXd fc_f_dists_; // distances to the plane of feasible control force convex
    Eigen::VectorXd fc_t_dists_; // distances to the plane of feasible control torque convex
    double fc_f_min_;
    double fc_t_min_;
    double fc_f_min_thre_;
    double fc_t_min_thre_;

    // mutex
    std::mutex mutex_cog_;
    std::mutex mutex_cog2baselink_;
    std::mutex mutex_inertia_;
    std::mutex mutex_rotor_origin_;
    std::mutex mutex_rotor_normal_;
    std::mutex mutex_seg_tf_;
    std::mutex mutex_desired_baselink_rot_;


    //private functions
    void getParamFromRos();
    void kinematicsInit();
    void stabilityInit();
    void staticsInit();


    KDL::RigidBodyInertia inertialSetup(const KDL::TreeElement& tree_element);
    void jointSegmentSetupRecursive(const KDL::TreeElement& tree_element, std::vector<std::string> current_joints);
    void makeJointSegmentMap();

    KDL::Frame forwardKinematicsImpl(std::string link, const KDL::JntArray& joint_positions) const;
    std::map<std::string, KDL::Frame> fullForwardKinematicsImpl(const KDL::JntArray& joint_positions);

  protected:
    virtual void updateRobotModelImpl(const KDL::JntArray& joint_positions);

    void setCog(const KDL::Frame cog)
    {
      std::lock_guard<std::mutex> lock(mutex_cog_);
      cog_ = cog;
    }
    void setCog2Baselink(const KDL::Frame cog2baselink_transform)
    {
      std::lock_guard<std::mutex> lock(mutex_cog2baselink_);
      cog2baselink_transform_ = cog2baselink_transform;
    }
    void setInertia(const KDL::RotationalInertia inertia)
    {
      std::lock_guard<std::mutex> lock(mutex_inertia_);
      link_inertia_cog_ = inertia;
    }
   void setRotorsNormalFromCog(const std::vector<KDL::Vector> rotors_normal_from_cog)
    {
      std::lock_guard<std::mutex> lock(mutex_rotor_normal_);
      rotors_normal_from_cog_ = rotors_normal_from_cog;
    }
   void setRotorsOriginFromCog(const std::vector<KDL::Vector> rotors_origin_from_cog)
    {
      std::lock_guard<std::mutex> lock(mutex_rotor_origin_);
      rotors_origin_from_cog_ = rotors_origin_from_cog;
    }
    void setSegmentsTf(const std::map<std::string, KDL::Frame> seg_tf_map)
    {
      std::lock_guard<std::mutex> lock(mutex_seg_tf_);
      seg_tf_map_ = seg_tf_map;
    }

    void setStaticThrust(const Eigen::VectorXd static_thrust) {static_thrust_ = static_thrust;}
    void setThrustWrenchMatrix(const Eigen::MatrixXd q_mat) {q_mat_ = q_mat;}

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

  template<> inline KDL::Frame RobotModel::getCog()
  {
    std::lock_guard<std::mutex> lock(mutex_cog_);
    return cog_;
  }

  template<> inline Eigen::Affine3d RobotModel::getCog()
  {
    return aerial_robot_model::kdlToEigen(RobotModel::getCog<KDL::Frame>());
  }

  template<> inline geometry_msgs::TransformStamped RobotModel::getCog()
  {
    return aerial_robot_model::kdlToMsg(RobotModel::getCog<KDL::Frame>());
  }

  template<> inline tf2::Transform RobotModel::getCog()
  {
    return aerial_robot_model::kdlToTf2(RobotModel::getCog<KDL::Frame>());
  }

  template<> inline KDL::Frame RobotModel::getCog2Baselink()
  {
    std::lock_guard<std::mutex> lock(mutex_cog2baselink_);
    return cog2baselink_transform_;
  }

  template<> inline Eigen::Affine3d RobotModel::getCog2Baselink()
  {
    return aerial_robot_model::kdlToEigen(RobotModel::getCog2Baselink<KDL::Frame>());
  }

  template<> inline geometry_msgs::TransformStamped RobotModel::getCog2Baselink()
  {
    return aerial_robot_model::kdlToMsg(RobotModel::getCog2Baselink<KDL::Frame>());
  }

  template<> inline tf2::Transform RobotModel::getCog2Baselink()
  {
    return aerial_robot_model::kdlToTf2(RobotModel::getCog2Baselink<KDL::Frame>());
  }

  template<> inline KDL::Rotation RobotModel::getCogDesireOrientation()
  {
    std::lock_guard<std::mutex> lock(mutex_desired_baselink_rot_);
    return cog_desire_orientation_;
  }

  template<> inline Eigen::Matrix3d RobotModel::getCogDesireOrientation()
  {
    return aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>());
  }

  template<> inline KDL::RotationalInertia RobotModel::getInertia()
  {
    std::lock_guard<std::mutex> lock(mutex_inertia_);
    return link_inertia_cog_;
  }

  template<> inline Eigen::Matrix3d RobotModel::getInertia()
  {
    return aerial_robot_model::kdlToEigen(RobotModel::getInertia<KDL::RotationalInertia>());
  }

  template<> inline std::vector<KDL::Vector> RobotModel::getRotorsNormalFromCog()
  {
    std::lock_guard<std::mutex> lock(mutex_rotor_normal_);
    return rotors_normal_from_cog_;
  }

  template<> inline std::vector<Eigen::Vector3d> RobotModel::getRotorsNormalFromCog()
  {
    return aerial_robot_model::kdlToEigen(RobotModel::getRotorsNormalFromCog<KDL::Vector>());
  }

  template<> inline std::vector<geometry_msgs::PointStamped> RobotModel::getRotorsNormalFromCog()
  {
    return aerial_robot_model::kdlToMsg(RobotModel::getRotorsNormalFromCog<KDL::Vector>());
  }

  template<> inline std::vector<tf2::Vector3> RobotModel::getRotorsNormalFromCog()
  {
    return aerial_robot_model::kdlToTf2(RobotModel::getRotorsNormalFromCog<KDL::Vector>());
  }

  template<> inline std::vector<KDL::Vector> RobotModel::getRotorsOriginFromCog()
  {
    std::lock_guard<std::mutex> lock(mutex_rotor_origin_);
    return rotors_origin_from_cog_;
  }

  template<> inline std::vector<Eigen::Vector3d> RobotModel::getRotorsOriginFromCog()
  {
    return aerial_robot_model::kdlToEigen(RobotModel::getRotorsOriginFromCog<KDL::Vector>());
  }

  template<> inline std::vector<geometry_msgs::PointStamped> RobotModel::getRotorsOriginFromCog()
  {
    return aerial_robot_model::kdlToMsg(RobotModel::getRotorsOriginFromCog<KDL::Vector>());
  }

  template<> inline std::vector<tf2::Vector3> RobotModel::getRotorsOriginFromCog()
  {
    return aerial_robot_model::kdlToTf2(RobotModel::getRotorsOriginFromCog<KDL::Vector>());
  }
} // namespace aerial_robot_model
