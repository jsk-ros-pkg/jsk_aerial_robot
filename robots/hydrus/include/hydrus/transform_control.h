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


#ifndef TRANSFORM_CONTROL_H
#define TRANSFORM_CONTROL_H

/* ros */
#include <ros/ros.h>

#include <spinal/RollPitchYawTerms.h>
#include <aerial_robot_msgs/FourAxisGain.h>
#include <sensor_msgs/JointState.h>
#include <hydrus/AddExtraModule.h>
#include <spinal/DesireCoord.h>
#include <spinal/PMatrixPseudoInverseWithInertia.h>
#include <std_msgs/UInt8.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf_conversions/tf_kdl.h>
#include <eigen_conversions/eigen_msg.h>

/* robot model */
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

/* for eigen cumputation */
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

/* util */
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <string>
#include <iostream>
#include <iomanip>
#include <cmath>


/* for dynamic reconfigure */
#include <dynamic_reconfigure/server.h>
#include <hydrus/LQIConfig.h>
#define LQI_GAIN_FLAG 0
#define LQI_RP_P_GAIN 1
#define LQI_RP_I_GAIN 2
#define LQI_RP_D_GAIN 3
#define LQI_Y_P_GAIN 4
#define LQI_Y_I_GAIN 5
#define LQI_Y_D_GAIN 6
#define LQI_Z_P_GAIN 7
#define LQI_Z_I_GAIN 8
#define LQI_Z_D_GAIN 9


class TransformController{
public:
  TransformController(ros::NodeHandle nh, ros::NodeHandle nh_private, bool callback_flag = true);
  ~TransformController();

  bool addExtraModule(int action, std::string module_name, std::string parent_link_name, geometry_msgs::Transform transform, geometry_msgs::Inertia inertia);

  double distThreCheck(bool verbose = false);
  bool modelling(bool verbose = false); //lagrange method

  bool hamiltonMatrixSolver(uint8_t lqi_mode);

  inline int getRotorNum(){return rotor_num_;}

  const Eigen::Vector3d& getRotorOirginFromCog(int index)
  {
    boost::lock_guard<boost::mutex> lock(origins_mutex_);
    return rotors_origin_from_cog_.at(index);
  }

  const std::vector<Eigen::Vector3d>& getRotorsOriginFromCog()
  {
    boost::lock_guard<boost::mutex> lock(origins_mutex_);
    return rotors_origin_from_cog_;
  }

  void setRotorsOriginFromCog(const std::vector<Eigen::Vector3d>& rotors_origin_from_cog)
  {
    boost::lock_guard<boost::mutex> lock(origins_mutex_);
    assert(rotors_origin_from_cog_.size() == rotors_origin_from_cog.size());
    rotors_origin_from_cog_ = rotors_origin_from_cog;
  }

  void setCog2Baselink(tf::Transform transform)
  {
    cog2baselink_transform_ = transform;
  }

  tf::Transform getCog2Baselink()
  {
    return cog2baselink_transform_;
  }

  tf::Transform getRoot2Link(std::string link, sensor_msgs::JointState state);

  inline void setBaselink(std::string baselink) { baselink_ = baselink;}

  Eigen::Matrix3d getInertia()
  {
    boost::lock_guard<boost::mutex> lock(inertia_mutex_);
    return links_inertia_;
  }
  void setInertia(Eigen::Matrix3d link_inertia)
  {
    boost::lock_guard<boost::mutex> lock(inertia_mutex_);
    links_inertia_ = link_inertia;
  }

  double getMass()
  {
    boost::lock_guard<boost::mutex> lock(mass_mutex_);
    return mass_;
  }

  void setMass(double mass)
  {
    boost::lock_guard<boost::mutex> lock(mass_mutex_);
    mass_ = mass;
  }

  tf::Transform getCog()
  {
    boost::lock_guard<boost::mutex> lock(cog_mutex_);
    return cog_;
  }

  void setCog(tf::Transform cog)
  {
    boost::lock_guard<boost::mutex> lock(cog_mutex_);
    cog_ = cog;
  }

  inline void setStableState(Eigen::VectorXd x) {stable_x_ = x;}
  const Eigen::VectorXd& getStableState() const {return stable_x_;}

  const Eigen::MatrixXd& getP() const  { return P_; }
  const Eigen::MatrixXd& getK() const { return K_; }
  inline uint8_t getLqiMode() { return lqi_mode_; }
  inline void setLqiMode(uint8_t lqi_mode) { lqi_mode_ = lqi_mode; }
  inline double getFMax() {return f_max_;}
  inline double getFMin() {return f_min_;}

  void setCogDesireOrientation(KDL::Rotation cog_desire_orientation)
  {
    cog_desire_orientation_ = cog_desire_orientation;
  }

  virtual void kinematics(sensor_msgs::JointState state);
  void param2controller();

  virtual bool overlapCheck(bool verbose = false){return true; }

  static constexpr uint8_t LQI_THREE_AXIS_MODE = 3;
  static constexpr uint8_t LQI_FOUR_AXIS_MODE = 4;

protected:

  ros::NodeHandle nh_,nh_private_;
  ros::Publisher rpy_gain_pub_;
  ros::Publisher four_axis_gain_pub_;
  ros::Publisher principal_axis_pub_;
  ros::Publisher cog_rotate_pub_; //for position control => to mocap
  ros::Publisher transform_pub_;
  ros::Publisher p_matrix_pseudo_inverse_inertia_pub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber desire_coordinate_sub_;
  ros::Subscriber realtime_control_sub_;

  boost::mutex mass_mutex_, cog_mutex_, origins_mutex_, inertia_mutex_;

  tf::TransformBroadcaster br_;
  bool callback_flag_;

  bool realtime_control_flag_;
  bool kinematics_flag_;

  bool kinematic_verbose_;
  bool control_verbose_;
  bool debug_verbose_;
  bool verbose_;

  /* robot model, kinematics */
  urdf::Model model_;
  KDL::Tree tree_;
  std::map<std::string, KDL::RigidBodyInertia> inertia_map_;
  tf::Transform cog2baselink_transform_;
  std::map<std::string, uint32_t> joint_map_;
  std::map<std::string /* module_name */, KDL::Segment> extra_module_map_;

  /* control */
  boost::thread control_thread_;
  double control_rate_;
  bool only_three_axis_mode_;
  bool gyro_moment_compensation_;

  /* base model config */
  int rotor_num_;
  std::string baselink_;
  std::string thrust_link_;

  /* dynamics */
  std::map<int, int> rotor_direction_;
  double m_f_rate_; //moment / force rate

  /* kinematics */
  double mass_;
  tf::Transform cog_;
  std::vector<Eigen::Vector3d> rotors_origin_from_cog_;
  Eigen::Matrix3d links_inertia_;
  ros::Time joint_state_stamp_;

  /* ros param init */
  void initParam();
  /* main control func */
  virtual void control();
  /* LQI parameter calculation */
  void lqi();

  /* dynamic reconfigure */
  void cfgLQICallback(hydrus::LQIConfig &config, uint32_t level);

  /* service */
  bool addExtraModuleCallback(hydrus::AddExtraModule::Request  &req,
                      hydrus::AddExtraModule::Response &res);

  void realtimeControlCallback(const std_msgs::UInt8ConstPtr & msg);
  void desireCoordinateCallback(const spinal::DesireCoordConstPtr & msg);
  virtual void jointStateCallback(const sensor_msgs::JointStateConstPtr& state);
  Eigen::MatrixXd P_;
  Eigen::MatrixXd K_;
  Eigen::MatrixXd P_orig_pseudo_inverse_; // for compensation of cross term in the rotional dynamics

  //Q
//8/12:r,r_d, p, p_d, y, y_d, z. z_d, r_i, p_i, y_i, z_i
  //6/9:r,r_d, p, p_d, z. z_d, r_i, p_i, z_i
  Eigen::VectorXd q_diagonal_;
  double q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_yaw_,strong_q_yaw_, q_yaw_d_,q_z_,q_z_d_;
  double q_roll_i_,q_pitch_i_,q_yaw_i_,q_z_i_;

  //R
  std::vector<double> r_;
  double var_thre_;
  double correlation_thre_;
  double f_max_, f_min_;

  uint8_t lqi_mode_;
  bool a_dash_eigen_calc_flag_;

  //stable state
  Eigen::VectorXd stable_x_;

  //cog desire orientation
  KDL::Rotation cog_desire_orientation_;

  //dynamic reconfigure
  dynamic_reconfigure::Server<hydrus::LQIConfig>* lqi_server_;
  dynamic_reconfigure::Server<hydrus::LQIConfig>::CallbackType dynamic_reconf_func_lqi_;

  //service
  ros::ServiceServer add_extra_module_service_;

  void addChildren(const KDL::SegmentMap::const_iterator segment);
};


#endif
