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

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "tf_conversions/tf_eigen.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <iomanip>
#include <cmath>

#include <aerial_robot_base/DesireCoord.h>
#include <aerial_robot_msgs/FourAxisGain.h>
#include <hydrus_transform_control/RollPitchYawGain.h>
#include <hydrus_transform_control/RollPitchYawGains.h>
#include <hydrus_transform_control/AddExtraModule.h>

#include <string>
//* for eigen cumputation
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>

#include <std_msgs/Float32.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

//* for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <hydrus_transform_control/LQIConfig.h>
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

class ElementModel{
public:
  ElementModel(){}
  ElementModel(int link, float weight, Eigen::Matrix3d inertia, Eigen::Vector3d origin_offset = Eigen::Vector3d::Zero(), bool verbose = false)
  {
    weight_ = weight;
    inertia_ = inertia;
    origin_offset_ = origin_offset;
    link_ = link;

    if(verbose)
      {
        std::cout << "weight:" << std::endl;
        std::cout << weight_ << std::endl;
        std::cout << "inertia:" << std::endl;
        std::cout << inertia_ << std::endl;
        std::cout << "offset:" << std::endl;
        std::cout << origin_offset_ << std::endl;
      }
  }
  ~ElementModel(){}

  inline float getWeight(){return weight_;}
  inline Eigen::Matrix3d getInertia(){return inertia_;}
  inline  void setOrigin(Eigen::Vector3d origin){origin_ = origin;}
  inline Eigen::Vector3d getOffset(){return origin_offset_;}
  inline Eigen::Vector3d getOrigin(){return origin_;}
  inline int getLink(){return link_;}

protected:
  float weight_;
  Eigen::Matrix3d inertia_;
  Eigen::Vector3d origin_;
  Eigen::Vector3d origin_offset_;
  int link_;
};

class TransformController{
public:
  TransformController(ros::NodeHandle nh, ros::NodeHandle nh_private, bool callback_flag = true);
  ~TransformController();

  void realtimeControlCallback(const std_msgs::UInt8ConstPtr & msg);
  void yawGainCallback(const std_msgs::UInt8ConstPtr & msg);

  void addExtraModule(int extra_module_link, float extra_module_mass, float extra_module_offset);
  void cogComputation(const std::vector<tf::StampedTransform>& transforms);
  void principalInertiaComputation(const std::vector<tf::StampedTransform>& transforms, bool continuous_flag = true);

  bool distThreCheck();
  bool distThreCheckFromJointValues(const std::vector<double>& joint_values, int joint_offset = 0,bool continous_flag = true);
  std::vector<tf::StampedTransform> transformsFromJointValues(const std::vector<double>& joint_values, int joint_offset = 0);

  bool stabilityCheck(bool verbose = false); //lagrange method

  bool hamiltonMatrixSolver(uint8_t lqi_mode);

  inline double getLinkLength(){return link_length_;}
  inline int getLinkNum(){return link_num_;}

  void getLinksOriginFromCog(std::vector<Eigen::Vector3d>& links_origin_from_cog)
  {
    boost::lock_guard<boost::mutex> lock(origins_mutex_);
    int size = links_origin_from_cog_.size();
    for(int i=0; i< size; i++)
      links_origin_from_cog = links_origin_from_cog_;

  }

  void setLinksOriginFromCog(const std::vector<Eigen::Vector3d>& links_origin_from_cog)
  {
    boost::lock_guard<boost::mutex> lock(origins_mutex_);
    links_origin_from_cog_ = links_origin_from_cog;
  }

  Eigen::Matrix3d getPrincipalInertia()
  {
    boost::lock_guard<boost::mutex> lock(inertia_mutex_);
    return links_principal_inertia_;
  }
  void setPrincipalInertia(Eigen::Matrix3d principal_inertia)
  {
    boost::lock_guard<boost::mutex> lock(inertia_mutex_);
    links_principal_inertia_ = principal_inertia;
  }

  Eigen::Matrix3d getRotateMatrix()
  {
    boost::lock_guard<boost::mutex> lock(rm_mutex_);
    return rotate_matrix_;
  }

  void setRotateMatrix(Eigen::Matrix3d rotate_matrix)
  {
    boost::lock_guard<boost::mutex> lock(rm_mutex_);
    rotate_matrix_ = rotate_matrix;
  }

  Eigen::Vector3d getCog()
  {
    boost::lock_guard<boost::mutex> lock(cog_mutex_);
    return cog_;
  }

  void setCog(Eigen::Vector3d cog)
  {
    boost::lock_guard<boost::mutex> lock(cog_mutex_);
    cog_ = cog;
  }

  Eigen::MatrixXd getU()
  {
    return U_;
  }

  Eigen::MatrixXd getK()
  {
    if(lqi_mode_ == LQI_THREE_AXIS_MODE) return K9_; 
    if(lqi_mode_ == LQI_FOUR_AXIS_MODE) return K12_; 
  }

  void setK(Eigen::MatrixXd K, uint8_t lqi_mode)
  {
    lqi_mode_ = lqi_mode;
    if(lqi_mode_ == LQI_THREE_AXIS_MODE) K9_ = K; 
    if(lqi_mode_ == LQI_FOUR_AXIS_MODE) K12_ = K; 
  }

  inline uint8_t getLqiMode()
  {
    return lqi_mode_;
  }

  void setLqiMode(uint8_t lqi_mode)
  {
    lqi_mode_ = lqi_mode;
  }

  //really  bad!!
  void setRotateAngle(float angle_cos, float angle_sin)
  {
    cog_matrix_(0, 0) = angle_cos;
    cog_matrix_(1, 0) = angle_sin;
  }
  void getRotateAngle(float& angle_cos, float& angle_sin)
  {
    angle_cos = cog_matrix_(0, 0);
    angle_sin = cog_matrix_(1, 0);
  }

  void param2contoller();

  //only for link4
  double getUDeterminant(){return U_.determinant();}

  const static uint8_t LQI_FOUR_AXIS_MODE = 0;
  const static uint8_t LQI_THREE_AXIS_MODE = 1;

private:

  ros::NodeHandle nh_,nh_private_;
  ros::Publisher rpy_gain_pub_;
  ros::Publisher four_axis_gain_pub_;
  ros::Publisher principal_axis_pub_;
  ros::Publisher cog_rotate_pub_; //for position control => to mocap
  ros::Subscriber realtime_control_sub_;
  ros::Subscriber yaw_gain_sub_;

  boost::mutex rm_mutex_, cog_mutex_, origins_mutex_, inertia_mutex_;

  tf::TransformListener tf_;
  bool callback_flag_;

  bool realtime_control_flag_;

  bool kinematic_verbose_;
  bool control_verbose_;
  bool debug_verbose_;

  /* control */
  boost::thread control_thread_;
  double control_rate_;
  bool only_three_axis_mode_;

  /* base model config */
  int link_num_;
  int joint_num_;
  std::vector<int> rotor_direction_;

  std::string rpy_gain_pub_name_;
  std::string yaw_pos_gain_sub_name_;


  //dynamics config
  double m_f_rate_; //moment / force rate
  double f_pwm_rate_; //force / pwm rate
  double f_pwm_offset_; //force / pwm offset
  double pwm_rate_; //percentage

  double link_length_;
  double link_base_rod_length_; //the length of the lik base rod
  //double link_ring_diameter_; //the diameter of the protector
  double ring_radius_; //the offset from the center of link base to the protector_end
  double link_joint_offset_; //the offset from the center of joint to the mass center of joint

  double link_base_rod_common_mass_; //link rod
  std::vector<double> link_base_rod_mass_; //each weight of link base rod is different
  double link_base_ring_mass_; //protector ring
  double link_base_two_ring_holder_mass_; //protector holder
  double link_base_center_mass_; //motor+esc+battery+etc
  double link_joint_mass_; //two_holder + servo motor
  double all_mass_;
  int extra_module_num_;
  std::vector<ElementModel> link_base_model_;
  std::vector<ElementModel> link_joint_model_;
  std::vector<ElementModel> extra_module_model_;

  std::string root_link_name_;
  int root_link_;
  std::vector<std::string> links_name_;

  Eigen::Vector3d cog_;
  std::vector<Eigen::Vector3d> links_origin_from_cog_; 

  Eigen::Matrix3d links_inertia_, links_principal_inertia_;

  Eigen::Matrix3d rotate_matrix_;
  Eigen::Matrix3d cog_matrix_;
  double rotate_angle_;

  /* ros param init */
  void initParam();
  /* main control func */
  void control();
  /* kinematics calculation */
  bool kinematics();
  /* LQI parameter calculation */
  void lqi();

  void visualization();
  void cogCoordPublish();

  /* dynamic reconfigure */
  void cfgLQICallback(hydrus_transform_control::LQIConfig &config, uint32_t level);

  /* service */
  bool addExtraModuleCallback(hydrus_transform_control::AddExtraModule::Request  &req,
                      hydrus_transform_control::AddExtraModule::Response &res);

  int sgn(double value){ return  (value / fabs(value));}

  //8/12:r,r_d, p, p_d, y, y_d, z. z_d, r_i, p_i, y_i, z_i
  //6/9:r,r_d, p, p_d, z. z_d, r_i, p_i, z_i
  Eigen::MatrixXd U_;

  Eigen::MatrixXd A8_;
  Eigen::MatrixXd B8_;
  Eigen::MatrixXd C8_;
  Eigen::MatrixXd A12_aug_;
  Eigen::MatrixXd B12_aug_;
  Eigen::MatrixXd C12_aug_;
  Eigen::MatrixXd Q12_;

  Eigen::MatrixXd A6_;
  Eigen::MatrixXd B6_;
  Eigen::MatrixXd C6_;
  Eigen::MatrixXd A9_aug_;
  Eigen::MatrixXd B9_aug_;
  Eigen::MatrixXd C9_aug_;
  Eigen::MatrixXd Q9_;

  Eigen::MatrixXd R_;

  Eigen::MatrixXd K12_;
  Eigen::MatrixXd K9_;

  //Q
  double q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_yaw_,strong_q_yaw_, q_yaw_d_,q_z_,q_z_d_;
  double q_roll_i_,q_pitch_i_,q_yaw_i_,q_z_i_;

  //R
  std::vector<double> r_;
  double dist_thre_;
  double f_max_, f_min_;

  uint8_t lqi_mode_;
  bool a_dash_eigen_calc_flag_;


  //dynamic reconfigure
  dynamic_reconfigure::Server<hydrus_transform_control::LQIConfig>* lqi_server_;
  dynamic_reconfigure::Server<hydrus_transform_control::LQIConfig>::CallbackType dynamic_reconf_func_lqi_;

  //service
  ros::ServiceServer add_extra_module_service_;
};


#endif
