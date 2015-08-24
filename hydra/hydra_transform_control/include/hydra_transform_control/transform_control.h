#ifndef TRANSFORM_CONTROL_H
#define TRANSFORM_CONTROL_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "tf_conversions/tf_eigen.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>

#include <string>
//* for eigen cumputation 
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/Eigenvalues>

#include <hydra_transform_control/HydraParam.h>
#include <std_msgs/UInt16.h>

#include <std_msgs/Float32.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

class ElementModel{
 public:
  ElementModel(){}
 ElementModel(ros::NodeHandle nh, ros::NodeHandle nh_private, float weight, Eigen::Matrix3d inertia, Eigen::Vector3d origin_offset = Eigen::Vector3d::Zero()) : nh_(nh), nhp_(nh_private)
    {
      weight_ = weight;
      inertia_ = inertia;
      origin_offset_ = origin_offset;
    }
  ~ElementModel(){}

  inline float getWeight(){return weight_;}
  inline Eigen::Matrix3d getInertia(){return inertia_;}
  inline  void setOrigin(Eigen::Vector3d origin){origin_ = origin;}
  inline Eigen::Vector3d getOffset(){return origin_offset_;}
  inline Eigen::Vector3d getOrigin(){return origin_;}

 protected:
  ros::NodeHandle nh_, nhp_;
  float weight_;
  Eigen::Matrix3d inertia_;
  Eigen::Vector3d origin_;
  Eigen::Vector3d origin_offset_;
};


class TransformController{
 public:
  TransformController(ros::NodeHandle nh, ros::NodeHandle nh_private, bool callback_flag = true);
  ~TransformController();

  void cogComputation(std::vector<tf::StampedTransform> transforms);
  void principalInertiaComputation(std::vector<tf::StampedTransform> transforms, bool continuous_flag = true);

  double getLinkLength();
  double getPropellerDiameter(); 
  int getLinkNum();


  std::vector<Eigen::Vector3d> links_origin_from_cog_; 
  std::vector<Eigen::Vector3d> getLinksOriginFromCog()
    {
      boost::lock_guard<boost::mutex> lock(origins_mutex_);
      return links_origin_from_cog_;
    }
  void setLinksOriginFromCog(std::vector<Eigen::Vector3d> links_origin_from_cog)
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


 private:

  ros::NodeHandle nh_,nh_private_;
  ros::Publisher transform_control_pub_;
  ros::Publisher principal_axis_pub_;
  ros::Publisher cog_rotate_pub_; //for position control => to mocap

  boost::mutex rm_mutex_, cog_mutex_, origins_mutex_, inertia_mutex_;

  tf::TransformListener tf_;
  double control_rate_;
  double tf_pub_rate_;
  ros::Timer  control_timer_;
  ros::Timer  tf_pub_timer_;

  ros::Time system_tf_time_;

  bool debug_log_;
  bool debug2_log_;

  //base model config

  int link_num_;
  std::vector<int> propeller_direction_;
  std::vector<int> propeller_order_;

  //dynamics config
  double m_f_rate_; //moment / force rate
  double f_pwm_rate_; //force / pwm rate
  double f_pwm_offset_; //force / pwm offset

  double link_length_;
  double link_base_rod_length_; //the length of the lik base rod
  //double link_ring_diameter_; //the diameter of the protector
  double link_base_two_end_offset_; //the offset from the center of link base to the protector_end
  double link_joint_offset_; //the offset from the center of joint to the mass center of joint
  double controller1_offset_; //the offset from the pos of controller1 to the root link
  double controller2_offset_; //the offset from the pos of controller2 to the related link

  double link_base_rod_mid_mass_; //protector ring
  std::vector<double> link_base_rod_mass_; //each weight of link base rod is different
  double link_base_ring_mass_; //protector ring
  double link_base_two_end_mass_; //protector holder
  double link_base_center_mass_; //motor+esc+battery+etc
  double link_joint_mass_; //two_holder + servo motor
  double controller1_mass_; //kduino
  double controller2_mass_; //odroid
  double all_mass_;
  std::vector<ElementModel> link_base_model_;
  std::vector<ElementModel> link_joint_model_;
  std::vector<ElementModel> controller_model_;

  std::string root_link_name_;
  std::vector<std::string> links_name_;

  Eigen::Vector3d cog_;

  Eigen::Matrix3d links_inertia_, links_principal_inertia_;

  Eigen::Matrix3d rotate_matrix_;
  Eigen::Matrix3d cog_matrix_;
  double rotate_angle_;




  void controlFunc(const ros::TimerEvent & e);  
  void tfPubFunc(const ros::TimerEvent & e);
  void visualization();
  int sgn(double value){ return  (value / fabs(value));}

  void cogCoordPublish();
  void initParam();

  

  void param2contoller();


  //lqi
  boost::thread lqi_thread_;
  bool lqi_flag_;
  double lqi_thread_rate_;
  void lqi();
  bool hamiltonMatrixSolver();
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd C_;
  Eigen::MatrixXd A_aug_;
  Eigen::MatrixXd B_aug_;
  Eigen::MatrixXd C_aug_;

  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;

  //Q
  double q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_yaw_,q_yaw_d_,q_z_,q_z_d_;
  double q_roll_i_,q_pitch_i_,q_yaw_i_,q_z_i_;
  
  //R
  std::vector<double> r_;


  //shift
  double alfa_;
};


#endif
