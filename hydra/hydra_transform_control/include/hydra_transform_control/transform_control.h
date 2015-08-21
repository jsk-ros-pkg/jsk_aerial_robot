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

#include <hydra_transform_control/HydraParam.h>
#include <std_msgs/UInt16.h>

#include <std_msgs/Float32.h>
#include <boost/thread/mutex.hpp>

class ElementModel{
 public:
  ElementModel(){}
 ElementModel(ros::NodeHandle nh, ros::NodeHandle nh_private, float weight, Eigen::Matrix<double, 3, 3> inertia, Eigen::Matrix<double, 3, 1> origin_offset = Eigen::Vector3d::Zero()) : nh_(nh), nhp_(nh_private)
    {
      weight_ = weight;
      inertia_ = inertia;
      origin_offset_ = origin_offset;
    }
  ~ElementModel(){}

  inline float getWeight(){return weight_;}
  inline Eigen::Matrix<double, 3, 3> getInertia(){return inertia_;}
  inline  void setOrigin(Eigen::Matrix<double, 3, 1> origin){origin_ = origin;}
  inline Eigen::Matrix<double, 3, 1> getOffset(){return origin_offset_;}
  inline Eigen::Matrix<double, 3, 1> getOrigin(){return origin_;}

 protected:
  ros::NodeHandle nh_, nhp_;
  float weight_;
  Eigen::Matrix<double, 3, 3> inertia_;
  Eigen::Matrix<double, 3, 1> origin_;
  Eigen::Matrix<double, 3, 1> origin_offset_;
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

  void getLinksOriginFromCog(std::vector<Eigen::Matrix<double, 3, 1> > &links_origin);

  std::vector<Eigen::Matrix<double, 3, 1> > links_origin_from_cog_; //temporary

  Eigen::Matrix<double, 3, 3> getRotateMatrix()
    {
      boost::lock_guard<boost::mutex> lock(rm_mutex_);
      return rotate_matrix_;
    }
  void setRotateMatrix(Eigen::Matrix<double, 3, 3> rotate_matrix)
    {
      boost::lock_guard<boost::mutex> lock(rm_mutex_);
      rotate_matrix_ = rotate_matrix;
    }

  Eigen::Matrix<double, 3, 1> getCog()
    {
      boost::lock_guard<boost::mutex> lock(cog_mutex_);
      return cog_;
    }
  void setCog(Eigen::Matrix<double, 3, 1> cog)
    {
      boost::lock_guard<boost::mutex> lock(cog_mutex_);
      cog_ = cog;
    }


 private:

  ros::NodeHandle nh_,nh_private_;
  ros::Publisher transform_control_pub_;
  ros::Publisher principal_axis_pub_;
  ros::Publisher cog_rotate_pub_; //for position control => to mocap

  boost::mutex rm_mutex_, cog_mutex_;

  tf::TransformListener tf_;
  double control_rate_;
  double tf_pub_rate_;
  ros::Timer  control_timer_;
  ros::Timer  tf_pub_timer_;

  ros::Time system_tf_time_;

  bool debug_log_;

  //base model config

  int link_num_;
  std::vector<int> propeller_direction_;
  std::vector<int> propeller_order_;


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


  Eigen::Matrix<double, 3, 1> cog_;


  Eigen::Matrix<double, 3, 3> links_inertia_, links_principal_inertia_, init_links_principal_inertia_;


  Eigen::Matrix<double, 3, 3> rotate_matrix_;
  Eigen::Matrix<double, 3, 3> cog_matrix_;
  double rotate_angle_;




  void controlFunc(const ros::TimerEvent & e);  
  void tfPubFunc(const ros::TimerEvent & e);
  void visualization();
  int sgn(double value){ return  (value / fabs(value));}

  void cogCoordPublish();
  void initParam();

  void param2contoller();

};


#endif
