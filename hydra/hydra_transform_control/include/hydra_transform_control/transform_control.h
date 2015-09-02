/*
1. the std::vector should be sent by reference(getLinksOriginFromCog)
 */

#ifndef TRANSFORM_CONTROL_H
#define TRANSFORM_CONTROL_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "tf_conversions/tf_eigen.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>

#include <aerial_robot_base/YawThrottleGain.h>
#include <aerial_robot_msgs/RollPitchYawGain.h>

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

void realtimeControlCallback(const std_msgs::UInt8ConstPtr & msg);

void cogComputation(const std::vector<tf::StampedTransform>& transforms);
  void principalInertiaComputation(const std::vector<tf::StampedTransform>& transforms, bool continuous_flag = true);

  bool distThreCheck();
  bool distThreCheckFromJointValues(const std::vector<double>& joint_values, int joint_offset,bool continous_flag = true);
    std::vector<tf::StampedTransform> transformsFromJointValues(const std::vector<double>& joint_values, int joint_offset);

  bool stabilityCheck(bool debug = false);

  bool hamiltonMatrixSolver(uint8_t lqi_mode);

  inline double getLinkLength(){return link_length_;}
  inline int getLinkNum(){return link_num_;}


  void getLinksOriginFromCog(std::vector<Eigen::Vector3d>& links_origin_from_cog)
    {
#if 0
      if(multi_thread_flag_)
        {
          boost::lock_guard<boost::mutex> lock(origins_mutex_);
          int size = links_origin_from_cog_.size();
          for(int i=0; i< size; i++)
            links_origin_from_cog = links_origin_from_cog_;
        }
      else
        {
          int size = links_origin_from_cog_.size();
          for(int i=0; i< size; i++)
            links_origin_from_cog = links_origin_from_cog_;
        }
#endif
          boost::lock_guard<boost::mutex> lock(origins_mutex_);
          int size = links_origin_from_cog_.size();
          for(int i=0; i< size; i++)
            links_origin_from_cog = links_origin_from_cog_;

    }

  void setLinksOriginFromCog(const std::vector<Eigen::Vector3d>& links_origin_from_cog)
  {
#if 0
    if(multi_thread_flag_)
      {
        boost::lock_guard<boost::mutex> lock(origins_mutex_);
        links_origin_from_cog_ = links_origin_from_cog;
      }
    else
      {
        links_origin_from_cog_ = links_origin_from_cog;
      }
#endif
    boost::lock_guard<boost::mutex> lock(origins_mutex_);
    links_origin_from_cog_ = links_origin_from_cog;
  }

  Eigen::Matrix3d getPrincipalInertia()
    {
#if 0
      if(multi_thread_flag_)
        {
          boost::lock_guard<boost::mutex> lock(inertia_mutex_);
          return links_principal_inertia_;
        }
      else
        {
          return links_principal_inertia_;
        }
#endif
      boost::lock_guard<boost::mutex> lock(inertia_mutex_);
      return links_principal_inertia_;
    }
  void setPrincipalInertia(Eigen::Matrix3d principal_inertia)
  {
#if 0
    if(multi_thread_flag_)
      {
        boost::lock_guard<boost::mutex> lock(inertia_mutex_);
        links_principal_inertia_ = principal_inertia;
      }
    else
      {
        links_principal_inertia_ = principal_inertia;
      }
#endif
    boost::lock_guard<boost::mutex> lock(inertia_mutex_);
    links_principal_inertia_ = principal_inertia;
    }


  Eigen::Matrix3d getRotateMatrix()
    {
#if 0
    if(multi_thread_flag_)
      {
        boost::lock_guard<boost::mutex> lock(rm_mutex_);
        return rotate_matrix_;
      }
    else
      {
        return rotate_matrix_;
      }
#endif
    boost::lock_guard<boost::mutex> lock(rm_mutex_);
    return rotate_matrix_;
    }

  void setRotateMatrix(Eigen::Matrix3d rotate_matrix)
  {
#if 0
    if(multi_thread_flag_)
      {
        boost::lock_guard<boost::mutex> lock(rm_mutex_);
        rotate_matrix_ = rotate_matrix;
      }
    else
      {
        rotate_matrix_ = rotate_matrix;
      }
#endif
    boost::lock_guard<boost::mutex> lock(rm_mutex_);
    rotate_matrix_ = rotate_matrix;
  }

  Eigen::Vector3d getCog()
    {
#if 0
      if(multi_thread_flag_)
        {
          boost::lock_guard<boost::mutex> lock(cog_mutex_);
          return cog_;
        }

      else
        {
          return cog_;
        }
#endif
      boost::lock_guard<boost::mutex> lock(cog_mutex_);
      return cog_;
    }

  void setCog(Eigen::Vector3d cog)
    {
#if 0
      if(multi_thread_flag_)
        {
          boost::lock_guard<boost::mutex> lock(cog_mutex_);
          cog_ = cog;
        }
      else { cog_ = cog;}
#endif
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

  const static uint8_t LQI_FOUR_AXIS_MODE = 0;
  const static uint8_t LQI_THREE_AXIS_MODE = 1;

 private:

  ros::NodeHandle nh_,nh_private_;
  ros::Publisher rpy_gain_pub_;
  ros::Publisher yaw_throttle_gain_pub_;
  ros::Publisher principal_axis_pub_;
  ros::Publisher cog_rotate_pub_; //for position control => to mocap
  ros::Subscriber realtime_control_sub_;

  boost::mutex rm_mutex_, cog_mutex_, origins_mutex_, inertia_mutex_;

  tf::TransformListener tf_;
  double control_rate_;
  double tf_pub_rate_;
  ros::Timer  control_timer_;
  ros::Timer  tf_pub_timer_;

  ros::Time system_tf_time_;

  bool realtime_control_flag_;

  bool callback_flag_;
  //bool multi_thread_flag_;

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
  int root_link_;
  std::vector<std::string> links_name_;

  Eigen::Vector3d cog_;
  std::vector<Eigen::Vector3d> links_origin_from_cog_; 

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

  //lqi
  boost::thread lqi_thread_;
  bool lqi_flag_;
  double lqi_thread_rate_;
  void lqi();


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

  Eigen::MatrixXd R4_;

  Eigen::MatrixXd K12_;
  Eigen::MatrixXd K9_;

  //Q
  double q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_yaw_,q_yaw_d_,q_z_,q_z_d_;
  double q_roll_i_,q_pitch_i_,q_yaw_i_,q_z_i_;

  //R
  std::vector<double> r_;

  //distance_thresold
  double alfa_;
  double dist_thre_;
  double f_max_, f_min_;

  uint8_t lqi_mode_;

};


#endif
