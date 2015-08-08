#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

//* ros
#include <ros/ros.h>
#include <aerial_robot_base/ImuQu.h>

//* for kalman filter
#include <Eigen/Core>
//#include <Eigen/LU>
#include <Eigen/Dense>

#include <iostream>
#include <queue>

#include <aerial_robot_base/digital_filter.h>
//* for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <aerial_robot_msgs/DynamicReconfigureLevels.h>
#include <aerial_robot_base/StateKalmanFilterConfig.h>

//* for mutex
#include <boost/version.hpp>
#include <boost/thread/mutex.hpp>
#if BOOST_VERSION>105200
 #include <boost/thread/lock_guard.hpp>
#endif
class KalmanFilterPosVelAcc : public Filter 
{
 public:
  KalmanFilterPosVelAcc(ros::NodeHandle nh, ros::NodeHandle nh_private, std::string filter_id, bool DynamicReconf = false);
  ~KalmanFilterPosVelAcc();

  bool prediction(double input, ros::Time stamp);
  bool correction(double measurement, ros::Time stamp);
  //only velocity
  bool correctionOnlyVelocity(double measurement, ros::Time stamp);
  //for time synchronized state
  void imuQuCorrection(ros::Time check_time_stamp, double measurement, int type = 0);
  void imuQuOnlyPrediction(ros::Time check_time_stamp); //for bad measurement step
  bool imuQuPrediction(ros::Time check_time_stamp);
  void imuQuPush(aerial_robot_base::ImuQuPtr imu_qu_msg_ptr);


  inline double getEstimatePos(){  return estimate_state_(0);}
  inline double getEstimateVel(){  return estimate_state_(1);}
  inline double getCorrectPos(){  return correct_state_(0);}
  inline double getCorrectVel(){  return correct_state_(1);}

  inline double getPredictPos(){  return predict_state_(0);}
  inline double getPredictVel(){  return predict_state_(1);}

  void getEstimateCovariance(float* covarianceMatrix);

  bool getFilteringStartFlag();
  void setInputStartFlag();
  void setMeasureStartFlag(bool flag);
  void setInitImuBias(double initBias);
  void setInitState(double init_pos, double init_vel);

  //dynamic reconfigure
  void cfgCallback(aerial_robot_base::StateKalmanFilterConfig &config, uint32_t level);
  
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::NodeHandle nhp_axis_;


  double dt_;
  double sigma_acc_;
  //  double sigma_optical_flow_;
  double sigma_laser_;
  Eigen::Vector2d estimate_state_; //0:Position, 1:Velocity
  Eigen::Vector2d correct_state_;  //0:Position, 1:_velocity
  Eigen::Vector2d predict_state_; //0:Position, 1:_velocity
  Eigen::Matrix2d prediction_noise_covariance_, estimate_covariance_;
  Eigen::Matrix<double, 1, 1> measurement_noise_covariance_;
  Eigen::Matrix<double, 1, 1> measurement_only_velocity_noise_covariance_;
  Eigen::Matrix<double, 1, 1> inovation_covariance_;
  Eigen::Matrix<double, 2, 1> kalman_gain_;
  
  Eigen::Matrix2d state_transition_model_;
  Eigen::Matrix<double, 1, 2> observation_model_;
  Eigen::Matrix<double, 1, 2> observation_only_velocity_model_;
  Eigen::Matrix<double, 2, 1> control_input_model_;

  //time synchronized state
  std::queue<aerial_robot_base::ImuQuPtr> imu_qu_;

  ros::Time kalman_filter_stamp_;
  std::string id_;

  //filtering start flag
  bool input_start_flag_;
  bool measure_start_flag_;

  //dynamic reconfigure
  dynamic_reconfigure::Server<aerial_robot_base::StateKalmanFilterConfig>* server_;
  dynamic_reconfigure::Server<aerial_robot_base::StateKalmanFilterConfig>::CallbackType dynamic_reconf_func_;

  //for mutex
  boost::mutex kf_mutex_;
  boost::mutex queue_mutex_;

  double input_sigma_;
  double measure_sigma_;

  //bias effect
  double acc_bias_;

  double imu_hz_;
  void rosParamInit();
};

class KalmanFilterPosVelAccBias : public Filter 
{
 public:
  KalmanFilterPosVelAccBias(ros::NodeHandle nh, ros::NodeHandle nh_private, std::string filterID, bool DynamicReconf = false);

  ~KalmanFilterPosVelAccBias();

  bool prediction(double input, ros::Time stamp);
  double correction(double measurement, ros::Time stamp);
  //only velocity
  double correctionOnlyVelocity(double measurement, ros::Time stamp);

  //for time synchronized state
  void imuQuCorrection(ros::Time check_time_stamp, double measurement, int type = 0);
  void imuQuOnlyPrediction(ros::Time check_time_stamp); //for bad measurement step
  bool imuQuPrediction(ros::Time check_time_stamp);
  void imuQuPush(aerial_robot_base::ImuQuPtr imu_qu_msg_ptr);


  inline double getEstimatePos(){  return estimate_state_(0);}
  inline double getEstimateVel(){  return estimate_state_(1);}
  inline double getEstimateBias(){  return estimate_state_(2);}
  inline double getCorrectPos(){  return correct_state_(0);}
  inline double getCorrectVel(){  return correct_state_(1);}
  inline double getPredictPos(){  return predict_state_(0);}
  inline double getPredictVel(){  return predict_state_(1);}


  void getEstimateCovariance(float* covarianceMatrix);

  inline bool getFilteringStartFlag()
  {
    if(input_start_flag_ && measure_start_flag_)
      return true;
    else 
      return false;
  }
  inline void setInputStartFlag(){  input_start_flag_ = true; }
  inline void setMeasureStartFlag(bool flag){  measure_start_flag_ = flag; }

  void setInitImuBias(double initBias);
  void setInitState(double init_pos, double init_vel);
  void reset();

  //dynamic reconfigure
  void cfgCallback(aerial_robot_base::StateKalmanFilterConfig &config, uint32_t level);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::NodeHandle nhp_axis_;
  

  double dt_;
  double sigma_acc_;
  double sigma_optical_flow_;
  double sigma_acc_bias_;
  double sigma_laser_;
  Eigen::Vector3d estimate_state_; //0:Position, 1:Velocity, 2:Bias
  Eigen::Vector3d correct_state_;  //0:Position, 1:Velocity, 2:Bias
  Eigen::Vector3d predict_state_; //0:Position, 1:Velocity, 2:Bias
  Eigen::Matrix3d prediction_noise_covariance_, estimate_covariance_;
  Eigen::Matrix<double, 1, 1> measurement_noise_covariance_;
  Eigen::Matrix<double, 1, 1> measurement_only_velocity_noise_covariance_;
  Eigen::Matrix<double, 1, 1> inovation_covariance_;
  Eigen::Matrix<double, 3, 1> kalman_gain_;
  
  Eigen::Matrix3d state_transition_model_;
  Eigen::Matrix<double, 1, 3> observation_model_;
  Eigen::Matrix<double, 1, 3> observation_only_velocity_model_;
  Eigen::Matrix<double, 3, 1>  control_input_model_;

  //time synchronized state
  std::queue <aerial_robot_base::ImuQuPtr> imu_qu_;
  Eigen::Vector2d estimate_state_time_sync; //0:Position, 1:Velocity


  bool input_start_flag_;
  bool measure_start_flag_;


  ros::Time kalman_filter_stamp_;
  std::string id_;
  //dynamic reconfigure
  dynamic_reconfigure::Server<aerial_robot_base::StateKalmanFilterConfig>* server_;
  dynamic_reconfigure::Server<aerial_robot_base::StateKalmanFilterConfig>::CallbackType dynamic_reconf_func_;

  boost::mutex kf_mutex_;
  boost::mutex queue_mutex_;

  double input_sigma_;
  double bias_sigma_;
  double measure_sigma_;
  double imu_hz_;
  void rosParamInit();
};

#endif
