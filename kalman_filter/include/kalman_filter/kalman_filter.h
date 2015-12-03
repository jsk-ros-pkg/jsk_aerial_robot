/*
cannot resolve the problem that seperates files to kalman_filter.h and kalman_filter.cpp
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

//* ros
#include <ros/ros.h>

//* for kalman filter
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

//* for mutex
#include <boost/version.hpp>
#include <boost/thread/mutex.hpp>
#if BOOST_VERSION>105200
 #include <boost/thread/lock_guard.hpp>
#endif

using namespace Eigen;
using namespace std;

template <size_t state_dim, size_t input_dim, size_t measure_dim>
class KalmanFilter
{
 public:
  KalmanFilter(ros::NodeHandle nh, ros::NodeHandle nh_private)
    {
  dt_ = 1 / input_hz_;
  input_start_flag_ = false;
  measure_start_flag_ = false;


  estimate_state_ = MatrixXd::Zero(state_dim, 1);
  correct_state_ = MatrixXd::Zero(state_dim, 1);
  predict_state_ = MatrixXd::Zero(state_dim, 1);

  estimate_covariance_ = MatrixXd::Zero(state_dim, state_dim);
  inovation_covariance_ = MatrixXd::Zero(measure_dim, measure_dim);
  kalman_gain_  = MatrixXd::Zero(state_dim, measure_dim);

    }
  ~KalmanFilter(){}

  bool prediction(Matrix<double, input_dim, 1> input)
  {
  if(getFilteringFlag())
    {
      boost::lock_guard<boost::mutex> lock(kf_mutex_);

      Matrix<double, state_dim, 1> estimate_hat_state = state_transition_model_ * estimate_state_ + control_input_model_ * input;
      estimate_state_ = estimate_hat_state;

      Matrix<double, state_dim, state_dim> estimate_bar_covariance
        = state_transition_model_ * estimate_covariance_ * state_transition_model_.transpose() 
        + prediction_noise_covariance_;
      estimate_covariance_ = estimate_bar_covariance;

      Matrix<double, state_dim, 1> predict_hat_state
        = state_transition_model_ * predict_state_ + control_input_model_ * input ;
      predict_state_ = predict_hat_state;

      return true;
    }

    return false;

  }
  bool correction(Matrix<double, measure_dim, 1> measurement)
  {
  if(getFilteringFlag())
    {
      boost::lock_guard<boost::mutex> lock(kf_mutex_);

      inovation_covariance_ = observation_model_ * estimate_covariance_ * observation_model_.transpose()
        + measurement_noise_covariance_;
  
      kalman_gain_ = estimate_covariance_ * observation_model_.transpose() * inovation_covariance_.inverse();

      Matrix<double, state_dim, 1> estimate_state = estimate_state_ + kalman_gain_ * (measurement - observation_model_ * estimate_state_);
      estimate_state_ = estimate_state;
      correct_state_ = estimate_state;

      Matrix<double, state_dim, state_dim> I; I.setIdentity();
      Matrix<double, state_dim, state_dim> estimate_covariance_tmp;
      estimate_covariance_tmp = (I - kalman_gain_ * observation_model_) * estimate_covariance_;
      estimate_covariance_ = estimate_covariance_tmp;

      return true;
    }

    return false;

  }

  void setEstimateState(const Matrix<double, state_dim, 1>& state)
  {
    boost::lock_guard<boost::mutex> lock(kf_mutex_);
    estimate_state_ = state;
  }


  inline void setInputSigma( Matrix<double, input_dim, 1> input_sigma) { input_sigma_ = input_sigma; }
  inline void setMeasureSigma( Matrix<double, measure_dim, 1> measure_sigma) { measure_sigma_ = measure_sigma; }

  void setPredictionNoiseCovariance()
  {
    //wrong!!!!!!
    Matrix<double, input_dim, input_dim> input_sigma_m = (input_sigma_.col(0)).asDiagonal();
    prediction_noise_covariance_ = control_input_model_ * (input_sigma_m * input_sigma_m) * control_input_model_.transpose();
  }

  void setMeasurementNoiseCovariance()
  {
    Matrix<double, measure_dim, measure_dim> measure_sigma_m = (measure_sigma_.col(0)).asDiagonal();
    measurement_noise_covariance_ = measure_sigma_m * measure_sigma_m;
  }

  inline void setCorrectState(const Matrix<double, state_dim, 1>& state) {  correct_state_ = state; }
  inline void setPredictState(const Matrix<double, state_dim, 1>& state) { predict_state_ = state; }

  Matrix<double, state_dim, 1> getEstimateState()
    {
      boost::lock_guard<boost::mutex> lock(kf_mutex_);
      return estimate_state_;
    }

  inline Matrix<double, state_dim, 1> getCorrectState() { return correct_state_; }
  inline Matrix<double, state_dim, 1> getPredictState() { return predict_state_; }
  inline Matrix<double, state_dim, state_dim> getEstimateCovariance(){ return estimate_covariance_;}

  inline void setStateTransitionModel(Matrix<double, state_dim, state_dim> state_transition_model){state_transition_model_ = state_transition_model;}
  inline void setControlInputModel(Matrix<double, state_dim, input_dim> control_input_model){control_input_model_ = control_input_model;}
  inline void setObservationModel(Matrix<double, measure_dim, state_dim> observation_model){observation_model_ = observation_model;}

  inline void setInputFlag(bool flag = true){input_start_flag_ = flag; }
  inline void setMeasureFlag(bool flag = true){ measure_start_flag_ = flag;}

  bool getFilteringFlag()
  {
    if(input_start_flag_ && measure_start_flag_)
      return true;
    else 
      return false;
  }

  inline void setInitState(double state_value, int no)
  { 
    estimate_state_(no,0) = state_value;
    correct_state_(no,0) = state_value;
    predict_state_(no,0) = state_value;
  }
  void setInitState(Matrix<double, state_dim, 1> init_state)
  {
  boost::lock_guard<boost::mutex> lock(kf_mutex_);
  estimate_state_ = init_state;
  correct_state_ = init_state;
  predict_state_ = init_state;

  }

 protected:

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  double dt_, input_hz_;



  Matrix<double, input_dim, 1>  input_sigma_;
  Matrix<double, measure_dim, 1>  measure_sigma_;

  Matrix<double, state_dim, 1> estimate_state_;
  Matrix<double, state_dim, 1> correct_state_;
  Matrix<double, state_dim, 1> predict_state_;


  Matrix<double, state_dim, state_dim> prediction_noise_covariance_, estimate_covariance_;

  Matrix<double, measure_dim, measure_dim> measurement_noise_covariance_;
  Matrix<double, measure_dim, measure_dim> inovation_covariance_;
  Matrix<double, state_dim, measure_dim> kalman_gain_;

  Matrix<double, state_dim, state_dim> state_transition_model_;
  Matrix<double, state_dim, input_dim> control_input_model_;
  Matrix<double, measure_dim, state_dim> observation_model_;



  //filtering start flag
  bool input_start_flag_;
  bool measure_start_flag_;

  //for mutex
  boost::mutex kf_mutex_;
};


#endif
