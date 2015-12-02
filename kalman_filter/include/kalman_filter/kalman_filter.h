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
  KalmanFilter(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~KalmanFilter(){}

  bool prediction(Matrix<double, input_dim, 1> input);
  bool correction(Matrix<double, measure_dim, 1> measurement);

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
    prediction_noise_covariance_ = (input_sigma_ * input_sigma_.transpose()) * (control_input_model_ * control_input_model_.transpose());
    measurement_noise_covariance_ = (measure_sigma_ * measure_sigma_.transpose()) * (observation_model_.transpose() * observation_model_);
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

  inline void  setStateTransitionModel(Matrix<double, state_dim, state_dim> state_transition_model){state_transition_model_ = state_transition_model;}
  inline void setControlInputModel(Matrix<double, state_dim, input_dim> control_input_model){control_input_model_ = control_input_model;}
  inline void setObservationModel(Matrix<double, measure_dim, state_dim> observation_model){observation_model_ = observation_model;}

  inline void setInputFlag(bool flag = true){input_start_flag_ = flag; }
  inline void setMeasureFlag(bool flag = true){ measure_start_flag_ = flag;}

  void getFilteringFlag()
  {
    if(input_start_flag_ && measure_start_flag_)
      return true;
    else 
      return false;
  }

  void setInitState(Matrix<double, state_dim, 1> init_state);

 private:
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

  string id_;

  //filtering start flag
  bool input_start_flag_;
  bool measure_start_flag_;

  //for mutex
  boost::mutex kf_mutex_;
};


#endif
