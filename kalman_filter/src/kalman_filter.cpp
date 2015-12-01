#include "kalman_filter/kalman_filter.h"

template <size_t state_dim, size_t input_dim, size_t measure_dim> KalmanFilter<state_dim, input_dim, measure_dim>::KalmanFilter(ros::NodeHandle nh, ros::NodeHandle nh_private): nh_(nh, "kalman_filter"), nhp_(nh_private, "kalman_filter")
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

 template <size_t state_dim, size_t input_dim, size_t measure_dim> bool KalmanFilter<state_dim, input_dim, measure_dim>::prediction(Matrix<double, input_dim, 1> input)
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
        = state_transition_model_ * predict_state_ + input * control_input_model_;
      predict_state_ = predict_hat_state;

      return true;
    }

    return false;
}

template <size_t state_dim, size_t input_dim, size_t measure_dim> bool KalmanFilter<state_dim, input_dim, measure_dim>::correction(Matrix<double, measure_dim, 1> measurement)
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

template <size_t state_dim, size_t input_dim, size_t measure_dim> void KalmanFilter<state_dim, input_dim, measure_dim>::setInitState(Matrix<double, state_dim, 1> init_state)
{
  boost::lock_guard<boost::mutex> lock(kf_mutex_);
  estimate_state_ = init_state;
  correct_state_ = init_state;
  predict_state_ = init_state;
}


