/*
cannot resolve the problem that seperates files to kalman_filter.h and kalman_filter.cpp => cannot do in catkin?
 */

#ifndef KALMAN_FILTER_PLUGIN_H
#define KALMAN_FILTER_PLUGIN_H

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

namespace kf_base_plugin
{
  class KalmanFilter
  {
  public:
    virtual void initialize(ros::NodeHandle nh, string suffix, int id )  = 0;

    void baseInit(ros::NodeHandle nh, string suffix, int id )
    {
      id_ = id;
      nh_ = nh;
      nhp_ = ros::NodeHandle(nh, suffix);

      state_transition_model_ = MatrixXd::Zero(state_dim_, state_dim_);
      control_input_model_ = MatrixXd::Zero(state_dim_, input_dim_);
      observation_model_ = MatrixXd::Zero(measure_dim_, state_dim_);

      input_sigma_ = MatrixXd::Zero(input_dim_, 1);
      measure_sigma_ = MatrixXd::Zero(measure_dim_, 1);

      estimate_state_ = MatrixXd::Zero(state_dim_, 1);
      correct_state_ = MatrixXd::Zero(state_dim_, 1);
      predict_state_ = MatrixXd::Zero(state_dim_, 1);

      estimate_covariance_ = MatrixXd::Zero(state_dim_, state_dim_);
      prediction_noise_covariance_ = MatrixXd::Zero(state_dim_, state_dim_);

      measurement_noise_covariance_ = MatrixXd::Zero(measure_dim_, measure_dim_);
      inovation_covariance_ = MatrixXd::Zero(measure_dim_, measure_dim_);
      kalman_gain_  = MatrixXd::Zero(state_dim_, measure_dim_);

      rosParamInit();
      kfModelInit();

      std::string ns = nhp_.getNamespace();
      ROS_ERROR("nhp namespace: %s", ns.c_str());
    }

    void kfModelInit()
    {
      dt_ = 1 / input_hz_;
      input_start_flag_ = false;
      measure_start_flag_ = false;


      updateModelFromDt(dt_);

      setPredictionNoiseCovariance();
      setMeasurementNoiseCovariance();


      std::cout << "state_transition_model_" << std::endl <<  state_transition_model_ << std::endl;
      std::cout << "control_input_model_" << std::endl << control_input_model_  << std::endl;
      std::cout << "observation_model_" << std::endl << observation_model_  << std::endl;

      std::cout << "estimate_covariance_" << std::endl << estimate_covariance_  << std::endl;
      std::cout << "measurement_noise_covariance_" << std::endl << measurement_noise_covariance_  << std::endl;
      std::cout << "prediction_noise_covariance_" << std::endl << prediction_noise_covariance_  << std::endl;

      std::cout << "inovation_covariance_" << std::endl << inovation_covariance_  << std::endl;

    }

    virtual ~KalmanFilter(){}

    bool prediction(MatrixXd  input)
    {
      if(getFilteringFlag())
        {

          boost::lock_guard<boost::mutex> lock(kf_mutex_);

          MatrixXd estimate_hat_state = state_transition_model_ * estimate_state_ + control_input_model_ * input;
          estimate_state_ = estimate_hat_state;

          MatrixXd estimate_bar_covariance
            = state_transition_model_ * estimate_covariance_ * state_transition_model_.transpose() 
            + prediction_noise_covariance_;
          estimate_covariance_ = estimate_bar_covariance;

          MatrixXd predict_hat_state
            = state_transition_model_ * predict_state_ + control_input_model_ * input ;
          predict_state_ = predict_hat_state;

          return true;
        }

      return false;

    }

    bool correction(MatrixXd measurement, bool debug = false)
    {
      if(getFilteringFlag())
        {
          boost::lock_guard<boost::mutex> lock(kf_mutex_);

          inovation_covariance_ = observation_model_ * estimate_covariance_ * observation_model_.transpose()
            + measurement_noise_covariance_;
  
          kalman_gain_ = estimate_covariance_ * observation_model_.transpose() * inovation_covariance_.inverse();

          MatrixXd estimate_state = estimate_state_ + kalman_gain_ * (measurement - observation_model_ * estimate_state_);
          estimate_state_ = estimate_state;
          correct_state_ = estimate_state;

          MatrixXd I = MatrixXd::Identity(state_dim_, state_dim_);
          MatrixXd estimate_covariance_tmp = (I - kalman_gain_ * observation_model_) * estimate_covariance_;
          estimate_covariance_ = estimate_covariance_tmp;

          if(debug)
            {
              std::cout << "estimate_state_" << std::endl <<  estimate_state_ << std::endl;
              std::cout << "kalman_gain_" << std::endl << kalman_gain_  << std::endl;

              std::cout << "state_transition_model_" << std::endl <<  state_transition_model_ << std::endl;
              std::cout << "control_input_model_" << std::endl << control_input_model_  << std::endl;
              std::cout << "observation_model_" << std::endl << observation_model_  << std::endl;

              std::cout << "estimate_covariance_" << std::endl << estimate_covariance_  << std::endl;
              std::cout << "measurement_noise_covariance_" << std::endl << measurement_noise_covariance_  << std::endl;
              std::cout << "inovation_covariance_" << std::endl << inovation_covariance_  << std::endl;

            }
          return true;
        }
      return false;
    }

    void setEstimateState(const MatrixXd& state)
    {
      boost::lock_guard<boost::mutex> lock(kf_mutex_);
      estimate_state_ = state;
    }


    void setInputSigma( MatrixXd input_sigma) 
    {
      input_sigma_ = input_sigma;  
      setPredictionNoiseCovariance();
    }

    void setMeasureSigma( MatrixXd measure_sigma) 
    { 
      measure_sigma_ = measure_sigma; 
      setMeasurementNoiseCovariance();
    }

    void setPredictionNoiseCovariance()
    {
      //wrong!!!!!!
      MatrixXd input_sigma_m = (input_sigma_.col(0)).asDiagonal();
      prediction_noise_covariance_ = control_input_model_ * (input_sigma_m * input_sigma_m) * control_input_model_.transpose();
    }

    void setMeasurementNoiseCovariance()
    {
      MatrixXd measure_sigma_m = (measure_sigma_.col(0)).asDiagonal();
      measurement_noise_covariance_ = measure_sigma_m * measure_sigma_m;
    }

    inline void setCorrectState(const MatrixXd& state) {  correct_state_ = state; }
    inline void setPredictState(const MatrixXd& state) { predict_state_ = state; }

    MatrixXd getEstimateState()
      {
        boost::lock_guard<boost::mutex> lock(kf_mutex_);
        return estimate_state_;
      }

    inline MatrixXd getCorrectState() { return correct_state_; }
    inline MatrixXd getPredictState() { return predict_state_; }
    inline MatrixXd getEstimateCovariance(){ return estimate_covariance_;}

    inline int getStateDim(){return state_dim_;}
    inline int getInputDim(){return input_dim_;}
    inline int getMeasureDim(){return measure_dim_;}

    inline void setStateTransitionModel(MatrixXd state_transition_model){state_transition_model_ = state_transition_model;}
    inline void setControlInputModel(MatrixXd control_input_model){control_input_model_ = control_input_model;}
    inline void setObservationModel(MatrixXd observation_model){observation_model_ = observation_model;}

    inline void setInputFlag(bool flag = true){input_start_flag_ = flag; }
    inline void setMeasureFlag(bool flag = true){ measure_start_flag_ = flag;}

    inline bool getFilteringFlag()
    {
      if(input_start_flag_ && measure_start_flag_)
        return true;
      else 
        return false;
    }

    void setInitState(double state_value, int no)
    { 
      boost::lock_guard<boost::mutex> lock(kf_mutex_);
      estimate_state_(no,0) = state_value;
      correct_state_(no,0) = state_value;
      predict_state_(no,0) = state_value;
    }

    inline void setInitState(MatrixXd init_state)
    {
      boost::lock_guard<boost::mutex> lock(kf_mutex_);
      estimate_state_ = init_state;
      correct_state_ = init_state;
      predict_state_ = init_state;

    }

    virtual void setCorrectMode(uint8_t correct_mode) = 0;
    virtual void updateModelFromDt(double dt) = 0;


  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    int state_dim_, input_dim_,  measure_dim_;
    double dt_, input_hz_;
    int correct_mode_;
    int id_;

    MatrixXd input_sigma_,   measure_sigma_;
    MatrixXd estimate_state_, correct_state_, predict_state_;

    MatrixXd prediction_noise_covariance_, estimate_covariance_;

    MatrixXd measurement_noise_covariance_;
    MatrixXd inovation_covariance_;
    MatrixXd kalman_gain_;

    MatrixXd state_transition_model_;
    MatrixXd control_input_model_;
    MatrixXd observation_model_;

    //filtering start flag
    bool input_start_flag_;
    bool measure_start_flag_;

    //for mutex
    boost::mutex kf_mutex_;

    KalmanFilter(){}


    void rosParamInit()
    {
      std::string ns = nhp_.getNamespace();

      if (!nhp_.getParam ("correct_mode", correct_mode_))
        correct_mode_ = 0;
      printf("%s: correct_mode  is %d\n", ns.c_str(), correct_mode_);


      /*
      if (!nhp_.getParam ("state_dim", state_dim_))
        state_dim_ = 2;
      printf("%s: state_dim  is %d\n", ns.c_str(), state_dim_);

      if (!nhp_.getParam ("input_dim", input_dim_))
        input_dim_ = 2;
      printf("%s: input_dim  is %d\n", ns.c_str(), input_dim_);

      if (!nhp_.getParam ("measure_dim", measure_dim_))
        measure_dim_ = 2;
      printf("%s: measure_dim  is %d\n", ns.c_str(), measure_dim_);
      */


      for(int i = 0; i < input_dim_; i ++)
        {
          std::stringstream input_sigma_no;
          input_sigma_no << i + 1;
          if (!nhp_.getParam (std::string("input_sigma") + input_sigma_no.str(), input_sigma_(i,0)))
            input_sigma_(i,0) = 0.0;
          printf("%s: input_sigma_ is %.4f\n", ns.c_str(), input_sigma_(i,0));
        }

      for(int i = 0; i < measure_dim_; i ++)
        {
          std::stringstream measure_sigma_no;
          measure_sigma_no << i + 1;
          if (!nhp_.getParam (std::string("measure_sigma") + measure_sigma_no.str(), measure_sigma_(i,0)))
            measure_sigma_(i,0) = 0.0;
          printf("%s: measure_sigma_ is %.4f\n", ns.c_str(), measure_sigma_(i,0));
        }

      if (!nhp_.getParam ("input_hz", input_hz_))
        input_hz_ = 100.0;
      printf("%s: input_hz_ is %.4f\n",nhp_.getNamespace().c_str(), input_hz_);
      dt_ = 1.0 / input_hz_;
    }

  };
};
#endif
