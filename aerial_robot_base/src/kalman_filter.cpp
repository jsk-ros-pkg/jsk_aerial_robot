#include "aerial_robot_base/kalman_filter.h"

KalmanFilterPosVelAcc::KalmanFilterPosVelAcc(ros::NodeHandle nh, ros::NodeHandle nh_private, std::string filter_id, bool dynamic_reconf): nh_(nh, "kalman_filter"), nhp_(nh_private, "kalman_filter"), nhp_axis_(nh_private, "kalman_filter/" + filter_id), id_(filter_id)
{
  rosParamInit();

  //init 
  sigma_acc_ = input_sigma_; sigma_laser_ = measure_sigma_;
  dt_ = 1 / imu_hz_;
  input_start_flag_ = false;
  measure_start_flag_ = false;

  if(dynamic_reconf)
    {
      server_ = new dynamic_reconfigure::Server<aerial_robot_base::StateKalmanFilterConfig>(nhp_axis_);
      dynamic_reconf_func_ = boost::bind(&KalmanFilterPosVelAcc::cfgCallback, this, _1, _2);
      server_->setCallback(dynamic_reconf_func_);
    }

  estimate_state_(0) = 0; //position initial
  estimate_state_(1) = 0; //velocity initial

  correct_state_(0) = 0; //position initial
  correct_state_(1) = 0; //velocity initial

  predict_state_(0) = 0; //position initial
  predict_state_(1) = 0; //velocity initial

  state_transition_model_ << 1, dt_, 0, 1;
  control_input_model_ << (dt_ * dt_)/2, dt_;
  observation_model_ << 1, 0;
  observation_only_velocity_model_ << 0, 1;

  prediction_noise_covariance_ = (sigma_acc_ * sigma_acc_) * control_input_model_ * control_input_model_.transpose();
  estimate_covariance_ << 0, 0, 0, 0;
  inovation_covariance_ << 0;
  measurement_noise_covariance_(0) = sigma_laser_ * sigma_laser_;
  //bad !!
  measurement_only_velocity_noise_covariance_(0) = sigma_laser_ * sigma_laser_;
  kalman_gain_ << 0, 0;

}

KalmanFilterPosVelAcc::~KalmanFilterPosVelAcc()
{
}

bool KalmanFilterPosVelAcc::prediction(double input, ros::Time stamp)
{
  boost::lock_guard<boost::mutex> lock(kf_mutex_);

  if(getFilteringFlag())
    {

      Eigen::Vector2d estimate_hat_state
        = state_transition_model_ * estimate_state_ + input * control_input_model_;
      estimate_state_ = estimate_hat_state;

      Eigen::Matrix2d estimate_bar_covariance
        = state_transition_model_ * estimate_covariance_ * state_transition_model_.transpose() 
        + prediction_noise_covariance_;
      estimate_covariance_ = estimate_bar_covariance;

      //debug
      Eigen::Vector2d predict_hat_state
        = state_transition_model_ * predict_state_ + input * control_input_model_;
      predict_state_ = predict_hat_state;

      return true;
    }
  else 
    return false;
}

bool KalmanFilterPosVelAcc::correction(double measurement, ros::Time stamp)
{
  boost::lock_guard<boost::mutex> lock(kf_mutex_);
  if(getFilteringFlag())
    {

      inovation_covariance_ = observation_model_ * estimate_covariance_ * observation_model_.transpose()
        + measurement_noise_covariance_;
  

      kalman_gain_ = estimate_covariance_ * observation_model_.transpose() * inovation_covariance_.inverse();



      Eigen::Vector2d estimate_state_tmp 
        = estimate_state_ + kalman_gain_ * (measurement - observation_model_ * estimate_state_);
      estimate_state_ = estimate_state_tmp;
      correct_state_ = estimate_state_tmp;


      Eigen::Matrix2d I; I.setIdentity();
      Eigen::Matrix2d estimate_covariance_tmp;
      estimate_covariance_tmp = (I - kalman_gain_ * observation_model_) * estimate_covariance_;
      estimate_covariance_ = estimate_covariance_tmp;

      return true;
    }
  else 
    return false;
}

bool KalmanFilterPosVelAcc::correctionOnlyVelocity(double measurement, ros::Time timeStamp)
{
  boost::lock_guard<boost::mutex> lock(kf_mutex_);
  if(getFilteringFlag())
    {

      inovation_covariance_ = observation_only_velocity_model_ * estimate_covariance_ * observation_only_velocity_model_.transpose() + measurement_only_velocity_noise_covariance_;

      kalman_gain_ = estimate_covariance_ * observation_only_velocity_model_.transpose() * inovation_covariance_.inverse();

      Eigen::Vector2d estimate_state_tmp 
        = estimate_state_ + kalman_gain_ * (measurement - observation_only_velocity_model_ * estimate_state_);
      estimate_state_ = estimate_state_tmp;
      correct_state_ = estimate_state_tmp;


      Eigen::Matrix2d I; I.setIdentity();
      Eigen::Matrix2d estimate_covariance_tmp;
      estimate_covariance_tmp = (I - kalman_gain_ * observation_only_velocity_model_) * estimate_covariance_;
      estimate_covariance_ = estimate_covariance_tmp;

      return true;
    }
  else 
    {
      return false;
    }
}



void KalmanFilterPosVelAcc::imuQuPush(aerial_robot_base::ImuQuPtr imu_qu_msg_ptr)
{
  boost::lock_guard<boost::mutex> lock(queue_mutex_);
  if(getFilteringFlag())
    {
      imu_qu_.push(imu_qu_msg_ptr);
    }
  return;
}

bool KalmanFilterPosVelAcc::imuQuPrediction(ros::Time check_time_stamp)
{
  boost::lock_guard<boost::mutex> lock(queue_mutex_);


  if(imu_qu_.empty())
    {
      return false;
    }

  if(check_time_stamp.toSec() > (imu_qu_.front()->stamp.toSec()))
    { 
      prediction((double)imu_qu_.front()->acc, imu_qu_.front()->stamp);
      imu_qu_.pop();

      return true;
    }
  else
    return false;
}


//for bad measurement step
void KalmanFilterPosVelAcc::imuQuOnlyPrediction(ros::Time check_time_stamp)
{
  if(getFilteringFlag())
    {
      while(1)
        {
          if(!imuQuPrediction(check_time_stamp))
            break;
        }
    }
  return;
}



//for time synchronized state
void KalmanFilterPosVelAcc::imuQuCorrection(ros::Time check_time_stamp, double measurement, int type)
{
  if(getFilteringFlag())
    {

      while(1)
        {
          if(!imuQuPrediction(check_time_stamp))
            break;
        }

      if(type == 0) // position measurement
        correction(measurement, check_time_stamp);
      if(type == 1) // velocity measurement
        correctionOnlyVelocity(measurement, check_time_stamp);

    }
  return;
}


void KalmanFilterPosVelAcc::cfgCallback(aerial_robot_base::StateKalmanFilterConfig &config, uint32_t level)
{

  if(config.kalmanFilterFlag == true)
    {
      printf("%s axis kf without bias, ", id_.c_str());

      switch(level)
        {
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_INPUT_SIGMA:
          sigma_acc_ = config.inputSigma;

          prediction_noise_covariance_ 
            = (sigma_acc_ * sigma_acc_) * control_input_model_ * control_input_model_.transpose();
          printf("change the input sigma\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_MEASURE_SIGMA:
          sigma_laser_  = config.measureSigma;
          measurement_noise_covariance_(0) = sigma_laser_ * sigma_laser_;
          printf("change the measure sigma\n");
          break;
        default :
          printf("\n");
          break;
        }
    }

}

void KalmanFilterPosVelAcc::getEstimateCovariance(float* covariance_matrix)
{
  covariance_matrix[0] = estimate_covariance_(0,0);
  covariance_matrix[1] = estimate_covariance_(0,1);
  covariance_matrix[2] = estimate_covariance_(1,0);
  covariance_matrix[3] = estimate_covariance_(1,1);
}


void KalmanFilterPosVelAcc::setInitImuBias(double init_bias)
{
  //set bias
  acc_bias_ = init_bias;
  // start filtering . danger!
  setInputFlag();
}

void KalmanFilterPosVelAcc::setInputFlag()
{
  boost::lock_guard<boost::mutex> lock(kf_mutex_);
  input_start_flag_ = true;
}

void KalmanFilterPosVelAcc::setMeasureFlag(bool flag)
{
  boost::lock_guard<boost::mutex> lock(kf_mutex_);
  measure_start_flag_ = flag;
}


bool KalmanFilterPosVelAcc::getFilteringFlag()
{
  if(input_start_flag_ && measure_start_flag_)
    return true;
  else 
    return false;
}

void KalmanFilterPosVelAcc::setInitState(double init_pos, double init_vel)
{
  estimate_state_(0) = init_pos; //position initial
  estimate_state_(1) = init_vel; //position initial

  correct_state_(0) = init_pos; //position initial
  correct_state_(1) = init_vel; //position initial

  predict_state_(0) = init_pos; //position initial
  predict_state_(1) = init_vel; //position initial


}

void KalmanFilterPosVelAcc::rosParamInit()
{
  std::string ns = nhp_axis_.getNamespace();
  ns.append(" without bias");
  if (!nhp_axis_.getParam ("input_sigma", input_sigma_))
    input_sigma_ = 0.0;
  printf("%s: input_sigma_ is %.4f\n", ns.c_str(), input_sigma_);
  if (!nhp_axis_.getParam ("measure_sigma", measure_sigma_))
    measure_sigma_ = 0.0;
  printf("%s: measure_sigma_ is %.4f\n", ns.c_str(), measure_sigma_);
  if (!nhp_.getParam ("imu_hz", imu_hz_))
    imu_hz_ = 100;
  printf("%s: imu_hz_ is %.4f\n",nhp_.getNamespace().c_str(), imu_hz_);
}



KalmanFilterPosVelAccBias::KalmanFilterPosVelAccBias(ros::NodeHandle nh, 
                                                   ros::NodeHandle nh_private, 
                                                   std::string filter_id,
                                                   bool dynamic_reconf):nh_(nh, "kalman_filter"), nhp_(nh_private, "kalman_filter"), nhp_axis_(nh_private, "kalman_filter/" + filter_id), id_(filter_id) 
{
  rosParamInit();

  //init
  sigma_acc_ = input_sigma_;
  sigma_acc_bias_ = bias_sigma_;
  sigma_laser_ = measure_sigma_;
  dt_ = 1 / imu_hz_;

  input_start_flag_ = false;
  measure_start_flag_ = false;


  //dynamic reconfigure
  if(dynamic_reconf)
    {
      server_ = new dynamic_reconfigure::Server<aerial_robot_base::StateKalmanFilterConfig>(nhp_axis_);
      dynamic_reconf_func_ = boost::bind(&KalmanFilterPosVelAccBias::cfgCallback, this, _1, _2);
      server_->setCallback(dynamic_reconf_func_);
    }


  estimate_state_(0) = 0; //position initial
  estimate_state_(1) = 0; //position initial
  estimate_state_(2) = 0; //bias inital

  correct_state_(0) = 0; //position initial
  correct_state_(1) = 0; //velocity initial
  correct_state_(2) = 0; //bias initial

  predict_state_(0) = 0; //position initial
  predict_state_(1) = 0; //velocity initial
  predict_state_(2) = 0; //bias initial

  state_transition_model_ << 1, dt_, -dt_*dt_/2, 0, 1, -dt_, 0, 0, 1;
  control_input_model_ << (dt_ * dt_)/2, dt_, 0;
  observation_model_ << 1, 0, 0;
  observation_only_velocity_model_ << 0, 1, 0;

  prediction_noise_covariance_ = (sigma_acc_ * sigma_acc_) * control_input_model_ * control_input_model_.transpose();

  prediction_noise_covariance_(2,2) = sigma_acc_bias_ * sigma_acc_bias_;

  estimate_covariance_ << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  inovation_covariance_ << 0;
  measurement_noise_covariance_(0) = sigma_laser_ * sigma_laser_;
  //bad !!
  measurement_only_velocity_noise_covariance_(0) = sigma_laser_ * sigma_laser_;

  kalman_gain_ << 0, 0, 0;

  kalman_filter_stamp_ = ros::Time::now();

}


KalmanFilterPosVelAccBias::~KalmanFilterPosVelAccBias()
{
}

void KalmanFilterPosVelAccBias::setInputFlag()
{
  boost::lock_guard<boost::mutex> lock(kf_mutex_);
  input_start_flag_ = true;
}

void KalmanFilterPosVelAccBias::setMeasureFlag(bool flag)
{
  boost::lock_guard<boost::mutex> lock(kf_mutex_);
  measure_start_flag_ = flag;
}


bool KalmanFilterPosVelAccBias::prediction(double input, ros::Time stamp)
{
  boost::lock_guard<boost::mutex> lock(kf_mutex_);
  if(getFilteringFlag())
    {

      Eigen::Vector3d estimate_hat_state
        = state_transition_model_ * estimate_state_ + input * control_input_model_;
      estimate_state_ = estimate_hat_state;

      Eigen::Matrix3d estimate_bar_covariance 
        = state_transition_model_ * estimate_covariance_ * state_transition_model_.transpose() 
        + prediction_noise_covariance_;
      estimate_covariance_ = estimate_bar_covariance;

      //debug
      Eigen::Vector3d predict_hat_state
        = state_transition_model_ * predict_state_ + input * control_input_model_;
      predict_state_ = predict_hat_state;

      kalman_filter_stamp_ = stamp;

      return true;
    }
  else 
    return false;
}

//for bad measurement step
void KalmanFilterPosVelAccBias::imuQuOnlyPrediction(ros::Time check_time_stamp)
{
  if(getFilteringFlag())
    {
      while(1)
        {
          if(!imuQuPrediction(check_time_stamp))
            break;
        }
    }
  return;
}



double KalmanFilterPosVelAccBias::correction(double measurement, ros::Time stamp)
{
  boost::lock_guard<boost::mutex> lock(kf_mutex_);
  if(getFilteringFlag())
    {

      inovation_covariance_ = observation_model_ * estimate_covariance_ * observation_model_.transpose()
        + measurement_noise_covariance_;
  

      kalman_gain_ = estimate_covariance_ * observation_model_.transpose() * inovation_covariance_.inverse();


      Eigen::Vector3d estimate_state_tmp 
        = estimate_state_ + kalman_gain_ * (measurement - observation_model_ * estimate_state_);
      estimate_state_ = estimate_state_tmp;
      correct_state_ = estimate_state_tmp;

      Eigen::Matrix3d I; I.setIdentity();
      Eigen::Matrix3d estimate_covariance_tmp;
      estimate_covariance_tmp = (I - kalman_gain_ * observation_model_) * estimate_covariance_;
      estimate_covariance_ = estimate_covariance_tmp;


      return (kalman_filter_stamp_.toSec() - stamp.toSec());
    }
  else 
    {
      return 0;
    }
}

double KalmanFilterPosVelAccBias::correctionOnlyVelocity(double measurement, ros::Time stamp)
{
  boost::lock_guard<boost::mutex> lock(kf_mutex_);
  if(getFilteringFlag())
    {

      inovation_covariance_ = observation_only_velocity_model_ * estimate_covariance_ * observation_only_velocity_model_.transpose()
        + measurement_only_velocity_noise_covariance_;

      kalman_gain_ = estimate_covariance_ * observation_only_velocity_model_.transpose() * inovation_covariance_.inverse();


      Eigen::Vector3d estimate_state_tmp 
        = estimate_state_ + kalman_gain_ * (measurement - observation_only_velocity_model_ * estimate_state_);
      estimate_state_ = estimate_state_tmp;
      correct_state_ = estimate_state_tmp;

      Eigen::Matrix3d I; I.setIdentity();
      Eigen::Matrix3d estimate_covariance_tmp;
      estimate_covariance_tmp = (I - kalman_gain_ * observation_only_velocity_model_) * estimate_covariance_;
      estimate_covariance_ = estimate_covariance_tmp;

      return (kalman_filter_stamp_.toSec() - stamp.toSec());
    }
  else 
    {
      return 0;
    }
}
  


void KalmanFilterPosVelAccBias::imuQuPush(aerial_robot_base::ImuQuPtr imu_qu_msg_ptr)
{
  boost::lock_guard<boost::mutex> lock(queue_mutex_);
  if(getFilteringFlag())
    {
      imu_qu_.push(imu_qu_msg_ptr);
    }
  return;
}

bool KalmanFilterPosVelAccBias::imuQuPrediction(ros::Time check_time_stamp)
{
  boost::lock_guard<boost::mutex> lock(queue_mutex_);
  if(imu_qu_.empty())
    {
      return false;
    }

  if(check_time_stamp.toSec() > (imu_qu_.front()->stamp.toSec()))
    { 
      prediction((double)imu_qu_.front()->acc, imu_qu_.front()->stamp);
      imu_qu_.pop();

      return true;
    }
  else
    return false;
}

//for time synchronized state
void KalmanFilterPosVelAccBias::imuQuCorrection(ros::Time check_time_stamp, double measurement, int type)
{
  if(getFilteringFlag())
    {
      while(1)
        {
          if(!imuQuPrediction(check_time_stamp))
            break;
        }

      if(type == 0) // position measurement
        correction(measurement, check_time_stamp);
      if(type == 1) // velocity measurement
        correctionOnlyVelocity(measurement, check_time_stamp);
    }
  return;
}

void KalmanFilterPosVelAccBias::cfgCallback(aerial_robot_base::StateKalmanFilterConfig &config, uint32_t level)
{

  if(config.kalmanFilterFlag == true)
    {
      printf("%s axis kf with bias, ", id_.c_str());
      switch(level)
        {
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_INPUT_SIGMA:
          sigma_acc_ = config.inputSigma;
          //+*+*+*+*+ confirm?? 
          prediction_noise_covariance_ 
            = (sigma_acc_ * sigma_acc_) * control_input_model_ * control_input_model_.transpose();
          prediction_noise_covariance_(2,2) = sigma_acc_bias_ * sigma_acc_bias_;
          printf("change the input sigma\n");
          std::cout << "Prediction Noise Covariance :\n" << prediction_noise_covariance_ << std::endl;
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_BIAS_SIGMA:
          sigma_acc_bias_ = config.biasSigma;
          prediction_noise_covariance_ 
            = (sigma_acc_ * sigma_acc_) * control_input_model_ * control_input_model_.transpose();
          prediction_noise_covariance_(2,2) = sigma_acc_bias_ * sigma_acc_bias_;
          printf("change the bias sigma\n");
          std::cout << "Prediction Noise Covariance :\n" << prediction_noise_covariance_ << std::endl;
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_MEASURE_SIGMA:
          sigma_laser_   = config.measureSigma;
          measurement_noise_covariance_(0) = sigma_laser_ * sigma_laser_;
          printf("change the measure sigma\n");
          break;
        default :
          printf("\n");
          break;
        }
    }

}

void KalmanFilterPosVelAccBias::getEstimateCovariance(float* covarianceMatrix)
{
  covarianceMatrix[0] = estimate_covariance_(0,0);
  covarianceMatrix[1] = estimate_covariance_(0,1);
  covarianceMatrix[2] = estimate_covariance_(0,2);
  covarianceMatrix[3] = estimate_covariance_(1,0);
  covarianceMatrix[4] = estimate_covariance_(1,1);
  covarianceMatrix[5] = estimate_covariance_(1,2);
  covarianceMatrix[6] = estimate_covariance_(2,0);
  covarianceMatrix[7] = estimate_covariance_(2,1);
  covarianceMatrix[8] = estimate_covariance_(2,2);
}

void KalmanFilterPosVelAccBias::setInitImuBias(double init_bias)
{
  //set bias
  estimate_state_(2) = init_bias; //bias inital
  correct_state_(2) = init_bias; //bias initial
  predict_state_(2) = init_bias; //bias initial

  // start filtering . danger!
  setInputFlag();
}



void KalmanFilterPosVelAccBias::setInitState(double init_pos, double init_vel)
{
  estimate_state_(0) = init_pos; //position initial
  estimate_state_(1) = init_vel; //position initial

  correct_state_(0) = init_pos; //position initial
  correct_state_(1) = init_vel; //position initial

  predict_state_(0) = init_pos; //position initial
  predict_state_(1) = init_vel; //position initial

}

void KalmanFilterPosVelAccBias::rosParamInit()
{
  std::string ns = nhp_axis_.getNamespace();
  ns.append(" with bias");
  if (!nhp_axis_.getParam ("input_sigma", input_sigma_))
    input_sigma_ = 0.0;
  printf("%s: input_sigma_ is %.4f\n", ns.c_str(), input_sigma_);
  if (!nhp_axis_.getParam ("bias_sigma", bias_sigma_))
    bias_sigma_ = 0.0;
  printf("%s: bias_sigma_ is %.4f\n", ns.c_str(), bias_sigma_);
  if (!nhp_axis_.getParam ("measure_sigma", measure_sigma_))
    measure_sigma_ = 0.0;
  printf("%s: measure_sigma_ is %.4f\n", ns.c_str(), measure_sigma_);
  if (!nhp_.getParam ("imu_hz", imu_hz_))
    imu_hz_ = 100;
  printf("%s: imu_hz_ is %.4f\n",nhp_.getNamespace().c_str(), imu_hz_);

}

