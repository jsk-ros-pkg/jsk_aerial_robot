#include "kalman_filter/kf_pos_vel_acc.h"

KalmanFilterPosVelAcc::KalmanFilterPosVelAcc(ros::NodeHandle nh, ros::NodeHandle nh_private, std::string filter_id, uint8_t correct_mode): KalmanFilter<2, 1, 1>(nh, nh_private), nhp_axis_(nh_private, "kalman_filter/" + filter_id), id_(filter_id)
{
  //this->KalmanFilter<2, 1, 1>(nh, nh_private);
  rosParamInit();

  //init 
  correct_mode_ = correct_mode;

  server_ = new dynamic_reconfigure::Server<kalman_filter::KalmanFilterPosVelAccConfig>(nhp_axis_);
  dynamic_reconf_func_ = boost::bind(&KalmanFilterPosVelAcc::cfgCallback, this, _1, _2);
  server_->setCallback(dynamic_reconf_func_);

  Matrix2d state_transition_model; 
  state_transition_model << 1, dt_, 0, 1;
  setStateTransitionModel(state_transition_model);

  Matrix<double, 2 ,1> control_input_model; 
  control_input_model << (dt_ * dt_)/2, dt_;
  setControlInputModel(control_input_model);

  Matrix<double, 2 ,1>   observation_model;
  if(correct_mode_ == CORRECT_POS) observation_model << 1, 0;
  else if(correct_mode_ == CORRECT_VEL) observation_model << 0, 1;
  setObservationModel(observation_model);

  setPredictionNoiseCovariance();
  setMeasurementNoiseCovariance();
}

void KalmanFilterPosVelAcc::cfgCallback(kalman_filter::KalmanFilterPosVelAccConfig &config, uint32_t level)
{

  if(config.kalmanFilterFlag == true)
    {
      printf("%s axis kf without bias, ", id_.c_str());

      switch(level)
        {
        case kalman_filter::KalmanFilterPosVelAccConf::RECONFIGURE_INPUT_SIGMA:
          input_sigma_(0,0) = config.inputSigma;
          setPredictionNoiseCovariance();
          printf("change the input sigma\n");
          break;
        case kalman_filter::KalmanFilterPosVelAccConf::RECONFIGURE_MEASURE_SIGMA:
          measure_sigma_(0,0)  = config.measureSigma;
          setMeasurementNoiseCovariance();
          printf("change the measure sigma\n");
          break;
        default :
          printf("\n");
          break;
        }
    }

}

void KalmanFilterPosVelAcc::rosParamInit()
{
  std::string ns = nhp_axis_.getNamespace();
  ns.append(" without bias");
  if (!nhp_axis_.getParam ("input_sigma", input_sigma_(0,0)))
    input_sigma_(0,0) = 0.0;
  printf("%s: input_sigma_ is %.4f\n", ns.c_str(), input_sigma_(0,0));
  if (!nhp_axis_.getParam ("measure_sigma", measure_sigma_(0,0)))
    measure_sigma_(0,0) = 0.0;
  printf("%s: measure_sigma_ is %.4f\n", ns.c_str(), measure_sigma_(0,0));
  if (!nhp_.getParam ("input_hz", input_hz_))
    input_hz_ = 100;
  printf("%s: input_hz_ is %.4f\n",nhp_.getNamespace().c_str(), input_hz_);
}


 void KalmanFilterPosVelAcc::setCorrectMode(uint8_t correct_mode)
{
  correct_mode_ = correct_mode;

  Matrix<double, 2 ,1>   observation_model;
  if(correct_mode_ == CORRECT_POS) observation_model << 1, 0;
  else if(correct_mode_ == CORRECT_VEL) observation_model << 0, 1;
  setObservationModel(observation_model);
}


KalmanFilterPosVelAccBias::KalmanFilterPosVelAccBias(ros::NodeHandle nh, 
                                                   ros::NodeHandle nh_private, 
                                                   std::string filter_id,
                                                     uint8_t correct_mode):KalmanFilter<3, 2, 1>(nh, nh_private), nhp_axis_(nh_private, "kalman_filter/" + filter_id), id_(filter_id) 
{
  rosParamInit();

  //init 
  correct_mode_ = correct_mode;

  server_ = new dynamic_reconfigure::Server<kalman_filter::KalmanFilterPosVelAccBiasConfig>(nhp_axis_);
  dynamic_reconf_func_ = boost::bind(&KalmanFilterPosVelAccBias::cfgCallback, this, _1, _2);
  server_->setCallback(dynamic_reconf_func_);

  Matrix3d state_transition_model; 
  state_transition_model << 1, dt_, -dt_*dt_/2, 0, 1, -dt_, 0, 0, 1;
  setStateTransitionModel(state_transition_model);

  Matrix<double, 3, 2> control_input_model; 
  control_input_model << (dt_ * dt_)/2, 0, dt_, 0, 0, 1;
  setControlInputModel(control_input_model);

  Matrix<double, 3, 1> observation_model;
  if(correct_mode_ == CORRECT_POS) observation_model << 1, 0, 0;
  else if(correct_mode_ == CORRECT_VEL) observation_model << 0, 1, 0;
  setObservationModel(observation_model);

  setPredictionNoiseCovariance();
  setMeasurementNoiseCovariance();


}

void KalmanFilterPosVelAccBias::cfgCallback(kalman_filter::KalmanFilterPosVelAccBiasConfig &config, uint32_t level)
{
  if(config.kalmanFilterFlag == true)
    {
      printf("%s axis kf with bias, ", id_.c_str());

      switch(level)
        {
        case kalman_filter::KalmanFilterPosVelAccConf::RECONFIGURE_INPUT_SIGMA:
          input_sigma_(0,0) = config.input1Sigma;
          setPredictionNoiseCovariance();
          printf("change the input1 sigma\n");
          break;
        case kalman_filter::KalmanFilterPosVelAccConf::RECONFIGURE_BIAS_SIGMA:
          input_sigma_(1,0) = config.input2Sigma;
          setPredictionNoiseCovariance();
          printf("change the input2 sigma\n");
          break;
        case kalman_filter::KalmanFilterPosVelAccConf::RECONFIGURE_MEASURE_SIGMA:
          measure_sigma_(0,0)  = config.measureSigma;
          setMeasurementNoiseCovariance();
          printf("change the measure sigma\n");
          break;
        default :
          printf("\n");
          break;
        }
    }
}

void KalmanFilterPosVelAccBias::setInitImuBias(double init_bias)
{
  boost::lock_guard<boost::mutex> lock(kf_mutex_);
  estimate_state_(2,0) = init_bias;
  correct_state_(2,0) = init_bias;
  predict_state_(2,0) = init_bias;
}

 void KalmanFilterPosVelAccBias::setCorrectMode(uint8_t correct_mode)
{
  correct_mode_ = correct_mode;

  Matrix<double, 3, 1> observation_model;
  if(correct_mode_ == CORRECT_POS) observation_model << 1, 0, 0;
  else if(correct_mode_ == CORRECT_VEL) observation_model << 0, 1, 0;
  setObservationModel(observation_model);
}


void KalmanFilterPosVelAccBias::rosParamInit()
{
  std::string ns = nhp_axis_.getNamespace();
  ns.append(" with bias");
  if (!nhp_axis_.getParam ("input1_sigma", input_sigma_(0,0)))
    input_sigma_(0,0) = 0.0;
  printf("%s: input1_sigma_ is %.4f\n", ns.c_str(), input_sigma_(0,0));
  if (!nhp_axis_.getParam ("input2_sigma", input_sigma_(1,0)))
    input_sigma_(1,0) = 0.0;
  printf("%s: input2_sigma_ is %.4f\n", ns.c_str(), input_sigma_(1,0));
  if (!nhp_axis_.getParam ("measure_sigma", measure_sigma_(0,0)))
    measure_sigma_(0,0) = 0.0;
  printf("%s: measure_sigma_ is %.4f\n", ns.c_str(), measure_sigma_(0,0));
  if (!nhp_.getParam ("input_hz", input_hz_))
    input_hz_ = 100;
  printf("%s: input_hz_ is %.4f\n",nhp_.getNamespace().c_str(), input_hz_);

}

