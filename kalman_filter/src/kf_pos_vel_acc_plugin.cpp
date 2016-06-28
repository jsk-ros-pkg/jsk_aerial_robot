#include <pluginlib/class_list_macros.h>
#include <kalman_filter/kf_base_plugin.h>
#include <kalman_filter/kf_pos_vel_acc_plugin.h>


namespace kf_pos_vel_acc_plugin
{
  void KalmanFilterPosVelAcc::initialize(ros::NodeHandle nh, string suffix, int id)
  {
    state_dim_ = 2;
    input_dim_ = 1;
    measure_dim_ = 1;
    baseInit(nh, suffix, id);

    //cfg init
    server_ = new dynamic_reconfigure::Server<kalman_filter::KalmanFilterPosVelAccConfig>(nhp_);
    dynamic_reconf_func_ = boost::bind(&KalmanFilterPosVelAcc::cfgCallback, this, _1, _2);
    server_->setCallback(dynamic_reconf_func_);

  }

  void KalmanFilterPosVelAcc::setCorrectMode(uint8_t correct_mode)
  {
    correct_mode_ = correct_mode;

    Matrix<double, 1 ,2>   observation_model;
    if(correct_mode_ == CORRECT_POS) observation_model << 1, 0;
    else if(correct_mode_ == CORRECT_VEL) observation_model << 0, 1;
    setObservationModel(observation_model);
  }

  void KalmanFilterPosVelAcc::updateModelFromDt(double dt)
  {
    dt_ = dt;

    Matrix2d state_transition_model; 
    state_transition_model << 1, dt_, 0, 1;
    setStateTransitionModel(state_transition_model);

    Matrix<double, 2 ,1> control_input_model; 
    control_input_model << (dt_ * dt_)/2, dt_;
    setControlInputModel(control_input_model);

    Matrix<double, 1 ,2>   observation_model;
    if(correct_mode_ == CORRECT_POS) observation_model << 1, 0;
    else if(correct_mode_ == CORRECT_VEL) observation_model << 0, 1;
    setObservationModel(observation_model);


    // std::cout << "estimate_state_" << std::endl <<  estimate_state_ << std::endl;
    // std::cout << "kalman_gain_" << std::endl << kalman_gain_  << std::endl;

    // std::cout << "prediction noise covariance_" << std::endl << prediction_noise_covariance_  << std::endl;

    // std::cout << "state_transition_model_" << std::endl <<  state_transition_model_ << std::endl;
    // std::cout << "control_input_model_" << std::endl << control_input_model_  << std::endl;
    // std::cout << "observation_model_" << std::endl << observation_model_  << std::endl;

    // std::cout << "estimate_covariance_" << std::endl << estimate_covariance_  << std::endl;
    // std::cout << "measurement_noise_covariance_" << std::endl << measurement_noise_covariance_  << std::endl;
    // std::cout << "inovation_covariance_" << std::endl << inovation_covariance_  << std::endl;

  }

  void KalmanFilterPosVelAcc::cfgCallback(kalman_filter::KalmanFilterPosVelAccConfig &config, uint32_t level)
  {
    if(config.kalmanFilterFlag == true)
      {
        printf("cfg update, node: %s ", (nhp_.getNamespace()).c_str());

        switch(level)
          {
          case 1:  // INPUT_SIGMA = 1
            input_sigma_(0,0) = config.inputSigma;
            setPredictionNoiseCovariance();
            printf("change the input sigma\n");
            break;
          case 3:  // MEASURE_SIGMA = 3
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

  void KalmanFilterPosVelAccBias::initialize(ros::NodeHandle nh, string suffix, int id)
  {
    state_dim_ = 3;
    input_dim_ = 2;
    measure_dim_ = 1;
    baseInit(nh, suffix, id);

    //cfg init
    server_ = new dynamic_reconfigure::Server<kalman_filter::KalmanFilterPosVelAccBiasConfig>(nhp_);
    dynamic_reconf_func_ = boost::bind(&KalmanFilterPosVelAccBias::cfgCallback, this, _1, _2);
    server_->setCallback(dynamic_reconf_func_);

  }

  void KalmanFilterPosVelAccBias::setCorrectMode(uint8_t correct_mode)
  {
    correct_mode_ = correct_mode;

    Matrix<double, 1, 3> observation_model;
    if(correct_mode_ == CORRECT_POS) observation_model << 1, 0, 0;
    else if(correct_mode_ == CORRECT_VEL) observation_model << 0, 1, 0;
    setObservationModel(observation_model);
  }

  void KalmanFilterPosVelAccBias::updateModelFromDt(double dt)
  {
    dt_ = dt;

    Matrix3d state_transition_model; 
    state_transition_model << 1, dt_, -dt_*dt_/2, 0, 1, -dt_, 0, 0, 1;
    setStateTransitionModel(state_transition_model);

    Matrix<double, 3, 2> control_input_model; 
    control_input_model << (dt_ * dt_)/2, 0, dt_, 0, 0, 1;
    setControlInputModel(control_input_model);

    Matrix<double, 1, 3> observation_model;
    if(correct_mode_ == CORRECT_POS) observation_model << 1, 0, 0;
    else if(correct_mode_ == CORRECT_VEL) observation_model << 0, 1, 0;
    setObservationModel(observation_model);

    //std::cout << "estimate_state_" << std::endl <<  estimate_state_ << std::endl;
    // std::cout << "kalman_gain_" << std::endl << kalman_gain_  << std::endl;

    // std::cout << "prediction noise covariance_" << std::endl << prediction_noise_covariance_  << std::endl;

    // std::cout << "state_transition_model_" << std::endl <<  state_transition_model_ << std::endl;
    // std::cout << "control_input_model_" << std::endl << control_input_model_  << std::endl;
    // std::cout << "observation_model_" << std::endl << observation_model_  << std::endl;

    // std::cout << "estimate_covariance_" << std::endl << estimate_covariance_  << std::endl;
    // std::cout << "measurement_noise_covariance_" << std::endl << measurement_noise_covariance_  << std::endl;
    // std::cout << "inovation_covariance_" << std::endl << inovation_covariance_  << std::endl;

  }

  void KalmanFilterPosVelAccBias::cfgCallback(kalman_filter::KalmanFilterPosVelAccBiasConfig &config, uint32_t level)
  {
    if(config.kalmanFilterFlag == true)
      {
        printf("cfg update, node: %s", (nhp_.getNamespace()).c_str());

        switch(level)
          {
          case 1:  // INPUT_SIGMA = 1
            input_sigma_(0,0) = config.input1Sigma;
            setPredictionNoiseCovariance();
            printf("change the input1 sigma\n");
            break;
          case 2:  // BIAS_SIGMA = 2
            input_sigma_(1,0) = config.input2Sigma;
            setPredictionNoiseCovariance();
            printf("change the input2 sigma\n");
            break;
          case 3:  // MEASURE_SIGMA = 3
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

};


PLUGINLIB_EXPORT_CLASS(kf_pos_vel_acc_plugin::KalmanFilterPosVelAcc, kf_base_plugin::KalmanFilter);
PLUGINLIB_EXPORT_CLASS(kf_pos_vel_acc_plugin::KalmanFilterPosVelAccBias, kf_base_plugin::KalmanFilter);
