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

    Matrix<double, 2 ,1>   observation_model;
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

    Matrix<double, 2 ,1>   observation_model;
    if(correct_mode_ == CORRECT_POS) observation_model << 1, 0;
    else if(correct_mode_ == CORRECT_VEL) observation_model << 0, 1;
    setObservationModel(observation_model);
  }

  void KalmanFilterPosVelAcc::cfgCallback(kalman_filter::KalmanFilterPosVelAccConfig &config, uint32_t level)
  {
    if(config.kalmanFilterFlag == true)
      {
        printf("cfg update, node: %s ", (nhp_.getNamespace()).c_str());

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

    Matrix<double, 3, 1> observation_model;
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

    Matrix<double, 3, 1> observation_model;
    if(correct_mode_ == CORRECT_POS) observation_model << 1, 0, 0;
    else if(correct_mode_ == CORRECT_VEL) observation_model << 0, 1, 0;
    setObservationModel(observation_model);

  }

  void KalmanFilterPosVelAccBias::cfgCallback(kalman_filter::KalmanFilterPosVelAccBiasConfig &config, uint32_t level)
  {
    if(config.kalmanFilterFlag == true)
      {
        printf("cfg update, node: %s", (nhp_.getNamespace()).c_str());

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

};


PLUGINLIB_EXPORT_CLASS(kf_pos_vel_acc_plugin::KalmanFilterPosVelAcc, kf_base_plugin::KalmanFilter);
PLUGINLIB_EXPORT_CLASS(kf_pos_vel_acc_plugin::KalmanFilterPosVelAccBias, kf_base_plugin::KalmanFilter);
