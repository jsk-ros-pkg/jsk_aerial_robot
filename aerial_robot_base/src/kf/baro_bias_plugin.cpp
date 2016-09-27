#include <pluginlib/class_list_macros.h>
#include <aerial_robot_base/kf/baro_bias_plugin.h>

namespace kf_baro_bias_plugin
{
  void KalmanFilterBaroBias::initialize(ros::NodeHandle nh, string suffix, int id)
  {
    state_dim_ = 1;
    input_dim_ = 1;
    measure_dim_ = 1;
    baseInit(nh, suffix, id);

    //cfg init
    server_ = new dynamic_reconfigure::Server<aerial_robot_base::KalmanFilterBaroBiasConfig>(nhp_);
    dynamic_reconf_func_ = boost::bind(&KalmanFilterBaroBias::cfgCallback, this, _1, _2);
    server_->setCallback(dynamic_reconf_func_);
  }

  void KalmanFilterBaroBias::updateModelFromDt(double dt)
  {
    dt_ = dt;

    Matrix<double, 1, 1> state_transition_model;
    state_transition_model << 0;
    setStateTransitionModel(state_transition_model);

    Matrix<double, 1, 1> control_input_model;
    control_input_model << 1;
    setControlInputModel(control_input_model);

    Matrix<double, 1, 1> observation_model;
    observation_model << 1;
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

  void KalmanFilterBaroBias::cfgCallback(aerial_robot_base::KalmanFilterBaroBiasConfig &config, uint32_t level)
  {
    if(config.kalman_filter_flag == true)
      {
        printf("cfg update, node: %s ", (nhp_.getNamespace()).c_str());

        switch(level)
          {
          case 1:  // INPUT_SIGMA = 1
            input_sigma_(0,0) = config.input_sigma;
            setPredictionNoiseCovariance();
            printf("change the input sigma\n");
            break;
          case 2:  // MEASURE_SIGMA = 2
            measure_sigma_(0,0)  = config.measure_sigma;
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

PLUGINLIB_EXPORT_CLASS(kf_baro_bias_plugin::KalmanFilterBaroBias, kf_base_plugin::KalmanFilter);
