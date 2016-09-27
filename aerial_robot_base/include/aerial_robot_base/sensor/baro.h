#ifndef BAROMETER_MODULE_H
#define BAROMETER_MODULE_H

#include <ros/ros.h>
#include <aerial_robot_base/basic_state_estimation.h>
#include <aerial_robot_base/sensor/sensor_base_plugin.h>
#include <kalman_filter/digital_filter.h>
#include <aerial_robot_base/States.h>
#include <aerial_robot_msgs/Barometer.h>

namespace sensor_plugin
{
  class Barometer :public sensor_base_plugin::SensorBase
  {
  public:
    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator)
    {
      nh_ = ros::NodeHandle(nh, "barometer");
      nhp_ = ros::NodeHandle(nhp, "barometer");
      estimator_ = estimator;
      baseRosParamInit();
      rosParamInit();

      barometer_pub_ = nh_.advertise<aerial_robot_base::States>("data",10);
      barometer_sub_ = nh_.subscribe<aerial_robot_msgs::Barometer>(barometer_sub_name_, 1, &Barometer::baroCallback, this, ros::TransportHints().tcpNoDelay());

      kf_loader_ptr_ = boost::shared_ptr< pluginlib::ClassLoader<kf_base_plugin::KalmanFilter> >(new pluginlib::ClassLoader<kf_base_plugin::KalmanFilter>("kalman_filter", "kf_base_plugin::KalmanFilter"));
      baro_bias_kf_  = kf_loader_ptr_->createInstance("aerial_robot_base/kf_baro_bias");
      baro_bias_kf_->initialize(nh_, std::string(""), 0);

      iir_filter_ = IirFilter((float)rx_freq_, (float)cutoff_freq_);
      iir_high_filter_ = IirFilter((float)rx_freq_, (float)high_cutoff_freq_);

      raw_pos_z_ = 0;
      prev_raw_pos_z_ = 0;
      raw_vel_z_ = 0;
      pos_z_ = 0;
      prev_pos_z_ = 0;
      vel_z_ = 0;
      high_filtered_pos_z_ = 0;
      prev_high_filtered_pos_z_ = 0;
      high_filtered_vel_z_ = 0;
      inflight_state_ = false;
      temp_ = 0;
    }

    ~Barometer() {}

    Barometer() {}
  private:
    ros::Subscriber barometer_sub_;
    ros::Publisher barometer_pub_;

    std::string barometer_sub_name_;
    double baro_noise_sigma_, baro_bias_noise_sigma_;

    /* the kalman filter for the baro bias estimation */
    boost::shared_ptr< pluginlib::ClassLoader<kf_base_plugin::KalmanFilter> > kf_loader_ptr_;
    boost::shared_ptr<kf_base_plugin::KalmanFilter> baro_bias_kf_;

    /* the iir filter for the barometer for the first filtering stage */
    IirFilter iir_filter_;
    IirFilter iir_high_filter_;
    double rx_freq_;
    double cutoff_freq_, high_cutoff_freq_;

    /* base variables */
    bool inflight_state_; //the flag for the inflight state
    double raw_pos_z_, prev_raw_pos_z_;
    double pos_z_, prev_pos_z_;
    double raw_vel_z_, vel_z_;
    double high_filtered_pos_z_, prev_high_filtered_pos_z_, high_filtered_vel_z_;
    double temp_; // the temperature of the chip

    void baroCallback(const aerial_robot_msgs::BarometerConstPtr & baro_msg)
    {
      static int init_flag = true;
      static int calibrate_cnt = 0;
      static double previous_secs;
      double current_secs = baro_msg->stamp.toSec();

      raw_pos_z_ = baro_msg->altitude;
      temp_ = baro_msg->temperature;

      /*First Filtering: IIR filter */
      /* position */
      iir_filter_.filterFunction(raw_pos_z_, pos_z_);
      iir_high_filter_.filterFunction(raw_pos_z_, high_filtered_pos_z_);
      /* velocity */
      raw_vel_z_ = (raw_pos_z_ - prev_raw_pos_z_)/(current_secs - previous_secs);
      vel_z_ = (pos_z_ - prev_pos_z_)/(current_secs - previous_secs);
      high_filtered_vel_z_ = (high_filtered_pos_z_ - prev_high_filtered_pos_z_)/(current_secs - previous_secs);

      /* the true initial phase for baro based estimattion for inflight state */
      /* since the value of pressure will decrease during the rising of the propeller rotation speed */
      if(estimator_->getFlyingFlag() && high_filtered_vel_z_ > 0)
        {
          inflight_state_ = true;
          ROS_WARN("barometer: start the inflight barometer height estimation");

          /* the initialization of the baro bias kf filter */
          Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1); 
          temp(0,0) = baro_bias_noise_sigma_;
          baro_bias_kf_->setInputSigma(temp);
          baro_bias_kf_->setMeasureFlag();
          baro_bias_kf_->setInitState(-pos_z_, 0);
        }

      /* the estimate phase: bias or height estimate */
      Eigen::Matrix<double, 1, 1> sigma_temp = Eigen::MatrixXd::Zero(1, 1);
      Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1);

      Eigen::Matrix<double,2,1> kf_z_state;
      Eigen::Matrix<double,3,1> kfb_z_state;

      switch(estimator_->getHeightEstimateMode())
        {
        case(BasicEstimator::ONLY_BARO_MODE):
          if(estimate_mode_ & (1 << EGOMOTION_ESTIMATION_MODE))
            {
              for(int i = 0; i < estimator_->getFuserEgomotionNo(); i++)
                {
                  if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Z_W)))
                    {
                      if(!estimator_->getFuserEgomotion(i)->getFilteringFlag())
                        {
                          ROS_FATAL("baro: can not estiamte the height by baro(ONLY_BARO_MODE), because the filtering flag is not activated");
                          return;
                        }
                      /* We should set the sigma every time, since we may have several different sensors to correct the kalman filter(e.g. vo + opti, laser + baro) */
                      sigma_temp(0,0) = baro_noise_sigma_;
                      estimator_->getFuserEgomotion(i)->setMeasureSigma(sigma_temp);
                      temp(0, 0) = pos_z_ + (baro_bias_kf_->getEstimateState())(0,0);
                      estimator_->getFuserEgomotion(i)->correction(temp);

                      /* set the state */
                      if(estimator_->getFuserEgomotionPluginName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                        {
                          if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Z_W))) 
                            {
                              kfb_z_state = estimator_->getFuserEgomotion(i)->getEstimateState();
                              estimator_->setEEState(BasicEstimator::Z_W, 0, kfb_z_state(0,0));
                              estimator_->setEEState(BasicEstimator::Z_W, 1, kfb_z_state(1,0));
                            }
                        }
                      if(estimator_->getFuserEgomotionPluginName(i) == "kalman_filter/kf_pose_vel_acc")
                        {
                          if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Z_W))) 
                            {
                              kf_z_state = estimator_->getFuserEgomotion(i)->getEstimateState();
                              estimator_->setEEState(BasicEstimator::Z_W, 0, kf_z_state(0,0));
                              estimator_->setEEState(BasicEstimator::Z_W, 1, kf_z_state(1,0));
                            }
                        }

                    }
                }
            }
          if(estimate_mode_ & (1 << EXPERIMENT_MODE))
            {
              for(int i = 0; i < estimator_->getFuserExperimentNo(); i++)
                {
                  if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Z_W)))
                    {
                      if(!estimator_->getFuserExperiment(i)->getFilteringFlag())
                        {
                          ROS_FATAL("baro: can not estiamte the height by baro(ONLY_BARO_MODE), because the filtering flag is not activated");
                          return;
                        }

                      sigma_temp(0,0) = baro_noise_sigma_;
                      estimator_->getFuserExperiment(i)->setMeasureSigma(sigma_temp);
                      temp(0, 0) = pos_z_ + (baro_bias_kf_->getEstimateState())(0,0);
                      estimator_->getFuserExperiment(i)->correction(temp);

                      if(estimator_->getFuserExperimentPluginName(i) == "kalman_filter/kf_pose_vel_acc")
                        {
                          if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Z_W))) 
                            {
                              kf_z_state = estimator_->getFuserExperiment(i)->getEstimateState();
                              estimator_->setEXState(BasicEstimator::Z_W, 0, kf_z_state(0,0));
                              estimator_->setEXState(BasicEstimator::Z_W, 1, kf_z_state(1,0));
                            }
                        }
                    }
                }
            }
          break;
        case(BasicEstimator::WITHOUT_BARO_MODE):
          baro_bias_kf_->prediction(Eigen::MatrixXd::Zero(1, 1));
          temp(0, 0) = estimator_->getEEState(BasicEstimator::Z_W, 0) - pos_z_;
          baro_bias_kf_->correction(temp);
          break;
        case(BasicEstimator::WITH_BARO_MODE):
          //TODO: this is another part, maybe we have to use another package:
          //http://wiki.ros.org/ethzasl_sensor_fusion
          break;
        default:
          break;
        }

      //publish
      aerial_robot_base::States baro_data;
      baro_data.header.stamp = baro_msg->stamp;

      aerial_robot_base::State z_state;
      z_state.id = "z";
      z_state.raw_pos = raw_pos_z_;
      z_state.raw_vel = raw_vel_z_;
      z_state.pos = pos_z_;
      z_state.vel = vel_z_;
      z_state.kf_pos = kf_z_state(0, 0);
      z_state.kf_vel = kf_z_state(1, 0);
      z_state.kfb_pos = kfb_z_state(0, 0);
      z_state.kfb_vel = kfb_z_state(1, 0);
      z_state.kfb_bias = (baro_bias_kf_->getEstimateState())(0,0);
      z_state.reserves.push_back(high_filtered_pos_z_);
      z_state.reserves.push_back(high_filtered_vel_z_);
      baro_data.states.push_back(z_state);
      barometer_pub_.publish(baro_data);

      //更新
      previous_secs = current_secs;
      prev_raw_pos_z_ = raw_pos_z_;
      prev_pos_z_ = pos_z_;
      prev_high_filtered_pos_z_ = high_filtered_pos_z_;
    }

    void rosParamInit()
    {
      nhp_.param("barometer_sub_name", barometer_sub_name_, std::string("distance"));
      printf("barometer sub name is %s\n", barometer_sub_name_.c_str());

      nhp_.param("baro_noise_sigma", baro_noise_sigma_, 0.05 );
      printf("baro noise sigma  is %f\n", baro_noise_sigma_);

      nhp_.param("baro_bias_noise_sigma", baro_bias_noise_sigma_, 0.001 );
      printf("baro_bias noise sigma  is %f\n", baro_bias_noise_sigma_);

      nhp_.param("rx_freq", rx_freq_, 100.0 );
      printf("rx freq of barometer  is %f\n", rx_freq_);

      nhp_.param("cutoff_freq", cutoff_freq_, 10.0 );
      printf("cutoff freq of barometer  is %f\n", cutoff_freq_);

      nhp_.param("high_cutoff_freq", high_cutoff_freq_, 1.0 );
      printf("high_cutoff freq of barometer  is %f\n", high_cutoff_freq_);
    }
  };

};
#endif



