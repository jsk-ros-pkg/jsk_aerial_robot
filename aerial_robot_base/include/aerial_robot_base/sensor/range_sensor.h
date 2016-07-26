#ifndef RNAGE_SENSOR_MODULE_H
#define RNAGE_SENSOR_MODULE_H

#include <ros/ros.h>
#include <aerial_robot_base/basic_state_estimation.h>
#include <aerial_robot_base/sensor/sensor_base_plugin.h>
#include <aerial_robot_base/States.h>
#include <sensor_msgs/Range.h>

namespace sensor_plugin
{
  class RangeSensor :public sensor_base_plugin::SensorBase
  {
  public:
    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator)
    {
      nh_ = ros::NodeHandle(nh, "range_sensor");
      nhp_ = ros::NodeHandle(nhp, "range_sensor");
      estimator_ = estimator;
      baseRosParamInit();
      rosParamInit();

      range_sensor_pub_ = nh_.advertise<aerial_robot_base::States>("data",10);
      range_sensor_sub_ = nh_.subscribe<sensor_msgs::Range>(range_sensor_sub_name_, 1, &RangeSensor::rangeCallback, this, ros::TransportHints().tcpNoDelay());

      raw_pos_z_ = 0;
      prev_raw_pos_z_ = 0;
      raw_vel_z_ = 0;

      height_offset_ = 0;

    }

    ~RangeSensor() {}

    RangeSensor() {}
  private:
    ros::Subscriber range_sensor_sub_;
    ros::Publisher range_sensor_pub_;

    std::string range_sensor_sub_name_;
    double range_noise_sigma_;
    float raw_pos_z_, prev_raw_pos_z_;
    float raw_vel_z_;
    float height_init_offset_;
    int calibrate_cnt_;

    //for terrain check
    double ground_variance_of_noise_sigma_; // 10 times
    double check_duration_;
    float height_offset_;

    void rangeCallback(const sensor_msgs::RangeConstPtr & range_msg)
    {

      static int init_flag = true;
      static int calibrate_cnt = 0;
      static double previous_secs;
      double current_secs = range_msg->header.stamp.toSec();

      //**** Optical flow によるmeasuring flag のenable化
      if(init_flag)
        {
          if(estimate_mode_ & (1 << EGOMOTION_ESTIMATION_MODE))
            {
              for(int i = 0; i < estimator_->getFuserEgomotionNo(); i++)

                {
                  if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Z_W)))
                    estimator_->getFuserEgomotion(i)->setMeasureFlag();
                }
            }
          if(estimate_mode_ & (1 << EXPERIMENT_MODE))
            {

              for(int i = 0; i < estimator_->getFuserExperimentNo(); i++)
                {
                  if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Z_W)))

                    estimator_->getFuserExperiment(i)->setMeasureFlag();
                }
            }
          calibrate_cnt = calibrate_cnt_;
          init_flag = false;
        }


      //**** calibrate phase
      if(calibrate_cnt > 0)
        {
          calibrate_cnt--;
          height_offset_ += range_msg->range;
          if(calibrate_cnt == 0) 
            {
              height_offset_ /= (float)calibrate_cnt_;
              ROS_WARN("range_sensor: %f", height_offset_);
            }
          return;
        }

      //**** 高さ方向情報の更新
      raw_pos_z_ = range_msg->range - height_offset_;
      raw_vel_z_ = (raw_pos_z_ - prev_raw_pos_z_) / (current_secs - previous_secs);

      Eigen::Matrix<double, 1, 1> sigma_temp = Eigen::MatrixXd::Zero(1, 1);
      Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1);

      if(estimate_mode_ & (1 << EGOMOTION_ESTIMATION_MODE))
        {
          for(int i = 0; i < estimator_->getFuserEgomotionNo(); i++)
            {
              if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Z_W)))
                {
                  sigma_temp(0,0) = range_noise_sigma_;
                  estimator_->getFuserEgomotion(i)->setMeasureSigma(sigma_temp);
                  temp(0, 0) = raw_pos_z_;
                  estimator_->getFuserEgomotion(i)->correction(temp);

                }
            }
        }

      if(estimate_mode_ & (1 << EXPERIMENT_MODE))
        {
          for(int i = 0; i < estimator_->getFuserExperimentNo(); i++)
            {
              if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Z_W)))
                {
                  sigma_temp(0,0) = range_noise_sigma_;
                  estimator_->getFuserExperiment(i)->setMeasureSigma(sigma_temp);
                  temp(0, 0) = raw_pos_z_;
                  estimator_->getFuserExperiment(i)->correction(temp);
                }
            }
        }

      //publish
      aerial_robot_base::States range_data;
      range_data.header.stamp = range_msg->header.stamp;

      aerial_robot_base::State z_state;
      z_state.id = "z";
      z_state.raw_pos = raw_pos_z_;
      z_state.raw_vel = raw_vel_z_;

      Eigen::Matrix<double,2,1> kf_x_state;
      Eigen::Matrix<double,2,1> kf_y_state;
      Eigen::Matrix<double,2,1> kf_z_state;
      Eigen::Matrix<double,3,1> kfb_x_state;
      Eigen::Matrix<double,3,1> kfb_y_state;
      Eigen::Matrix<double,3,1> kfb_z_state;

      if(estimate_mode_ & (1 << EGOMOTION_ESTIMATION_MODE))
        {
          for(int i = 0; i < estimator_->getFuserEgomotionNo(); i++)
            {
              if(estimator_->getFuserEgomotionPluginName(i) == "kalman_filter/kf_pose_vel_acc_bias")
                {//temporary
                  if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Z_W))) 
                    {
                      kfb_z_state = estimator_->getFuserEgomotion(i)->getEstimateState();
                      estimator_->setEEState(BasicEstimator::Z_W, 0, kfb_z_state(0,0));
                      estimator_->setEEState(BasicEstimator::Z_W, 1, kfb_z_state(1,0));
                    }
                }
              if(estimator_->getFuserEgomotionPluginName(i) == "kalman_filter/kf_pose_vel_acc")
                {//temporary
                  if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Z_W))) 
                    {
                      kf_z_state = estimator_->getFuserEgomotion(i)->getEstimateState();
                      estimator_->setEEState(BasicEstimator::Z_W, 0, kf_z_state(0,0));
                      estimator_->setEEState(BasicEstimator::Z_W, 1, kf_z_state(1,0));
                    }
                }
            }
        }
      else if(estimate_mode_ & (1 << EXPERIMENT_MODE))
        {
          for(int i = 0; i < estimator_->getFuserExperimentNo(); i++)
            {
              if(estimator_->getFuserExperimentPluginName(i) == "kalman_filter/kf_pose_vel_acc")
                {//temporary
                  if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Z_W))) 
                    {
                      kf_z_state = estimator_->getFuserExperiment(i)->getEstimateState();
                      estimator_->setEXState(BasicEstimator::Z_W, 0, kf_z_state(0,0));
                      estimator_->setEXState(BasicEstimator::Z_W, 1, kf_z_state(1,0));
                    }
                }
            }
        }

      z_state.kf_pos = kf_z_state(0, 0);
      z_state.kf_vel = kf_z_state(1, 0);
      z_state.kfb_pos = kfb_z_state(0, 0);
      z_state.kfb_vel = kfb_z_state(1, 0);
      range_data.states.push_back(z_state);
      range_sensor_pub_.publish(range_data);

      //更新
      previous_secs = current_secs;
      prev_raw_pos_z_ = raw_pos_z_;
    }

    void terrainCheck()
    {
      //TODO: time 1s -> check the new ground with 10x sigma(e.g. 10x 0.005 = 0.05)
    }


    void rosParamInit()
    {
      nhp_.param("range_sensor_sub_name", range_sensor_sub_name_, std::string("distance"));
      printf("range noise sigma  is %s\n", range_sensor_sub_name_.c_str());

      nhp_.param("range_noise_sigma", range_noise_sigma_, 0.005 );
      printf("range noise sigma  is %f\n", range_noise_sigma_);

      //for terrain check
      nhp_.param("ground_variance_of_noise_sigma", ground_variance_of_noise_sigma_, 10.0 );
      printf("ground variance of  noise sigma  is %f\n", ground_variance_of_noise_sigma_);

      nhp_.param("check_duration", check_duration_, 1.0);
      printf("check duration  is %f\n", check_duration_); 

      nhp_.param("calibrate_cnt", calibrate_cnt_, 100);
      printf("check duration  is %d\n", calibrate_cnt_); 

    }
  };

};
#endif



