#ifndef OPTICAL_FLOW_MODULE_H
#define OPTICAL_FLOW_MODULE_H

#include <ros/ros.h>
#include <aerial_robot_base/basic_state_estimation.h>
#include <aerial_robot_base/sensor/sensor_base_plugin.h>
#include <aerial_robot_base/sensor/range_sensor.h>
#include <aerial_robot_base/States.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int32.h>

namespace sensor_plugin
{
  class OpticalFlow :public sensor_base_plugin::SensorBase
  {
  public:
    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, std::vector< boost::shared_ptr<sensor_base_plugin::SensorBase> > sensors, std::vector<std::string> sensor_names, int sensor_index)
    {
      baseParamInit(nh, nhp, estimator, sensor_names[sensor_index], sensor_index);

      rosParamInit();

      optical_flow_pub_ = nh_.advertise<aerial_robot_base::States>("data",10);
      optical_flow_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("/opt_flow", 1, &OpticalFlow::opticalFlowCallback, this, ros::TransportHints().tcpNoDelay());

      vel_x_ = 0;
      flow_x_ = 0;

      vel_y_ = 0;
      flow_y_ = 0;

      quality_ = 0;
      metric_scale_ = 0;

      /* find the range sensor */
      std::vector< boost::shared_ptr<sensor_base_plugin::SensorBase> >::iterator it = sensors.begin();
      while(it != sensors.end())
        {
          size_t i = std::distance(sensors.begin(), it);
          if(sensor_names[i] == std::string("sensor_plugin/range_sensor"))
            {
              range_sensor_ = boost::static_pointer_cast< sensor_plugin::RangeSensor >(*it);
              ROS_WARN("optical flow: find the range sensor");
            }
          it++;
        }

      /* if we can not find range sensor, we stop optical flow */
      if(range_sensor_ == NULL)
        {
          ROS_FATAL("can not start since we do not have range sensor");
          optical_flow_sub_.shutdown();
        }

      sensor_correction_flag_ = false;

    }

    ~OpticalFlow() {}

    OpticalFlow() {}
  private:
    ros::Publisher optical_flow_pub_;
    ros::Subscriber optical_flow_sub_;
    ros::Time optical_flow_stamp_;

    double x_axis_direction_;
    double y_axis_direction_;

    double opt_noise_sigma_;

    float vel_x_;
    float flow_x_;

    float vel_y_;
    float flow_y_;

    float quality_;
    float metric_scale_;

    bool sensor_correction_flag_;

    geometry_msgs::TwistStamped optical_flow_msg_;

    boost::shared_ptr<sensor_plugin::RangeSensor> range_sensor_;

    void opticalFlowCallback(const geometry_msgs::TwistStampedConstPtr & optical_flow_msg)
    {

      static int init_flag = true;

      static bool stop_flag = true;
      static double previous_secs;
      static double special_increment = 0;
      optical_flow_msg_ = *optical_flow_msg;
      double current_secs = optical_flow_msg_.header.stamp.toSec();

      //**** Global Sensor Fusion Flag Check
      //** the correction will continue after the range sensor become insane (landing phase.) 
      //** should change?
      if(range_sensor_->getSensorSanity() == sensor_plugin::RangeSensor::TOTAL_INSANE && sensor_correction_flag_ && (estimator_->getLandedFlag() || !estimator_->getSensorFusionFlag()))
        {
          ROS_INFO("optical flow: stop correciton");
          for(vector<int>::iterator  it = estimate_indices_.begin(); it != estimate_indices_.end(); ++it )
            {
              estimator_->getFuserEgomotion(*it)->setMeasureFlag(false);
              estimator_->getFuserEgomotion(*it)->resetState();
            }
          for(vector<int>::iterator  it = experiment_indices_.begin(); it != experiment_indices_.end(); ++it )
            {
              estimator_->getFuserExperiment(*it)->setMeasureFlag(false);
              estimator_->getFuserExperiment(*it)->resetState();
            }
          sensor_correction_flag_ = false;
        }

      //* update here
      metric_scale_ = optical_flow_msg_.twist.angular.z;
      quality_ = optical_flow_msg_.twist.linear.z;


      if(metric_scale_ == 0) //have abosolte scale, dont need calculation
        {
          vel_x_ = x_axis_direction_ * optical_flow_msg_.twist.angular.x;
          vel_y_ = y_axis_direction_ * optical_flow_msg_.twist.angular.y;
        }
      else
        {//TODO!: this is scaling, but we lack the compensation of gyro.
          vel_x_ = x_axis_direction_ * optical_flow_msg_.twist.linear.x * metric_scale_;
          vel_y_ = y_axis_direction_ * optical_flow_msg_.twist.linear.y * metric_scale_;
        }

      flow_x_ = x_axis_direction_ * optical_flow_msg_.twist.linear.x;
      flow_y_ = y_axis_direction_ * optical_flow_msg_.twist.linear.y;


      /*  start sensor fusion condition */
      if(range_sensor_->getSensorSanity() > sensor_plugin::RangeSensor::TOTAL_INSANE && !sensor_correction_flag_)
        {
          Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1);
          temp(0,0) = opt_noise_sigma_;
          if(estimate_mode_ & (1 << EGOMOTION_ESTIMATION_MODE))
            {
              for(int i = 0; i < estimator_->getFuserEgomotionNo(); i++)
                {
                  if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_W)) || (estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::X_B)))
                    {
                      estimate_indices_.push_back(i);
                      //specila process for px4flow
                      if(metric_scale_ == 0) 
                        estimator_->getFuserEgomotion(i)->setInitState(flow_x_, 1);
                      else
                        estimator_->getFuserEgomotion(i)->setInitState(vel_x_, 1);

                      estimator_->getFuserEgomotion(i)->setMeasureSigma(temp);
                      estimator_->getFuserEgomotion(i)->setMeasureFlag();
                    }
                  else if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_W)) || (estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Y_B)))
                    {
                      estimate_indices_.push_back(i);
                      //specila process for py4flow
                      if(metric_scale_ == 0)
                        estimator_->getFuserEgomotion(i)->setInitState(flow_y_, 1);
                      else
                        estimator_->getFuserEgomotion(i)->setInitState(vel_y_, 1);

                      estimator_->getFuserEgomotion(i)->setMeasureSigma(temp);
                      estimator_->getFuserEgomotion(i)->setMeasureFlag();
                    }
                }
            }

          if(estimate_mode_ & (1 << EXPERIMENT_MODE))
            {
              for(int i = 0; i < estimator_->getFuserExperimentNo(); i++)
                {
                  if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_W)) || (estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_B)))
                    {
                      experiment_indices_.push_back(i);
                      estimator_->getFuserExperiment(i)->setMeasureSigma(temp);
                      //specila process for px4flow
                      if(metric_scale_ == 0) 
                        estimator_->getFuserEgomotion(i)->setInitState(flow_x_, 1);
                      else
                        estimator_->getFuserEgomotion(i)->setInitState(vel_x_, 1);
                      estimator_->getFuserExperiment(i)->setMeasureFlag();
                    }
                  else if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_W)) || (estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_B)))
                    {
                      experiment_indices_.push_back(i);
                      estimator_->getFuserExperiment(i)->setMeasureSigma(temp);
                      //specila process for py4flow
                      if(metric_scale_ == 0)
                        estimator_->getFuserEgomotion(i)->setInitState(flow_y_, 1);
                      else
                        estimator_->getFuserEgomotion(i)->setInitState(vel_y_, 1);
                      estimator_->getFuserExperiment(i)->setMeasureFlag();
                    }
                }
            }
          sensor_correction_flag_ = true;
        }

      //** the condition which we can correction */
      if(!sensor_correction_flag_) return;
      /* not sure whether this is right */
      if(range_sensor_->getValueState() != sensor_plugin::RangeSensor::NORMAL) return;

      estimateProcess();

      //publish
      aerial_robot_base::States opt_data;
      opt_data.header.stamp = optical_flow_msg_.header.stamp;

      aerial_robot_base::State x_state;
      x_state.id = "x";
      x_state.raw_vel = vel_x_;
      x_state.vel = flow_x_;

      aerial_robot_base::State y_state;
      y_state.id = "y";
      y_state.raw_vel = vel_y_;
      y_state.vel = flow_y_;

      for(vector<int>::iterator  it = estimate_indices_.begin(); it != estimate_indices_.end(); ++it )
        {
          if((estimator_->getFuserEgomotionId(*it) & (1 << BasicEstimator::X_B))) 
            {
              estimator_->setEEState(BasicEstimator::X_B, 1, (estimator_->getFuserEgomotion(*it)->getEstimateState())(1,0));
              estimator_->setEEState(BasicEstimator::X_B, 0, (estimator_->getFuserEgomotion(*it)->getEstimateState())(0,0));
            }
          if((estimator_->getFuserEgomotionId(*it) & (1 << BasicEstimator::X_W))) 
            {
              estimator_->setEEState(BasicEstimator::X_W, 1, (estimator_->getFuserEgomotion(*it)->getEstimateState())(1,0));
              estimator_->setEEState(BasicEstimator::X_W, 0, (estimator_->getFuserEgomotion(*it)->getEstimateState())(0,0));
            }
          if((estimator_->getFuserEgomotionId(*it) & (1 << BasicEstimator::Y_B))) 
            {
              estimator_->setEEState(BasicEstimator::Y_B, 1, (estimator_->getFuserEgomotion(*it)->getEstimateState())(1,0));
              estimator_->setEEState(BasicEstimator::Y_B, 0,  (estimator_->getFuserEgomotion(*it)->getEstimateState())(0,0));
            }
          if((estimator_->getFuserEgomotionId(*it) & (1 << BasicEstimator::Y_W))) 
            {
              estimator_->setEEState(BasicEstimator::Y_W, 1, (estimator_->getFuserEgomotion(*it)->getEstimateState())(1,0));
              estimator_->setEEState(BasicEstimator::Y_W, 0, (estimator_->getFuserEgomotion(*it)->getEstimateState())(0,0));
            }
        }
      for(vector<int>::iterator  it = experiment_indices_.begin(); it != experiment_indices_.end(); ++it )
        {
          if((estimator_->getFuserExperimentId(*it) & (1 << BasicEstimator::X_B))) 
            estimator_->setEXState(BasicEstimator::X_B, 1, (estimator_->getFuserExperiment(*it)->getEstimateState())(1,0));
          if((estimator_->getFuserExperimentId(*it) & (1 << BasicEstimator::X_W))) 
            estimator_->setEXState(BasicEstimator::X_W, 1, (estimator_->getFuserExperiment(*it)->getEstimateState())(1,0));
          if((estimator_->getFuserExperimentId(*it) & (1 << BasicEstimator::Y_B))) 
            estimator_->setEXState(BasicEstimator::Y_B, 1, (estimator_->getFuserExperiment(*it)->getEstimateState())(1,0));
          if((estimator_->getFuserExperimentId(*it) & (1 << BasicEstimator::Y_W))) 
            estimator_->setEXState(BasicEstimator::Y_W, 1, (estimator_->getFuserExperiment(*it)->getEstimateState())(1,0));
        }

      x_state.kf_pos = estimator_->getEEState(BasicEstimator::X_W, 0);
      x_state.kf_vel = estimator_->getEEState(BasicEstimator::X_W, 1);
      x_state.kfb_pos = estimator_->getEEState(BasicEstimator::X_B, 0);
      x_state.kfb_vel = estimator_->getEEState(BasicEstimator::X_B, 1);

      y_state.kf_pos = estimator_->getEEState(BasicEstimator::Y_W, 0);
      y_state.kf_vel = estimator_->getEEState(BasicEstimator::Y_W, 1);
      y_state.kfb_pos = estimator_->getEEState(BasicEstimator::Y_B, 0);
      y_state.kfb_vel = estimator_->getEEState(BasicEstimator::Y_B, 1);


      opt_data.states.push_back(x_state);
      opt_data.states.push_back(y_state);

      optical_flow_pub_.publish(opt_data);

      //update
      previous_secs = current_secs;
      updateHealthStamp(current_secs);
    }

    void estimateProcess()
    {
      if(!estimate_flag_) return;

      Eigen::Matrix<double, 1, 1> sigma_temp = Eigen::MatrixXd::Zero(1, 1); 
      sigma_temp(0,0) = opt_noise_sigma_;
      Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1); 

      for(vector<int>::iterator  it = estimate_indices_.begin(); it != estimate_indices_.end(); ++it )
        {
          if((estimator_->getFuserEgomotionId(*it) & (1 << BasicEstimator::X_W)) || (estimator_->getFuserEgomotionId(*it) & (1 << BasicEstimator::X_B)))
            {
              estimator_->getFuserEgomotion(*it)->setMeasureSigma(sigma_temp);
              estimator_->getFuserEgomotion(*it)->setCorrectMode(1);

              if(metric_scale_ == 0)
                {
                  //we have absolute scale for optical flow, like px4flow using sonar
                  ROS_INFO("Oko");
                  if(quality_ == 0 || vel_x_ == 0) temp(0, 0) = flow_x_;
                  else temp(0, 0) = vel_x_;
                }
              else
                temp(0, 0) = vel_x_;

              estimator_->getFuserEgomotion(*it)->correction(temp);
            }
          else if((estimator_->getFuserEgomotionId(*it) & (1 << BasicEstimator::Y_W)) || (estimator_->getFuserEgomotionId(*it) & (1 << BasicEstimator::Y_B)))
            {
              estimator_->getFuserEgomotion(*it)->setMeasureSigma(sigma_temp);
              estimator_->getFuserEgomotion(*it)->setCorrectMode(1);

              if(metric_scale_ == 0)
                {
                  //we have absolute scale for optical flow, like px4flow using sonar
                  if(quality_ == 0 || vel_y_ == 0) temp(0, 0) = flow_y_;
                  else temp(0, 0) = vel_y_;
                }
              else
                temp(0, 0) = vel_y_;

              estimator_->getFuserEgomotion(*it)->correction(temp);
            }
        }
      for(vector<int>::iterator  it = experiment_indices_.begin(); it != experiment_indices_.end(); ++it )
        {
          if((estimator_->getFuserExperimentId(*it) & (1 << BasicEstimator::X_W)) || (estimator_->getFuserExperimentId(*it) & (1 << BasicEstimator::X_B)))
            {
              estimator_->getFuserExperiment(*it)->setMeasureSigma(sigma_temp);
              estimator_->getFuserExperiment(*it)->setCorrectMode(1);

              if(metric_scale_ == 0)
                {
                  if(quality_ == 0 || vel_x_ == 0) temp(0, 0) = flow_x_;
                  else temp(0, 0) = vel_x_;
                }
              else
                temp(0, 0) = vel_x_;

              estimator_->getFuserExperiment(*it)->correction(temp);
            }
          else if((estimator_->getFuserExperimentId(*it) & (1 << BasicEstimator::Y_W)) || (estimator_->getFuserExperimentId(*it) & (1 << BasicEstimator::Y_B)))
            {
              estimator_->getFuserExperiment(*it)->setMeasureSigma(sigma_temp);
              estimator_->getFuserExperiment(*it)->setCorrectMode(1);

              if(metric_scale_ == 0)
                {
                  if(quality_ == 0 || vel_y_ == 0) temp(0, 0) = flow_y_;
                  else temp(0, 0) = vel_y_;
                }
              else
                temp(0, 0) = vel_y_;

              estimator_->getFuserExperiment(*it)->correction(temp);
            }
        }
    }

  void rosParamInit()
    {

    nhp_.param("opt_noise_sigma", opt_noise_sigma_, 0.01 );
    printf("level vel noise sigma  is %f\n", opt_noise_sigma_);

    std::string ns = nhp_.getNamespace();
    if (!nhp_.getParam ("x_axis_direction", x_axis_direction_))
      x_axis_direction_ = 1.0;
    printf("%s: x_axis direction_ is %.3f\n", ns.c_str(), x_axis_direction_);

    if (!nhp_.getParam ("y_axis_direction", y_axis_direction_))
      y_axis_direction_ = 1.0; //-1 is default
    printf("%s: y_axis direction_ is %.3f\n", ns.c_str(), y_axis_direction_);

  }
};

};
#endif



