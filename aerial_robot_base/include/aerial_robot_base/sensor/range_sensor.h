#ifndef RNAGE_SENSOR_MODULE_H
#define RNAGE_SENSOR_MODULE_H

#include <ros/ros.h>
#include <aerial_robot_base/basic_state_estimation.h>
#include <aerial_robot_base/sensor/sensor_base_plugin.h>
#include <aerial_robot_base/States.h>
#include <sensor_msgs/Range.h>

using namespace std;

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

      state_ = NORMAL;
      first_outlier_val_ = 0;
      t_ab_ = 0;
      t_ab_incre_ = 0;
      height_offset_ = 0;
      hardware_height_offset_ = 0;
      sensor_hz_ = 0;

      /* set the barometer to the mode of sensor-fusion mode(range-sensor dominant) */
      //estimator_->setRangeSenorFlag(true);
      estimator_->setHeightEstimateMode(BasicEstimator::WITHOUT_BARO_MODE);
    }

    ~RangeSensor() {}

    RangeSensor() {}


    /* the state of the height sensor value in terms of estimation  */
    const static uint8_t NORMAL = 0;
    const static uint8_t ABNORMAL = 1;
    const static uint8_t MAX_EXCEED = 2;

  private:
    ros::Subscriber range_sensor_sub_;
    ros::Publisher range_sensor_pub_;

    string range_sensor_sub_name_;

    double range_noise_sigma_;
    float raw_pos_z_, prev_raw_pos_z_;
    float raw_vel_z_;
    int calibrate_cnt_;

    /* for terrain check */
    bool terrain_check_; // the flag to enable or disable the terrain check
    double outlier_noise_; // the check threshold between the estimated value and sensor value: e.g. 10 times
    double inlier_noise_; // the check threshold between the estimated value and sensor value: e.g. 5 times
    double check_du1_; // the duration to check the sensor value in terms of the big noise, short time(e.g. 0.1s)
    double check_du2_; // the duration to check the sensor value in terms of the new flat terrain, long time(e.g. 1s)
    uint8_t state_; // the sensor state(normal or abnormal)
    double t_ab_; // the time when the state becomes abnormal.
    double t_ab_incre_; // the increment time from  t_ab_.
    double first_outlier_val_; // the first sensor value during the check_du2_;

    float hardware_height_offset_; // the offset of the sensor value due to the attachment(hardware);
    float height_offset_;

    void rangeCallback(const sensor_msgs::RangeConstPtr & range_msg)
    {
      static int init_flag = true;
      static int calibrate_cnt = calibrate_cnt_;
      static double previous_secs = range_msg->header.stamp.toSec();
      double current_secs = range_msg->header.stamp.toSec();

      //**** calibrate phase
      if(calibrate_cnt > 0)
        {
          calibrate_cnt--;
          sensor_hz_ += (current_secs - previous_secs);
          hardware_height_offset_ -= range_msg->range;

          if(calibrate_cnt == 0)
            {
              sensor_hz_ /= (float)(calibrate_cnt_ - 1);
              hardware_height_offset_ /= (float)calibrate_cnt_;
              /* the initial height offset is equal with the hardware initial height offset */
              height_offset_ = hardware_height_offset_;
              ROS_WARN("range_sensor: the hardware height offset: %f, the hz is %f", hardware_height_offset_, 1.0 / sensor_hz_);

              if(estimate_mode_ & (1 << EGOMOTION_ESTIMATION_MODE))
                {
                  for(int i = 0; i < estimator_->getFuserEgomotionNo(); i++)
                    {
                      if((estimator_->getFuserEgomotionId(i) & (1 << BasicEstimator::Z_W)))
                        {
                          estimator_->getFuserEgomotion(i)->setMeasureFlag();
                          estimate_indices_.push_back(i);
                        }
                    }
                }
              if(estimate_mode_ & (1 << EXPERIMENT_MODE))
                {
                  for(int i = 0; i < estimator_->getFuserExperimentNo(); i++)
                    {
                      if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Z_W)))
                        {
                          estimator_->getFuserExperiment(i)->setMeasureFlag();
                          experiment_indices_.push_back(i);
                        }
                    }
                }
            }
        }

      //**** update  phase
      raw_pos_z_ = range_msg->range + height_offset_;
      raw_vel_z_ = (raw_pos_z_ - prev_raw_pos_z_) / (current_secs - previous_secs);
      if(!terrain_check_) state_ = NORMAL;

      /* update */
      previous_secs = current_secs;
      prev_raw_pos_z_ = raw_pos_z_;
      if(calibrate_cnt > 0) return;

      /* flow chat 2 */
      switch(state_)
        {
        case NORMAL:
          /* flow chat 1 */
          /* check the height of the sensor when the height exceeds the max value */
          if(range_msg->range > range_msg->max_range && terrain_check_)
            {
              state_ = MAX_EXCEED;
              estimator_->setHeightEstimateMode(BasicEstimator::ONLY_BARO_MODE);
              ROS_WARN("exceed the range of the sensor");
              break;
            }

          /* flow chat 2 */
          /* check the difference between the estimated value and */
          if(fabs((estimator_->getFuserEgomotion(estimate_indices_[0])->getEstimateState())(0,0) - raw_pos_z_) > outlier_noise_ && terrain_check_)
            {
              t_ab_ = current_secs;
              t_ab_incre_ = current_secs;
              first_outlier_val_ = range_msg->range;
              state_ = ABNORMAL;
              ROS_WARN("range sensor: we find the outlier value in NORMAL mode, switch to ABNORMAL mode, the sensor value is %f, the estimator value is %f, the first outlier value is %f", raw_pos_z_, (estimator_->getFuserEgomotion(estimate_indices_[0])->getEstimateState())(0,0), first_outlier_val_);
              break;
            }

          /* general update */
          estimateProcess();
          break;
        case ABNORMAL:

          /* update the t_ab_incre_ for the second level oulier check */
          if(fabs(range_msg->range - first_outlier_val_) > outlier_noise_)
            {
              ROS_WARN("range sensor: update the t_ab_incre and first_outlier_val_, since the sensor value is vibrated, sensor value:%f, first outlier value: %f", range_msg->range, first_outlier_val_);
              t_ab_incre_ = current_secs;
              first_outlier_val_ = range_msg->range;
            }

          if(current_secs - t_ab_ < check_du1_)
            {
              /* the first level to check the outlier: recover to the last normal mode */
              /* we don't have to update the height_offset */
              if(fabs((estimator_->getFuserEgomotion(estimate_indices_[0])->getEstimateState())(0,0) - raw_pos_z_) < inlier_noise_)
                {
                  state_ = NORMAL; 
                  estimateProcess();
                }
            }
          else
            {
              /* the second level to check the outlier: find new terrain */
              if(current_secs - t_ab_incre_ > check_du2_)
                {
                  state_ = NORMAL;
                  height_offset_ = (estimator_->getFuserEgomotion(estimate_indices_[0])->getEstimateState())(0,0) - range_msg->range;
                  /* also update the landing height */
                  estimator_->setLandingHeight(height_offset_ - hardware_height_offset_);
                  ROS_WARN("We we find the new terrain, the new height_offset is %f", height_offset_);
                }
            }
          break;
        case MAX_EXCEED:
          /* the sensor value is below the max value ath the MAX_EXCEED state */
          /*we first turn back to ABNORMAL mode to verify the validity of the value */
          if(range_msg->range < range_msg->max_range) state_ = ABNORMAL;
          break;
        default:
          break;
        }

      /* publish phase */
      aerial_robot_base::States range_data;
      range_data.header.stamp = range_msg->header.stamp;

      aerial_robot_base::State z_state;
      z_state.id = "z";
      z_state.raw_pos = raw_pos_z_;
      z_state.raw_vel = raw_vel_z_;

      for(vector<int>::iterator  it = estimate_indices_.begin(); it != estimate_indices_.end(); ++it )
        {
          estimator_->setEEState(BasicEstimator::Z_W, 0, (estimator_->getFuserEgomotion(*it)->getEstimateState())(0,0));
          estimator_->setEEState(BasicEstimator::Z_W, 1, (estimator_->getFuserEgomotion(*it)->getEstimateState())(1,0));
          z_state.kfb_pos = (estimator_->getFuserEgomotion(*it)->getEstimateState())(0,0);
          z_state.kfb_vel = (estimator_->getFuserEgomotion(*it)->getEstimateState())(1,0);
          if(estimator_->getFuserEgomotionPluginName(*it) == "kalman_filter/kf_pose_vel_acc_bias")
            z_state.kfb_bias = (estimator_->getFuserEgomotion(*it)->getEstimateState())(2,0);
        }
      for(vector<int>::iterator  it = experiment_indices_.begin(); it != experiment_indices_.end(); ++it )
        {
          estimator_->setEXState(BasicEstimator::Z_W, 0, (estimator_->getFuserExperiment(*it)->getEstimateState())(0,0));
          estimator_->setEXState(BasicEstimator::Z_W, 1, (estimator_->getFuserExperiment(*it)->getEstimateState())(1,0));
          z_state.reserves.push_back((estimator_->getFuserEgomotion(*it)->getEstimateState())(0,0));
          z_state.reserves.push_back((estimator_->getFuserEgomotion(*it)->getEstimateState())(1,0));
        }

      range_data.states.push_back(z_state);
      range_sensor_pub_.publish(range_data);
    }

    void estimateProcess()
    {
      if(!estimate_flag_) return;

      Eigen::Matrix<double, 1, 1> sigma_temp = Eigen::MatrixXd::Zero(1, 1);
      Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1);

      for(vector<int>::iterator  it = estimate_indices_.begin(); it != estimate_indices_.end(); ++it )
        {
          sigma_temp(0,0) = range_noise_sigma_;
          estimator_->getFuserEgomotion(*it)->setMeasureSigma(sigma_temp);
          temp(0, 0) = raw_pos_z_;
          estimator_->getFuserEgomotion(*it)->correction(temp);
        }
      for(vector<int>::iterator  it = experiment_indices_.begin(); it != experiment_indices_.end(); ++it )
        {
          sigma_temp(0,0) = range_noise_sigma_;
          estimator_->getFuserExperiment(*it)->setMeasureSigma(sigma_temp);
          temp(0, 0) = raw_pos_z_;
          estimator_->getFuserExperiment(*it)->correction(temp);
        }
    }

    void rosParamInit()
    {
      nhp_.param("range_sensor_sub_name", range_sensor_sub_name_, string("distance"));
      cout << "range noise sigma is " << range_sensor_sub_name_ << endl;

      nhp_.param("range_noise_sigma", range_noise_sigma_, 0.005 );
      cout << "range noise sigma is " <<  range_noise_sigma_ << endl;

      nhp_.param("calibrate_cnt", calibrate_cnt_, 100);
      printf("check duration  is %d\n", calibrate_cnt_); 

      /* for terrain and outlier check */
      nhp_.param("terrain_check", terrain_check_, false);
      cout << "terrain check is " <<  (terrain_check_?string("true"):string("false")) << endl;

      nhp_.param("outlier_noise", outlier_noise_, 0.1); // [m]
      cout << "outlier noise sigma is " << outlier_noise_ << endl;

      nhp_.param("inlier_noise", inlier_noise_, 0.06); // [m]
      cout << "inlier noise sigma is " << inlier_noise_ << endl;

      nhp_.param("check_du1", check_du1_, 0.1); // [sec]
      cout << "check duration1 is " << check_du1_ << endl;

      nhp_.param("check_du2", check_du2_, 1.0); // [sec]
      cout << "check duration2 is " << check_du2_ << endl;
    }
  };

};
#endif



