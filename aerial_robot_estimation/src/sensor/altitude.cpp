// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <aerial_robot_estimation/sensor/base_plugin.h>
#include <kalman_filter/kf_pos_vel_acc_plugin.h>
#include <sensor_msgs/Range.h>
#include <spinal/Barometer.h>

using namespace Eigen;
using namespace std;

namespace
{
  int calibrate_cnt;
  double range_previous_secs;
}

namespace sensor_plugin
{
  class Alt :public sensor_plugin::SensorBase
  {
  public:
    void initialize(ros::NodeHandle nh,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    string sensor_name, int index)
    {
      SensorBase::initialize(nh, robot_model, estimator, sensor_name, index);
      rosParamInit();

      kf_loader_ptr_ = boost::shared_ptr< pluginlib::ClassLoader<kf_plugin::KalmanFilter> >(new pluginlib::ClassLoader<kf_plugin::KalmanFilter>("kalman_filter", "kf_plugin::KalmanFilter"));
      baro_bias_kf_  = kf_loader_ptr_->createInstance("aerial_robot_base/kf_baro_bias");
      baro_bias_kf_->initialize(string(""), 0);

      baro_lpf_filter_ = IirFilter(sample_freq_, cutoff_freq_);
      baro_lpf_high_filter_ = IirFilter(sample_freq_, high_cutoff_freq_);

      /* terrain check */
      if(!terrain_check_with_baro_) state_on_terrain_ = NORMAL;

      /* range sensor */
      std::string topic_name;
      getParam<std::string>("range_sensor_sub_name", topic_name, string("distance"));
      range_sensor_sub_ = nh_.subscribe(topic_name, 10, &Alt::rangeCallback, this);

      alt_mode_sub_ = indexed_nhp_.subscribe("estimate_alt_mode", 1, &Alt::altEstimateModeCallback, this);


      /* barometer */
      //barometer_sub_ = nh_.subscribe(barometer_sub_name_, 1, &Alt::baroCallback, this);
    }

    ~Alt() {}

    Alt():
      sensor_plugin::SensorBase(),
      /* range sensor */
      raw_range_sensor_value_(0),
      raw_range_pos_z_(0),
      prev_raw_range_pos_z_(0),
      raw_range_vel_z_(0),
      min_range_(0),
      max_range_(0),
      range_sensor_sanity_(TOTAL_SANE),
      range_sensor_offset_(0),
      range_sensor_hz_(0),
      /* barometer */
      raw_baro_pos_z_(0),
      baro_pos_z_(0),
      prev_raw_baro_pos_z_(0),
      baro_vel_z_(0),
      raw_baro_vel_z_(0),
      baro_temp_(0),
      high_filtered_baro_pos_z_(0),
      prev_high_filtered_baro_pos_z_(0),
      high_filtered_baro_vel_z_(0),
      /* terrain state */
      alt_estimate_mode_(ONLY_BARO_MODE),
      state_on_terrain_(NORMAL),
      inflight_state_(false),
      first_outlier_val_(0),
      t_ab_(0),
      t_ab_incre_(0),
      height_offset_(0)
    {
      alt_state_.states.resize(2);
      alt_state_.states[0].id = "range_sensor";
      alt_state_.states[0].state.resize(2);
      alt_state_.states[1].id = "baro";
      alt_state_.states[1].state.resize(3);

      /* set health chan num */
      setHealthChanNum(2);
    }

    int getRangeSensorSanity(){return range_sensor_sanity_;}
    int getStateOnTerrain(){return state_on_terrain_;}

    /* the height estimation related function */
    static constexpr uint8_t ONLY_BARO_MODE = 0; //we estimate the height only based the baro, but the bias of baro is constexprant(keep the last eistamted value)
    static constexpr uint8_t WITH_BARO_MODE = 1; //we estimate the height using range sensor etc. without the baro, but we are estimate the bias of baro
    static constexpr uint8_t WITHOUT_BARO_MODE = 2; //we estimate the height using range sensor etc. with the baro, also estimating the bias of baro


    /* the state of the height sensor value in terms of estimation  */
    static constexpr uint8_t NORMAL = 0;
    static constexpr uint8_t ABNORMAL = 1;
    static constexpr uint8_t MAX_EXCEED = 2;

    /* the criteria to express the sanity of the range sensor */
    /* e.g. the sonar which is attached at the bottom of the uav can not measure the distance at the moment before takeoff and after landing. so the sensr(value) is insane. Also, we have to consider that the sensor may become insane at the moment of landing or to low. */
    static constexpr int TOTAL_INSANE = -1; // the state before takeoff and after landing
    static constexpr int POTENTIALLY_INSANE = 0; // the inflight state which can be insane potentially
    static constexpr int TOTAL_SANE = 1; // the totally sane state.

  private:
    /* ros */
    ros::Subscriber alt_mode_sub_;
    /* range sensor */
    ros::Subscriber range_sensor_sub_;
    /* barometer */
    ros::Subscriber barometer_sub_;
    ros::Publisher barometer_pub_;

    /* ros param */
    /* range sensor */
    tf::Vector3 range_origin_; /* the origin of range based on cog of UAV */
    bool no_height_offset_;
    double range_noise_sigma_;
    int calibrate_cnt_;
    /* barometer */
    string barometer_sub_name_;
    double baro_noise_sigma_, baro_bias_noise_sigma_;
    /* the iir filter for the barometer for the first filtering stage */
    double sample_freq_, cutoff_freq_, high_cutoff_freq_;

    /* base variables */
    /* range sensor */
    double raw_range_sensor_value_;
    double raw_range_pos_z_, prev_raw_range_pos_z_, raw_range_vel_z_;
    double min_range_, max_range_;
    double ascending_check_range_; /* merge range around the min range */
    float range_sensor_offset_; // the offset of the sensor value due to the attachment(hardware);
    float range_sensor_hz_;
    int range_sensor_sanity_; /* for the (initial) sanity of the senser in terms of the attachment hardware */

    /* barometer */
    /* the kalman filter for the baro bias estimation */
    boost::shared_ptr< pluginlib::ClassLoader<kf_plugin::KalmanFilter> > kf_loader_ptr_;
    boost::shared_ptr<kf_plugin::KalmanFilter> baro_bias_kf_;
    IirFilter baro_lpf_filter_, baro_lpf_high_filter_;
    bool inflight_state_; //the flag for the inflight state
    double raw_baro_pos_z_, baro_pos_z_, prev_raw_baro_pos_z_, prev_baro_pos_z_;
    double raw_baro_vel_z_, baro_vel_z_;
    double high_filtered_baro_pos_z_, prev_high_filtered_baro_pos_z_, high_filtered_baro_vel_z_;
    double baro_temp_; // the temperature of the chip

    /* for vo validity check */
    double max_flight_height_;

    /* for terrain check */
    int alt_estimate_mode_;
    bool terrain_check_with_baro_; // the flag to enable or disable the terrain check
    double outlier_threshold_; // the check threshold between the estimated value and sensor value: e.g. 10 times
    double inlier_threshold_; // the check threshold between the estimated value and sensor value: e.g. 5 times
    double check_du1_; // the duration to check the sensor value in terms of the big noise, short time(e.g. 0.1s)
    double check_du2_; // the duration to check the sensor value in terms of the new flat terrain, long time(e.g. 1s)
    uint8_t state_on_terrain_; // the sensor state(normal or abnormal)
    double t_ab_; // the time when the state becomes abnormal.
    double t_ab_incre_; // the increment time from  t_ab_.
    double first_outlier_val_; // the first sensor value during the check_du2_;
    float height_offset_; /* general offset between esimated height and range sensor value, maybe change beacause of the terrain */

    aerial_robot_msgs::States alt_state_;

    void rangeCallback(const sensor_msgs::RangeConstPtr & range_msg)
    {
      double current_secs = range_msg->header.stamp.toSec();

      if(getStatus() != Status::ACTIVE && estimator_->getForceAttControlFlag())
        {
          sensor_status_ = Status::ACTIVE;
          ROS_WARN("Force activate range sensor in force att contorl mode");
        }

      if(!updateBaseLink2SensorTransform()) return;

      /* consider the orientation of the uav */
#if 0
      float roll = (estimator_->getState(State::ROLL_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE))[0];
      float pitch = (estimator_->getState(State::PITCH_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE))[0];
      /* add the offset from the base_link to the sensor */
      tf::Matrix3x3 tilt_r; tilt_r.setRPY(roll, pitch, 0);
      double raw_range_sensor_value = cos(roll) * cos(pitch) * range_msg->range - (tilt_r * sensor_tf_.getOrigin()).z();
#endif

      raw_range_sensor_value_ = -(estimator_->getOrientation(Frame::BASELINK, aerial_robot_estimation::EGOMOTION_ESTIMATE) * (sensor_tf_* tf::Vector3(0, 0, range_msg->range))).z();

      /* calibrate phase */
      if(calibrate_cnt > 0)
        {
          if(calibrate_cnt == calibrate_cnt_)
            {
              range_previous_secs = current_secs;
              setStatus(Status::INIT);
            }

          calibrate_cnt--;
          sensor_hz_ += (current_secs - range_previous_secs);
          range_sensor_offset_ -= raw_range_sensor_value_;

          /* initialize */
          if(calibrate_cnt == 0)
            {
              /* set the range of the sensor value */
              max_range_ = range_msg->max_range;
              min_range_ = range_msg->min_range;

              /* check the sanity of the range sensor value */
              if(range_msg->max_range <= range_msg->min_range)
                {
                  calibrate_cnt = 1;
                  ROS_ERROR("range sensor: the min/max range is not correct");
                  return;
                }

              if(max_flight_height_ < 0) max_flight_height_ = max_range_;

              sensor_hz_ /= (float)(calibrate_cnt_ - 1);
              range_sensor_offset_ /= (float)calibrate_cnt_;
              /* the initial height offset is equal with the hardware initial height offset */
              height_offset_ = range_sensor_offset_;

              /* check the sanity of the first height (height_offset) */
              if(-height_offset_ < min_range_ || -height_offset_ > max_range_)
                {
                  /* sonar sensor should be here */
                  ROS_WARN("the range sensor is attached close to the ground");

                  range_sensor_sanity_ = TOTAL_INSANE;

                  /* the offset (init) should be 0 */
                  height_offset_ = 0;
                  range_sensor_offset_ = 0;

                  /* set the undescending mode because we may only use imu for z(alt) estimation */
                  estimator_->setUnDescendMode(true);
                }

              /* set the height offset to be zero, if the sensor is too closed to the ground */
              if(no_height_offset_)
                {
                  height_offset_ = 0;
                  range_sensor_offset_ = 0;
                }

              /* fuser for 0: egomotion, 1: experiment */
              for(int mode = 0; mode < 2; mode++)
                {
                  if(!getFuserActivate(mode)) continue;

                  for(auto& fuser : estimator_->getFuser(mode))
                    {
                      boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
                      int id = kf->getId();

                      if(id & (1 << State::Z_BASE))
                        {
                          kf->setMeasureFlag();
                          kf->setInitState(raw_range_sensor_value_ + height_offset_, 0);
                        }
                    }
                }

              /* change the alt estimate mode */
              alt_estimate_mode_ = WITHOUT_BARO_MODE;


              /* set the status for Z (altitude) */
              estimator_->setStateStatus(State::Z_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE, true);

              setStatus(Status::ACTIVE); //active

              ROS_WARN("%s: the range sensor offset: %f, initial sanity: %s, the hz is %f, estimate mode is %d",
                       (range_msg->radiation_type == sensor_msgs::Range::ULTRASOUND)?string("sonar sensor").c_str():string("infrared sensor").c_str(), range_sensor_offset_, range_sensor_sanity_?string("true").c_str():string("false").c_str(), 1.0 / sensor_hz_, alt_estimate_mode_);

              range_previous_secs = current_secs;
              prev_raw_range_pos_z_ = raw_range_pos_z_;
              return;
            }
        }

      /* update */
      raw_range_pos_z_ = raw_range_sensor_value_ + height_offset_;
      raw_range_vel_z_ = (raw_range_pos_z_ - prev_raw_range_pos_z_) / (current_secs - range_previous_secs);


      /* only for sonar in takeoff and landing phase
         before takeoff: toal_sane -> total_insane
         takeoff: total_insane -> potentially_insance
         landing: potentially_insance -> total_insane */
      switch(range_sensor_sanity_)
        {
        case TOTAL_INSANE:
          if(!estimator_->getSensorFusionFlag())
            {
              /* this is for the repeat mode */
              if(!estimator_->getSensorFusionFlag()) calibrate_cnt = 0;

              for(int mode = 0; mode < 2; mode++)
                {
                  if(!getFuserActivate(mode)) continue;

                  for(auto& fuser : estimator_->getFuser(mode))
                    {
                      boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
                      int id = kf->getId();
                      if(id & (1 << State::Z_BASE))
                        {
                          kf->setMeasureFlag(false);
                          kf->resetState();
                        }
                    }
                }
              return;
            }

          /* first ascending phase */
          if(raw_range_pos_z_ < min_range_ + ascending_check_range_ &&
             raw_range_pos_z_ > min_range_ &&
             prev_raw_range_pos_z_ < min_range_ &&
             prev_raw_range_pos_z_ > min_range_ - ascending_check_range_)
            {

              ROS_WARN("insanity %s: confirm ascending to sanity height, start sf correction process, previous height: %f", (range_msg->radiation_type == sensor_msgs::Range::ULTRASOUND)?string("sonar sensor").c_str():string("infrared sensor").c_str(), prev_raw_range_pos_z_);

              /* release the non-descending mode, use the range sensor for z(alt) estimation */
              estimator_->setUnDescendMode(false);

              for(int mode = 0; mode < 2; mode++)
                {
                  if(!getFuserActivate(mode)) continue;

                  for(auto& fuser : estimator_->getFuser(mode))
                    {
                      ROS_INFO("debug sonar: init test:");
                      boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
                      int id = kf->getId();
                      if(id & (1 << State::Z_BASE))
                        {
                          kf->setInitState(raw_range_pos_z_, 0);
                          kf->setMeasureFlag();
                        }
                    }
                }

              range_sensor_sanity_ = POTENTIALLY_INSANE;
            }
          return;
        case POTENTIALLY_INSANE:
          if(prev_raw_range_pos_z_ < min_range_ + ascending_check_range_ &&
             prev_raw_range_pos_z_ > min_range_ &&
             raw_range_pos_z_ < min_range_ &&
             raw_range_pos_z_ > min_range_ - ascending_check_range_)
            {
              ROS_WARN("potential insane %s: confirm descending to insace zone, stop sf correction process, previous height: %f", (range_msg->radiation_type == sensor_msgs::Range::ULTRASOUND)?string("sonar sensor").c_str():string("infrared sensor").c_str(), prev_raw_range_pos_z_);

              range_sensor_sanity_ = TOTAL_INSANE;
              return;
            }
          break;
        default:
          break;
        }

      /* terrain check and height estimate */
      alt_state_.header.stamp.fromSec(range_msg->header.stamp.toSec());
      if(terrainProcess(current_secs))
        rangeEstimateProcess();

      /* publish phase */
      alt_state_.states[0].state[0].x = raw_range_pos_z_;
      alt_state_.states[0].state[0].y = raw_range_vel_z_;

      state_pub_.publish(alt_state_);
      updateHealthStamp(1); //channel: 1

      /* update */
      range_previous_secs = current_secs;
      prev_raw_range_pos_z_ = raw_range_pos_z_;
    }

    void rangeEstimateProcess()
    {
      if(getStatus() == Status::INVALID) return;

      Matrix<double, 1, 1> temp = MatrixXd::Zero(1, 1);

      for(int mode = 0; mode < 2; mode++)
        {
          if(!getFuserActivate(mode)) continue;

          for(auto& fuser : estimator_->getFuser(mode))
            {
              string plugin_name = fuser.first;
              boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
              int id = kf->getId();
              if(id & (1 << State::Z_BASE))
                {
                  if(plugin_name == "kalman_filter/kf_pos_vel_acc")
                    {
                      /* correction */
                      VectorXd measure_sigma(1); measure_sigma << range_noise_sigma_;
                      VectorXd meas(1); meas <<  raw_range_pos_z_;
                      vector<double> params = {kf_plugin::POS};

                      kf->correction(meas, measure_sigma,
                                     time_sync_?(alt_state_.header.stamp.toSec()):-1, params);
                    }
                }
            }
        }
    }

    bool terrainProcess(double current_secs)
    {
      if(getStatus() == Status::INVALID) return false;

      boost::shared_ptr<kf_plugin::KalmanFilter> kf = nullptr;
      if(!getFuserActivate(aerial_robot_estimation::EGOMOTION_ESTIMATE))
        {
          ROS_ERROR("range sensor is not used in EGOMOTION_ESTIMATE mode");
          return false;
        }

      for(auto& fuser : estimator_->getFuser(aerial_robot_estimation::EGOMOTION_ESTIMATE))
        {
          if(fuser.second->getId() & (1 << State::Z_BASE))
            {
              if(!kf) kf = fuser.second;
              else
                {
                  ROS_ERROR("more than one kaman filter estimating z axis is detected");
                  return false;
                }
            }
        }

      if(terrain_check_with_baro_)
        {
          switch(state_on_terrain_)
            {
            case NORMAL:
              /* flow chat 1 */
              /* check the height of the range sensor when height exceeds limitation */
              if(raw_range_sensor_value_ < min_range_ || raw_range_sensor_value_ > max_range_)
                {
                  state_on_terrain_ = MAX_EXCEED;
                  alt_estimate_mode_ = ONLY_BARO_MODE;
                  ROS_WARN("exceed the range of the sensor, change to only baro estimate mode");
                  //break;
                  return false;
                }

              /* flow chat 2 */
              /* check the difference between the estimated value and raw range value */
              if(fabs((kf->getEstimateState())(0) - raw_range_pos_z_) > outlier_threshold_)
                {
                  t_ab_ = current_secs;
                  t_ab_incre_ = current_secs;
                  first_outlier_val_ = raw_range_sensor_value_;
                  state_on_terrain_ = ABNORMAL;
                  ROS_WARN("range sensor: we find the outlier value in NORMAL mode, switch to ABNORMAL mode, the sensor value is %f, the estimator value is %f, the first outlier value is %f", raw_range_pos_z_, (kf->getEstimateState())(0), first_outlier_val_);
                  //break;
                  return false;
                }

              //break;
              return true;
            case ABNORMAL:

              /* update the t_ab_incre_ for the second level oulier check */
              if(fabs(raw_range_sensor_value_ - first_outlier_val_) > outlier_threshold_)
                {
                  ROS_WARN("range sensor: update the t_ab_incre and first_outlier_val_, since the sensor value is vibrated, sensor value:%f, first outlier value: %f", raw_range_sensor_value_, first_outlier_val_);
                  t_ab_incre_ = current_secs;
                  first_outlier_val_ = raw_range_sensor_value_;
                }

              if(current_secs - t_ab_ < check_du1_)
                {
                  /* the first level to check the outlier: recover to the last normal mode */
                  /* we don't have to update the height_offset */
                  if(fabs((kf->getEstimateState())(0) - raw_range_pos_z_) < inlier_threshold_)
                    {
                      state_on_terrain_ = NORMAL;
                      return true;
                    }
                }
              else
                {
                  /* the second level to check the outlier: find new terrain */
                  if(current_secs - t_ab_incre_ > check_du2_)
                    {
                      state_on_terrain_ = NORMAL;
                      height_offset_ = (kf->getEstimateState())(0) - raw_range_sensor_value_;
                      ROS_WARN("We we find the new terrain, the new height_offset is %f", height_offset_);
                      return true;
                    }
                }
              break;
            case MAX_EXCEED:
              /* the sensor value is below the max value ath the MAX_EXCEED state */
              /*we first turn back to ABNORMAL mode to verify the validity of the value */
              if(raw_range_sensor_value_ < max_range_ && raw_range_sensor_value_ > min_range_) state_on_terrain_ = ABNORMAL;
              return false;
            default:
              return false;
            }
        }
      else
        {
          if(estimator_->getForceAttControlFlag()) return true;

          /*
            heuristic check method:
            check the validity of visual odometry by range sensor, with the assumption that there is no terrain change
          */
          if(estimator_->getVoHandlers().size() > 0 && raw_range_pos_z_ > max_flight_height_)
            {
              bool vo_active = false;
              for(const auto& handler: estimator_->getVoHandlers())
                {
                  if(handler->getStatus() == Status::ACTIVE)
                    vo_active = true;
                }

              if(vo_active)
                {
                  /* TODO: find the invalid vo sensor, and only reset the invalid one */
                  ROS_WARN("reset all vo sensor, because the value of range sensor exceeds the max flight height: %f, prev raw range pos z: %f, kf pos z: %f", raw_range_sensor_value_, prev_raw_range_pos_z_, (kf->getEstimateState())(0));
                  for(const auto& handler: estimator_->getVoHandlers()) handler->reset();
                  return true;
                }
            }
          return true;
        }
      return false;
    }

    void baroCallback(const spinal::BarometerConstPtr & baro_msg)
    {
      static double baro_previous_secs;
      double current_secs = baro_msg->stamp.toSec();

      raw_baro_pos_z_ = baro_msg->altitude;
      baro_temp_ = baro_msg->temperature;

      /*First Filtering: IIR filter */
      /* position */
      baro_pos_z_ = baro_lpf_filter_.filterFunction(raw_baro_pos_z_);
      high_filtered_baro_pos_z_ = baro_lpf_high_filter_.filterFunction(raw_baro_pos_z_);
      /* velocity */
      raw_baro_vel_z_ = (raw_baro_pos_z_ - prev_raw_baro_pos_z_)/(current_secs - baro_previous_secs);
      baro_vel_z_ = (baro_pos_z_ - prev_baro_pos_z_)/(current_secs - baro_previous_secs);
      high_filtered_baro_vel_z_ = (high_filtered_baro_pos_z_ - prev_high_filtered_baro_pos_z_)/(current_secs - baro_previous_secs);

      /* the true initial phase for baro based estimattion for inflight state */
      /* since the value of pressure will decrease during the rising of the propeller rotation speed */
      if(((alt_estimate_mode_ == ONLY_BARO_MODE  && high_filtered_baro_vel_z_ > 0.1)
          || alt_estimate_mode_ == WITHOUT_BARO_MODE)
         && estimator_->getFlyingFlag() && !inflight_state_)
        {//the inflight state should be with the velocity of 0.1(up)
          inflight_state_ = true;
          ROS_WARN("barometer: start the inflight barometer height estimation");

          /* the initialization of the baro bias kf filter */
          VectorXd input_sigma(1); input_sigma << baro_bias_noise_sigma_;
          baro_bias_kf_->setPredictionNoiseCovariance(input_sigma);
          baro_bias_kf_->setInputFlag();
          baro_bias_kf_->setMeasureFlag();
          baro_bias_kf_->setInitState(-baro_pos_z_, 0);
        }
      /* reset */
      baroEstimateProcess(baro_msg->stamp);

      /* publish */
      alt_state_.header.stamp = baro_msg->stamp;
      alt_state_.states[1].state[0].x = raw_baro_pos_z_;
      alt_state_.states[1].state[0].y = raw_baro_vel_z_;
      alt_state_.states[1].state[1].x = baro_pos_z_;
      alt_state_.states[1].state[1].y = baro_vel_z_;
      alt_state_.states[1].state[2].x = high_filtered_baro_pos_z_;
      alt_state_.states[1].state[2].y = high_filtered_baro_vel_z_;

      state_pub_.publish(alt_state_);

      /* update */
      baro_previous_secs = current_secs;
      prev_raw_baro_pos_z_ = raw_baro_pos_z_;
      prev_baro_pos_z_ = baro_pos_z_;
      prev_high_filtered_baro_pos_z_ = high_filtered_baro_pos_z_;
      updateHealthStamp(0); //channel: 0
    }

    void baroEstimateProcess(ros::Time stamp)
    {
      if(getStatus() == Status::INVALID) return;

      if(!inflight_state_) return;

      switch(alt_estimate_mode_)
        {
        case ONLY_BARO_MODE :
          for(int mode = 0; mode < 2; mode++)
            {
              if(!getFuserActivate(mode)) continue;

              for(auto& fuser : estimator_->getFuser(mode))
                {
                  string plugin_name = fuser.first;
                  boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
                  int id = kf->getId();
                  if(id & (1 << State::Z_BASE))
                    {
                      if(!kf->getFilteringFlag())
                        {
                          //ROS_FATAL("baro: can not estiamte the height by baro(ONLY_BARO_MODE), because the filtering flag is not activated");
                          return;
                        }
                      /* We should set the sigma every time, since we may have several different sensors to correct the kalman filter(e.g. vo + opti, laser + baro) */

                      if(plugin_name == "kalman_filter/kf_pos_vel_acc")
                        {
                          /* correction */
                          VectorXd measure_sigma(1); measure_sigma << baro_noise_sigma_;
                          VectorXd meas(1); meas <<  baro_pos_z_ + (baro_bias_kf_->getEstimateState())(0);
                          vector<double> params = {kf_plugin::POS};
                          kf->correction(meas, measure_sigma, -1, params);

                        }

                      /* set the state */
                      // VectorXd state = kf->getEstimateState();
                      // estimator_->setState(State::Z_BASE, mode, 0, state(0));
                      // estimator_->setState(State::Z_BASE, mode, 1, state(1));
                      //alt_state_.states[0].state[1].x = state(0);
                      //alt_state_.states[0].state[1].y = state(1);
                      alt_state_.states[0].state[1].z = (baro_bias_kf_->getEstimateState())(0);

                    }
                }
            }
          break;
        case WITHOUT_BARO_MODE:
          {
            baro_bias_kf_->prediction(VectorXd::Zero(1), stamp.toSec());
            VectorXd meas(1);  meas << (estimator_->getState(State::Z_BASE, 0))[0] - baro_pos_z_;
            baro_bias_kf_->correction(meas, VectorXd::Zero(1), -1);
          }
          break;
        case WITH_BARO_MODE:
          //TODO: this is another part, maybe we have to use another package:
          //http://wiki.ros.org/ethzasl_sensor_fusion
          break;
        default:
          break;
        }
    }

    /* force to change the estimate mode */
    void altEstimateModeCallback(const std_msgs::UInt8ConstPtr & mode_msg)
    {
      alt_estimate_mode_ = mode_msg->data;
      ROS_INFO("change the height estimate mode: %d", alt_estimate_mode_);
    }

    void rosParamInit()
    {
      /* range sensor */
      getParam<double>("range_noise_sigma", range_noise_sigma_, 0.005);
      getParam<int>("calibrate_cnt", calibrate_cnt_, 100);
      getParam<bool>("no_height_offset", no_height_offset_, true);

      calibrate_cnt = calibrate_cnt_;

      /* first ascending process: check range */
      getParam<double>("ascending_check_range", ascending_check_range_, 0.1); // [m]

      /* for vo validity check */
      getParam<double>("max_flight_height", max_flight_height_, -1); // [m]

      /* for terrain and outlier check */
      getParam<double>("outlier_threshold", outlier_threshold_, 1.0); // [m]
      getParam<double>("inlier_threshold", inlier_threshold_, 0.5); // [m]
      getParam<bool>("terrain_check_with_baro", terrain_check_with_baro_, false);
      getParam<double>("check_du1", check_du1_, 0.1); // [sec]
      getParam<double>("check_du2", check_du2_, 1.0); // [sec]

      /* barometer */
      getParam<std::string>("barometer_sub_name", barometer_sub_name_, string("/baro"));
      getParam<double>("baro_noise_sigma", baro_noise_sigma_, 0.05 );
      getParam<double>("baro_bias_noise_sigma", baro_bias_noise_sigma_, 0.001 );
      getParam<double>("sample_freq", sample_freq_, 100.0 );
      getParam<double>("cutoff_freq", cutoff_freq_, 10.0 );
      getParam<double>("high_cutoff_freq", high_cutoff_freq_, 1.0 );
    }

    void changeStatus(bool flag)
    {
      if(flag)
        {
          sensor_status_ = Status::ACTIVE;
          ROS_INFO_STREAM(nhp_.getNamespace() << ", set to active");
        }
      else
        {
          sensor_status_ = Status::INVALID;
          ROS_INFO_STREAM(nhp_.getNamespace() << ", set to invalid");
        }
    }

  };

};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Alt, sensor_plugin::SensorBase);




