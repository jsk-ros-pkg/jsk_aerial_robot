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

/* ros */
#include <ros/ros.h>

/* base class */
#include <aerial_robot_base/sensor_base_plugin.h>

/* message filter */
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/* ros msg */
#include <sensor_msgs/Range.h>
#include <aerial_robot_base/States.h>
#include <geometry_msgs/TwistStamped.h>

namespace sensor_plugin
{
  class OpticalFlow :public sensor_plugin::SensorBase
  {
  public:

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, geometry_msgs::TwistStamped> SyncPolicy;


    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, string sensor_name)
    {

      baseParamInit(nh, nhp, estimator, sensor_name);
      rosParamInit();

      /* ros publisher of aerial_robot_base::State */
      opt_state_pub_ = nh_.advertise<aerial_robot_base::States>("data",10);

      /* message filter for synchronize two message : optical flow and range */
      range_sub_.subscribe(nh, range_sub_topic_name_, 1);
      opt_sub_.subscribe(nh, opt_sub_topic_name_, 1);

      sync_ = boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(100)));
      sync_->connectInput(range_sub_, opt_sub_);
      sync_->registerCallback(&OpticalFlow::sensorCallback, this);
    }

    ~OpticalFlow() {}
    OpticalFlow():
      vel_w_(0, 0, 0),
      flow_w_(0, 0, 0),
      vel_b_(0, 0, 0),
      flow_b_(0, 0, 0),
      pos_z_(0),
      quality_(0),
      metric_scale_(0),
      sensor_correction_flag_(false)
    {
      opt_state_.states.resize(2);
      opt_state_.states[0].id = "x";
      opt_state_.states[0].state.resize(3);
      opt_state_.states[1].id = "y";
      opt_state_.states[1].state.resize(3);
    }

    static constexpr int SPECIAL_SCALE = -1;
    static constexpr int ABSOLUTE_SCALE = 0;

  private:
    /* ros */
    boost::shared_ptr< message_filters::Synchronizer<SyncPolicy> > sync_;
    message_filters::Subscriber<sensor_msgs::Range> range_sub_;
    message_filters::Subscriber<geometry_msgs::TwistStamped> opt_sub_;
    ros::Publisher opt_state_pub_;

    /* ros param */
    string range_sub_topic_name_, opt_sub_topic_name_;
    tf::Matrix3x3 frame_orientation_;
    double opt_noise_sigma_;
    double range_min_margin_;

    /* base var */
    ros::Time opt_stamp_;
    tf::Vector3 vel_b_, vel_w_;
    tf::Vector3 flow_b_, flow_w_;
    float pos_z_;
    float quality_;
    float metric_scale_;
    bool sensor_correction_flag_;

    aerial_robot_base::States opt_state_;

    void sensorCallback(const sensor_msgs::Range::ConstPtr & range_msg, const geometry_msgs::TwistStamped::ConstPtr & opt_msg)
    {
      static double previous_secs;

      double current_secs = opt_msg->header.stamp.toSec();

      /* range sensor */
      /* consider the orientation of the uav */
      float roll_b = (estimator_->getState(BasicEstimator::ROLL_W, 0))[0];
      float pitch_b = (estimator_->getState(BasicEstimator::PITCH_W, 0))[0];
      float yaw_b = (estimator_->getState(BasicEstimator::YAW_W, 0))[0];
      ROS_INFO("range debug: uav orientation is [%f, %f, %f]", roll_b, pitch_b, yaw_b);
      pos_z_ = cos(roll_b) * cos(pitch_b) * range_msg->range;
      bool range_sanity = (range_msg->min_range + range_min_margin_ < range_msg->range && range_msg->max_range > range_msg->range)?true:false;

      /* optical flow */
      metric_scale_ = opt_msg->twist.angular.z;
      if(metric_scale_ < SPECIAL_SCALE)
        {
          ROS_ERROR("unknonw scale for optical flow ");
          return;
        }
      quality_ = opt_msg->twist.linear.z;
      /* twist.linear: optical_flow; twist.angular: velocity */
      /* twist.linear.z: quality; twist.angular.z: matric_scale */
      flow_b_ = frame_orientation_ * tf::Vector3(opt_msg->twist.angular.x, opt_msg->twist.angular.y, 0);
      if(metric_scale_ == ABSOLUTE_SCALE || metric_scale_ == SPECIAL_SCALE)
        vel_b_ = frame_orientation_ * tf::Vector3(opt_msg->twist.linear.x, opt_msg->twist.linear.y, 0);
      else
        vel_b_ = frame_orientation_ * tf::Vector3(opt_msg->twist.linear.x, opt_msg->twist.linear.y, 0) * metric_scale_ / pos_z_;

      /* for general optical flow model */

      tf::Matrix3x3 yaw_orientation;
      yaw_orientation.setRPY(0, 0, yaw_b);
      flow_w_ = yaw_orientation * flow_b_;
      vel_w_ = yaw_orientation * vel_b_;

      /* reset to false */
      if(!range_sanity && sensor_correction_flag_ &&
         (estimator_->getLandedFlag() || !estimator_->getSensorFusionFlag()))
        {
          ROS_INFO("optical flow: stop correciton");

          for(int mode = 0; mode < 2; mode++)
            {
              if(!getFuserActivate(mode)) continue;

              for(auto& fuser : estimator_->getFuser(mode))
                {
                  boost::shared_ptr<kf_base_plugin::KalmanFilter> kf = fuser.second;
                  int id = kf->getId();
                  /* do not reset x_w and y_w which is estimated by other sensr such as rtk-gps */
                  if((id & (1 << BasicEstimator::X_B)) || (id & (1 << BasicEstimator::Y_B)))
                    {
                      kf->setMeasureFlag(false);
                      kf->resetState();
                    }
                }
            }
          sensor_correction_flag_ = false;
        }

      /*  start sensor fusion condition */
      if(range_sanity && !sensor_correction_flag_)
        {
          Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1);
          temp(0,0) = opt_noise_sigma_;

          for(int mode = 0; mode < 2; mode++)
            {
              if(!getFuserActivate(mode)) continue;

              for(auto& fuser : estimator_->getFuser(mode))
                {
                  boost::shared_ptr<kf_base_plugin::KalmanFilter> kf = fuser.second;
                  int id = kf->getId();

                  if((id & (1 << BasicEstimator::X_W)) ||
                     (id & (1 << BasicEstimator::Y_W)) ||
                     (id & (1 << BasicEstimator::X_B)) ||
                     (id & (1 << BasicEstimator::Y_B)) )
                    {
                      uint8_t b_shift = 0;
                      tf::Vector3 vel, flow;

                      if(id <= (1 << BasicEstimator::Y_W)) /* world frame */
                        {
                          vel = vel_w_;
                          flow = flow_w_;
                          b_shift = BasicEstimator::Y_W;
                        }
                      else  /* board frame */
                        {
                          vel = vel_b_;
                          flow = flow_b_;
                          b_shift = BasicEstimator::Y_B;
                        }

                      /* specila process for px4flow */
                      if(metric_scale_ == SPECIAL_SCALE)
                        kf->setInitState(flow[id >> b_shift], 1);
                      else
                        kf->setInitState(vel[id >> b_shift], 1);

                      kf->setMeasureSigma(temp);
                      kf->setMeasureFlag();
                    }
                }
            }
          sensor_correction_flag_ = true;
        }

      estimateProcess();

      /* publish */
      opt_state_.header.stamp = opt_msg->header.stamp;

      for(int axis = 0; axis < 2; axis++)
        {
          opt_state_.states[axis].state[0].x = vel_b_[axis];
          opt_state_.states[axis].state[0].y = vel_w_[axis];
        }
      opt_state_pub_.publish(opt_state_);

      /* update */
      previous_secs = current_secs;
      updateHealthStamp(current_secs);
    }

    void estimateProcess()
    {
      if(!estimate_flag_) return;

      //** the condition which we can correction */
      if(!sensor_correction_flag_) return;

      Eigen::Matrix<double, 1, 1> sigma_temp = Eigen::MatrixXd::Zero(1, 1); 
      sigma_temp(0,0) = opt_noise_sigma_;
      Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1); 

      for(int mode = 0; mode < 2; mode++)
        {
          if(!getFuserActivate(mode)) continue;

          for(auto& fuser : estimator_->getFuser(mode))
            {
              boost::shared_ptr<kf_base_plugin::KalmanFilter> kf = fuser.second;
              int id = kf->getId();

              if((id & (1 << BasicEstimator::X_W)) ||
                 (id & (1 << BasicEstimator::Y_W)) ||
                 (id & (1 << BasicEstimator::X_B)) ||
                 (id & (1 << BasicEstimator::Y_B)) )
                {
                  kf->setMeasureSigma(sigma_temp);
                  kf->setCorrectMode(1);

                  uint8_t b_shift = 0;
                  tf::Vector3 vel, flow;
                  if(id <= (1 << BasicEstimator::Y_W)) /* world frame */
                    {
                      vel = vel_w_;
                      flow = flow_w_;
                      b_shift = BasicEstimator::Y_W;
                    }
                  else  /* board frame */
                    {
                      vel = vel_b_;
                      flow = flow_b_;
                      b_shift = BasicEstimator::Y_B;
                    }

                  /* special process for px4flow */
                  if((metric_scale_ == SPECIAL_SCALE) &&
                     (quality_ == 0 || vel[id >> b_shift] == 0))
                    temp(0, 0) = flow[id >> b_shift];
                  else temp(0, 0) = vel[id >> b_shift];

                  kf->correction(temp);

                  MatrixXd state = kf->getEstimateState();

                  /* set data */
                  estimator_->setState(id >> b_shift, mode, 0, state(0,0));
                  estimator_->setState(id >> b_shift, mode, 1, state(1,0));
                  opt_state_.states[id >> b_shift].state[mode + 1].x = state(0, 0);
                  opt_state_.states[id >> b_shift].state[mode + 1].y = state(1, 0);
                }
            }
        }
    }

    void rosParamInit()
    {
      std::string ns = nhp_.getNamespace();
      nhp_.param("range_min_margin", range_min_margin_, 0.0);

      double frame_roll, frame_pitch, frame_yaw;
      nhp_.param("frame_roll", frame_roll, 0.0);
      nhp_.param("frame_pitch", frame_pitch, 0.0);
      nhp_.param("frame_yaw", frame_yaw, 0.0);
      if(param_verbose_) cout << ns << ": frame rpy: [" << frame_roll << ", " << frame_pitch << ", " << frame_yaw << "]"  << endl;
      frame_orientation_.setRPY(frame_roll, frame_pitch, frame_yaw);

      nhp_.param("opt_noise_sigma", opt_noise_sigma_, 0.01 );
      if(param_verbose_) cout << ns << ": level vel noise sigma is " <<  opt_noise_sigma_ << endl;

      nhp_.param("range_sub_topic_name", range_sub_topic_name_, string("range") );
      if(param_verbose_) cout << ns << ": range sub topic name is " <<  range_sub_topic_name_ << endl;
      nhp_.param("opt_sub_topic_name", opt_sub_topic_name_, string("opt") );
      if(param_verbose_) cout << ns << ": opt sub topic name is " <<  opt_sub_topic_name_ << endl;
    }
  };

};




