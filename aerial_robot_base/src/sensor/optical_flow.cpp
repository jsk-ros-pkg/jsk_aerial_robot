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

/* kalman filters */
#include <kalman_filter/kf_pos_vel_acc_plugin.h>

/* ros msg */
#include <geometry_msgs/Vector3Stamped.h>

namespace sensor_plugin
{
  class OpticalFlow :public sensor_plugin::SensorBase
  {
  public:

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, string sensor_name)
    {
      SensorBase::initialize(nh, nhp, estimator, sensor_name);
      rosParamInit();

      /* ros subscriber: optical flow */
      opt_sub_ = nh_.subscribe(opt_sub_topic_name_, 1, &OpticalFlow::optCallback, this, ros::TransportHints().udp());

      /* ros publisher: aerial_robot_base::State */
      opt_state_pub_ = nh_.advertise<aerial_robot_base::States>("data",10);
    }

    ~OpticalFlow() {}
    OpticalFlow():
      vel_(0, 0, 0), quality_(0)
    {
      opt_state_.states.resize(2);
      opt_state_.states[0].id = "x";
      opt_state_.states[0].state.resize(2);
      opt_state_.states[1].id = "y";
      opt_state_.states[1].state.resize(2);
    }

  private:
    /* ros */
    ros::Subscriber opt_sub_;
    ros::Publisher opt_state_pub_;

    /* ros param */
    string opt_sub_topic_name_;
    double opt_noise_sigma_;
    double height_thresh_;
    double vel_thresh_;

    /* base var */
    float quality_;
    tf::Vector3 vel_;

    aerial_robot_base::States opt_state_;

    void optCallback(const geometry_msgs::Vector3Stamped::ConstPtr & opt_msg)
    {
      static double previous_secs;
      static bool first_flag = true;
      double current_secs = opt_msg->header.stamp.toSec();

      //ROS_INFO("optical flow time: %f", opt_msg->header.stamp.toSec());

      /* only do egmotion estimate mode */
      if(!getFuserActivate(BasicEstimator::EGOMOTION_ESTIMATE))
        {
          ROS_WARN_THROTTLE(1,"Optical Flow: no egmotion estimate mode");
          return;
        }

      /* update/check the state status of the xy state */
      if(estimator_->getState(State::Z_BASE, BasicEstimator::EGOMOTION_ESTIMATE)[0] < height_thresh_)
        {
          ROS_WARN_THROTTLE(1,"Optical Flow: bad height to use opticla flow");

          /* stop kalman filter */
          if(!first_flag)
            {
              estimator_->setStateStatus(State::X_BASE, BasicEstimator::EGOMOTION_ESTIMATE, false);
              estimator_->setStateStatus(State::Y_BASE, BasicEstimator::EGOMOTION_ESTIMATE, false);

              first_flag = true;
            }
          return;
        }

      if(first_flag)
        {
          first_flag = false;
          ROS_WARN("Optical Flow: start/restart, height: %f", height_thresh_);

          if(!estimator_->getStateStatus(State::X_BASE, BasicEstimator::EGOMOTION_ESTIMATE) ||
             !estimator_->getStateStatus(State::Y_BASE, BasicEstimator::EGOMOTION_ESTIMATE))
            {
              ROS_WARN("Optical Flow: start/restart kalman filter");

              for(auto& fuser : estimator_->getFuser(BasicEstimator::EGOMOTION_ESTIMATE))
                {
                  string plugin_name = fuser.first;
                  boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
                  int id = kf->getId();

                  if(plugin_name == "kalman_filter/kf_pos_vel_acc_bias" ||
                    plugin_name == "kalman_filter/kf_pos_vel_acc")
                    {
                      if((id & (1 << State::X_BASE)) || (id & (1 << State::Y_BASE)) )
                        {
                          if(time_sync_) kf->setTimeSync(true);

                          kf->setInitState(vel_[id >> (State::X_BASE + 1)], 1);
                          kf->setMeasureFlag();
                        }
                    }

                  if(plugin_name == "aerial_robot_base/kf_xy_roll_pitch_bias")
                    {
                      if((id & (1 << State::X_BASE)) && (id & (1 << State::Y_BASE)) )
                        {
                          if(time_sync_) kf->setTimeSync(true);
                          VectorXd init_state(6);
                          init_state << 0, vel_[0], 0, vel_[1], 0, 0;
                          kf->setInitState(init_state);
                          kf->setMeasureFlag();
                        }
                    }

                }
            }

          /* set the status */
          estimator_->setStateStatus(State::X_BASE, BasicEstimator::EGOMOTION_ESTIMATE, true);
          estimator_->setStateStatus(State::Y_BASE, BasicEstimator::EGOMOTION_ESTIMATE, true);
        }

      quality_ = opt_msg->vector.z;

      /* get the optical flow frame orientation towards the world frame */
      tf::Matrix3x3 orien = estimator_->getOrientation(BasicEstimator::EGOMOTION_ESTIMATE);
      /* change the velocity based on the world frame  */
      vel_ = orien * baselink_transform_.getBasis() * tf::Vector3(opt_msg->vector.x, opt_msg->vector.y, 0);
      vel_.setZ(0);

      /* publish */
      opt_state_.header.stamp = opt_msg->header.stamp;
      estimateProcess(opt_state_.header.stamp);
      //tf::Vector3 vel = vel_;
      //vel_ = orien.transpose() * vel;

      for(int axis = 0; axis < 2; axis++) opt_state_.states[axis].state[0].y = vel_[axis];
      opt_state_pub_.publish(opt_state_);

      /* update */
      previous_secs = current_secs;
      updateHealthStamp(current_secs);
    }

    void estimateProcess(ros::Time stamp)
    {
      tf::Vector3 estimate_vel(estimator_->getState(State::X_BASE, BasicEstimator::EGOMOTION_ESTIMATE)[1],
                               estimator_->getState(State::Y_BASE, BasicEstimator::EGOMOTION_ESTIMATE)[1],
                               0);

      if((vel_ - estimate_vel).length() > vel_thresh_)
        {
          ROS_WARN("Optical Flow: bad vel estimate, since the diff is too big: %f, curr: [%f, %f, %f] vs prev: [%f, %f, %f]", (vel_ - estimate_vel).length(), vel_.x(), vel_.y(), vel_.z(), estimate_vel.x(), estimate_vel.y(), estimate_vel.z());
          return;
        }
      //double t = ros::Time::now().toSec();
      for(auto& fuser : estimator_->getFuser(BasicEstimator::EGOMOTION_ESTIMATE))
        {
          boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
          string plugin_name = fuser.first;
          int id = kf->getId();

          if(plugin_name == "kalman_filter/kf_pos_vel_acc" ||
             plugin_name == "kalman_filter/kf_pos_vel_acc_bias")
            {
              if((id & (1 << State::X_BASE)) ||  (id & (1 << State::Y_BASE)))
                {
                  /* set noise sigma */
                  VectorXd measure_sigma(1);
                  measure_sigma << opt_noise_sigma_;
                  kf->setMeasureSigma(measure_sigma);

                  /* correction */
                  int index = id >> (State::X_BASE + 1);
                  VectorXd meas(1); meas <<  vel_[index];
                  vector<double> params = {kf_plugin::VEL};
                  /* time sync and delay process: get from kf time stamp */
                  if(time_sync_ && delay_ < 0) stamp.fromSec(kf->getTimestamp() + delay_);
                  //ROS_INFO("opt stamp: %f, imu stamp: %f, diff: %f", stamp.toSec(), kf->getTimestamp(), kf->getTimestamp() - stamp.toSec());
                  kf->correction(meas, params, stamp.toSec());

                  VectorXd state = kf->getEstimateState();
                  estimator_->setState(index + 3, BasicEstimator::EGOMOTION_ESTIMATE, 0, state(0));
                  estimator_->setState(index + 3, BasicEstimator::EGOMOTION_ESTIMATE, 1, state(1));
                  opt_state_.states[index].state[1].x = state(0);
                  opt_state_.states[index].state[1].y = state(1);

                  if(plugin_name == "kalman_filter/kf_pos_vel_acc_bias")
                    opt_state_.states[index].state[1].z = state(2); /* bias */
                }
            }

          if(plugin_name == "aerial_robot_base/kf_xy_roll_pitch_bias")
            {
              if((id & (1 << State::X_BASE)) && (id & (1 << State::Y_BASE)))
                {
                  /* set noise sigma */
                  VectorXd measure_sigma(2);
                  measure_sigma << opt_noise_sigma_, opt_noise_sigma_;
                  kf->setMeasureSigma(measure_sigma);

                  /* correction */
                  VectorXd meas(2); meas <<  vel_[0], vel_[1];
                  vector<double> params = {kf_plugin::VEL};
                  /* time sync and delay process: get from kf time stamp */
                  if(time_sync_ && delay_ < 0) stamp.fromSec(kf->getTimestamp() + delay_);
                  kf->correction(meas, params, stamp.toSec());

                  VectorXd state = kf->getEstimateState();
                  /* temp */
                  estimator_->setState(State::X_BASE, BasicEstimator::EGOMOTION_ESTIMATE, 0, state(0));
                  estimator_->setState(State::X_BASE, BasicEstimator::EGOMOTION_ESTIMATE, 1, state(1));
                  estimator_->setState(State::Y_BASE, BasicEstimator::EGOMOTION_ESTIMATE, 0, state(2));
                  estimator_->setState(State::Y_BASE, BasicEstimator::EGOMOTION_ESTIMATE, 1, state(3));

                  opt_state_.states[0].state[1].x = state(0);
                  opt_state_.states[0].state[1].y = state(1);
                  opt_state_.states[0].state[1].z = state(4);

                  opt_state_.states[1].state[1].x = state(2);
                  opt_state_.states[1].state[1].y = state(3);
                  opt_state_.states[1].state[1].z = state(5);
                }
            }
        }
      //ROS_INFO("opt correciton: %f", ros::Time::now().toSec() - t)
    }

    void rosParamInit()
    {
      std::string ns = nhp_.getNamespace();
      nhp_.param("opt_noise_sigma", opt_noise_sigma_, 0.01 );
      if(param_verbose_) cout << ns << ": level vel noise sigma is " <<  opt_noise_sigma_ << endl;

      nhp_.param("height_thresh", height_thresh_, 0.4 );
      if(param_verbose_) cout << ns << ": height thresh is " <<  height_thresh_ << endl;

      nhp_.param("vel_thresh", vel_thresh_, 1.0);
      if(param_verbose_) cout << ns << ": vel thresh is " <<  vel_thresh_ << endl;

      nhp_.param("opt_sub_topic_name", opt_sub_topic_name_, string("opt") );
      if(param_verbose_) cout << ns << ": opt sub topic name is " <<  opt_sub_topic_name_ << endl;
    }
  };

};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::OpticalFlow, sensor_plugin::SensorBase);



