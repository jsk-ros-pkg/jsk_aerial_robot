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
#include <nav_msgs/Odometry.h>

namespace sensor_plugin
{
  class VisualOdometry :public sensor_plugin::SensorBase
  {
  public:

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, string sensor_name)
    {
      SensorBase::initialize(nh, nhp, estimator, sensor_name);
      rosParamInit();

      /* ros publisher: aerial_robot_base::State */
      vo_state_pub_ = nh_.advertise<aerial_robot_base::States>("data",10);

      control_timer_ = nhp_.createTimer(ros::Duration(1.0 / control_rate_), &VisualOdometry::controlFunc,this);
    }

    ~VisualOdometry(){}
    VisualOdometry():init_time_(true), pos_(0,0,0)
    {
      r_.setIdentity();
      init_orien_.setIdentity();

      vo_state_.states.resize(3);
      vo_state_.states[0].id = "x";
      vo_state_.states[0].state.resize(2);
      vo_state_.states[1].id = "y";
      vo_state_.states[1].state.resize(2);
      vo_state_.states[2].id = "z";
      vo_state_.states[2].state.resize(2);
    }

  private:
    /* ros */
    ros::Subscriber vo_sub_;
    ros::Publisher vo_state_pub_;
    ros::Timer  control_timer_;

    /* ros param */
    tf::Vector3 pos_;
    string vo_sub_topic_name_;
    double vo_noise_sigma_;
    double height_thresh_;
    double control_rate_;
    bool relative_odom_;
    tf::Matrix3x3 r_, init_orien_;
    bool init_time_;

    aerial_robot_base::States vo_state_;

    void voCallback(const nav_msgs::Odometry::ConstPtr & vo_msg)
    {
      /* only do egmotion estimate mode */
      if(!getFuserActivate(BasicEstimator::EGOMOTION_ESTIMATE))
        {
          ROS_WARN_THROTTLE(1,"Optical Flow: no egmotion estimate mode");
          return;
        }

      tf::Vector3 raw_pos;
      tf::pointMsgToTF(vo_msg->pose.pose.position, raw_pos);
      if(relative_odom_)
        pos_ = init_orien_ * baselink_transform_.getBasis() * raw_pos;
      else
        pos_ = raw_pos;

      tf::Matrix3x3 raw_r;
      r_ = init_orien_ * baselink_transform_.getBasis() * raw_r;

      if(init_time_)
        {
          init_time_ = false;

          /* record the init state from the baselink(CoG) of UAV */
          init_orien_ = estimator_->getOrientation(Frame::BASELINK, BasicEstimator::EGOMOTION_ESTIMATE);
          if(relative_odom_)
            pos_ = init_orien_ * baselink_transform_.getBasis() * raw_pos; //do again

          if(!estimator_->getStateStatus(State::X_BASE, BasicEstimator::EGOMOTION_ESTIMATE) ||
             !estimator_->getStateStatus(State::Y_BASE, BasicEstimator::EGOMOTION_ESTIMATE))
            {
              ROS_WARN("VO: start/restart kalman filter");

              for(auto& fuser : estimator_->getFuser(BasicEstimator::EGOMOTION_ESTIMATE))
                {
                  boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
                  int id = kf->getId();
                  if((id & (1 << State::X_BASE)) || (id & (1 << State::Y_BASE)) )
                    {
                      if(time_sync_) kf->setTimeSync(true);

                      if(id & (1 << State::X_BASE)) kf->setInitState(pos_[0], 0);
                      if(id & (1 << State::Y_BASE)) kf->setInitState(pos_[1], 0);
                      kf->setMeasureFlag();
                    }
                }
            }
          estimator_->setStateStatus(State::X_BASE, BasicEstimator::EGOMOTION_ESTIMATE, true);
          estimator_->setStateStatus(State::Y_BASE, BasicEstimator::EGOMOTION_ESTIMATE, true);
          estimator_->setStateStatus(State::YAW_BASE, BasicEstimator::EGOMOTION_ESTIMATE, true);

        }

      estimateProcess(vo_msg->header.stamp);

      /* publish */
      vo_state_.header.stamp = vo_msg->header.stamp;

      for(int axis = 0; axis < 2; axis++) vo_state_.states[axis].state[0].x = pos_[axis];
      vo_state_pub_.publish(vo_state_);

      /* update */
      updateHealthStamp(vo_msg->header.stamp.toSec());
    }

    void estimateProcess(ros::Time stamp)
    {
      tfScalar r,p,y;
      r_.getRPY(r,p,y);
      estimator_->setState(State::YAW_BASE, BasicEstimator::EGOMOTION_ESTIMATE, 0, y);

      for(auto& fuser : estimator_->getFuser(BasicEstimator::EGOMOTION_ESTIMATE))
        {
          string plugin_name = fuser.first;
          boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
          int id = kf->getId();

          if((id & (1 << State::X_BASE)) || (id & (1 << State::Y_BASE)))
            {
              if((id & (1 << State::X_BASE)) && (id & (1 << State::Y_BASE)))
                {
                  // TODO
                }
              else
                {

                  if(plugin_name == "kalman_filter/kf_pos_vel_acc" ||
                     plugin_name == "kalman_filter/kf_pos_vel_acc_bias")
                    {
                      /* set noise sigma */
                      VectorXd measure_sigma(1);
                      measure_sigma << vo_noise_sigma_;
                      kf->setMeasureSigma(measure_sigma);

                      /* correction */
                      uint8_t index = 0;
                      VectorXd meas(1); meas << pos_[id >> (State::X_BASE + 1)];
                      vector<double> params = {kf_plugin::POS};
                      /* time sync and delay process: get from kf time stamp */
                      if(time_sync_ && delay_ < 0)
                        stamp.fromSec(kf->getTimestamp() + delay_);
                      kf->correction(meas, params, stamp.toSec());
                      VectorXd state = kf->getEstimateState();

                      /* set data */
                      if(id & (1 << State::X_BASE)) index = State::X_BASE;
                      if(id & (1 << State::Y_BASE)) index = State::Y_BASE;
                      estimator_->setState(index, BasicEstimator::EGOMOTION_ESTIMATE, 0, state(0));
                      estimator_->setState(index, BasicEstimator::EGOMOTION_ESTIMATE, 1, state(1));
                      vo_state_.states[id >> (State::X_BASE + 1)].state[1].x = state(0);
                      vo_state_.states[id >> (State::X_BASE + 1)].state[1].y = state(1);
                    }
                }
            }
        }
    }

    void controlFunc(const ros::TimerEvent & e)
    {
      static bool start_vo = false;

      /* update/check the state status of the xy state */
      if(estimator_->getState(State::Z_BASE, BasicEstimator::EGOMOTION_ESTIMATE)[0] < height_thresh_)
        {
          ROS_WARN_THROTTLE(1,"VO: bad height to use vo");
          if(start_vo)
            {
              start_vo = false;
              estimator_->setStateStatus(State::X_BASE, BasicEstimator::EGOMOTION_ESTIMATE, false);
              estimator_->setStateStatus(State::Y_BASE, BasicEstimator::EGOMOTION_ESTIMATE, false);
              estimator_->setStateStatus(State::YAW_BASE, BasicEstimator::EGOMOTION_ESTIMATE, false);
              /* stop subscribe */
              vo_sub_.shutdown();
              ROS_WARN("VO: shutdown the subscribe");
            }
        }
      else
        {
          if(!start_vo)
            {
              start_vo = true;
              init_time_ = true;
              ROS_WARN("VO: start/restart");

              /* ros subscriber: vo */
              vo_sub_ = nh_.subscribe(vo_sub_topic_name_, 1, &VisualOdometry::voCallback, this);
            }
        }
    }

    void rosParamInit()
    {
      std::string ns = nhp_.getNamespace();
      nhp_.param("vo_noise_sigma", vo_noise_sigma_, 0.01 );
      if(param_verbose_) cout << ns << ": vo noise sigma is " <<  vo_noise_sigma_ << endl;

      nhp_.param("height_thresh", height_thresh_, 0.8);
      if(param_verbose_) cout << ns << ": height thresh is " <<  height_thresh_ << endl;

      nhp_.param("control_rate", control_rate_, 20.0);
      if(param_verbose_) cout << ns << ": height thresh is " <<  control_rate_ << endl;

      nhp_.param("relative_odom", relative_odom_, true);
      if(param_verbose_) cout << ns << ": relative odom flag is " <<  relative_odom_ << endl;

      nhp_.param("vo_sub_topic_name", vo_sub_topic_name_, string("vo") );
      if(param_verbose_) cout << ns << ": vo sub topic name is " <<  vo_sub_topic_name_ << endl;
    }
  };

};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::VisualOdometry, sensor_plugin::SensorBase);



