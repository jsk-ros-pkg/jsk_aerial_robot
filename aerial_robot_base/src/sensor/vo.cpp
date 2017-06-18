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
      r_cog_.setIdentity();
      r_b_.setIdentity();
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
    tf::Matrix3x3 r_cog_, r_b_, init_orien_;
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
      pos_ = init_orien_ * baselink_transform_.getBasis() * raw_pos;

      /* hardcoding: x_imu = y_zed, y_imu = -z_zed, z_imu = -x_zed */
      tf::Matrix3x3 raw_r(tf::Quaternion(vo_msg->pose.pose.orientation.y,
                                         -vo_msg->pose.pose.orientation.z,
                                         -vo_msg->pose.pose.orientation.x,
                                         vo_msg->pose.pose.orientation.w));
      r_b_ = init_orien_ * raw_r;
      r_b_ = raw_r;
      r_cog_ = r_b_ * estimator_->getRRootlink2Cog();

      tfScalar r,p,y;
      r_b_.getRPY(r,p,y);

      if(init_time_)
        {
          init_time_ = false;

          /* record the init state */
          init_orien_.setRPY(estimator_->getState(BasicEstimator::ROLL_W_B, BasicEstimator::EGOMOTION_ESTIMATE)[0],
                             estimator_->getState(BasicEstimator::PITCH_W_B, BasicEstimator::EGOMOTION_ESTIMATE)[0],
                             estimator_->getState(BasicEstimator::YAW_W_B, BasicEstimator::EGOMOTION_ESTIMATE)[0]);
          pos_ = init_orien_ * baselink_transform_.getBasis() * raw_pos; //do again

          if(!estimator_->getStateStatus(BasicEstimator::X_W, BasicEstimator::EGOMOTION_ESTIMATE) ||
             !estimator_->getStateStatus(BasicEstimator::Y_W, BasicEstimator::EGOMOTION_ESTIMATE))
            {
              ROS_WARN("VO: start/restart kalman filter");

              for(auto& fuser : estimator_->getFuser(BasicEstimator::EGOMOTION_ESTIMATE))
                {
                  boost::shared_ptr<kf_base_plugin::KalmanFilter> kf = fuser.second;
                  int id = kf->getId();
                  if((id & (1 << BasicEstimator::X_W)) || (id & (1 << BasicEstimator::Y_W)) )
                    {
                      kf->setInitState(pos_[id >> 1], 0);
                      kf->setMeasureFlag();
                    }
                }
            }
          estimator_->setStateStatus(BasicEstimator::X_W, BasicEstimator::EGOMOTION_ESTIMATE, true);
          estimator_->setStateStatus(BasicEstimator::Y_W, BasicEstimator::EGOMOTION_ESTIMATE, true);
          estimator_->setStateStatus(BasicEstimator::YAW_W, BasicEstimator::EGOMOTION_ESTIMATE, true);
          estimator_->setStateStatus(BasicEstimator::YAW_W_B, BasicEstimator::EGOMOTION_ESTIMATE, true);
        }

      estimateProcess();

      /* publish */
      vo_state_.header.stamp = vo_msg->header.stamp;

      for(int axis = 0; axis < 2; axis++) vo_state_.states[axis].state[0].x = pos_[axis];
      vo_state_pub_.publish(vo_state_);

      /* update */
      updateHealthStamp(vo_msg->header.stamp.toSec());
    }

    void estimateProcess()
    {
      tfScalar r,p,y;
      r_b_.getRPY(r,p,y);
      estimator_->setState(BasicEstimator::YAW_W_B, BasicEstimator::EGOMOTION_ESTIMATE, 0, y);

      r_cog_.getRPY(r,p,y);
      estimator_->setState(BasicEstimator::YAW_W, BasicEstimator::EGOMOTION_ESTIMATE, 0, y);

      Eigen::Matrix<double, 1, 1> sigma_temp = Eigen::MatrixXd::Zero(1, 1);
      sigma_temp(0,0) = vo_noise_sigma_;
      Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1);

      for(auto& fuser : estimator_->getFuser(BasicEstimator::EGOMOTION_ESTIMATE))
        {
          boost::shared_ptr<kf_base_plugin::KalmanFilter> kf = fuser.second;
          int id = kf->getId();

          if((id & (1 << BasicEstimator::X_W)) || (id & (1 << BasicEstimator::Y_W)))
            {
              kf->setMeasureSigma(sigma_temp);
              kf->setCorrectMode(0);

              temp(0, 0) = pos_[id >> 1];
              kf->correction(temp);

              MatrixXd state = kf->getEstimateState();

              /* set data */
              estimator_->setState(id >> 1, BasicEstimator::EGOMOTION_ESTIMATE, 0, state(0,0));
              estimator_->setState(id >> 1, BasicEstimator::EGOMOTION_ESTIMATE, 1, state(1,0));
              vo_state_.states[id >> 1].state[1].x = state(0, 0);
              vo_state_.states[id >> 1].state[1].y = state(1, 0);
            }
        }
    }

    void controlFunc(const ros::TimerEvent & e)
    {
      static bool start_vo = false;

      /* update/check the state status of the xy state */
      if(estimator_->getState(BasicEstimator::Z_W, BasicEstimator::EGOMOTION_ESTIMATE)[0] < height_thresh_)
        {
          ROS_WARN_THROTTLE(1,"VO: bad height to use vo");
          if(start_vo)
            {
              start_vo = false;
              estimator_->setStateStatus(BasicEstimator::X_W, BasicEstimator::EGOMOTION_ESTIMATE, false);
              estimator_->setStateStatus(BasicEstimator::Y_W, BasicEstimator::EGOMOTION_ESTIMATE, false);
              estimator_->setStateStatus(BasicEstimator::YAW_W, BasicEstimator::EGOMOTION_ESTIMATE, false);
              estimator_->setStateStatus(BasicEstimator::YAW_W_B, BasicEstimator::EGOMOTION_ESTIMATE, false);

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

      nhp_.param("vo_sub_topic_name", vo_sub_topic_name_, string("vo") );
      if(param_verbose_) cout << ns << ": vo sub topic name is " <<  vo_sub_topic_name_ << endl;
    }


  };

};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::VisualOdometry, sensor_plugin::SensorBase);



