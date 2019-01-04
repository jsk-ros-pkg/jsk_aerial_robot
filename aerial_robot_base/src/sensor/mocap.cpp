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
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace Eigen;
using namespace std;

namespace
{
  bool first_flag = true;
  ros::Time previous_time;
};


namespace sensor_plugin
{
  class Mocap : public sensor_plugin::SensorBase
  {
  public:
    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, string sensor_name)
    {
      SensorBase::initialize(nh, nhp, estimator, sensor_name);
      rosParamInit();

      //low pass filter
      for(size_t i = 0; i < lpf_pos_vel_.size(); i ++)
        lpf_pos_vel_[i] = IirFilter((float)rx_freq_, (float)cutoff_pos_freq_,(float)cutoff_vel_freq_);
      lpf_psi_ = IirFilter((float)rx_freq_, (float)cutoff_vel_freq_);

      std::string topic_name;

      nhp_.param("mocap_pub_name", topic_name, std::string("data"));
      pose_stamped_pub_ = nh_.advertise<aerial_robot_msgs::States>(topic_name, 5);
      nhp_.param("mocap_sub_name", topic_name, std::string("/aerial_robot/pose"));
      mocap_sub_ = nh_.subscribe(topic_name, 1, &Mocap::poseCallback, this, ros::TransportHints().udp());
      nhp_.param("ground_truth_sub_name", topic_name, std::string("ground_truth"));
      ground_truth_sub_ = nh_.subscribe(topic_name, 1, &Mocap::groundTruthCallback, this);
    }

    ~Mocap() {}

    Mocap():
      raw_pos_(0, 0, 0),
      raw_vel_(0, 0, 0),
      pos_(0, 0, 0),
      vel_(0, 0, 0),
      prev_raw_pos_(0, 0, 0),
      prev_raw_vel_(0, 0, 0),
      prev_psi_(0)
    {
      ground_truth_pose_.states.resize(6);
      ground_truth_pose_.states[0].id = "x";
      ground_truth_pose_.states[0].state.resize(2);
      ground_truth_pose_.states[1].id = "y";
      ground_truth_pose_.states[1].state.resize(2);
      ground_truth_pose_.states[2].id = "z";
      ground_truth_pose_.states[2].state.resize(2);
      ground_truth_pose_.states[3].id = "roll";
      ground_truth_pose_.states[3].state.resize(2);
      ground_truth_pose_.states[4].id = "pitch";
      ground_truth_pose_.states[4].state.resize(2);
      ground_truth_pose_.states[5].id = "yaw";
      ground_truth_pose_.states[5].state.resize(2);
    }

    static constexpr int TIME_SYNC_CALIB_COUNT = 10;

  private:
    /* ros */
    ros::Publisher  pose_stamped_pub_;
    ros::Subscriber mocap_sub_, ground_truth_sub_;

    /* ros param */
    double rx_freq_;
    double cutoff_pos_freq_;
    double cutoff_vel_freq_;

    double pos_noise_sigma_, angle_noise_sigma_, acc_bias_noise_sigma_;

    array<IirFilter, 3> lpf_pos_vel_; /* x, y, z */
    IirFilter lpf_psi_;

    tf::Vector3 raw_pos_, raw_vel_;
    tf::Vector3 pos_, vel_;
    double prev_psi_;

    tf::Vector3 prev_raw_pos_, prev_raw_vel_;

    /* ros msg */
    aerial_robot_msgs::States ground_truth_pose_;

    void poseCallback(const geometry_msgs::PoseStampedConstPtr & msg)
    {
      tf::pointMsgToTF(msg->pose.position, raw_pos_);

      tf::Quaternion q;
      tf::quaternionMsgToTF(msg->pose.orientation, q);
      tfScalar r = 0, p = 0, y = 0;
      tf::Matrix3x3(q).getRPY(r, p, y);
      tf::Vector3 euler = tf::Vector3(r, p, y);

      if(!first_flag)
        {
          float delta_t = msg->header.stamp.toSec() - previous_time.toSec();
          raw_vel_ = (raw_pos_ - prev_raw_pos_) / delta_t;
          double psi_err = euler[2] - prev_psi_;
          if(psi_err > M_PI)  psi_err -= 2 * M_PI;
          else if(psi_err < -M_PI)  psi_err += 2 * M_PI;
          double raw_psi_vel = psi_err / delta_t;

          double psi_vel = 0;

          /* pos */
          for(int i = 0; i < 3; i++)
            lpf_pos_vel_[i].filterFunction(raw_pos_[i], pos_[i], raw_vel_[i], vel_[i]);

          lpf_psi_.filterFunction(raw_psi_vel, psi_vel);

          /* euler */
          estimator_->setState(State::YAW_BASE, BasicEstimator::GROUND_TRUTH, 0, euler[2]);
          if(estimate_mode_ & (1 << BasicEstimator::EXPERIMENT_ESTIMATE))
            estimator_->setState(State::YAW_BASE, BasicEstimator::EXPERIMENT_ESTIMATE, 0, euler[2]);

          ground_truth_pose_.header.stamp = msg->header.stamp;

          for(int i = 0; i < 6; i++)
            {
              if(i < 3)
                {
                  ground_truth_pose_.states[i].state[0].x = raw_pos_[i];
                  ground_truth_pose_.states[i].state[0].y = raw_vel_[i];
                  ground_truth_pose_.states[i].state[1].x = pos_[i];
                  ground_truth_pose_.states[i].state[1].y = vel_[i];
                }
              else
                {
                  ground_truth_pose_.states[i].state[0].x = euler[i - 3];
                  if(i == 5) ground_truth_pose_.states[i].state[0].y = psi_vel;
                }
            }

          /* estimation */
          estimateProcess(msg->header.stamp);
          pose_stamped_pub_.publish(ground_truth_pose_);
        }

      if(first_flag)
        {
          first_flag = false;
          init(raw_pos_);
        }


      prev_raw_pos_ = raw_pos_;
      prev_raw_vel_ = raw_vel_;
      prev_psi_ = euler[2];

      previous_time = msg->header.stamp;
      /* consider the remote wirleess transmission, we use the local time server */
      updateHealthStamp();
    }

    void groundTruthProcess()
    {
      /* base link */
      estimator_->setPos(Frame::BASELINK, BasicEstimator::GROUND_TRUTH, pos_);
      estimator_->setVel(Frame::BASELINK, BasicEstimator::GROUND_TRUTH, vel_);

      /* CoG */
      /* 2017.7.25: calculate the state in COG frame using the Baselink frame */
      /* pos_cog = pos_baselink - R * pos_cog2baselink */
      int estimate_mode = BasicEstimator::GROUND_TRUTH;
      estimator_->setPos(Frame::COG, estimate_mode,
                         estimator_->getPos(Frame::BASELINK, estimate_mode)
                         + estimator_->getOrientation(Frame::BASELINK, estimate_mode)
                         * estimator_->getCog2Baselink().inverse().getOrigin());
      /* vel_cog = vel_baselink - R * (w x pos_cog2baselink) */
      estimator_->setVel(Frame::COG, estimate_mode,
                         estimator_->getVel(Frame::BASELINK, estimate_mode)
                         + estimator_->getOrientation(Frame::BASELINK, estimate_mode)
                         * (estimator_->getAngularVel(Frame::BASELINK, estimate_mode).cross(estimator_->getCog2Baselink().inverse().getOrigin())));
    }

    void groundTruthCallback(const nav_msgs::OdometryConstPtr & msg)
    {
      tf::pointMsgToTF(msg->pose.pose.position, pos_);

      if(!first_flag)
        {
          tf::vector3MsgToTF(msg->twist.twist.linear, vel_);

          /* baselink */
          tf::Quaternion q;
          tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
          tfScalar r = 0, p = 0, y = 0;
          tf::Matrix3x3(q).getRPY(r, p, y);
          tf::Vector3 euler = tf::Vector3(r, p, y);
          tf::Vector3 omega;
          tf::vector3MsgToTF(msg->twist.twist.angular, omega);

          for(int i = 0; i < 3; i++)
            {
              float angle = euler[i];
              float vel = omega[i];
              lpf_pos_vel_[i].filterFunction(angle, euler[i], vel, omega[i]);

              /* only yaw can not use the LPF */
              if(i == 2) euler[2] = angle;
            }

          estimator_->setEuler(Frame::BASELINK, BasicEstimator::GROUND_TRUTH, euler);
          estimator_->setAngularVel(Frame::BASELINK, BasicEstimator::GROUND_TRUTH, omega);

          /* cog */
          (tf::Matrix3x3(q) * estimator_->getCog2Baselink().getBasis().inverse()).getRPY(r, p, y);
          euler.setValue(r, p, y);
          estimator_->setEuler(Frame::COG, BasicEstimator::GROUND_TRUTH, euler);
          estimator_->setAngularVel(Frame::COG, BasicEstimator::GROUND_TRUTH, estimator_->getCog2Baselink().getBasis() * omega);

          groundTruthProcess();
        }

      if(first_flag)
        {
          first_flag = false;
          init(pos_);
        }

      /* consider the remote wirleess transmission, we use the local time server */
      updateHealthStamp();
    }

    void rosParamInit()
    {
      std::string ns = nhp_.getNamespace();

      nhp_.param("pos_noise_sigma", pos_noise_sigma_, 0.001 );
      if(param_verbose_) cout << ns << ": pos noise sigma  is " << pos_noise_sigma_ << endl;

      nhp_.param("acc_bias_noise_sigma", acc_bias_noise_sigma_, 0.0);
      if(param_verbose_) cout << ns << ": acc noise sigma  is " << acc_bias_noise_sigma_ << endl;

      nhp_.param("angle_sigma", pos_noise_sigma_, 0.001 );
      if(param_verbose_) cout << ns << ": pos noise sigma  is " << pos_noise_sigma_ << endl;

      nhp_.param("rx_freq", rx_freq_, 100.0);
      if(param_verbose_) cout << ns << ": rx_freq  is " << rx_freq_ << endl;
      nhp_.param("cutoff_pos_freq", cutoff_pos_freq_, 20.0);
      if(param_verbose_) cout << ns << ": cutoff_pos_freq  is " << cutoff_pos_freq_ << endl;
      nhp_.param("cutoff_vel_freq", cutoff_vel_freq_, 20.0);
      if(param_verbose_) cout << ns << ": cutoff_vel_freq  is " << cutoff_vel_freq_ << endl;
    }

    void init(tf::Vector3 init_pos)
    {
      /* set ground truth */
      estimator_->setStateStatus(State::X_BASE, BasicEstimator::GROUND_TRUTH, true);
      estimator_->setStateStatus(State::Y_BASE, BasicEstimator::GROUND_TRUTH, true);
      estimator_->setStateStatus(State::Z_BASE, BasicEstimator::GROUND_TRUTH, true);
      estimator_->setStateStatus(State::YAW_BASE, BasicEstimator::GROUND_TRUTH, true);
      estimator_->setStateStatus(State::YAW_COG, BasicEstimator::GROUND_TRUTH, true);

      if(estimate_mode_ & (1 << BasicEstimator::EXPERIMENT_ESTIMATE))
        {
          estimator_->setStateStatus(State::X_BASE, BasicEstimator::EXPERIMENT_ESTIMATE, true);
          estimator_->setStateStatus(State::Y_BASE, BasicEstimator::EXPERIMENT_ESTIMATE, true);
          estimator_->setStateStatus(State::Z_BASE, BasicEstimator::EXPERIMENT_ESTIMATE, true);
          estimator_->setStateStatus(State::YAW_BASE, BasicEstimator::EXPERIMENT_ESTIMATE, true);
          estimator_->setStateStatus(State::YAW_COG, BasicEstimator::EXPERIMENT_ESTIMATE, true);

          for(auto& fuser : estimator_->getFuser(BasicEstimator::EXPERIMENT_ESTIMATE))
            {
              string plugin_name = fuser.first;
              boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
              int id = kf->getId();

              /* x, y, z */
              if(plugin_name == "kalman_filter/kf_pos_vel_acc")
                {
                  if(id < (1 << State::ROLL_COG))
                    {
                      if(time_sync_) kf->setTimeSync(true);
                      kf->setInitState(init_pos[id >> (State::X_BASE + 1)], 0);
                    }
                }

              if(plugin_name == "aerial_robot_base/kf_xy_roll_pitch_bias")
                {
                  if((id & (1 << State::X_BASE)) && (id & (1 << State::Y_BASE)))
                    {
                      VectorXd init_state(6);
                      init_state << init_pos[0], 0, init_pos[1], 0, 0, 0;
                      kf->setInitState(init_state);
                    }
                }
              kf->setMeasureFlag();
            }
        }
    }

    void estimateProcess(ros::Time stamp)
    {
      if(!estimate_flag_) return;

      if((estimate_mode_ & (1 << BasicEstimator::GROUND_TRUTH)))
        groundTruthProcess();

      /* start experiment estimation */
      if(!(estimate_mode_ & (1 << BasicEstimator::EXPERIMENT_ESTIMATE))) return;


      for(auto& fuser : estimator_->getFuser(BasicEstimator::EXPERIMENT_ESTIMATE))
        {
          string plugin_name = fuser.first;
          boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
          int id = kf->getId();

          /* x_w, y_w, z_w */
          if(id < (1 << State::ROLL_COG))
            {
              if(plugin_name == "kalman_filter/kf_pos_vel_acc")
                {
                  int index = id >> (State::X_BASE + 1);

                  if(kf->getInputDim() == 1 && acc_bias_noise_sigma_ > 0)
                    {
                      /* special process since we have to refer kf->getInputSigma() which is assinged from IMU */
                      VectorXd input_noise_sigma(2);
                      input_noise_sigma << (kf->getInputSigma())(0), acc_bias_noise_sigma_;
                      kf->setInputSigma(input_noise_sigma);
                    }

                  /* set noise sigma */
                  VectorXd measure_sigma(1);
                  measure_sigma << pos_noise_sigma_;
                  kf->setMeasureSigma(measure_sigma);

                  /* correction */
                  VectorXd meas(1); meas << raw_pos_[index];
                  vector<double> params = {kf_plugin::POS};
                  kf->correction(meas, stamp.toSec(), params);
                  VectorXd state = kf->getEstimateState();
                  estimator_->setState(index + 3, BasicEstimator::EXPERIMENT_ESTIMATE, 0, state(0));
                  estimator_->setState(index + 3, BasicEstimator::EXPERIMENT_ESTIMATE, 1, state(1));
                }

              if(plugin_name == "aerial_robot_base/kf_xy_roll_pitch_bias")
                {
                  if((id & (1 << State::X_BASE)) && (id & (1 << State::Y_BASE)))
                    {
                      /* set noise sigma */
                      VectorXd measure_sigma(2);
                      measure_sigma << pos_noise_sigma_, pos_noise_sigma_;
                      kf->setMeasureSigma(measure_sigma);

                      /* correction */
                      VectorXd meas(2); meas <<  raw_pos_[0], raw_pos_[1];
                      vector<double> params = {kf_plugin::POS};
                      /* time sync and delay process: get from kf time stamp */
                      kf->correction(meas, stamp.toSec(), params);

                      VectorXd state = kf->getEstimateState();
                      /* temp */
                      estimator_->setState(State::X_BASE, BasicEstimator::EXPERIMENT_ESTIMATE, 0, state(0));
                      estimator_->setState(State::X_BASE, BasicEstimator::EXPERIMENT_ESTIMATE, 1, state(1));
                      estimator_->setState(State::Y_BASE, BasicEstimator::EXPERIMENT_ESTIMATE, 0, state(2));
                      estimator_->setState(State::Y_BASE, BasicEstimator::EXPERIMENT_ESTIMATE, 1, state(3));
                    }
                }
            }
        }
    }
  };
};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Mocap, sensor_plugin::SensorBase);













