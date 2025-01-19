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
#include <aerial_robot_estimation/sensor/imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <kalman_filter/kf_pos_vel_acc_plugin.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

using namespace Eigen;
using namespace std;

namespace
{
  bool first_flag = true;
  bool ground_truth_first_flag = true;
  ros::Time previous_time;
};

namespace sensor_plugin
{
  class Mocap : public sensor_plugin::SensorBase
  {
  public:
    void initialize(ros::NodeHandle nh,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    string sensor_name, int index)
    {
      SensorBase::initialize(nh, robot_model, estimator, sensor_name, index);
      rosParamInit();

      //low pass filter
      lpf_pos_ = IirFilter(sample_freq_, cutoff_pos_freq_, 3);
      lpf_vel_ = IirFilter(sample_freq_, cutoff_vel_freq_, 3);
      lpf_angular_ = IirFilter(sample_freq_, cutoff_vel_freq_, 3);

      std::string topic_name;
      getParam<std::string>("mocap_sub_name", topic_name, std::string("pose"));
      ros::TransportHints hint = ros::TransportHints();
      bool mocap_udp;
      getParam<bool>("mocap_udp", mocap_udp, false);
      if (mocap_udp) {
	hint = ros::TransportHints().udp();
	ROS_INFO("use UDP for mocap subscribe");
      }

      mocap_sub_ = nh_.subscribe(topic_name, 1, &Mocap::poseCallback, this, hint); // buffer size 1: only need the latest value.
      nhp_.param("ground_truth_sub_name", topic_name, std::string("ground_truth"));
      ground_truth_sub_ = nh_.subscribe(topic_name, 1, &Mocap::groundTruthCallback, this, ros::TransportHints().tcpNoDelay());
    }

    ~Mocap() {}

    Mocap():
      sensor_plugin::SensorBase(),
      raw_pos_(0, 0, 0),
      raw_vel_(0, 0, 0),
      pos_(0, 0, 0),
      vel_(0, 0, 0),
      prev_raw_pos_(0, 0, 0),
      prev_raw_vel_(0, 0, 0),
      prev_raw_q_(0, 0, 0, 1),
      receive_groundtruth_odom_(false)
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
    ros::Subscriber mocap_sub_, ground_truth_sub_;

    /* ros param */
    double sample_freq_;
    double cutoff_pos_freq_;
    double cutoff_vel_freq_;

    double pos_noise_sigma_, angle_noise_sigma_, acc_bias_noise_sigma_;

    IirFilter lpf_pos_; /* x, y, z */
    IirFilter lpf_vel_; /* x, y, z */
    IirFilter lpf_angular_; /* yaw angular velocity */

    tf::Vector3 raw_pos_, raw_vel_;
    tf::Vector3 pos_, vel_;

    tf::Vector3 prev_raw_pos_, prev_raw_vel_;
    tf::Quaternion prev_raw_q_;

    bool receive_groundtruth_odom_;

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

          /* TODO: not working
          tf::Quaternion q_delta = prev_raw_q_.inverse() * q;
          tf::Vector3 raw_omega = q_delta.getAxis() * q_delta.getAngle() / delta_t;
          */

          tf::Matrix3x3 r_delta(prev_raw_q_.inverse() * q);
          r_delta.getRPY(r, p, y);
          tf::Vector3 raw_omega(r/delta_t, p/delta_t, y/delta_t);

          /* lpf */
          pos_ = lpf_pos_.filterFunction(raw_pos_);
          vel_ = lpf_vel_.filterFunction(raw_vel_);


          /* euler and omega */
          tf::Vector3 omega = raw_omega;
          if(!receive_groundtruth_odom_)
            {
              omega = lpf_angular_.filterFunction(raw_omega);
              estimator_->setState(State::YAW_BASE, aerial_robot_estimation::GROUND_TRUTH, 0, euler[2]);
            }
          if(estimate_mode_ & (1 << aerial_robot_estimation::EXPERIMENT_ESTIMATE))
            estimator_->setState(State::YAW_BASE, aerial_robot_estimation::EXPERIMENT_ESTIMATE, 0, euler[2]);

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
                  ground_truth_pose_.states[i].state[0].y = raw_omega[i - 3];
                  ground_truth_pose_.states[i].state[1].y = omega[i - 3];
                }
            }

          /* estimation */
          estimateProcess(msg->header.stamp);
          state_pub_.publish(ground_truth_pose_);
        }

      if(first_flag)
        {
          lpf_pos_.setInitValues(raw_pos_); //init pos filter with the first value
          init(raw_pos_);
          first_flag = false;
        }


      prev_raw_pos_ = raw_pos_;
      prev_raw_vel_ = raw_vel_;
      prev_raw_q_ = q;

      previous_time = msg->header.stamp;
      /* consider the remote wirleess transmission, we use the local time server */
      updateHealthStamp();
    }

    void setGroundTruthPosVel(tf::Vector3 baselink_pos, tf::Vector3 baselink_vel)
    {
      /* base link */
      estimator_->setPos(Frame::BASELINK, aerial_robot_estimation::GROUND_TRUTH, baselink_pos);
      estimator_->setVel(Frame::BASELINK, aerial_robot_estimation::GROUND_TRUTH, baselink_vel);

      /* CoG */
      /* 2017.7.25: calculate the state in COG frame using the Baselink frame */
      /* pos_cog = pos_baselink - R * pos_cog2baselink */
      tf::Transform cog2baselink_tf;
      tf::transformKDLToTF(robot_model_->getCog2Baselink<KDL::Frame>(), cog2baselink_tf);

      int estimate_mode = aerial_robot_estimation::GROUND_TRUTH;
      estimator_->setPos(Frame::COG, estimate_mode,
                         estimator_->getPos(Frame::BASELINK, estimate_mode)
                         + estimator_->getOrientation(Frame::BASELINK, estimate_mode)
                         * cog2baselink_tf.inverse().getOrigin());
      /* vel_cog = vel_baselink - R * (w x pos_cog2baselink) */
      estimator_->setVel(Frame::COG, estimate_mode,
                         estimator_->getVel(Frame::BASELINK, estimate_mode)
                         + estimator_->getOrientation(Frame::BASELINK, estimate_mode)
                         * (estimator_->getAngularVel(Frame::BASELINK, estimate_mode).cross(cog2baselink_tf.inverse().getOrigin())));

      auto pos = estimator_->getPos(Frame::COG, estimate_mode);
    }

    void groundTruthCallback(const nav_msgs::OdometryConstPtr & msg)
    {
      tf::Vector3 baselink_pos, baselink_vel;
      tf::pointMsgToTF(msg->pose.pose.position, baselink_pos);
      tf::vector3MsgToTF(msg->twist.twist.linear, baselink_vel);

      if(!ground_truth_first_flag)
        {
          /* baselink */
          tf::Quaternion q;
          tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
          tfScalar r = 0, p = 0, y = 0;
          tf::Matrix3x3(q).getRPY(r, p, y);
          tf::Vector3 euler(r, p, y);

          tf::Vector3 omega;
          tf::vector3MsgToTF(msg->twist.twist.angular, omega);
          // This LPF simulates the smoothing of gyro in spinal (e.g., dragon, which use angular to do attitude control in ROS as well as spinal,)
          omega = lpf_angular_.filterFunction(omega);

          estimator_->setEuler(Frame::BASELINK, aerial_robot_estimation::GROUND_TRUTH, euler);
          estimator_->setAngularVel(Frame::BASELINK, aerial_robot_estimation::GROUND_TRUTH, omega);

          /* cog */
          tf::Transform cog2baselink_tf;
          tf::transformKDLToTF(robot_model_->getCog2Baselink<KDL::Frame>(), cog2baselink_tf);
          (tf::Matrix3x3(q) * cog2baselink_tf.inverse().getBasis()).getRPY(r, p, y);
          euler.setValue(r, p, y);
          estimator_->setEuler(Frame::COG, aerial_robot_estimation::GROUND_TRUTH, euler);
          estimator_->setAngularVel(Frame::COG, aerial_robot_estimation::GROUND_TRUTH, cog2baselink_tf.getBasis() * omega); // TODO: check the vibration

          setGroundTruthPosVel(baselink_pos, baselink_vel);
        }

      if(ground_truth_first_flag)
        {
          ground_truth_first_flag = false;
          init(baselink_pos);
          receive_groundtruth_odom_ = true;

          for(const auto& handler: estimator_->getImuHandlers())
            boost::dynamic_pointer_cast<sensor_plugin::Imu>(handler)->treatImuAsGroundTruth(false);
        }
    }

    void rosParamInit()
    {
      getParam<double>("pos_noise_sigma", pos_noise_sigma_, 0.001 );
      getParam<double>("acc_bias_noise_sigma", acc_bias_noise_sigma_, 0.0);
      getParam<double>("sample_freq", sample_freq_, 100.0);
      getParam<double>("cutoff_pos_freq", cutoff_pos_freq_, 20.0);
      getParam<double>("cutoff_vel_freq", cutoff_vel_freq_, 20.0);
    }

    void init(tf::Vector3 init_pos)
    {
      /* set ground truth */
      estimator_->setStateStatus(State::X_BASE, aerial_robot_estimation::GROUND_TRUTH, true);
      estimator_->setStateStatus(State::Y_BASE, aerial_robot_estimation::GROUND_TRUTH, true);
      estimator_->setStateStatus(State::Z_BASE, aerial_robot_estimation::GROUND_TRUTH, true);
      estimator_->setStateStatus(State::YAW_BASE, aerial_robot_estimation::GROUND_TRUTH, true);
      estimator_->setStateStatus(State::YAW_COG, aerial_robot_estimation::GROUND_TRUTH, true);

      if(estimate_mode_ & (1 << aerial_robot_estimation::EXPERIMENT_ESTIMATE))
        {
          estimator_->setStateStatus(State::X_BASE, aerial_robot_estimation::EXPERIMENT_ESTIMATE, true);
          estimator_->setStateStatus(State::Y_BASE, aerial_robot_estimation::EXPERIMENT_ESTIMATE, true);
          estimator_->setStateStatus(State::Z_BASE, aerial_robot_estimation::EXPERIMENT_ESTIMATE, true);
          estimator_->setStateStatus(State::YAW_BASE, aerial_robot_estimation::EXPERIMENT_ESTIMATE, true);
          estimator_->setStateStatus(State::YAW_COG, aerial_robot_estimation::EXPERIMENT_ESTIMATE, true);

          for(auto& fuser : estimator_->getFuser(aerial_robot_estimation::EXPERIMENT_ESTIMATE))
            {
              string plugin_name = fuser.first;
              boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
              int id = kf->getId();

              /* x, y, z */
              if(plugin_name == "kalman_filter/kf_pos_vel_acc")
                {
                  if(id < (1 << State::ROLL_COG))
                      kf->setInitState(init_pos[id >> (State::X_BASE + 1)], 0);
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
      if(sensor_status_ == Status::INVALID) return;

      if((estimate_mode_ & (1 << aerial_robot_estimation::GROUND_TRUTH)) && !receive_groundtruth_odom_)
        {
          setGroundTruthPosVel(pos_, vel_);
        }

      /* start experiment estimation */
      if(!(estimate_mode_ & (1 << aerial_robot_estimation::EXPERIMENT_ESTIMATE))) return;

      for(auto& fuser : estimator_->getFuser(aerial_robot_estimation::EXPERIMENT_ESTIMATE))
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

                  if(kf->getStateDim() == 2 && acc_bias_noise_sigma_ > 0)
                    {
                      if(kf->getPredictionNoiseCovariance().rows() == 0)
                        return;

                      VectorXd input_noise_sigma(2);
                      input_noise_sigma << kf->getPredictionNoiseCovariance()(0, 0),
                        acc_bias_noise_sigma_;

                      kf->setPredictionNoiseCovariance(input_noise_sigma);
                      kf->setInitState(raw_pos_[index], 0);
                    }

                  /* correction */
                  VectorXd measure_sigma(1);
                  measure_sigma << pos_noise_sigma_;
                  VectorXd meas(1); meas << raw_pos_[index];
                  vector<double> params = {kf_plugin::POS};
                  kf->correction(meas, measure_sigma, -1, params); //no time sync
                  // VectorXd state = kf->getEstimateState();
                  // estimator_->setState(index + 3, aerial_robot_estimation::EXPERIMENT_ESTIMATE, 0, state(0));
                  // estimator_->setState(index + 3, aerial_robot_estimation::EXPERIMENT_ESTIMATE, 1, state(1));
                }

              if(plugin_name == "aerial_robot_base/kf_xy_roll_pitch_bias")
                {
                  if((id & (1 << State::X_BASE)) && (id & (1 << State::Y_BASE)))
                    {
                      /* correction */
                      VectorXd measure_sigma(2);
                      measure_sigma << pos_noise_sigma_, pos_noise_sigma_;
                      VectorXd meas(2); meas <<  raw_pos_[0], raw_pos_[1];
                      vector<double> params = {kf_plugin::POS};
                      /* time sync and delay process: get from kf time stamp */
                      kf->correction(meas, measure_sigma, -1, params); // no time sync

                      VectorXd state = kf->getEstimateState();
                      /* temp */
                      estimator_->setState(State::X_BASE, aerial_robot_estimation::EXPERIMENT_ESTIMATE, 0, state(0));
                      estimator_->setState(State::X_BASE, aerial_robot_estimation::EXPERIMENT_ESTIMATE, 1, state(1));
                      estimator_->setState(State::Y_BASE, aerial_robot_estimation::EXPERIMENT_ESTIMATE, 0, state(2));
                      estimator_->setState(State::Y_BASE, aerial_robot_estimation::EXPERIMENT_ESTIMATE, 1, state(3));
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













