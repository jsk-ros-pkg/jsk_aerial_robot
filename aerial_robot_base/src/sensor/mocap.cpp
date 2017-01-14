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

#ifndef MOCAP_H_
#define MOCAP_H_

/* ros */
#include <ros/ros.h>

/* base class */
#include <aerial_robot_base/sensor_base_plugin.h>

/* ros msg */
#include <aerial_robot_base/DesireCoord.h>
#include <aerial_robot_base/States.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>

using namespace Eigen;
using namespace std;

namespace sensor_plugin
{
  class Mocap : public sensor_plugin::SensorBase
  {
  public:

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, std::vector< boost::shared_ptr<sensor_plugin::SensorBase> > sensors, std::vector<std::string> sensor_names, int sensor_index)
    {
      baseParamInit(nh, nhp, estimator, sensor_names[sensor_index], sensor_index);
      estimator_ = estimator;

      rosParamInit();

      mocap_sub_ = nh_.subscribe("/aerial_robot/pose", 1, &Mocap::poseCallback, this, ros::TransportHints().udp());

      cog_offset_sub_ = nh_.subscribe(cog_rotate_sub_name_, 5, &Mocap::cogOffsetCallback, this);

      pose_stamped_pub_ = nh_.advertise<aerial_robot_base::States>(pub_name_, 5);

      ground_truth_pose_.states.resize(6);

      //low pass filter
      for(size_t i = 0; i < lpf_pos_vel_.size(); i ++)
        lpf_pos_vel_[i] = IirFilter((float)rx_freq_, (float)cutoff_pos_freq_,(float)cutoff_vel_freq_);

      for(size_t i = 0; i < lpf_acc_.size(); i ++)
        lpf_acc_[i] = IirFilter((float)rx_freq_, (float)cutoff_pos_freq_);
    }

    ~Mocap() {}

    Mocap():
      raw_pos_(0, 0, 0),
      raw_vel_(0, 0, 0),
      raw_acc_(0, 0, 0),
      raw_euler_(0, 0, 0),
      raw_omega_(0, 0, 0),
      pos_(0, 0, 0),
      vel_(0, 0, 0),
      acc_(0, 0, 0),
      euler_(0, 0, 0),
      omega_(0, 0, 0),
      prev_raw_pos_(0, 0, 0),
      prev_raw_vel_(0, 0, 0),
      prev_raw_euler_(0, 0, 0),
      pos_offset_(0, 0, 0),
      cog_offset_angle_(0)
    {}

    static constexpr int TIME_SYNC_CALIB_COUNT = 10;

  private:
    /* ros */
    ros::Publisher  pose_stamped_pub_;
    ros::Subscriber mocap_sub_;
    ros::Subscriber cog_offset_sub_;

    /* ros param */
    double rx_freq_;
    double cutoff_pos_freq_;
    double cutoff_vel_freq_;
    std::string pub_name_;
    std::string cog_rotate_sub_name_;

    /* base var */
    BasicEstimator* estimator_;

    double pos_noise_sigma_, angle_noise_sigma_;

    array<IirFilter, 4> lpf_pos_vel_; /* x, y, z, yaw */
    array<IirFilter, 3> lpf_acc_;

    tf::Vector3 raw_pos_, raw_vel_, raw_acc_, raw_euler_, raw_omega_;
    tf::Vector3 pos_, vel_, acc_, euler_, omega_;

    tf::Vector3 prev_raw_pos_, prev_raw_vel_, prev_raw_euler_;
    tf::Vector3 pos_offset_;
    float cog_offset_angle_;

    /* ros msg */
    aerial_robot_base::States ground_truth_pose_;

    void poseCallback(const geometry_msgs::PoseStampedConstPtr & msg)
    {
      static bool first_flag = true;
      static ros::Time previous_time;

      if(!first_flag)
        {
          float delta_t = msg->header.stamp.toSec() - previous_time.toSec();
          raw_pos_ = tf::Vector3(msg->pose.position.x, msg->pose.position.y,
                                 msg->pose.position.z - pos_offset_.z());
          raw_vel_ = (raw_pos_ - prev_raw_pos_) / delta_t;
          raw_acc_ = (raw_vel_ - prev_raw_vel_) / delta_t;

          tf::Quaternion q(msg->pose.orientation.x,
                           msg->pose.orientation.y,
                           msg->pose.orientation.z,
                           msg->pose.orientation.w);
          tfScalar r = 0, p = 0, y = 0;
          tf::Matrix3x3(q).getRPY(r, p, y);
          raw_euler_.setValue(r, p, y);

          float y_offset = 0;
          if(raw_euler_[2] - prev_raw_euler_[2] > M_PI) y_offset = -2 * M_PI;
          else if(raw_euler_[2] - prev_raw_euler_[2] < -M_PI) y_offset = 2 * M_PI;

          raw_omega_ = (raw_euler_ + tf::Vector3(0, 0, y_offset) - prev_raw_euler_) / delta_t;

          for(int i = 0; i < 4; i++)
            {
              if(i == 3) /* yaw */
                lpf_pos_vel_[i].filterFunction(raw_euler_[2], euler_[2], raw_omega_[2], omega_[2]);
              else
                lpf_pos_vel_[i].filterFunction(raw_pos_[i], pos_[i], raw_vel_[i], vel_[i]);
            }
          for(int i = 0; i < 3; i++)
            lpf_acc_[i].filterFunction(raw_acc_[i], acc_[i]);

          estimator_->setGTState(BasicEstimator::X_W, 0, pos_.x());
          estimator_->setGTState(BasicEstimator::Y_W, 0, pos_.y());
          estimator_->setGTState(BasicEstimator::Z_W, 0, pos_.z());

          float yaw_b = raw_euler_[2] + cog_offset_angle_;
          if (yaw_b > M_PI) yaw_b -= 2 * M_PI;
          if (yaw_b < -M_PI) yaw_b += 2 * M_PI;
          estimator_->setGTState(BasicEstimator::YAW_W_COG, 0, yaw_b);
          estimator_->setGTState(BasicEstimator::YAW_W_B, 0, raw_euler_[2]);

          if(estimate_mode_ & (1 << EXPERIMENT_MODE))
            {
              estimator_->setEXState(BasicEstimator::YAW_W_COG, 0, yaw_b);
              estimator_->setEXState(BasicEstimator::YAW_W_B, 0, raw_euler_[2]);
            }

          estimator_->setGTState(BasicEstimator::X_W, 1, vel_.x());
          estimator_->setGTState(BasicEstimator::Y_W, 1, vel_.y());
          estimator_->setGTState(BasicEstimator::ROLL_W, 0, raw_euler_[0]);
          estimator_->setGTState(BasicEstimator::PITCH_W, 0, raw_euler_[1]);
          estimator_->setGTState(BasicEstimator::Z_W, 1, vel_.z());
          estimator_->setGTState(BasicEstimator::YAW_W_COG, 1, omega_[2]); //bad!!
          estimator_->setGTState(BasicEstimator::YAW_W_B, 1, omega_[2]);




          ground_truth_pose_.header.stamp = msg->header.stamp;

          for(int i = 0; i < 6; i++)
            {
              switch(i)
                {
                case 0:
                    ground_truth_pose_.states[i].id = "x";
                    break;
                case 1:
                    ground_truth_pose_.states[i].id = "y";
                    break;
                case 2:
                    ground_truth_pose_.states[i].id = "z";
                    break;
                case 3:
                    ground_truth_pose_.states[i].id = "roll";
                    break;
                case 4:
                    ground_truth_pose_.states[i].id = "pitch";
                    break;
                case 5:
                    ground_truth_pose_.states[i].id = "yaw";
                    break;
                default:
                  break;
                }

              if(i < 3)
                {
                  ground_truth_pose_.states[i].raw_pos = raw_pos_[i];
                  ground_truth_pose_.states[i].raw_vel = raw_vel_[i];
                  ground_truth_pose_.states[i].pos = pos_[i];
                  ground_truth_pose_.states[i].vel = vel_[i];

                  ground_truth_pose_.states[i].reserves.resize(0);
                  ground_truth_pose_.states[i].reserves.push_back(acc_[i]);
                  ground_truth_pose_.states[i].reserves.push_back(raw_acc_[i]);
                }
              else
                {
                  ground_truth_pose_.states[i].raw_pos = raw_euler_[i];
                  ground_truth_pose_.states[i].raw_vel = raw_omega_[i];
                  ground_truth_pose_.states[i].pos = euler_[i];
                  ground_truth_pose_.states[i].vel = omega_[i];
                }
            }

          /* estimation */
          estimateProcess();
          pose_stamped_pub_.publish(ground_truth_pose_);
        }

      if(first_flag)
        {
          prev_raw_pos_.setValue(msg->pose.position.x,
                             msg->pose.position.y,
                             msg->pose.position.z);

          pos_offset_.setValue(msg->pose.position.x,
                               msg->pose.position.y,
                               msg->pose.position.z);

          /* set ground truth */
          estimator_->setStateStatus(BasicEstimator::X_W, BasicEstimator::GROUND_TRUTH);
          estimator_->setStateStatus(BasicEstimator::Y_W, BasicEstimator::GROUND_TRUTH);
          estimator_->setStateStatus(BasicEstimator::Z_W, BasicEstimator::GROUND_TRUTH);
          estimator_->setStateStatus(BasicEstimator::YAW_W_COG, BasicEstimator::GROUND_TRUTH);
          estimator_->setStateStatus(BasicEstimator::YAW_W_B, BasicEstimator::GROUND_TRUTH);

          if(estimate_mode_ & (1 << EXPERIMENT_MODE))
            {
              Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1);

              for(int i = 0; i < estimator_->getFuserExperimentNum(); i++)
                {
                  if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::X_W)) ||
                     (estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Y_W)) ||
                     (estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::Z_W)))
                    {
                      experiment_indices_.push_back(i);
                      temp(0,0) = pos_noise_sigma_;
                      estimator_->getFuserExperiment(i)->setMeasureSigma(temp);
                      estimator_->getFuserExperiment(i)->setMeasureFlag();
                    }
                  if((estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::YAW_W_COG)) ||
                     (estimator_->getFuserExperimentId(i) & (1 << BasicEstimator::YAW_W_B)))
                    {
                      experiment_indices_.push_back(i);
                      temp(0,0) = angle_noise_sigma_;
                      estimator_->getFuserExperiment(i)->setMeasureSigma(temp);
                      estimator_->getFuserExperiment(i)->setMeasureFlag();
                    }
                }
            }
          first_flag = false;
        }

      prev_raw_pos_ = raw_pos_;
      prev_raw_vel_ = raw_vel_;
      prev_raw_euler_ = raw_euler_;

      previous_time = msg->header.stamp;
      /* consider the remote wirleess transmission, we use the local time server */
      updateHealthStamp(ros::Time::now().toSec());
    }

    void cogOffsetCallback(aerial_robot_base::DesireCoord offset_msg)
    {
      cog_offset_angle_ =  - offset_msg.yaw; //temporarily, cog coord is parent, board to cog
    }

    void rosParamInit(  )
    {
      std::string ns = nhp_.getNamespace();

      nhp_.param("pos_noise_sigma", pos_noise_sigma_, 0.001 );
      if(param_verbose_) cout << "pos noise sigma  is " << pos_noise_sigma_ << endl;

      nhp_.param("angle_sigma", pos_noise_sigma_, 0.001 );
      if(param_verbose_) cout << "pos noise sigma  is " << pos_noise_sigma_ << endl;

      nhp_.param("pub_name", pub_name_, std::string("ground_truth/pose"));
      nhp_.param("cog_rotate_sub_name", cog_rotate_sub_name_, std::string("/desire_coordinate"));

      nhp_.param("rx_freq", rx_freq_, 100.0);
      nhp_.param("cutoff_pos_freq", cutoff_pos_freq_, 20.0);
      nhp_.param("cutoff_vel_freq", cutoff_vel_freq_, 20.0);
    }

    void estimateProcess()
    {
      if(!estimate_flag_) return;


      Eigen::Matrix<double, 1, 1> sigma_temp = Eigen::MatrixXd::Zero(1, 1);
      Eigen::Matrix<double, 1, 1> temp = Eigen::MatrixXd::Zero(1, 1);

      for(vector<int>::iterator it = experiment_indices_.begin(); it != experiment_indices_.end(); ++it )
        {

          if((estimator_->getFuserExperimentId(*it) & (1 << BasicEstimator::X_W)))
            {
              sigma_temp(0,0) = pos_noise_sigma_;
              estimator_->getFuserExperiment(*it)->setMeasureSigma(sigma_temp);
              temp(0, 0) = raw_pos_.x();
              estimator_->getFuserExperiment(*it)->correction(temp);
              ground_truth_pose_.states[0].reserves.push_back((estimator_->getFuserExperiment(*it)->getEstimateState())(0,0));
              ground_truth_pose_.states[0].reserves.push_back((estimator_->getFuserExperiment(*it)->getEstimateState())(1,0));
            }
          else if((estimator_->getFuserExperimentId(*it) & (1 << BasicEstimator::Y_W)))
            {
              sigma_temp(0,0) = pos_noise_sigma_;
              estimator_->getFuserExperiment(*it)->setMeasureSigma(sigma_temp);

              temp(0, 0) = raw_pos_.y();
              estimator_->getFuserExperiment(*it)->correction(temp);

              ground_truth_pose_.states[1].reserves.push_back((estimator_->getFuserExperiment(*it)->getEstimateState())(0,0));
              ground_truth_pose_.states[1].reserves.push_back((estimator_->getFuserExperiment(*it)->getEstimateState())(1,0));
            }
          else if((estimator_->getFuserExperimentId(*it) & (1 << BasicEstimator::Z_W)))
            {
              sigma_temp(0,0) = pos_noise_sigma_;
              estimator_->getFuserExperiment(*it)->setMeasureSigma(sigma_temp);

              temp(0, 0) = raw_pos_.z();
              estimator_->getFuserExperiment(*it)->correction(temp);

              ground_truth_pose_.states[2].reserves.push_back((estimator_->getFuserExperiment(*it)->getEstimateState())(0,0));
              ground_truth_pose_.states[2].reserves.push_back((estimator_->getFuserExperiment(*it)->getEstimateState())(1,0));
            }
          else if((estimator_->getFuserExperimentId(*it) & (1 << BasicEstimator::YAW_W_COG)))
            {
              sigma_temp(0,0) = angle_noise_sigma_;
              estimator_->getFuserExperiment(*it)->setMeasureSigma(sigma_temp);

              temp(0, 0) = raw_euler_[2] + cog_offset_angle_;
              estimator_->getFuserExperiment(*it)->correction(temp);

              ground_truth_pose_.states[5].reserves.push_back((estimator_->getFuserExperiment(*it)->getEstimateState())(0,0));
              ground_truth_pose_.states[5].reserves.push_back((estimator_->getFuserExperiment(*it)->getEstimateState())(1,0));

            }
          else if((estimator_->getFuserExperimentId(*it) & (1 << BasicEstimator::YAW_W_B)))
            {
              sigma_temp(0,0) = angle_noise_sigma_;
              estimator_->getFuserExperiment(*it)->setMeasureSigma(sigma_temp);

              temp(0, 0) = raw_euler_[2];
              estimator_->getFuserExperiment(*it)->correction(temp);

              ground_truth_pose_.states[5].reserves.push_back((estimator_->getFuserExperiment(*it)->getEstimateState())(0,0));
              ground_truth_pose_.states[5].reserves.push_back((estimator_->getFuserExperiment(*it)->getEstimateState())(1,0));
            }
        }
    }
  };
};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Mocap, sensor_plugin::SensorBase);

#endif












