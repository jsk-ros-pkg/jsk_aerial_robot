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
#include <aerial_robot_base/sensor/base_plugin.h>

/* ros msg */
#include <aerial_robot_msgs/Acc.h>
#include <geometry_msgs/Vector3.h>
#include <spinal/SimpleImu.h>
#include <spinal/Imu.h>
#include <sensor_msgs/Imu.h>

using namespace Eigen;
using namespace std;

namespace
{
  int bias_calib = 0;
  ros::Time prev_time;
}

namespace sensor_plugin
{
  class Imu : public sensor_plugin::SensorBase
  {
  public:
    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, StateEstimator* estimator, string sensor_name)
    {
      SensorBase::initialize(nh, nhp, estimator, sensor_name);
      rosParamInit();

      acc_pub_ = nh_.advertise<aerial_robot_msgs::Acc>("acc", 2);
      imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_pub_topic_name_, 1);
      state_pub_ = nh_.advertise<aerial_robot_msgs::States>("data", 1);

      imu_sub_ = nh_.subscribe<spinal::Imu>(imu_topic_name_, 1, &Imu::ImuCallback, this);
    }

    ~Imu () {}
    Imu ():
      calib_count_(200),
      acc_b_(0, 0, 0),
      euler_(0, 0, 0),
      omega_(0, 0, 0),
      mag_(0, 0, 0),
      acc_l_(0, 0, 0),
      acc_w_(0, 0, 0),
      acc_non_bias_w_(0, 0, 0),
      acc_bias_b_(0, 0, 0),
      acc_bias_l_(0, 0, 0),
      acc_bias_w_(0, 0, 0),
      sensor_dt_(0)
    {
      state_.states.resize(3);
      state_.states[0].id = "x";
      state_.states[0].state.resize(2);
      state_.states[1].id = "y";
      state_.states[1].state.resize(2);
      state_.states[2].id = "z";
      state_.states[2].state.resize(2);
    }

    inline tf::Vector3 getAttitude(uint8_t frame)  { return euler_; }
    inline ros::Time getStamp(){return imu_stamp_;}

  private:
    ros::Publisher  acc_pub_;
    ros::Publisher  imu_pub_;
    ros::Publisher  state_pub_;
    ros::Subscriber  imu_sub_, sub_imu_sub_;
    ros::Subscriber  imu_simple_sub_;

    /* rosparam */
    string imu_topic_name_;
    string imu_pub_topic_name_;

    int calib_count_;
    double acc_scale_, gyro_scale_, mag_scale_; /* the scale of sensor value */
    double level_acc_noise_sigma_, z_acc_noise_sigma_, level_acc_bias_noise_sigma_, z_acc_bias_noise_sigma_, angle_bias_noise_sigma_; /* sigma for kf */
    double landing_shock_force_thre_;     /* force */

    /* sensor internal */
    double sensor_dt_;

    /* imu */
    tf::Vector3 euler_; /* the euler angle of both body and cog frame */
    tf::Vector3 omega_; /* the omega both body and cog frame */
    tf::Vector3 mag_; /* the magnetometer both body and cog frame */
    /* acc */
    tf::Vector3 acc_b_; /* the acceleration in baselink frame */
    tf::Vector3 acc_l_; /* the acceleration in level frame as to baselink frame: previously is acc_i */
    tf::Vector3 acc_w_; /* the acceleration in world frame */
    tf::Vector3 acc_non_bias_w_; /* the acceleration without bias in world frame */
    /* acc bias */
    tf::Vector3 acc_bias_b_; /* the acceleration bias in baselink frame, only use z axis  */
    tf::Vector3 acc_bias_l_; /* the acceleration bias in level frame as to baselink frame: previously is acc_i */
    tf::Vector3 acc_bias_w_; /* the acceleration bias in world frame */

    aerial_robot_msgs::States state_; /* for debug */

    double calib_time_;

    ros::Time imu_stamp_;

    void ImuCallback(const spinal::ImuConstPtr& imu_msg)
    {
      imu_stamp_ = imu_msg->stamp;

      for(int i = 0; i < 3; i++)
        {
          acc_b_[i] = imu_msg->acc_data[i];
          euler_[i] = imu_msg->angles[i];
          omega_[i] = imu_msg->gyro_data[i];
          mag_[i] = imu_msg->mag_data[i];
        }

      imuDataConverter();
      updateHealthStamp();
    }

    void imuDataConverter()
    {
      if(imu_stamp_.toSec() <= prev_time.toSec())
        {
          ROS_WARN("IMU: bad timestamp. curr time stamp: %f, prev time stamp: %f",
                   imu_stamp_.toSec(), prev_time.toSec());
          return;
        }

      /* set the time internal */
      sensor_dt_ = imu_stamp_.toSec() - prev_time.toSec();

      tf::Matrix3x3 r; r.setRPY(euler_[0], euler_[1], euler_[2]);

      /* project acc onto level frame using body frame value */
      tf::Matrix3x3 orientation;
      orientation.setRPY(euler_[0], euler_[1], 0);
#if 1
      acc_l_ = orientation * acc_b_  - tf::Vector3(0, 0, StateEstimator::G); /* use x,y for factor4 and z for factor3 */
      //acc_l_.setZ((orientation * tf::Vector3(0, 0, acc_[Frame::BODY].z())).z() - StateEstimator::G);
#else  // use approximation
      acc_l_ = orientation * tf::Vector3(0, 0, acc_b_.z()) - tf::Vector3(0, 0, StateEstimator::G);
#endif

      if(estimator_->getLandingMode() &&
         !estimator_->getLandedFlag() &&
         acc_l_.z() > landing_shock_force_thre_)
        {
          ROS_WARN("imu: touch to ground");
          estimator_->setLandedFlag(true);
        }

      /* base link */
      /* roll & pitch */
      estimator_->setState(State::ROLL_BASE, StateEstimator::EGOMOTION_ESTIMATE, 0, euler_[0]);
      estimator_->setState(State::PITCH_BASE, StateEstimator::EGOMOTION_ESTIMATE, 0, euler_[1]);
      estimator_->setState(State::ROLL_BASE, StateEstimator::EXPERIMENT_ESTIMATE, 0, euler_[0]);
      estimator_->setState(State::PITCH_BASE, StateEstimator::EXPERIMENT_ESTIMATE, 0, euler_[1]);
      /* not good for ground truth mode, but do temporarily */
      estimator_->setState(State::ROLL_BASE, StateEstimator::GROUND_TRUTH, 0, euler_[0]);
      estimator_->setState(State::PITCH_BASE, StateEstimator::GROUND_TRUTH, 0, euler_[1]);

      /* yaw */
      if(!estimator_->getStateStatus(State::YAW_BASE, StateEstimator::EGOMOTION_ESTIMATE))
        estimator_->setState(State::YAW_BASE, StateEstimator::EGOMOTION_ESTIMATE, 0, euler_[2]);

      if(!estimator_->getStateStatus(State::YAW_BASE, StateEstimator::EXPERIMENT_ESTIMATE))
        estimator_->setState(State::YAW_BASE, StateEstimator::EXPERIMENT_ESTIMATE, 0, euler_[2]);

      estimator_->setAngularVel(Frame::BASELINK, StateEstimator::EGOMOTION_ESTIMATE, omega_);
      estimator_->setAngularVel(Frame::BASELINK, StateEstimator::EXPERIMENT_ESTIMATE, omega_);
      /* not good for ground truth mode, but do temporarily */
      estimator_->setAngularVel(Frame::BASELINK, StateEstimator::GROUND_TRUTH, omega_);

      /* COG */
      /* TODO: only imu can assign to cog state for estimate mode and experiment mode */
      double roll, pitch, yaw;
      (estimator_->getOrientation(Frame::BASELINK, StateEstimator::EGOMOTION_ESTIMATE) * estimator_->getCog2Baselink().getBasis().inverse()).getRPY(roll, pitch, yaw);
      estimator_->setEuler(Frame::COG, StateEstimator::EGOMOTION_ESTIMATE, tf::Vector3(roll, pitch, yaw));
      estimator_->setAngularVel(Frame::COG, StateEstimator::EGOMOTION_ESTIMATE, estimator_->getCog2Baselink().getBasis() * omega_);

      (estimator_->getOrientation(Frame::BASELINK, StateEstimator::EXPERIMENT_ESTIMATE) * estimator_->getCog2Baselink().getBasis().inverse()).getRPY(roll, pitch, yaw);
      estimator_->setEuler(Frame::COG, StateEstimator::EXPERIMENT_ESTIMATE, tf::Vector3(roll, pitch, yaw));
      estimator_->setAngularVel(Frame::COG, StateEstimator::EXPERIMENT_ESTIMATE, estimator_->getCog2Baselink().getBasis() * omega_);

      /* TODO: not good for ground truth mode, but do temporarily */
      (estimator_->getOrientation(Frame::BASELINK, StateEstimator::GROUND_TRUTH) * estimator_->getCog2Baselink().getBasis().inverse()).getRPY(roll, pitch, yaw);
      estimator_->setEuler(Frame::COG, StateEstimator::GROUND_TRUTH, tf::Vector3(roll, pitch, yaw));
      estimator_->setAngularVel(Frame::COG, StateEstimator::GROUND_TRUTH, estimator_->getCog2Baselink().getBasis() * omega_);

      /* bais calibration */
      if(bias_calib < calib_count_)
        {
          bias_calib ++;

          if(bias_calib == 100)
            {
              calib_count_ = calib_time_ / sensor_dt_;
              ROS_WARN("calib count is %d", calib_count_);

              /* check whether use imu yaw for contorl and estimation */
              if(!estimator_->getStateStatus(State::YAW_BASE, estimator_->getEstimateMode()))
                ROS_WARN("IMU: use imu mag-based yaw value for estimation");
            }

          /* acc bias */
          acc_bias_l_ += acc_l_;

          if(bias_calib == calib_count_)
            {
              acc_bias_l_ /= calib_count_;
              ROS_WARN("accX bias is %f, accY bias is %f, accZ bias is %f, dt is %f[sec]", acc_bias_l_.x(), acc_bias_l_.y(), acc_bias_l_.z(), sensor_dt_);

              /* fuser for 0: egomotion, 1: experiment */
              for(int mode = 0; mode < 2; mode++)
                {
                  if(!getFuserActivate(mode)) continue;

                  tf::Matrix3x3 orientation;
                  orientation.setRPY(0, 0, (estimator_->getState(State::YAW_BASE, mode))[0]);
                  acc_bias_w_ = orientation * acc_bias_l_;

                  for(auto& fuser : estimator_->getFuser(mode))
                    {
                      string plugin_name = fuser.first;
                      boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
                      int id = kf->getId();

                      bool start_predict = false;

                      if(plugin_name == "kalman_filter/kf_pos_vel_acc")
                        {
                          VectorXd input_noise_sigma(2);
                          if((id & (1 << State::X_BASE)) || (id & (1 << State::Y_BASE)))
                            {
                              input_noise_sigma << level_acc_noise_sigma_, level_acc_bias_noise_sigma_;
                              kf->setPredictionNoiseCovariance(input_noise_sigma);
                              if(level_acc_bias_noise_sigma_ > 0)
                                {
                                  if(id & (1 << State::X_BASE)) kf->setInitState(acc_bias_w_.x(),2);
                                  if(id & (1 << State::Y_BASE)) kf->setInitState(acc_bias_w_.y(),2);
                                }
                            }
                          if(id & (1 << State::Z_BASE))
                            {
                              input_noise_sigma << z_acc_noise_sigma_, z_acc_bias_noise_sigma_;
                              kf->setPredictionNoiseCovariance(input_noise_sigma);
                              if(z_acc_bias_noise_sigma_ > 0) kf->setInitState(acc_bias_w_.z(), 2);
                            }
                          start_predict = true;
                        }

                      if(plugin_name == "aerial_robot_base/kf_xy_roll_pitch_bias")
                        {
                          if((id & (1 << State::X_BASE)) && (id & (1 << State::Y_BASE)))
                            {
                              VectorXd input_noise_sigma(5);
                              input_noise_sigma <<
                                level_acc_noise_sigma_,
                                level_acc_noise_sigma_,
                                level_acc_noise_sigma_,
                                angle_bias_noise_sigma_,
                                angle_bias_noise_sigma_;
                              kf->setPredictionNoiseCovariance(input_noise_sigma);
                              start_predict = true;
                            }
                        }
                      if(start_predict)
                        {
                          kf->setPredictBufSize(1/sensor_dt_); //set the prediction handler buffer size
                          kf->setInputFlag();
                        }
                    }
                }
            }
        }

      if(bias_calib == calib_count_)
        {
          /* fuser for 0: egomotion, 1: experiment */
          for(int mode = 0; mode < 2; mode++)
            {
              if(getFuserActivate(mode))
                {
                  tf::Matrix3x3 orientation;
                  orientation.setRPY(0, 0, (estimator_->getState(State::YAW_BASE, mode))[0]);

                  acc_w_ = orientation * acc_l_;
                  acc_non_bias_w_ = orientation * (acc_l_ - acc_bias_l_);

                  for(auto& fuser : estimator_->getFuser(mode))
                    {
                      string plugin_name = fuser.first;
                      boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
                      int id = kf->getId();
                      vector<double> params = {sensor_dt_};

                      int axis;
                      bool predict = false;

                      if(plugin_name == "kalman_filter/kf_pos_vel_acc")
                        {
                          VectorXd input_val(1);
                          if(id & (1 << State::X_BASE))
                            {
                              input_val << ((level_acc_bias_noise_sigma_ > 0)?acc_w_.x(): acc_non_bias_w_.x());
                              axis = State::X_BASE;
                            }
                          else if(id & (1 << State::Y_BASE))
                            {
                              input_val << ((level_acc_bias_noise_sigma_ > 0)?acc_w_.y(): acc_non_bias_w_.y());
                              axis = State::Y_BASE;
                            }
                          else if(id & (1 << State::Z_BASE))
                            {
                              input_val << ((z_acc_bias_noise_sigma_ > 0)?acc_w_.z(): acc_non_bias_w_.z());
                              axis = State::Z_BASE;

                              /* considering the undescend mode, such as the phase of takeoff, the velocity should not below than 0 */
                              if(estimator_->getUnDescendMode() && (kf->getEstimateState())(1) < 0)
                                kf->resetState();

                              /* get the estiamted offset(bias) */
                              if(z_acc_bias_noise_sigma_ > 0) acc_bias_b_.setZ((kf->getEstimateState())(2));
                            }

                          kf->prediction(input_val, imu_stamp_.toSec(), params);
                          VectorXd estimate_state = kf->getEstimateState();
                          estimator_->setState(axis, mode, 0, estimate_state(0));
                          estimator_->setState(axis, mode, 1, estimate_state(1));
                        }

                      if(plugin_name == "aerial_robot_base/kf_xy_roll_pitch_bias")
                        {
                          if(id & (1 << State::X_BASE) && (id & (1 << State::Y_BASE)))
                            {
                              params.push_back(euler_[0]);
                              params.push_back(euler_[1]);
                              params.push_back(euler_[2]);
                              params.push_back(acc_b_[0]);
                              params.push_back(acc_b_[1]);
                              params.push_back(acc_b_[2] - acc_bias_b_.z());

                              VectorXd input_val(5);
                              input_val <<
                                acc_b_[0],
                                acc_b_[1],
                                acc_b_[2] - acc_bias_b_.z(),
                                0,
                                0;

                              kf->prediction(input_val, imu_stamp_.toSec(), params);
                              VectorXd estimate_state = kf->getEstimateState();
                              estimator_->setState(State::X_BASE, mode, 0, estimate_state(0));
                              estimator_->setState(State::X_BASE, mode, 1, estimate_state(1));
                              estimator_->setState(State::Y_BASE, mode, 0, estimate_state(2));
                              estimator_->setState(State::Y_BASE, mode, 1, estimate_state(3));
                            }
                        }
                    }
                }
            }

          /* TODO: set z acc: should use kf reuslt? */
          estimator_->setState(State::Z_BASE, StateEstimator::EGOMOTION_ESTIMATE, 2, acc_non_bias_w_.z());
          estimator_->setState(State::Z_BASE, StateEstimator::EXPERIMENT_ESTIMATE, 2, acc_non_bias_w_.z());

          /* 2017.7.25: calculate the state in COG frame using the Baselink frame */
          /* pos_cog = pos_baselink - R * pos_cog2baselink */
          int estimate_mode = StateEstimator::EGOMOTION_ESTIMATE;
          estimator_->setPos(Frame::COG, estimate_mode,
                             estimator_->getPos(Frame::BASELINK, estimate_mode)
                             + estimator_->getOrientation(Frame::BASELINK, estimate_mode)
                             * estimator_->getCog2Baselink().inverse().getOrigin());
          /* vel_cog = vel_baselink - R * (w x pos_cog2baselink) */
          estimator_->setVel(Frame::COG, estimate_mode,
                             estimator_->getVel(Frame::BASELINK, estimate_mode)
                             + estimator_->getOrientation(Frame::BASELINK, estimate_mode)
                             * (estimator_->getAngularVel(Frame::BASELINK, estimate_mode).cross(estimator_->getCog2Baselink().inverse().getOrigin())));


          estimate_mode = StateEstimator::EXPERIMENT_ESTIMATE;
          estimator_->setPos(Frame::COG, estimate_mode,
                               estimator_->getPos(Frame::BASELINK, estimate_mode)
                               + estimator_->getOrientation(Frame::BASELINK, estimate_mode)
                               * estimator_->getCog2Baselink().inverse().getOrigin());
          estimator_->setVel(Frame::COG, estimate_mode,
                             estimator_->getVel(Frame::BASELINK, estimate_mode)
                             + estimator_->getOrientation(Frame::BASELINK, estimate_mode)
                             * (estimator_->getAngularVel(Frame::BASELINK, estimate_mode).cross(estimator_->getCog2Baselink().inverse().getOrigin())));

          /* no acc, we do not have the angular acceleration */

          publishAccData();
          publishRosImuData();

          /* publish state date */
          state_.header.stamp = imu_stamp_;
          tf::Vector3 pos = estimator_->getPos(Frame::BASELINK, StateEstimator::EGOMOTION_ESTIMATE);
          tf::Vector3 vel = estimator_->getVel(Frame::BASELINK, StateEstimator::EGOMOTION_ESTIMATE);
          state_.states[0].state[0].x = pos.x();
          state_.states[1].state[0].x = pos.y();
          state_.states[2].state[0].x = pos.z();
          state_.states[0].state[0].y = vel.x();
          state_.states[1].state[0].y = vel.y();
          state_.states[2].state[0].y = vel.z();
          state_.states[0].state[0].z = acc_w_.x();
          state_.states[1].state[0].z = acc_w_.y();
          state_.states[2].state[0].z = acc_w_.z();
          pos = estimator_->getPos(Frame::BASELINK, StateEstimator::EXPERIMENT_ESTIMATE);
          vel = estimator_->getVel(Frame::BASELINK, StateEstimator::EXPERIMENT_ESTIMATE);
          state_.states[0].state[1].x = pos.x();
          state_.states[1].state[1].x = pos.y();
          state_.states[2].state[1].x = pos.z();
          state_.states[0].state[1].y = vel.x();
          state_.states[1].state[1].y = vel.y();
          state_.states[2].state[1].y = vel.z();
          state_.states[0].state[1].z = acc_w_.x();
          state_.states[1].state[1].z = acc_w_.y();
          state_.states[2].state[1].z = acc_w_.z();

          state_pub_.publish(state_);

        }
      prev_time = imu_stamp_;
    }

    void publishAccData()
    {
      aerial_robot_msgs::Acc acc_data;
      acc_data.header.stamp = imu_stamp_;

      tf::vector3TFToMsg(acc_b_, acc_data.acc_body_frame);
      tf::vector3TFToMsg(acc_w_, acc_data.acc_world_frame);
      tf::vector3TFToMsg(acc_non_bias_w_, acc_data.acc_non_bias_world_frame);

      acc_pub_.publish(acc_data);
    }

    void publishRosImuData()
    {
      sensor_msgs::Imu imu_data;
      imu_data.header.stamp = imu_stamp_;
      imu_data.orientation = tf::createQuaternionMsgFromRollPitchYaw(euler_[0], euler_[1], euler_[2]);
      tf::vector3TFToMsg(omega_, imu_data.angular_velocity);
      tf::vector3TFToMsg(acc_b_, imu_data.linear_acceleration);
      imu_pub_.publish(imu_data);
    }

    void rosParamInit()
    {
      std::string ns = nhp_.getNamespace();

      nhp_.param("imu_topic_name", imu_topic_name_, string("imu"));
      if(param_verbose_) cout << ns << ": imu topic name is " << imu_topic_name_.c_str() << endl;

      nhp_.param("level_acc_noise_sigma", level_acc_noise_sigma_, 0.01 );
      if(param_verbose_) cout << ns << ": level acc noise sigma  is " << level_acc_noise_sigma_ << endl;
      nhp_.param("z_acc_noise_sigma", z_acc_noise_sigma_, 0.01 );
      if(param_verbose_) cout << ns << ": z acc noise sigma  is " << z_acc_noise_sigma_ << endl;

      nhp_.param("level_acc_bias_noise_sigma", level_acc_bias_noise_sigma_, 0.01 );
      if(param_verbose_) cout << ns << ": level acc bias noise sigma  is " << level_acc_bias_noise_sigma_ << endl;

      nhp_.param("z_acc_bias_noise_sigma", z_acc_bias_noise_sigma_, 0.0);
      if(param_verbose_) cout << ns << ": z acc bias noise sigma  is " << z_acc_bias_noise_sigma_ << endl;

      nhp_.param("angle_bias_noise_sigma", angle_bias_noise_sigma_, 0.001 );
      if(param_verbose_) cout << ns << ": angle bias noise sigma  is " << angle_bias_noise_sigma_ << endl;

      nhp_.param("calib_time", calib_time_, 2.0 );
      if(param_verbose_) cout << ns << ": imu calib time is " << calib_time_ << endl;

      nhp_.param("landing_shock_force_thre", landing_shock_force_thre_, 5.0 );
      if(param_verbose_) cout << ns << ": landing shock force_thre is " << landing_shock_force_thre_ << endl;

      nhp_.param("imu_pub_topic_name_", imu_pub_topic_name_, string("/uav/baselink/imu"));

      /* important scale, record here
        {
          nhp_.param("acc_scale", acc_scale_, StateEstimator::G / 512.0);
          if(param_verbose_) cout << ns << ": acc scale is" << acc_scale_ << endl;
          nhp_.param("gyro_scale", gyro_scale_, (2279 * M_PI)/((32767.0 / 4.0f ) * 180.0));
          if(param_verbose_) cout << ns << ": gyro scale is" << gyro_scale_ << endl;
          nhp_.param("mag_scale", mag_scale_, 1200 / 32768.0);
          if(param_verbose_) cout << ns << ": mag scale is" << mag_scale_ << endl;
        }
      */

    }

  };


};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Imu, sensor_plugin::SensorBase);



