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
#include <aerial_robot_msgs/Acc.h>
#include <geometry_msgs/Vector3.h>
#include <spinal/SimpleImu.h>
#include <spinal/Imu.h>
#include <sensor_msgs/Imu.h>

using namespace Eigen;
using namespace std;

namespace sensor_plugin
{
  class Imu : public sensor_plugin::SensorBase
  {
  public:
    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, string sensor_name)
    {
      SensorBase::initialize(nh, nhp, estimator, sensor_name);
      rosParamInit();

      acc_pub_ = nh_.advertise<aerial_robot_msgs::Acc>("acc", 2);
      imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_pub_topic_name_, 1);

      if(imu_board_ == D_BOARD)
        {
          imu_sub_ = nh_.subscribe<spinal::Imu>(imu_topic_name_, 1, boost::bind(&Imu::ImuCallback, this, _1, false));
        }

      if(imu_board_ == KDUINO)
        {
          imu_sub_ = nh_.subscribe<spinal::SimpleImu>(imu_topic_name_, 1, &Imu::kduinoImuCallback, this);
        }

    }

    ~Imu () {}
    Imu ():
      calib_count_(100),
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
    { }

    const static uint8_t D_BOARD = 1;
    const static uint8_t KDUINO = 0;

    inline tf::Vector3 getAttitude(uint8_t frame)  { return euler_; }
    inline ros::Time getStamp(){return imu_stamp_;}

  private:
    ros::Publisher  acc_pub_;
    ros::Publisher  imu_pub_;
    ros::Subscriber  imu_sub_, sub_imu_sub_;
    ros::Subscriber  imu_simple_sub_;

    /* rosparam */
    string imu_topic_name_;
    string imu_pub_topic_name_;
    int imu_board_;

    int calib_count_;
    double acc_scale_, gyro_scale_, mag_scale_; /* the scale of sensor value */
    double level_acc_noise_sigma_, z_acc_noise_sigma_, acc_bias_noise_sigma_, angle_bias_noise_sigma_; /* sigma for kf */
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

    double calib_time_;

    ros::Time imu_stamp_;

    void ImuCallback(const spinal::ImuConstPtr& imu_msg, bool sub_imu_board)
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
      updateHealthStamp(imu_msg->stamp.toSec());
    }

    void kduinoImuCallback(const spinal::SimpleImuConstPtr& imu_msg)
    {
      imu_stamp_ = imu_msg->stamp;

      for(int i = 0; i < 3; i++)
        {
          /* acc */
          acc_b_[i] = imu_msg->accData[i] * acc_scale_;

          /* euler */
          if(i == 2)
            {
              euler_[i] = M_PI * imu_msg->angle[2] / 180.0;
            }
          else /* raw data is 10 times */
            euler_[0]  = M_PI * imu_msg->angle[0] / 10.0 / 180.0;
        }

      imuDataConverter();
    }

    void imuDataConverter()
    {
      static int bias_calib = 0;
      static ros::Time prev_time;

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
      acc_l_ = orientation * acc_b_  - tf::Vector3(0, 0, BasicEstimator::G); /* use x,y for factor4 and z for factor3 */
      //acc_l_.setZ((orientation * tf::Vector3(0, 0, acc_[Frame::BODY].z())).z() - BasicEstimator::G);
#else  // use approximation
      acc_l_ = orientation * tf::Vector3(0, 0, acc_b_.z()) - tf::Vector3(0, 0, BasicEstimator::G);
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
      estimator_->setState(State::ROLL_BASE, BasicEstimator::EGOMOTION_ESTIMATE, 0, euler_[0]);
      estimator_->setState(State::PITCH_BASE, BasicEstimator::EGOMOTION_ESTIMATE, 0, euler_[1]);
      estimator_->setState(State::ROLL_BASE, BasicEstimator::EXPERIMENT_ESTIMATE, 0, euler_[0]);
      estimator_->setState(State::PITCH_BASE, BasicEstimator::EXPERIMENT_ESTIMATE, 0, euler_[1]);
      /* not good for ground truth mode, but do temporarily */
      estimator_->setState(State::ROLL_BASE, BasicEstimator::GROUND_TRUTH, 0, euler_[0]);
      estimator_->setState(State::PITCH_BASE, BasicEstimator::GROUND_TRUTH, 0, euler_[1]);

      /* yaw */
      if(!estimator_->getStateStatus(State::YAW_BASE, BasicEstimator::EGOMOTION_ESTIMATE))
        estimator_->setState(State::YAW_BASE, BasicEstimator::EGOMOTION_ESTIMATE, 0, euler_[2]);

      if(!estimator_->getStateStatus(State::YAW_BASE, BasicEstimator::EXPERIMENT_ESTIMATE))
        estimator_->setState(State::YAW_BASE, BasicEstimator::EXPERIMENT_ESTIMATE, 0, euler_[2]);

      estimator_->setAngularVel(Frame::BASELINK, BasicEstimator::EGOMOTION_ESTIMATE, omega_);
      estimator_->setAngularVel(Frame::BASELINK, BasicEstimator::EXPERIMENT_ESTIMATE, omega_);
      /* not good for ground truth mode, but do temporarily */
      estimator_->setAngularVel(Frame::BASELINK, BasicEstimator::GROUND_TRUTH, omega_);

      /* COG */
      /* TODO: only imu can assign to cog state for estimate mode and experiment mode */
      double roll, pitch, yaw;
      (estimator_->getOrientation(Frame::BASELINK, BasicEstimator::EGOMOTION_ESTIMATE) * estimator_->getCog2Baselink().getBasis().inverse()).getRPY(roll, pitch, yaw);
      estimator_->setEuler(Frame::COG, BasicEstimator::EGOMOTION_ESTIMATE, tf::Vector3(roll, pitch, yaw));
      estimator_->setAngularVel(Frame::COG, BasicEstimator::EGOMOTION_ESTIMATE, estimator_->getCog2Baselink().getBasis() * omega_);

      (estimator_->getOrientation(Frame::BASELINK, BasicEstimator::EXPERIMENT_ESTIMATE) * estimator_->getCog2Baselink().getBasis().inverse()).getRPY(roll, pitch, yaw);
      estimator_->setEuler(Frame::COG, BasicEstimator::EXPERIMENT_ESTIMATE, tf::Vector3(roll, pitch, yaw));
      estimator_->setAngularVel(Frame::COG, BasicEstimator::EXPERIMENT_ESTIMATE, estimator_->getCog2Baselink().getBasis() * omega_);

      /* TODO: not good for ground truth mode, but do temporarily */
      (estimator_->getOrientation(Frame::BASELINK, BasicEstimator::GROUND_TRUTH) * estimator_->getCog2Baselink().getBasis().inverse()).getRPY(roll, pitch, yaw);
      estimator_->setEuler(Frame::COG, BasicEstimator::GROUND_TRUTH, tf::Vector3(roll, pitch, yaw));
      estimator_->setAngularVel(Frame::COG, BasicEstimator::GROUND_TRUTH, estimator_->getCog2Baselink().getBasis() * omega_);

      /* bais calibration */
      if(bias_calib < calib_count_)
        {
          bias_calib ++;

          if(bias_calib == 2)
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

                      if(plugin_name == "kalman_filter/kf_pos_vel_acc_bias")
                        {
                          VectorXd intput_noise_sigma(2);

                          /* do not update model here */
                          /* (boost::static_pointer_cast<kf_plugin::KalmanFilterPosVelAccBias>(kf))->updatePredictModel(sensor_dt_); */

                          if(id & (1 << State::X_BASE))
                            {
                              intput_noise_sigma << level_acc_noise_sigma_, acc_bias_noise_sigma_;
                              kf->setInitState(acc_bias_w_.x(), 2);
                            }
                          else if(id & (1 << State::Y_BASE))
                            {
                              intput_noise_sigma << level_acc_noise_sigma_, acc_bias_noise_sigma_;
                              kf->setInitState(acc_bias_w_.y(), 2);
                            }
                          else if((id & (1 << State::Z_BASE)))
                            {
                              intput_noise_sigma << z_acc_noise_sigma_, acc_bias_noise_sigma_;
                              kf->setInitState(acc_bias_w_.z(), 2);
                            }
                          kf->setInputSigma(intput_noise_sigma);
                          start_predict = true;
                        }

                      if(plugin_name == "kalman_filter/kf_pos_vel_acc")
                        {
                          VectorXd intput_noise_sigma(1);
                          if((id & (1 << State::X_BASE)) || (id & (1 << State::Y_BASE)))
                            intput_noise_sigma << level_acc_noise_sigma_;
                          else if((id & (1 << State::Z_BASE)))
                            intput_noise_sigma << z_acc_noise_sigma_;
                          kf->setInputSigma(intput_noise_sigma);
                          start_predict = true;
                        }

                      if(plugin_name == "aerial_robot_base/kf_xy_roll_pitch_bias")
                        {
                          if((id & (1 << State::X_BASE)) && (id & (1 << State::Y_BASE)))
                            {
                              VectorXd intput_noise_sigma(5);
                              intput_noise_sigma <<
                                level_acc_noise_sigma_,
                                level_acc_noise_sigma_,
                                level_acc_noise_sigma_,
                                angle_bias_noise_sigma_,
                                angle_bias_noise_sigma_;
                              kf->setInputSigma(intput_noise_sigma);
                              start_predict = true;
                            }
                        }

                      if(start_predict) kf->setInputFlag();
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
                          VectorXd intput_val(1);
                          if(id & (1 << State::X_BASE))
                            {
                              intput_val << (double)acc_non_bias_w_.x();
                              axis = State::X_BASE;
                            }
                          else if(id & (1 << State::Y_BASE))
                            {
                              intput_val << (double)acc_non_bias_w_.y();
                              axis = State::Y_BASE;
                            }
                          else if(id & (1 << State::Z_BASE))
                            {
                              intput_val << (double)acc_non_bias_w_.z();
                              axis = State::Z_BASE;

                              /* considering the undescend mode, such as the phase of takeoff, the velocity should not below than 0 */
                              if(estimator_->getUnDescendMode() && (kf->getEstimateState())(1) < 0)
                                kf->resetState();

                            }
                          kf->prediction(intput_val, params, imu_stamp_.toSec());
                          VectorXd estimate_state = kf->getEstimateState();
                          estimator_->setState(axis, mode, 0, estimate_state(0));
                          estimator_->setState(axis, mode, 1, estimate_state(1));
                        }

                      if(plugin_name == "kalman_filter/kf_pos_vel_acc_bias")
                        {
                          VectorXd intput_val(2);
                          if(id & (1 << State::X_BASE))
                            {
                              intput_val << acc_w_.x(), 0;
                              axis = State::X_BASE;
                            }
                          else if(id & (1 << State::Y_BASE))
                            {
                              intput_val << acc_w_.y(), 0;
                              axis = State::Y_BASE;
                            }
                          else if(id & (1 << State::Z_BASE))
                            {
                              intput_val << acc_w_.z(), 0;
                              axis = State::Z_BASE;

                              /* considering the undescend mode, such as the phase of takeoff, the velocity should not below than 0 */
                              if(estimator_->getUnDescendMode() && (kf->getEstimateState())(1) < 0)
                                kf->resetState();

                              /* get the estiamted offset(bias) */
                              acc_bias_b_.setZ((kf->getEstimateState())(2));
                            }
                          kf->prediction(intput_val, params, imu_stamp_.toSec());
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

                              VectorXd intput_val(5);
                              intput_val <<
                                acc_b_[0],
                                acc_b_[1],
                                acc_b_[2] - acc_bias_b_.z(),
                                0,
                                0;

                              kf->prediction(intput_val, params, imu_stamp_.toSec());
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
          estimator_->setState(State::Z_BASE, BasicEstimator::EGOMOTION_ESTIMATE, 2, acc_non_bias_w_.z());
          estimator_->setState(State::Z_BASE, BasicEstimator::EXPERIMENT_ESTIMATE, 2, acc_non_bias_w_.z());

          /* 2017.7.25: calculate the state in COG frame using the Baselink frame */
          /* pos_cog = pos_baselink - R * pos_cog2baselink */
          int estimate_mode = BasicEstimator::EGOMOTION_ESTIMATE;
          estimator_->setPos(Frame::COG, estimate_mode,
                             estimator_->getPos(Frame::BASELINK, estimate_mode)
                             + estimator_->getOrientation(Frame::BASELINK, estimate_mode)
                             * estimator_->getCog2Baselink().inverse().getOrigin());
          /* vel_cog = vel_baselink - R * (w x pos_cog2baselink) */
          estimator_->setVel(Frame::COG, estimate_mode,
                             estimator_->getVel(Frame::BASELINK, estimate_mode)
                             + estimator_->getOrientation(Frame::BASELINK, estimate_mode)
                             * (estimator_->getAngularVel(Frame::BASELINK, estimate_mode).cross(estimator_->getCog2Baselink().inverse().getOrigin())));


          estimate_mode = BasicEstimator::EXPERIMENT_ESTIMATE;
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
          publishFilteredImuData();
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

    void publishFilteredImuData()
    {
      sensor_msgs::Imu imu_data;
      imu_data.header.stamp = imu_stamp_;
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

      nhp_.param("acc_bias_noise_sigma", acc_bias_noise_sigma_, 0.01 );
      if(param_verbose_) cout << ns << ": acc bias noise sigma  is " << acc_bias_noise_sigma_ << endl;

      nhp_.param("angle_bias_noise_sigma", angle_bias_noise_sigma_, 0.001 );
      if(param_verbose_) cout << ns << ": angle bias noise sigma  is " << angle_bias_noise_sigma_ << endl;


      nhp_.param("calib_time", calib_time_, 2.0 );
      if(param_verbose_) cout << ns << ": imu calib time is " << calib_time_ << endl;

      nhp_.param("landing_shock_force_thre", landing_shock_force_thre_, 5.0 );
      if(param_verbose_) cout << ns << ": landing shock force_thre is " << landing_shock_force_thre_ << endl;

      nhp_.param("imu_board", imu_board_, 1);
      if(imu_board_ != D_BOARD)
        ROS_WARN(" imu board is %s\n", (imu_board_ == KDUINO)?"kduino":"other board");
      if(imu_board_ == KDUINO)
        {
          nhp_.param("acc_scale", acc_scale_, BasicEstimator::G / 512.0);
          if(param_verbose_) cout << ns << ": acc scale is" << acc_scale_ << endl;
          nhp_.param("gyro_scale", gyro_scale_, (2279 * M_PI)/((32767.0 / 4.0f ) * 180.0));
          if(param_verbose_) cout << ns << ": gyro scale is" << gyro_scale_ << endl;
          nhp_.param("mag_scale", mag_scale_, 1200 / 32768.0);
          if(param_verbose_) cout << ns << ": mag scale is" << mag_scale_ << endl;
        }
      nhp_.param("imu_pub_topic_name_", imu_pub_topic_name_, string("imu_filtered"));
    }

  };


};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Imu, sensor_plugin::SensorBase);



