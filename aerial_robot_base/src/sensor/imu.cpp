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
#include <aerial_robot_base/Acc.h>
#include <geometry_msgs/Vector3.h>
#include <aerial_robot_msgs/SimpleImu.h>
#include <aerial_robot_msgs/Imu.h>
#include <aerial_robot_base/DesireCoord.h>

using namespace Eigen;
using namespace std;

namespace sensor_plugin
{
  class Imu : public sensor_plugin::SensorBase
  {
  public:
    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, string sensor_name)
    {
      baseParamInit(nh, nhp, estimator, sensor_name);
      rosParamInit();

      acc_pub_ = nh_.advertise<aerial_robot_base::Acc>("acc", 2);

      cog_offset_sub_ = nh_.subscribe(cog_rotate_sub_name_, 5, &Imu::cogOffsetCallback, this);

      if(imu_board_ == D_BOARD)
        {
          imu_sub_ = nh_.subscribe<aerial_robot_msgs::Imu>(imu_topic_name_, 1, boost::bind(&Imu::ImuCallback, this, _1, false));
        }

      if(imu_board_ == KDUINO)
        {
          imu_sub_ = nh_.subscribe<aerial_robot_msgs::SimpleImu>(imu_topic_name_, 1, &Imu::kduinoImuCallback, this);
        }

    }

    ~Imu () {}
    Imu ():
      calib_count_(100),
      euler_(0, 0, 0),
      acc_b_(0, 0, 0),
      acc_l_(0, 0, 0),
      acc_w_(0, 0, 0),
      acc_non_bias_w_(0, 0, 0),
      acc_bias_l_(0, 0, 0),
      acc_bias_w_(0, 0, 0),
      omega_(0, 0, 0),
      mag_(0, 0, 0),
      cog_offset_angle_(0)
    { }

    const static uint8_t D_BOARD = 1;
    const static uint8_t KDUINO = 0;

    inline tf::Vector3 getEuler()  { return euler_; }
    inline ros::Time getStamp(){return imu_stamp_;}

    static constexpr float G = 9.797;

  private:
    ros::Publisher  acc_pub_;
    ros::Subscriber  imu_sub_, sub_imu_sub_;
    ros::Subscriber  imu_simple_sub_;
    ros::Subscriber cog_offset_sub_;

    /* rosparam */
    string imu_topic_name_;
    std::string cog_rotate_sub_name_;
    int imu_board_;

    int calib_count_;
    double acc_scale_, gyro_scale_, mag_scale_; /* the scale of sensor value */
    double level_acc_noise_sigma_, z_acc_noise_sigma_, acc_bias_noise_sigma_; /* sigma for kf */
    double landing_shock_force_thre_;     /* force */

    /* angle */
    tf::Vector3 euler_; /* the euler angle of cog frame: roll, pitch, yaw */
    /* acc */
    tf::Vector3 acc_b_; /* the acceleration in body frame(cog) */
    tf::Vector3 acc_l_; /* the acceleration in level frame as to body frame(cog): previously is acc_i */
    tf::Vector3 acc_w_; /* the acceleration in world frame */
    tf::Vector3 acc_non_bias_w_; /* the acceleration without bias in world frame */
    /* acc bias */
    tf::Vector3 acc_bias_l_; /* the acceleration bias in level frame as to body frame: previously is acc_i */
    tf::Vector3 acc_bias_w_; /* the acceleration bias in world frame */

    /* other imu sensor values */
    tf::Vector3 omega_; /* the omega in cog frame */
    tf::Vector3 mag_; /* the magnetometer value in cog frame */
    double calib_time_;
    float cog_offset_angle_; /* the offset between cog coord and frame coord */

    ros::Time imu_stamp_;

    void ImuCallback(const aerial_robot_msgs::ImuConstPtr& imu_msg, bool sub_imu_board)
    {
      imu_stamp_ = imu_msg->stamp;

      for(int i = 0; i < 3; i++)
        {
          euler_[i] = imu_msg->angles[i];
          acc_b_[i] = imu_msg->acc_data[i];
          omega_[i] = imu_msg->gyro_data[i];
          mag_[i] = imu_msg->mag_data[i];
        }

      imuDataConverter(imu_stamp_);
      updateHealthStamp(imu_msg->stamp.toSec());
    }

    void kduinoImuCallback(const aerial_robot_msgs::SimpleImuConstPtr& imu_msg)
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

      imuDataConverter(imu_stamp_);
    }

    void imuDataConverter(ros::Time stamp)
    {
      static int bias_calib = 0;
      static ros::Time prev_time;
      static double sensor_hz_ = 0;
      /* calculate accTran */
#if 1 // use x,y for factor4 and z for factor3
      tf::Matrix3x3 orientation;
      orientation.setRPY(euler_[0], euler_[1], 0);
      acc_l_ = orientation * acc_b_;
      acc_l_.setZ((orientation * tf::Vector3(0, 0, acc_b_.z())).z() - G);

#else  // use approximation
      acc_l_ = orientation * tf::Vector3(0, 0, acc_b_.z()) - tf::Vector3(0, 0, G);
#endif

      if(estimator_->getLandingMode() &&
         !estimator_->getLandedFlag() &&
         acc_l_.z() > landing_shock_force_thre_)
        {
          ROS_WARN("imu: touch to ground");
          estimator_->setLandedFlag(true);
        }

      /* roll & pitch */
      estimator_->setState(BasicEstimator::ROLL_W, BasicEstimator::EGOMOTION_ESTIMATE, 0, euler_[0]);
      estimator_->setState(BasicEstimator::ROLL_W, BasicEstimator::EXPERIMENT_ESTIMATE, 0, euler_[0]);
      estimator_->setState(BasicEstimator::ROLL_W, BasicEstimator::EGOMOTION_ESTIMATE, 1, omega_[0]);
      estimator_->setState(BasicEstimator::ROLL_W, BasicEstimator::EXPERIMENT_ESTIMATE, 1, omega_[0]);
      estimator_->setState(BasicEstimator::PITCH_W, BasicEstimator::EGOMOTION_ESTIMATE, 0, euler_[1]);
      estimator_->setState(BasicEstimator::PITCH_W, BasicEstimator::EXPERIMENT_ESTIMATE, 0, euler_[1]);
      estimator_->setState(BasicEstimator::PITCH_W, BasicEstimator::EGOMOTION_ESTIMATE, 1, omega_[1]);
      estimator_->setState(BasicEstimator::PITCH_W, BasicEstimator::EXPERIMENT_ESTIMATE, 1, omega_[1]);

      /* yaw */
      if(estimator_->getStateStatus(BasicEstimator::YAW_W) == BasicEstimator::RAW)
        {
          ROS_WARN("debug: use mcu imu yaw for cog");
          estimator_->setState(BasicEstimator::YAW_W, BasicEstimator::EGOMOTION_ESTIMATE, 0, euler_[2]);
          estimator_->setState(BasicEstimator::YAW_W, BasicEstimator::EXPERIMENT_ESTIMATE, 0, euler_[2]);
        }
      if(estimator_->getStateStatus(BasicEstimator::YAW_W_B) == BasicEstimator::RAW)
        {
          ROS_WARN("debug: use mcu imu yaw for board");
          estimator_->setState(BasicEstimator::YAW_W_B, BasicEstimator::EGOMOTION_ESTIMATE, 0, euler_[2] + cog_offset_angle_);
          estimator_->setState(BasicEstimator::YAW_W_B, BasicEstimator::EXPERIMENT_ESTIMATE, 0, euler_[2] + cog_offset_angle_);
        }

      /* bais calibration */
      if(bias_calib < calib_count_)
        {
          bias_calib ++;

          if(bias_calib == 1) prev_time = imu_stamp_;

          double time_interval = imu_stamp_.toSec() - prev_time.toSec();
          if(bias_calib == 2)
            {
              calib_count_ = calib_time_ / time_interval;
              ROS_WARN("calib count is %d, time interval is %f", calib_count_, time_interval);

              /* check whether use imu yaw for contorl and estimation */
              if(estimator_->getStateStatus(BasicEstimator::YAW_W) == BasicEstimator::RAW || estimator_->getStateStatus(BasicEstimator::YAW_W_B) == BasicEstimator::RAW)
                ROS_WARN("use imu mag-based yaw value for estimation and control");
            }

          /* hz estimation */
          sensor_hz_ += time_interval;

          /* acc bias */
          acc_bias_l_ += acc_l_;

          if(bias_calib == calib_count_)
            {
              acc_bias_l_ /= calib_count_;
              sensor_hz_ /= (calib_count_ - 1 );
              ROS_WARN("accX bias is %f, accY bias is %f, accZ bias is %f, hz is %f", acc_bias_l_.x(), acc_bias_l_.y(), acc_bias_l_.z(), sensor_hz_);

              /* fuser for 0: egomotion, 1: experiment */
              for(int mode = 0; mode < 2; mode++)
                {
                  if(!getFuserActivate(mode)) continue;

                  tf::Matrix3x3 orientation;
                  orientation.setRPY(0, 0, (estimator_->getState(BasicEstimator::YAW_W, mode))[0]);
                  tf::Vector3 acc_bias_w_ = orientation * acc_bias_l_;

                  for(auto& fuser : estimator_->getFuser(mode))
                    {
                      string plugin_name = fuser.first;
                      boost::shared_ptr<kf_base_plugin::KalmanFilter> kf = fuser.second;
                      kf->updateModelFromDt(sensor_hz_);
                      int id = kf->getId();

                      if(plugin_name == "kalman_filter/kf_pose_vel_acc_bias")
                        {
                          Matrix<double, 2, 1> temp = MatrixXd::Zero(2, 1);
                          temp(1,0) = acc_bias_noise_sigma_;
                          if(id & (1 << BasicEstimator::X_B))
                            {
                              temp(0,0) = level_acc_noise_sigma_;
                              kf->setInitState(acc_bias_l_.x(), 2);
                            }
                          else if(id & (1 << BasicEstimator::X_W))
                            {
                              temp(0,0) = level_acc_noise_sigma_;
                              kf->setInitState(acc_bias_w_.x(), 2);
                            }
                          else if(id & (1 << BasicEstimator::Y_B))
                            {
                              temp(0,0) = level_acc_noise_sigma_;
                              kf->setInitState(acc_bias_l_.y(), 2);
                            }
                          else if(id & (1 << BasicEstimator::Y_W))
                            {
                              kf->setInitState(acc_bias_w_.y(), 2);
                            }
                          else if((id & (1 << BasicEstimator::Z_W)))
                            {
                              temp(0,0) = z_acc_noise_sigma_;
                              kf->setInitState(acc_bias_w_.z(), 2);
                            }
                          kf->setInputSigma(temp);
                        }

                      if(plugin_name == "kalman_filter/kf_pose_vel_acc")
                        {
                          Matrix<double, 1, 1> temp = MatrixXd::Zero(1, 1);

                          if((id & (1 << BasicEstimator::X_W)) ||
                             (id & (1 << BasicEstimator::X_B)) ||
                             (id & (1 << BasicEstimator::Y_W)) ||
                             (id & (1 << BasicEstimator::Y_B)))
                            {
                              temp(0,0) = level_acc_noise_sigma_;
                            }
                          else if((id & (1 << BasicEstimator::Z_W)))
                            {
                              temp(0,0) = z_acc_noise_sigma_;
                            }
                          kf->setInputSigma(temp);
                        }
                      kf->setInputFlag();
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
                  orientation.setRPY(0, 0, (estimator_->getState(BasicEstimator::YAW_W, mode))[0]);

                  acc_w_ = orientation * acc_l_;
                  acc_non_bias_w_ = orientation * (acc_l_ - acc_bias_l_);

                  Matrix<double, 1, 1> temp = MatrixXd::Zero(1, 1);
                  Matrix<double, 2, 1> temp2 = MatrixXd::Zero(2, 1);

                  for(auto& fuser : estimator_->getFuser(mode))
                    {
                      string plugin_name = fuser.first;
                      boost::shared_ptr<kf_base_plugin::KalmanFilter> kf = fuser.second;
                      int id = kf->getId();

                      MatrixXd estimate_state;
                      int axis;

                      if(plugin_name == "kalman_filter/kf_pose_vel_acc")
                        {
                          if(id & (1 << BasicEstimator::X_W))
                            {
                              temp(0, 0) = (double)acc_non_bias_w_.x();
                              axis = BasicEstimator::X_W;
                            }
                          else if(id & (1 << BasicEstimator::X_B))
                            {
                              temp(0, 0) = (double)acc_l_.x();
                              axis = BasicEstimator::X_B;
                            }
                          else if(id & (1 << BasicEstimator::Y_W))
                            {
                              temp(0, 0) = (double)acc_non_bias_w_.y();
                              axis = BasicEstimator::Y_W;
                            }
                          else if(id & (1 << BasicEstimator::Y_B))
                            {
                              temp(0, 0) = (double)acc_l_.y();
                              axis = BasicEstimator::Y_B;
                            }
                          else if(id & (1 << BasicEstimator::Z_W))
                            {
                              temp(0, 0) = (double)acc_non_bias_w_.z();
                              axis = BasicEstimator::Z_W;

                              /* considering the undescend mode, such as the phase of takeoff, the velocity should not below than 0 */
                              if(estimator_->getUnDescendMode() && (kf->getEstimateState())(1,0) < 0)
                                kf->resetState();
                            }
                          kf->prediction(temp);
                        }

                      if(plugin_name == "kalman_filter/kf_pose_vel_acc_bias")
                        {
                          if(id & (1 << BasicEstimator::X_W))
                            {
                              temp2(0, 0) = (double)acc_w_.x();
                              axis = BasicEstimator::X_W;
                            }
                          else if(id & (1 << BasicEstimator::X_B))
                            {
                              temp2(0, 0) = (double)acc_l_.x();
                              axis = BasicEstimator::X_B;
                            }
                          else if(id & (1 << BasicEstimator::Y_W))
                            {
                              temp2(0, 0) = (double)acc_w_.y();
                              axis = BasicEstimator::Y_W;
                            }
                          else if(id & (1 << BasicEstimator::Y_B))
                            {
                              temp2(0, 0) = (double)acc_l_.y();
                              axis = BasicEstimator::Y_B;
                            }
                          else if(id & (1 << BasicEstimator::Z_W))
                            {
                              temp2(0, 0) = (double)acc_w_.z();
                              axis = BasicEstimator::Z_W;

                              /* considering the undescend mode, such as the phase of takeoff, the velocity should not below than 0 */
                              if(estimator_->getUnDescendMode() && (kf->getEstimateState())(1,0) < 0)
                                kf->resetState();

                            }
                          kf->prediction(temp2);
                        }
                      estimate_state = kf->getEstimateState();
                      estimator_->setState(axis, mode, 0, estimate_state(0,0));
                      estimator_->setState(axis, mode, 1, estimate_state(1,0));
                    }
                }
            }

          /* TODO: set z acc: should use kf reuslt? */
          estimator_->setState(BasicEstimator::Z_W, BasicEstimator::EGOMOTION_ESTIMATE, 2, acc_non_bias_w_.z());
          estimator_->setState(BasicEstimator::Z_W, BasicEstimator::EXPERIMENT_ESTIMATE, 2, acc_non_bias_w_.z());

          publishAccData(stamp);
        }
      prev_time = imu_stamp_;
    }

    void publishAccData(ros::Time stamp)
    {
      aerial_robot_base::Acc acc_data;
      acc_data.header.stamp = stamp;

      tf::vector3TFToMsg(acc_b_, acc_data.acc_body_frame);
      tf::vector3TFToMsg(acc_w_, acc_data.acc_world_frame);
      tf::vector3TFToMsg(acc_non_bias_w_, acc_data.acc_non_bias_world_frame);

      acc_pub_.publish(acc_data);
    }

    void rosParamInit()
    {
      string ns = nhp_.getNamespace();

      nhp_.param("imu_topic_name", imu_topic_name_, string("imu"));
      if(param_verbose_) cout << " imu topic name is " << imu_topic_name_.c_str() << endl;

      nhp_.param("level_acc_noise_sigma", level_acc_noise_sigma_, 0.01 );
      if(param_verbose_) cout << "level acc noise sigma  is " << level_acc_noise_sigma_ << endl;
      nhp_.param("z_acc_noise_sigma", z_acc_noise_sigma_, 0.01 );
      if(param_verbose_) cout << "z acc noise sigma  is " << z_acc_noise_sigma_ << endl;

      nhp_.param("acc_bias_noise_sigma", acc_bias_noise_sigma_, 0.01 );
      if(param_verbose_) cout << "acc bias noise sigma  is " << acc_bias_noise_sigma_ << endl;

      nhp_.param("calib_time", calib_time_, 2.0 );
      if(param_verbose_) cout << "imu calib time is " << calib_time_ << endl;

      nhp_.param("landing_shock_force_thre", landing_shock_force_thre_, 5.0 );
      if(param_verbose_) cout << "landing shock force_thre is " << landing_shock_force_thre_ << endl;

      nhp_.param("cog_rotate_sub_name", cog_rotate_sub_name_, std::string("/desire_coordinate"));

      nhp_.param("imu_board", imu_board_, 1);
      if(imu_board_ != D_BOARD)
        ROS_WARN(" imu board is %s\n", (imu_board_ == KDUINO)?"kduino":"other board");
      if(imu_board_ == KDUINO)
        {
          nhp_.param("acc_scale", acc_scale_, G / 512.0);
          if(param_verbose_) cout << "acc scale is" << acc_scale_ << endl;
          nhp_.param("gyro_scale", gyro_scale_, (2279 * M_PI)/((32767.0 / 4.0f ) * 180.0));
          if(param_verbose_) cout << "gyro scale is" << gyro_scale_ << endl;
          nhp_.param("mag_scale", mag_scale_, 1200 / 32768.0);
          if(param_verbose_) cout << "mag scale is" << mag_scale_ << endl;
        }
    }

    void cogOffsetCallback(aerial_robot_base::DesireCoord offset_msg)
    {
      cog_offset_angle_ = offset_msg.yaw; //offset is from cog to board
    }

  };


};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Imu, sensor_plugin::SensorBase);



