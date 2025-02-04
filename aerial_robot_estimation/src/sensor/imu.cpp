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

#include <aerial_robot_estimation/sensor/imu.h>
#include <aerial_robot_estimation/sensor/vo.h>

namespace
{
  int bias_calib = 0;
  ros::Time prev_time;
}

using namespace aerial_robot_estimation;

namespace sensor_plugin
{
  Imu::Imu ():
    sensor_plugin::SensorBase(),
    calib_count_(200),
    acc_b_(0, 0, 0),
    omega_(0, 0, 0),
    mag_(0, 0, 0),
    acc_bias_b_(0, 0, 0),
    sensor_dt_(0)
  {
    state_.states.resize(3);
    state_.states[0].id = "x";
    state_.states[0].state.resize(2);
    state_.states[1].id = "y";
    state_.states[1].state.resize(2);
    state_.states[2].id = "z";
    state_.states[2].state.resize(2);

    acc_w_.at(0) = tf::Vector3(0, 0, 0);
    acc_w_.at(1) = tf::Vector3(0, 0, 0);
    acc_non_bias_w_.at(0) = tf::Vector3(0, 0, 0);
    acc_non_bias_w_.at(1) = tf::Vector3(0, 0, 0);
    acc_bias_w_.at(0) = tf::Vector3(0, 0, 0);
    acc_bias_w_.at(1) = tf::Vector3(0, 0, 0);

    raw_rot_.setIdentity();
  }

  void Imu::initialize(ros::NodeHandle nh,
                       boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                       boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                       string sensor_name, int index)
  {
    SensorBase::initialize(nh, robot_model, estimator, sensor_name, index);
    rosParamInit();

    std::string topic_name;
    getParam<std::string>("imu_topic_name", topic_name, string("imu"));
    imu_sub_ = nh_.subscribe<spinal::Imu>(topic_name, 10, &Imu::ImuCallback, this);
    imu_pub_ = indexed_nhp_.advertise<sensor_msgs::Imu>(string("ros_converted"), 1);
    acc_pub_ = indexed_nhp_.advertise<aerial_robot_msgs::Acc>("acc_only", 2);
  }

  void Imu::ImuCallback(const spinal::ImuConstPtr& imu_msg)
  {
    imu_stamp_ = imu_msg->stamp;

    for(int i = 0; i < 3; i++)
      {
        if(std::isnan(imu_msg->acc[i]) || std::isnan(imu_msg->gyro[i]) || std::isnan(imu_msg->mag[i]))
          {
            ROS_ERROR_THROTTLE(1.0, "IMU plugin receives Nan value in IMU sensors !");
            return;
          }

        acc_b_[i] = imu_msg->acc[i]; // baselink frame
        omega_[i] = imu_msg->gyro[i];  // baselink frame
        mag_[i] = imu_msg->mag[i];  // baselink frame
       }

    if(std::isnan(imu_msg->quaternion[0]) || std::isnan(imu_msg->quaternion[1]) ||
       std::isnan(imu_msg->quaternion[2]) || std::isnan(imu_msg->quaternion[3]))
      {
        ROS_ERROR_THROTTLE(1.0, "IMU plugin receives Nan value in Quaternion!");
        return;
      }

    tf::Quaternion raw_q(imu_msg->quaternion[0], imu_msg->quaternion[1],
                         imu_msg->quaternion[2], imu_msg->quaternion[3]);
    raw_rot_ = tf::Matrix3x3(raw_q);

    estimateProcess();
    updateHealthStamp();
  }

  void Imu::estimateProcess()
  {
    if(imu_stamp_.toSec() <= prev_time.toSec())
      {
        ROS_WARN("IMU: bad timestamp. curr time stamp: %f, prev time stamp: %f",
                 imu_stamp_.toSec(), prev_time.toSec());
        return;
      }

    /* set the time internal */
    sensor_dt_ = imu_stamp_.toSec() - prev_time.toSec();

    tf::Transform cog2baselink_tf;
    tf::transformKDLToTF(robot_model_->getCog2Baselink<KDL::Frame>(), cog2baselink_tf);

    tf::Vector3 wx_b = raw_rot_.getRow(0);
    tf::Vector3 wy_b = raw_rot_.getRow(1);
    tf::Vector3 wz_b = raw_rot_.getRow(2);

    tf::Vector3 wz_c = cog2baselink_tf.getBasis() * wz_b;
    tf::Vector3 omega_c = cog2baselink_tf.getBasis() * omega_;

    // 1. mode for EGOMOTION_ESTIMATE and EXPERIMENT_ESTIMATE
    std::map<int, tf::Matrix3x3> rots;
    for (int i = 0; i < 2; i++)
      {
        // check if there is a refined (better) yaw estimation handler (e.g. VO, RTK-GPS)
        if (estimator_->hasRefinedYawEstimate(i))
          {
            // only update the wz_b vector (the vector only related to gravity)
            estimator_->setOrientationWzB(Frame::BASELINK, i, wz_b);
            estimator_->setOrientationWzB(Frame::COG, i, wz_c);
          }
        else
          {
            estimator_->setOrientation(Frame::BASELINK, i, raw_rot_);
            tf::Matrix3x3 rot_c = raw_rot_ * cog2baselink_tf.getBasis().transpose();
            estimator_->setOrientation(Frame::COG, i, rot_c);
          }

        // angular velocity
        estimator_->setAngularVel(Frame::BASELINK, i, omega_);
        estimator_->setAngularVel(Frame::COG, i, omega_c);

        // re-obtain the rotation and store to a map for later usage
        rots[i] = estimator_->getOrientation(Frame::BASELINK, i);

        acc_w_.at(i) = rots.at(i) * acc_b_ - tf::Vector3(0, 0, aerial_robot_estimation::G);
        acc_non_bias_w_.at(i) = acc_w_.at(i) - acc_bias_w_.at(i);
      }

    // 2. mode for GROUND_TRUTH
    if (!estimator_->hasGroundTruthOdom())
      {
        // the orientation is set from ground truth plugin (mocap)

        /* set baselink angular velocity for all axes using imu omega */
        estimator_->setAngularVel(Frame::BASELINK, GROUND_TRUTH, omega_);
        /* set cog angular velocity for all axes using imu omega */
        estimator_->setAngularVel(Frame::COG, GROUND_TRUTH, omega_c);
      }

    // 3. set the rotation and angular velocity for the temporal queue for other sensor with time delay
    estimator_->updateQueue(imu_stamp_.toSec(),
                            rots.at(EGOMOTION_ESTIMATE),
                            rots.at(EXPERIMENT_ESTIMATE),
                            omega_);


    /* bais calibration */
    if(bias_calib < calib_count_)
      {
        bias_calib ++;

        if(bias_calib == 100) // warm up for callback to be stable subscribe
          {
            calib_count_ = calib_time_ / sensor_dt_;
            ROS_WARN("calib count is %d", calib_count_);

            setStatus(Status::INIT); // start init
          }

        /* acc bias */
        for (int i = 0; i < 2; i++)
          acc_bias_w_.at(i) += acc_w_.at(i);

        if(bias_calib == calib_count_)
          {
            for (int i = 0; i < 2; i++)
              acc_bias_w_.at(i) /= calib_count_;

            tf::Matrix3x3 rot_inv = rots.at(EGOMOTION_ESTIMATE).inverse();
            tf::Vector3 acc_bias_b = rot_inv * acc_bias_w_.at(EGOMOTION_ESTIMATE);
            ROS_INFO("acc bias w.r.t body frame: [%f, %f, %f], dt: %f[sec]", acc_bias_b.x(), acc_bias_b.y(), acc_bias_b.z(), sensor_dt_);

            estimator_->setQueueSize(1/sensor_dt_);

            setStatus(Status::ACTIVE);

            /* fuser for 0: egomotion, 1: experiment */
            for(int mode = 0; mode < 2; mode++)
              {
                if(!getFuserActivate(mode)) continue;

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
                                if(id & (1 << State::X_BASE)) kf->setInitState(acc_bias_w_.at(mode).x(),2);
                                if(id & (1 << State::Y_BASE)) kf->setInitState(acc_bias_w_.at(mode).y(),2);
                              }
                          }
                        if(id & (1 << State::Z_BASE))
                          {
                            input_noise_sigma << z_acc_noise_sigma_, z_acc_bias_noise_sigma_;
                            kf->setPredictionNoiseCovariance(input_noise_sigma);
                            if(z_acc_bias_noise_sigma_ > 0) kf->setInitState(acc_bias_w_.at(mode).z(), 2);
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
                            input_val << ((level_acc_bias_noise_sigma_ > 0)?acc_w_.at(mode).x(): acc_non_bias_w_.at(mode).x());
                            axis = State::X_BASE;
                          }
                        else if(id & (1 << State::Y_BASE))
                          {
                            input_val << ((level_acc_bias_noise_sigma_ > 0)?acc_w_.at(mode).y(): acc_non_bias_w_.at(mode).y());
                            axis = State::Y_BASE;
                          }
                        else if(id & (1 << State::Z_BASE))
                          {
                            input_val << ((z_acc_bias_noise_sigma_ > 0)?acc_w_.at(mode).z(): acc_non_bias_w_.at(mode).z());
                            axis = State::Z_BASE;

                            /* considering the undescend mode, such as the phase of takeoff, the velocity should not below than 0 */
                            if(estimator_->getUnDescendMode() && (kf->getEstimateState())(1) < 0)
                              kf->resetState();

                            /* get the estiamted offset(bias) */
                            if(z_acc_bias_noise_sigma_ > 0) acc_bias_w_.at(mode).setZ((kf->getEstimateState())(2));
                          }

                        kf->prediction(input_val, imu_stamp_.toSec(), params);
                        VectorXd estimate_state = kf->getEstimateState();
                        estimator_->setState(axis, mode, 0, estimate_state(0));
                        estimator_->setState(axis, mode, 1, estimate_state(1));
                      }

                    if(plugin_name == "aerial_robot_base/kf_xy_roll_pitch_bias")
                      {
                        tf::Matrix3x3 rot = rots.at(mode);
                        tf::Vector3 acc_bias_b = rot.inverse() * acc_bias_w_.at(mode);

                        double r, p, y; rot.getRPY(r, p, y);
                        if(id & (1 << State::X_BASE) && (id & (1 << State::Y_BASE)))
                          {
                            params.push_back(r);
                            params.push_back(p);
                            params.push_back(y);
                            params.push_back(acc_b_[0]);
                            params.push_back(acc_b_[1]);
                            params.push_back(acc_b_[2] - acc_bias_b.z());

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
        for (int i = 0; i < 2; i++)
          estimator_->setState(State::Z_BASE, i, 2, acc_non_bias_w_.at(i).z());

        /* calculate the state in COG frame using the Baselink frame */
        /* TODO: the joint velocity */
        for (int i = 0; i < 2; i++)
          {
            estimator_->setPos(Frame::COG, i,
                               estimator_->getPos(Frame::BASELINK, i)
                               + estimator_->getOrientation(Frame::BASELINK, i)
                               * cog2baselink_tf.inverse().getOrigin());
            estimator_->setVel(Frame::COG, i,
                               estimator_->getVel(Frame::BASELINK, i)
                               + estimator_->getOrientation(Frame::BASELINK, i)
                               * (estimator_->getAngularVel(Frame::BASELINK, i).cross(cog2baselink_tf.inverse().getOrigin())));
          }

        publishAccData();
        publishRosImuData();

        /* publish state date */
        state_.header.stamp = imu_stamp_;
        for (int i = 0; i < 2; i++)
          {
            tf::Vector3 pos = estimator_->getPos(Frame::BASELINK, i);
            tf::Vector3 vel = estimator_->getVel(Frame::BASELINK, i);
            state_.states[0].state[i].x = pos.x();
            state_.states[1].state[i].x = pos.y();
            state_.states[2].state[i].x = pos.z();
            state_.states[0].state[i].y = vel.x();
            state_.states[1].state[i].y = vel.y();
            state_.states[2].state[i].y = vel.z();
            state_.states[0].state[i].z = acc_w_.at(i).x();
            state_.states[1].state[i].z = acc_w_.at(i).y();
            state_.states[2].state[i].z = acc_w_.at(i).z();
          }
        state_pub_.publish(state_);
      }
    prev_time = imu_stamp_;
  }

  void Imu::publishAccData()
  {
    aerial_robot_msgs::Acc acc_data;
    acc_data.header.stamp = imu_stamp_;

    tf::vector3TFToMsg(acc_b_, acc_data.acc_body_frame);
    tf::vector3TFToMsg(acc_w_.at(0), acc_data.acc_world_frame);
    tf::vector3TFToMsg(acc_non_bias_w_.at(0), acc_data.acc_non_bias_world_frame);

    acc_pub_.publish(acc_data);
  }

  void Imu::publishRosImuData()
  {
    sensor_msgs::Imu imu_data;
    imu_data.header.stamp = imu_stamp_;
    tf::Quaternion q;
    raw_rot_.getRotation(q);
    tf::quaternionTFToMsg(q, imu_data.orientation);
    tf::vector3TFToMsg(omega_, imu_data.angular_velocity);
    tf::vector3TFToMsg(acc_b_, imu_data.linear_acceleration);
    imu_pub_.publish(imu_data);
  }

  void Imu::rosParamInit()
  {
    std::string ns = nhp_.getNamespace();

    getParam<double>("level_acc_noise_sigma", level_acc_noise_sigma_, 0.01 );
    getParam<double>("z_acc_noise_sigma", z_acc_noise_sigma_, 0.01 );
    getParam<double>("level_acc_bias_noise_sigma", level_acc_bias_noise_sigma_, 0.01 );
    getParam<double>("z_acc_bias_noise_sigma", z_acc_bias_noise_sigma_, 0.0);
    getParam<double>("angle_bias_noise_sigma", angle_bias_noise_sigma_, 0.001 );
    getParam<double>("calib_time", calib_time_, 2.0 );

    /* important scale, record here
       {
       nhp_.param("acc_scale", acc_scale_, aerial_robot_estimation::G / 512.0);
       if(param_verbose_) cout << ns << ": acc scale is" << acc_scale_ << endl;
       nhp_.param("gyro_scale", gyro_scale_, (2279 * M_PI)/((32767.0 / 4.0f ) * 180.0));
       if(param_verbose_) cout << ns << ": gyro scale is" << gyro_scale_ << endl;
       nhp_.param("mag_scale", mag_scale_, 1200 / 32768.0);
       if(param_verbose_) cout << ns << ": mag scale is" << mag_scale_ << endl;
       }
    */

  }
};
/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Imu, sensor_plugin::SensorBase);



