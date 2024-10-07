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

namespace
{
  int bias_calib = 0;
  ros::Time prev_time;
  bool first_flag = true;
}

namespace sensor_plugin
{
  Imu::Imu ():
    sensor_plugin::SensorBase(),
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
    sensor_dt_(0),
    treat_imu_as_ground_truth_(true)
  {
    state_.states.resize(3);
    state_.states[0].id = "x";
    state_.states[0].state.resize(2);
    state_.states[1].id = "y";
    state_.states[1].state.resize(2);
    state_.states[2].id = "z";
    state_.states[2].state.resize(2);
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

    //low pass filter
    double sample_freq, cutoff_freq;
    getParam<double>("cutoff_freq", cutoff_freq, 20.0);
    getParam<double>("sample_freq", sample_freq, 200.0);
    lpf_omega_ = IirFilter(sample_freq, cutoff_freq, 3);

    // debug
    omega_filter_pub_ = indexed_nhp_.advertise<geometry_msgs::Vector3Stamped>(string("filter_angular_velocity"), 1);
  }

  void Imu::ImuCallback(const spinal::ImuConstPtr& imu_msg)
  {
    imu_stamp_ = imu_msg->stamp;
    tf::Vector3 filtered_omega;

    for(int i = 0; i < 3; i++)
      {
        if(std::isnan(imu_msg->acc_data[i]) || std::isnan(imu_msg->angles[i]) ||
           std::isnan(imu_msg->gyro_data[i]) || std::isnan(imu_msg->mag_data[i]))
          {
            ROS_ERROR_THROTTLE(1.0, "IMU sensor publishes Nan value!");
            return;
          }

        acc_b_[i] = imu_msg->acc_data[i];
        euler_[i] = imu_msg->angles[i];
        omega_[i] = imu_msg->gyro_data[i];
        mag_[i] = imu_msg->mag_data[i];
      }

    if(first_flag)
      {
        lpf_omega_.setInitValues(omega_);
        first_flag = false;
      }
    filtered_omega = lpf_omega_.filterFunction(omega_);
    geometry_msgs::Vector3Stamped omega_msg;
    omega_msg.header.stamp = imu_msg->stamp;
    tf::vector3TFToMsg(filtered_omega, omega_msg.vector);
    omega_filter_pub_.publish(omega_msg);

    // workaround: use raw roll&pitch omega (not filtered in spinal) for both angular and linear CoG velocity estimation, yaw is still filtered
    // note: this is different with hydrus-like control which use filtered omega for CoG estimation
    omega_.setZ(filtered_omega.z());

    // get filtered angular and linear velocity of CoG
    tf::Transform cog2baselink_tf;
    tf::transformKDLToTF(robot_model_->getCog2Baselink<KDL::Frame>(), cog2baselink_tf);
    int estimate_mode = estimator_->getEstimateMode();
    setFilteredOmegaCog(cog2baselink_tf.getBasis() * filtered_omega);
    setFilteredVelCog(estimator_->getVel(Frame::BASELINK, estimate_mode)
                      + estimator_->getOrientation(Frame::BASELINK, estimate_mode)
                      * (filtered_omega.cross(cog2baselink_tf.inverse().getOrigin())));

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

    tf::Matrix3x3 r; r.setRPY(euler_[0], euler_[1], euler_[2]);

    /* project acc onto level frame using body frame value */
    tf::Matrix3x3 orientation;
    orientation.setRPY(euler_[0], euler_[1], 0);
#if 1
    acc_l_ = orientation * acc_b_  - tf::Vector3(0, 0, aerial_robot_estimation::G); /* use x,y for factor4 and z for factor3 */
    //acc_l_.setZ((orientation * tf::Vector3(0, 0, acc_[Frame::BODY].z())).z() - aerial_robot_estimation::G);
#else  // use approximation
    acc_l_ = orientation * tf::Vector3(0, 0, acc_b_.z()) - tf::Vector3(0, 0, aerial_robot_estimation::G);
#endif

    /* base link */
    /* roll & pitch */
    estimator_->setState(State::ROLL_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE, 0, euler_[0]);
    estimator_->setState(State::PITCH_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE, 0, euler_[1]);
    estimator_->setState(State::ROLL_BASE, aerial_robot_estimation::EXPERIMENT_ESTIMATE, 0, euler_[0]);
    estimator_->setState(State::PITCH_BASE, aerial_robot_estimation::EXPERIMENT_ESTIMATE, 0, euler_[1]);

    /* yaw */
    if(!estimator_->getStateStatus(State::YAW_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE))
      estimator_->setState(State::YAW_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE, 0, euler_[2]);

    if(!estimator_->getStateStatus(State::YAW_BASE, aerial_robot_estimation::EXPERIMENT_ESTIMATE))
      estimator_->setState(State::YAW_BASE, aerial_robot_estimation::EXPERIMENT_ESTIMATE, 0, euler_[2]);

    estimator_->setAngularVel(Frame::BASELINK, aerial_robot_estimation::EGOMOTION_ESTIMATE, omega_);
    estimator_->setAngularVel(Frame::BASELINK, aerial_robot_estimation::EXPERIMENT_ESTIMATE, omega_);

    /* COG */
    /* TODO: only imu can assign to cog state for estimate mode and experiment mode */
    tf::Transform cog2baselink_tf;
    tf::transformKDLToTF(robot_model_->getCog2Baselink<KDL::Frame>(), cog2baselink_tf);

    double roll, pitch, yaw;
    (estimator_->getOrientation(Frame::BASELINK, aerial_robot_estimation::EGOMOTION_ESTIMATE) * cog2baselink_tf.inverse().getBasis()).getRPY(roll, pitch, yaw);
    estimator_->setEuler(Frame::COG, aerial_robot_estimation::EGOMOTION_ESTIMATE, tf::Vector3(roll, pitch, yaw));
    estimator_->setAngularVel(Frame::COG, aerial_robot_estimation::EGOMOTION_ESTIMATE, cog2baselink_tf.getBasis() * omega_);

    (estimator_->getOrientation(Frame::BASELINK, aerial_robot_estimation::EXPERIMENT_ESTIMATE) * cog2baselink_tf.inverse().getBasis()).getRPY(roll, pitch, yaw);
    estimator_->setEuler(Frame::COG, aerial_robot_estimation::EXPERIMENT_ESTIMATE, tf::Vector3(roll, pitch, yaw));
    estimator_->setAngularVel(Frame::COG, aerial_robot_estimation::EXPERIMENT_ESTIMATE, cog2baselink_tf.getBasis() * omega_);

    /* Ground Truth if necessary */
    if(treat_imu_as_ground_truth_)
      {
        /* set baselink angles for roll and pitch, yaw is obtained from mocap */
        estimator_->setState(State::ROLL_BASE, aerial_robot_estimation::GROUND_TRUTH, 0, euler_[0]);
        estimator_->setState(State::PITCH_BASE, aerial_robot_estimation::GROUND_TRUTH, 0, euler_[1]);
        /* set cog angles for all axes */
        (estimator_->getOrientation(Frame::BASELINK, aerial_robot_estimation::GROUND_TRUTH) * cog2baselink_tf.inverse().getBasis()).getRPY(roll, pitch, yaw);
        estimator_->setEuler(Frame::COG, aerial_robot_estimation::GROUND_TRUTH, tf::Vector3(roll, pitch, yaw));
        /* set baselink angular velocity for all axes using imu omega */
        estimator_->setAngularVel(Frame::BASELINK, aerial_robot_estimation::GROUND_TRUTH, omega_);
        /* set cog angular velocity for all axes using imu omega */
        estimator_->setAngularVel(Frame::COG, aerial_robot_estimation::GROUND_TRUTH, cog2baselink_tf.getBasis() * omega_);
      }

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
        acc_bias_l_ += acc_l_;

        if(bias_calib == calib_count_)
          {
            acc_bias_l_ /= calib_count_;
            ROS_WARN("accX bias is %f, accY bias is %f, accZ bias is %f, dt is %f[sec]", acc_bias_l_.x(), acc_bias_l_.y(), acc_bias_l_.z(), sensor_dt_);

            estimator_->setQueueSize(1/sensor_dt_);

            setStatus(Status::ACTIVE);

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
        estimator_->setState(State::Z_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE, 2, acc_non_bias_w_.z());
        estimator_->setState(State::Z_BASE, aerial_robot_estimation::EXPERIMENT_ESTIMATE, 2, acc_non_bias_w_.z());

        /* set the rotation and angular velocity for the temporal queue for other sensor with time delay */
        estimator_->updateQueue(imu_stamp_.toSec(), euler_[0], euler_[1], omega_);
        /* TODO: we ignore yaw becuase it is relatively slower than other axes in the case of under -actuated system */

        /* 2017.7.25: calculate the state in COG frame using the Baselink frame */
        /* pos_cog = pos_baselink - R * pos_cog2baselink */
        int estimate_mode = aerial_robot_estimation::EGOMOTION_ESTIMATE;
        estimator_->setPos(Frame::COG, estimate_mode,
                           estimator_->getPos(Frame::BASELINK, estimate_mode)
                           + estimator_->getOrientation(Frame::BASELINK, estimate_mode)
                           * cog2baselink_tf.inverse().getOrigin());
        estimator_->setVel(Frame::COG, estimate_mode,
                           estimator_->getVel(Frame::BASELINK, estimate_mode)
                           + estimator_->getOrientation(Frame::BASELINK, estimate_mode)
                           * (estimator_->getAngularVel(Frame::BASELINK, estimate_mode).cross(cog2baselink_tf.inverse().getOrigin())));


        estimate_mode = aerial_robot_estimation::EXPERIMENT_ESTIMATE;
        estimator_->setPos(Frame::COG, estimate_mode,
                           estimator_->getPos(Frame::BASELINK, estimate_mode)
                           + estimator_->getOrientation(Frame::BASELINK, estimate_mode)
                           * cog2baselink_tf.inverse().getOrigin());
        estimator_->setVel(Frame::COG, estimate_mode,
                           estimator_->getVel(Frame::BASELINK, estimate_mode)
                           + estimator_->getOrientation(Frame::BASELINK, estimate_mode)
                           * (estimator_->getAngularVel(Frame::BASELINK, estimate_mode).cross(cog2baselink_tf.inverse().getOrigin())));

        /* no acc, we do not have the angular acceleration */

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
            state_.states[0].state[i].z = acc_w_.x();
            state_.states[1].state[i].z = acc_w_.y();
            state_.states[2].state[i].z = acc_w_.z();
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
    tf::vector3TFToMsg(acc_w_, acc_data.acc_world_frame);
    tf::vector3TFToMsg(acc_non_bias_w_, acc_data.acc_non_bias_world_frame);

    acc_pub_.publish(acc_data);
  }

  void Imu::publishRosImuData()
  {
    sensor_msgs::Imu imu_data;
    imu_data.header.stamp = imu_stamp_;
    imu_data.orientation = tf::createQuaternionMsgFromRollPitchYaw(euler_[0], euler_[1], euler_[2]);
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



