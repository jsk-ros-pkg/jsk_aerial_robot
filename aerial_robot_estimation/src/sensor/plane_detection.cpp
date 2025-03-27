// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, JSK Lab
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

#include <aerial_robot_estimation/sensor/plane_detection.h>

namespace sensor_plugin
{
  PlaneDetection::PlaneDetection():
    SensorBase(),
    raw_plane_pos_z_(0.0),
    prev_raw_plane_pos_z_(0.0),
    raw_plane_vel_z_(0.0)
  {
    plane_detection_state_.states.resize(1);
    plane_detection_state_.states[0].id = "plane";
    plane_detection_state_.states[0].state.resize(2);
  }

  void PlaneDetection::initialize(ros::NodeHandle nh,
                                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                  string sensor_name, int index)
  {
    SensorBase::initialize(nh, robot_model, estimator, sensor_name, index);
    rosParamInit();
    getParam<std::string>("plane_detection_sub_topic_name", plane_detection_sub_topic_name_, std::string("output_coefficients"));
    plane_sub_ = nh_.subscribe(plane_detection_sub_topic_name_, 10, &PlaneDetection::planeCallback, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / timer_freq_), &PlaneDetection::timerCallback, this);
    queue_length_ = static_cast<int>(state_save_time_ * timer_freq_);
  }

  void PlaneDetection::timerCallback(const ros::TimerEvent& e)
  {
    pos_z_queue_.push_back(std::make_pair<double, double>(ros::Time::now().toSec(), static_cast<double>(estimator_->getPos(Frame::BASELINK, aerial_robot_estimation::EGOMOTION_ESTIMATE).z())));
    if (pos_z_queue_.size() > queue_length_) {
       pos_z_queue_.pop_front();
    }
  }

  void PlaneDetection::planeCallback(const jsk_recognition_msgs::ModelCoefficientsArray& msg)
  {
    /* only do egomotion estimate mode */
    if (!getFuserActivate(aerial_robot_estimation::EGOMOTION_ESTIMATE)) {
      ROS_WARN_THROTTLE(1, "Plane detection: no egmotion estimate mode");
      return;
    }

    /* check whether is force att control mode */
    if (estimator_->getForceAttControlFlag() && getStatus() == Status::ACTIVE) {
      setStatus(Status::INVALID);
    }

    if (msg.coefficients.size() == 0) {
      ROS_DEBUG("no plane found");
      return; //no plane found
    }

    if (!updateBaseLink2SensorTransform()) {
      ROS_DEBUG("tf update failed");
      return;
    }

    curr_timestamp_ = msg.header.stamp.toSec() + delay_;

    /* initialization */
    if (getStatus() == Status::INACTIVE) {

      // 1. check altimeter is initialized, and check the height
      bool alt_initialized = false;
      for (const auto& handler: estimator_->getAltHandlers()) {
        if (handler->getStatus() == Status::ACTIVE) {
          alt_initialized = true;
          break;
        }
      }
      if (!alt_initialized && estimator_->getAltHandlers().size() > 0) {
        ROS_WARN_THROTTLE(1, "vo: no altimeter is initialized, wait");
        return;
      }
      if (estimator_->getState(State::Z_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE)[0] < min_height_) {
        return;
      }

      // 2. check imu is initialized, and check the camera direction
      bool imu_initialized = false;
      for (const auto& handler: estimator_->getImuHandlers()) {
        if(handler->getStatus() == Status::ACTIVE) {
          imu_initialized = true;
          break;
        }
      }
      if (!imu_initialized) {
        ROS_WARN_THROTTLE(1, "vo: no imu is initialized, wait");
        return;
      }

      if (!isCameraAngleValid()) {
        ROS_DEBUG("invalid camera angle");
        return;
      }

      // 3. if the height and camera direction is OK, check a valid plane is found
      if (!findValidPlane(msg)) {
        ROS_DEBUG("invalid plane");
        return;
      }

      // 4. if a valid plane is found, initialize kalman filter

      // 4.1 if dynamic_offset is true, set height_offset
      if (dynamic_offset_) {
        bool good_timestamp_found = false;
        double uav_z_good;
        double pos_z_timestamp;

        for (const auto& pos_z : pos_z_queue_) {
          if (pos_z.first > msg.header.stamp.toSec()) {
            uav_z_good = pos_z.second;
            pos_z_timestamp = pos_z.first;
            good_timestamp_found = true;
            break;
          }
        }

        if (!good_timestamp_found) {
          uav_z_good = estimator_->getPos(Frame::BASELINK, aerial_robot_estimation::EGOMOTION_ESTIMATE).z();
          pos_z_timestamp = pos_z_queue_.back().first;
        }

        height_offset_ = uav_z_good - raw_plane_pos_z_;
        ROS_DEBUG("Set dynamic_offset. offset: %f, timestamp_diff: %f, good_timestamp_found: %s", height_offset_, static_cast<float>(std::abs(pos_z_timestamp - msg.header.stamp.toSec())), good_timestamp_found ? "true" : "false");
      }

      setStatus(Status::INIT);
      ROS_INFO_STREAM(indexed_nhp_.getNamespace()  << ": start kalman filter");
      /* fuser for 0: egomotion, 1: experiment */
      for (int mode = 0; mode < 2; mode++) {
        if (!getFuserActivate(mode)) continue;

        for (auto& fuser : estimator_->getFuser(mode)) {
          boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
          int id = kf->getId();

          if(id & (1 << State::Z_BASE)) {
            kf->setMeasureFlag();
            kf->setInitState(raw_plane_pos_z_, 0);
          }
        }
      }

      /* set the status for Z (altitude) */
      estimator_->setStateStatus(State::Z_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE, true);
      setStatus(Status::ACTIVE); //active
    } //end initialize

    /* kalman filter correction */
    plane_detection_state_.header.stamp.fromSec(msg.header.stamp.toSec());

    if (isCameraAngleValid() && findValidPlane(msg)) {
      raw_plane_vel_z_ = (raw_plane_pos_z_ - prev_raw_plane_pos_z_) / (curr_timestamp_ - prev_timestamp_);
      estimateProcess();

      /* publish state */
      plane_detection_state_.states[0].state[0].x = raw_plane_pos_z_;
      plane_detection_state_.states[0].state[0].y = raw_plane_vel_z_;
      state_pub_.publish(plane_detection_state_);
      updateHealthStamp(1);

      /* update */
      prev_timestamp_ = curr_timestamp_;
      prev_raw_plane_pos_z_ = raw_plane_pos_z_;
    }
  }

  bool PlaneDetection::findValidPlane(const jsk_recognition_msgs::ModelCoefficientsArray& msg)
  {
    tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::BASELINK, aerial_robot_estimation::EGOMOTION_ESTIMATE);
    double uav_z = estimator_->getPos(Frame::BASELINK, aerial_robot_estimation::EGOMOTION_ESTIMATE).z();
    bool valid_plane_found = false;
    double max_distance = -1e6;

    for (const auto& coeff : msg.coefficients) {
      tf::Vector3 norm_vec(coeff.values[0], coeff.values[1], coeff.values[2]);
      tf::Vector3 norm_vec_in_world_frame = uav_rot * (sensor_tf_.getBasis() * norm_vec) * -1;
      double distance = std::abs((uav_rot * (sensor_tf_ * (norm_vec * -coeff.values[3]))).z()) + height_offset_;

      // if this is the ground plane, the norm vector should be [0, 0, 1]
      double angle = norm_vec_in_world_frame.angle(tf::Vector3(0, 0, 1));
      if (std::abs(angle) < eps_angle_ &&
          std::abs(uav_z - distance) < distance_diff_thresh_ &&
          min_height_ < distance &&
          max_height_ > distance) {
        valid_plane_found = true;
        // find the farthest plane
        if (distance > max_distance) {
          max_distance = distance;
          raw_plane_pos_z_ = distance;
        }
      } else {
        ROS_DEBUG("angle: %f", std::abs(angle));
        ROS_DEBUG("distance: %f", distance);
        ROS_DEBUG("distance diff: %f", std::abs(uav_z - distance));
      }
    }

    return valid_plane_found;
  }

  bool PlaneDetection::isCameraAngleValid()
  {
    tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::BASELINK, aerial_robot_estimation::EGOMOTION_ESTIMATE);
    tf::Matrix3x3 sensor_tf_in_world_frame = uav_rot * sensor_tf_.getBasis();
    double camera_angle = sensor_tf_in_world_frame.getColumn(2).angle(tf::Vector3(0, 0, -1));
    return camera_angle < max_camera_angle_;
  }

  void PlaneDetection::estimateProcess()
  {
    if (getStatus() == Status::INVALID) {
      ROS_DEBUG("status invalid");
      return;
    }

    for (int mode = 0; mode < 2; mode++) {
      if (!getFuserActivate(mode)) continue;

      for (auto& fuser : estimator_->getFuser(mode)) {
        string plugin_name = fuser.first;
        boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
        int id = kf->getId();
        if(id & (1 << State::Z_BASE)) {
          if(plugin_name == "kalman_filter/kf_pos_vel_acc") {
            /* correction */
            Eigen::VectorXd measure_sigma(1);
            measure_sigma << plane_noise_sigma_;
            Eigen::VectorXd meas(1); meas <<  raw_plane_pos_z_;
            std::vector<double> params = {kf_plugin::POS};

            kf->correction(meas, measure_sigma,
                           time_sync_?(plane_detection_state_.header.stamp.toSec()):-1, params);
          }
        }
      }
    }
  }

  void PlaneDetection::rosParamInit()
  {
    getParam<double>("eps_angle", eps_angle_, 0.05);
    getParam<double>("distance_diff_thresh", distance_diff_thresh_, 0.4);
    getParam<double>("plane_noise_sigma", plane_noise_sigma_, 0.01);
    getParam<double>("min_height", min_height_, 0.5);
    getParam<double>("max_height", max_height_, 15.0);
    getParam<double>("max_camera_angle", max_camera_angle_, 0.4);
    getParam<bool>("dynamic_offset", dynamic_offset_, false);
    getParam<double>("height_offset", height_offset_, 0.0);
    getParam<int>("timer_freq", timer_freq_, 100);
    getParam<double>("state_save_time", state_save_time_, 1.2);

    if (dynamic_offset_) {
      height_offset_ = 0.0;
    }
  }
};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::PlaneDetection, sensor_plugin::SensorBase);
