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

#include <aerial_robot_base/sensor/gps.h>

namespace
{
  bool first_flag = true;

  /*
    Provides meters-per-degree latitude at a given latitude

    Args:
    lat: latitude
    Returns: meters-per-degree value
  */

  double mDegLat(double lat)
  {
    double lat_rad = lat * M_PI / 180.0;

    return 111132.09 - 566.05 * cos(2.0 * lat_rad) + 1.20 * cos(4.0 * lat_rad) - 0.002 * cos(6.0 * lat_rad);
  }

  /*
    Provides meters-per-degree longitude at a given latitude
    Args:
      lat: latitude in decimal degrees
    Returns: meters per degree longitude
  */

  double mDegLon(double lat)
  {
    double lat_rad = lat * M_PI/ 180.0;
    return 111415.13 * cos(lat_rad) - 94.55 * cos(3.0 * lat_rad) - 0.12 * cos(5.0 * lat_rad);
  }

};

namespace sensor_plugin
{
  void Gps::initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, string sensor_name)
  {
    SensorBase::initialize(nh, nhp, estimator, sensor_name);
    rosParamInit();

    /* ros publisher of aerial_robot_base::State */
    state_pub_ = nh_.advertise<aerial_robot_msgs::States>("data", 10);
    /* ros publisher of sensor_msgs::NavSatFix */
    gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/single_gps", 2);
    /* ros subscriber for gps */
    gps_sub_ = nh_.subscribe(gps_sub_name_, 5, &Gps::gpsCallback, this);
  }

  Gps::Gps():
    pos_(0, 0, 0),
    raw_pos_(0, 0, 0),
    prev_raw_pos_(0, 0, 0),
    vel_(0, 0, 0),
    raw_vel_(0, 0, 0)
  {
    gps_state_.states.resize(2);
    gps_state_.states[0].id = "x";
    gps_state_.states[0].state.resize(2);
    gps_state_.states[1].id = "y";
    gps_state_.states[1].state.resize(2);

    world_frame_.setIdentity();
  }

  void Gps::rosParamInit()
  {
    std::string ns = nhp_.getNamespace();

    nhp_.param("gps_sub_name", gps_sub_name_, string("/gps"));
    if(param_verbose_) cout << ns << ": rtk gps sub_name is:" << gps_sub_name_ << endl;

    nhp_.param("min_est_sat_num", min_est_sat_num_, 4);
    if(param_verbose_) cout << ns << ": min est sat num is " << min_est_sat_num_ << endl;

    nhp_.param("pos_noise_sigma", pos_noise_sigma_, 0.001);
    if(param_verbose_) cout << ns << ": pos noise sigma is " << pos_noise_sigma_ << endl;
    nhp_.param("vel_noise_sigma", vel_noise_sigma_, 0.1);
    if(param_verbose_) cout << ns << ": vel noise sigma is " << vel_noise_sigma_ << endl;

    nhp_.param("ned_flag", ned_flag_, true);
    if(param_verbose_) cout << ns << ": NED frame flag is " << ned_flag_ << endl;
    if(ned_flag_) world_frame_.setRPY(M_PI, 0, 0);
  }

  void Gps::gpsCallback(const spinal::Gps::ConstPtr & gps_msg)
  {

    if(!updateBaseLink2SensorTransform()) return;

    /* assignment lat/lon, velocity */
    curr_wgs84_point_ = geodesy::toMsg(gps_msg->location[0] / 1e7, gps_msg->location[1] / 1e7);
    tf::Vector3 raw_vel_temp(gps_msg->velocity[0], gps_msg->velocity[1], 0);
    raw_vel_ = world_frame_ * raw_vel_temp;

    /* fusion process */
    /* quit if the satellite number is too low */
    if(gps_msg->sat_num < min_est_sat_num_)
      {
        ROS_WARN_THROTTLE(1, "the satellite is not enough: %d", gps_msg->sat_num);
        if(!first_flag)
          {
            /* set the status */
            estimator_->setStateStatus(State::X_BASE, BasicEstimator::EGOMOTION_ESTIMATE, false);
            estimator_->setStateStatus(State::Y_BASE, BasicEstimator::EGOMOTION_ESTIMATE, false);
            first_flag = true;
          }
        return;
      }

    if(first_flag)
      {
        first_flag = false;

        if(!estimator_->getStateStatus(State::X_BASE, BasicEstimator::EGOMOTION_ESTIMATE) ||
           !estimator_->getStateStatus(State::Y_BASE, BasicEstimator::EGOMOTION_ESTIMATE))
          {
            ROS_WARN("GPS: start gps kalman filter");

            /* fuser for 0: egomotion, 1: experiment */
            for(int mode = 0; mode < 2; mode++)
              {
                if(!getFuserActivate(mode)) continue;

                for(auto& fuser : estimator_->getFuser(mode))
                  {
                    boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
                    int id = kf->getId();

                    string plugin_name = fuser.first;
                    if((id & (1 << State::X_BASE)) || (id & (1 << State::Y_BASE)))
                      {
                        if(plugin_name == "kalman_filter/kf_pos_vel_acc" ||
                           plugin_name == "kalman_filter/kf_pos_vel_acc_bias")
                          {
                            /* set velocity correction mode */
                            if(time_sync_) kf->setTimeSync(true);

                            kf->setInitState(raw_vel_[id >> (State::X_BASE + 1)], 1);
                            kf->setMeasureFlag();
                          }
                      }
                  }
              }
          }

        /* set home position */
        home_wgs84_point_ = curr_wgs84_point_;
        ROS_WARN("home lat/lon: [%f deg , %f deg]", home_wgs84_point_.latitude, home_wgs84_point_.longitude);

        /* set the status */
        estimator_->setStateStatus(State::X_BASE, BasicEstimator::EGOMOTION_ESTIMATE, true);
        estimator_->setStateStatus(State::Y_BASE, BasicEstimator::EGOMOTION_ESTIMATE, true);
      }
    else
      {
        /* get the position w.r.t. local frame (the origin is the initial takeoff place) */
        raw_pos_ = world_frame_ * Gps::wgs84ToNedLocalFrame(home_wgs84_point_, curr_wgs84_point_);
        //ROS_INFO("[%f, %f] -> [%f, %f]", curr_wgs84_point_.latitude, curr_wgs84_point_.longitude, raw_pos_.x(), raw_pos_.y());
        /* update the timestamp */
        gps_state_.header.stamp.fromSec(gps_msg->stamp.toSec() + ((time_sync_ && delay_ < 0)?delay_:0));

        if(!estimate_flag_) return;

        /* fuser for 0: egomotion, 1: experiment */
        for(int mode = 0; mode < 2; mode++)
          {
            if(!getFuserActivate(mode)) continue;

            for(auto& fuser : estimator_->getFuser(mode))
              {
                string plugin_name = fuser.first;
                boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;

                int id = kf->getId();
                if((id & (1 << State::X_BASE)) || (id & (1 << State::Y_BASE)))
                  {
                    if(plugin_name == "kalman_filter/kf_pos_vel_acc" ||
                       plugin_name == "kalman_filter/kf_pos_vel_acc_bias")
                      {
                        /* set noise sigma */
                        VectorXd measure_sigma(1);
                        measure_sigma << vel_noise_sigma_;
                        kf->setMeasureSigma(measure_sigma);

                        /* correction */
                        int index = id >> (State::X_BASE + 1);
                        VectorXd meas(1); meas <<  raw_vel_[index];
                        vector<double> params = {kf_plugin::VEL};

                        kf->correction(meas, gps_state_.header.stamp.toSec(), params);
                        VectorXd state = kf->getEstimateState();

                        estimator_->setState(index + 3, mode, 0, state(0));
                        estimator_->setState(index + 3, mode, 1, state(1));
                        gps_state_.states[index].state[0].x = raw_pos_[index];
                        gps_state_.states[index].state[0].y = raw_vel_[index];
                      }
                  }
              }
          }
        state_pub_.publish(gps_state_);
      }

    /* update */
    prev_raw_pos_ = raw_pos_;
    updateHealthStamp();
  }

  /*
    AlvinXY: Lat/Long to X/Y (NED)
    Converts Lat/Lon (WGS84) to Alvin XYs using a Mercator projection.
  */
  tf::Vector3 Gps::wgs84ToNedLocalFrame(geographic_msgs::GeoPoint base_point, geographic_msgs::GeoPoint target_point)
  {

    return tf::Vector3((target_point.latitude - base_point.latitude) * mDegLat(base_point.latitude), (target_point.longitude - base_point.longitude) * mDegLon(base_point.latitude), 0);
  }


  /*
    X/Y (NED) to Lat/Lon
    Converts Alvin XYs to Lat/Lon (WGS84) using a Mercator projection.
  */
  geographic_msgs::GeoPoint Gps::NedLocalFrameToWgs84(tf::Vector3 diff_pos, geographic_msgs::GeoPoint base_point)
  {
    geographic_msgs::GeoPoint target_point;

    target_point.longitude = diff_pos.y() / mDegLon(base_point.latitude) + base_point.longitude;
    target_point.latitude = diff_pos.x() / mDegLat(base_point.latitude) + base_point.latitude;
    return target_point;
  }

};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Gps, sensor_plugin::SensorBase);





