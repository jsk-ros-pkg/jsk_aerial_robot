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

#include <aerial_robot_estimation/sensor/gps.h>
#include <aerial_robot_estimation/sensor/vo.h>

namespace
{
  /* TODO: https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/blob/kinetic-devel/hector_gazebo_plugins/src/gazebo_ros_gps.cpp */
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
  Gps::Gps():
    sensor_plugin::SensorBase(),
    pos_(0, 0, 0),
    raw_pos_(0, 0, 0),
    prev_raw_pos_(0, 0, 0),
    vel_(0, 0, 0),
    raw_vel_(0, 0, 0),
    pos_offset_(0, 0, 0)
  {
    gps_state_.states.resize(2);
    gps_state_.states[0].id = "x";
    gps_state_.states[0].state.resize(1);
    gps_state_.states[1].id = "y";
    gps_state_.states[1].state.resize(1);
  }

  void Gps::initialize(ros::NodeHandle nh,
                       boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                       boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                       string sensor_name, int index)
  {
    SensorBase::initialize(nh, robot_model, estimator, sensor_name, index);
    rosParamInit();

    /* ros subscriber for gps */
    std::string topic_name;
    getParam<std::string>("gps_sub_name", topic_name, string("gps"));
    gps_sub_ = nh_.subscribe(topic_name, 5, &Gps::gpsCallback, this);

    getParam<std::string>("gps_full_sub_name", topic_name, string("gps_full"));
    // gps_full_sub_ = nh_.subscribe(topic_name, 5, &Gps::gpsFullCallback, this); // reserve

    getParam<std::string>("gps_ros_sub_name", topic_name, string("ros_fix"));
    gps_ros_sub_ = nh_.subscribe(topic_name, 5, &Gps::gpsRosCallback, this);

    if(is_rtk_gps_)
      {
        getParam<std::string>("rtk_pos_sub_name", topic_name, string("rtk_pos"));
        rtk_gps_sub_ = nh_.subscribe(topic_name, 5, &Gps::rtkGpsCallback, this);

        aerial_robot_msgs::State z_state;
        z_state.id = "z";
        z_state.state.resize(1);
        gps_state_.states.push_back(z_state);
      }
    else
      {
        /* ros publisher of sensor_msgs::NavSatFix */
        gps_pub_ = indexed_nhp_.advertise<sensor_msgs::NavSatFix>("ros_converted", 2);
      }
  }

  void Gps::rosParamInit()
  {
    std::string ns = nhp_.getNamespace();

    getParam<bool>("ned_flag", ned_flag_, true);
    getParam<bool>("only_use_vel", only_use_vel_, true);
    getParam<int>("min_est_sat_num", min_est_sat_num_, 4);
    getParam<double>("pos_noise_sigma", pos_noise_sigma_, 1.0);
    getParam<double>("vel_noise_sigma", vel_noise_sigma_, 0.1);
    getParam<bool>("rtk", is_rtk_gps_, false);
    getParam<bool>("rtk_offset", rtk_offset_, false);

    if(is_rtk_gps_) only_use_pos_ = true;
    else only_use_pos_ = false;
  }

  void Gps::gpsCallback(const spinal::Gps::ConstPtr & gps_msg)
  {

    if(!updateBaseLink2SensorTransform()) return;

    /* check other gps modules */
    bool has_rtk_gps = false;
    for(const auto& handler: estimator_->getGpsHandlers())
      {
        if(handler.get() == this) continue;

        if(boost::dynamic_pointer_cast<sensor_plugin::Gps>(handler)->isRtk())
          {
            has_rtk_gps = true;
            only_use_vel_= true; // if has RTK GPS, we do not use the position information of normal GPS for fusion
            ROS_DEBUG("has another RTK GPS");
            break;
          }
      }

    /* temporal update */
    double curr_timestamp = gps_msg->stamp.toSec() + delay_;

    /* assignment lat/lon, velocity */
    curr_wgs84_point_ = geodesy::toMsg(gps_msg->location[0], gps_msg->location[1]);
    if(!has_rtk_gps || is_rtk_gps_) estimator_->setCurrGpsPoint(curr_wgs84_point_);

    raw_vel_ = tf::Vector3(gps_msg->velocity[0], -gps_msg->velocity[1], 0); // NED frame -> XYZ frame

    /* to get the correction rotation and omega of baselink with the consideration of time delay */
    bool imu_initialized = false;
    for(const auto& handler: estimator_->getImuHandlers())
      {
        if(handler->getStatus() == Status::ACTIVE)
          {
            imu_initialized = true;
            break;
          }
      }
    if(!imu_initialized)
      {
        ROS_WARN_THROTTLE(1, "gps: the imu is not initialized, wait");
        return;
      }

    /* fusion process */
    /* quit if the satellite number is too low */
    if(gps_msg->sat_num >= min_est_sat_num_)
      {
        if(getStatus() == Status::INVALID) setStatus(prev_status_);
      }
    if(gps_msg->sat_num < min_est_sat_num_)
      {
        if(getStatus() == Status::ACTIVE) ROS_WARN_THROTTLE(1, "the satellite is not enough: %d", gps_msg->sat_num);
        setStatus(Status::INVALID);
      }

    if(getStatus() == Status::INACTIVE)
      {
        setStatus(Status::INIT);

        if(!is_rtk_gps_) activate();

        /* set base position */
        base_wgs84_point_ = curr_wgs84_point_;
        ROS_WARN("base lat/lon: [%f, %f] deg for %s GPS", base_wgs84_point_.latitude, base_wgs84_point_.longitude, is_rtk_gps_?std::string("RTK").c_str():std::string("normal").c_str());
        return;
      }

    if(is_rtk_gps_) return; // do not do esimation

    /* get the position and velocity  w.r.t. the local frame (the origin is the initial takeoff place) */
    tf::Matrix3x3 r; r.setIdentity();
    tf::Vector3 omega(0,0,0);
    int mode = aerial_robot_estimation::EGOMOTION_ESTIMATE;
    if(!estimator_->findRotOmega(curr_timestamp, mode, r, omega) && estimator_->getFlyingFlag())
      ROS_WARN_STREAM("gps: the omega is not updated from findRotOmega");

    raw_vel_ += r * (- omega.cross(sensor_tf_.getOrigin())); //offset from gps to baselink

    tf::Matrix3x3 convert_frame; convert_frame.setRPY(M_PI, 0, 0); // NED -> XYZ
    raw_pos_ = convert_frame * Gps::wgs84ToNedLocalFrame(base_wgs84_point_, curr_wgs84_point_) - r * sensor_tf_.getOrigin();

    /* update timestamp for estimation */
    curr_timestamp_ = curr_timestamp;

    estimateProcess();

    /* update the timestamp */
    gps_state_.header.stamp.fromSec(curr_timestamp);
    gps_state_.states[0].state[0].x = raw_pos_[0];
    gps_state_.states[0].state[0].y = raw_vel_[0];
    gps_state_.states[1].state[0].x = raw_pos_[1];
    gps_state_.states[1].state[0].y = raw_vel_[1];
    state_pub_.publish(gps_state_);

    /* update */
    prev_raw_pos_ = raw_pos_;
    updateHealthStamp();
  }

  void Gps::gpsFullCallback(const spinal::GpsFull::ConstPtr & gps_full_msg)
  {
    /* time */
    struct tm time = {0};
    time.tm_year = gps_full_msg->year - 1900;
    time.tm_mon = gps_full_msg->month - 1;
    time.tm_mday = gps_full_msg->day;
    time.tm_hour = gps_full_msg->hour;
    time.tm_min = gps_full_msg->min;
    time.tm_sec = gps_full_msg->sec;
    auto time_sec = mkgmtime(&time);

    ros::Time fix_ros_time;
    if (gps_full_msg->nano < 0)
      {
        fix_ros_time.sec = time_sec - 1;
        fix_ros_time.nsec = (uint32_t)(gps_full_msg->nano + 1e9);
      }
    else
      {
        fix_ros_time.sec = time_sec;
        fix_ros_time.nsec = (uint32_t)(gps_full_msg->nano);
      }

    ROS_DEBUG("gps utc time %f; spinal time: %f", fix_ros_time.toSec(), gps_full_msg->stamp.toSec() + delay_);

    spinal::Gps::Ptr gps_msg(new spinal::Gps());
    gps_msg->stamp = gps_full_msg->stamp;
    gps_msg->location[0] = gps_full_msg->location[0];
    gps_msg->location[1] = gps_full_msg->location[1];
    gps_msg->velocity[0] = gps_full_msg->velocity[0];
    gps_msg->velocity[1] = gps_full_msg->velocity[1];
    gps_msg->sat_num = gps_full_msg->sat_num;

    gpsCallback(boost::const_pointer_cast<const spinal::Gps>(gps_msg));
  }

  void Gps::gpsRosCallback(const sensor_msgs::NavSatFix::ConstPtr & gps_msg)
  {
    /* TODO: add velocity */
    spinal::Gps::Ptr spinal_gps_msg(new spinal::Gps());
    spinal_gps_msg->stamp = gps_msg->header.stamp;
    spinal_gps_msg->location[0] = gps_msg->latitude;
    spinal_gps_msg->location[1] = gps_msg->longitude;

    if(gps_msg->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX)
      spinal_gps_msg->sat_num = min_est_sat_num_; // temporarily
    gpsCallback(boost::const_pointer_cast<const spinal::Gps>(spinal_gps_msg));
  }


  void Gps::rtkGpsCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & rtk_gps_msg)
  {
    auto cov = rtk_gps_msg->pose.covariance;
    if(cov[0] == 0 || cov[7] == 0 || cov[14] == 0)
      {
        // TODO: implement dynamic recovery if RTK GPS is disconnected in air
        ROS_WARN_THROTTLE(1.0, "RTK GPS is not convergent, not recommend to fly. If this is the first bringup, please wait for few minute for covergence");
        return;
      }


    /* temporal update */
    curr_timestamp_ = rtk_gps_msg->header.stamp.toSec() + delay_;

    /* pos accuracy */
    pos_noise_sigma_ = cov[0];

    /* get the position */
    tf::Matrix3x3 r; r.setIdentity();
    tf::Vector3 omega(0,0,0);
    int mode = aerial_robot_estimation::EGOMOTION_ESTIMATE;
    if(!estimator_->findRotOmega(curr_timestamp_, mode, r, omega) && estimator_->getFlyingFlag())
      ROS_WARN_STREAM("gps: the omega is not updated from findRotOmega");

    tf::Vector3 raw_pos;
    tf::pointMsgToTF(rtk_gps_msg->pose.pose.position, raw_pos);

    raw_pos_ = raw_pos  - r * sensor_tf_.getOrigin();

    if(getStatus() == Status::INIT) pos_offset_ = raw_pos_;

    if(rtk_offset_) raw_pos_ -= pos_offset_;
    // TODO: add offset for z axis if we use altitude from RTK GPS

    if(getStatus() == Status::INIT)
      {
        activate();
        setStatus(Status::ACTIVE);
      }

    estimateProcess();

    /* update the timestamp */
    gps_state_.header.stamp.fromSec(curr_timestamp_);
    gps_state_.states[0].state[0].x = raw_pos_[0];
    gps_state_.states[1].state[0].x = raw_pos_[1];
    gps_state_.states[2].state[0].x = raw_pos_[2];
    state_pub_.publish(gps_state_);

    /* update */
    updateHealthStamp();
  }

  void Gps::estimateProcess()
  {
    if(getStatus() == Status::INVALID) return;

    /* collaboration wit VO */
    if(estimator_->getVoHandlers().size() > 0 && !only_use_vel_)
      {
        for(const auto& handler: estimator_->getVoHandlers())
          {
            if(handler->getStatus() == Status::ACTIVE &&
               boost::dynamic_pointer_cast<sensor_plugin::VisualOdometry>(handler)->odomPosMode())
              {
                ROS_WARN("GPS, vo pose odom mode, so only use vel");
                only_use_vel_ = true;
                break;
              }
          }
      }

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
                if(plugin_name == "kalman_filter/kf_pos_vel_acc")
                  {
                    /* correction */
                    VectorXd measure_sigma(1);
                    measure_sigma << vel_noise_sigma_;

                    int index = id >> (State::X_BASE + 1);

                    if(only_use_pos_)
                      {
                        VectorXd meas(1); meas <<  raw_pos_[index];
                        vector<double> params = {kf_plugin::POS};
                        VectorXd measure_sigma(1);
                        measure_sigma << pos_noise_sigma_;
                        kf->correction(meas, measure_sigma,
                                       time_sync_?(curr_timestamp_):-1, params);
                      }
                    else if(only_use_vel_)
                      {
                        VectorXd meas(1); meas <<  raw_vel_[index];
                        vector<double> params = {kf_plugin::VEL};
                        VectorXd measure_sigma(1);
                        measure_sigma << vel_noise_sigma_;

                        kf->correction(meas, measure_sigma,
                                       time_sync_?(curr_timestamp_):-1, params);
                      }
                    else
                      {
                        VectorXd measure_sigma(2);
                        measure_sigma << pos_noise_sigma_, vel_noise_sigma_;
                        VectorXd meas(2); meas <<  raw_pos_[index], raw_vel_[index];
                        vector<double> params = {kf_plugin::POS_VEL};

                        kf->correction(meas, measure_sigma,
                                       time_sync_?(curr_timestamp_):-1, params);
                      }
                  }
              }
          }
      }
  }

  void Gps::activate()
  {
    if(!estimator_->getStateStatus(State::X_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE) ||
       !estimator_->getStateStatus(State::Y_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE))
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
                    if(plugin_name == "kalman_filter/kf_pos_vel_acc")
                      {
                        kf->setInitState(raw_pos_[id >> (State::X_BASE + 1)], 0);
                        kf->setMeasureFlag();
                      }
                  }
              }
          }
      }

    /* set the status */
    estimator_->setStateStatus(State::X_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE, true);
    estimator_->setStateStatus(State::Y_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE, true);
    setStatus(Status::ACTIVE);
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


  time_t Gps::mkgmtime(struct tm * const tmp)
  {
    int            dir;
    int            bits;
    int            saved_seconds;
    time_t              t;
    struct tm           yourtm, *mytm;

    yourtm = *tmp;
    saved_seconds = yourtm.tm_sec;
    yourtm.tm_sec = 0;
    /*
    ** Calculate the number of magnitude bits in a time_t
    ** (this works regardless of whether time_t is
    ** signed or unsigned, though lint complains if unsigned).
    */
    for (bits = 0, t = 1; t > 0; ++bits, t <<= 1)
      ;
    /*
    ** If time_t is signed, then 0 is the median value,
    ** if time_t is unsigned, then 1 << bits is median.
    */
    t = (t < 0) ? 0 : ((time_t) 1 << bits);

    /* Some gmtime() implementations are broken and will return
     * NULL for time_ts larger than 40 bits even on 64-bit platforms
     * so we'll just cap it at 40 bits */
    if(bits > 40) bits = 40;

    for ( ; ; ) {
      mytm = gmtime(&t);

      if(!mytm) return -1;

      dir = tmcomp(mytm, &yourtm);
      if (dir != 0) {
        if (bits-- < 0)
          return -1;
        if (bits < 0)
          --t;
        else if (dir > 0)
          t -= (time_t) 1 << bits;
        else    t += (time_t) 1 << bits;
        continue;
      }
      break;
    }
    t += saved_seconds;
    return t;
  }

  int Gps::tmcomp(const struct tm * const  atmp, const struct tm * const btmp)
  {
    int    result;

    if ((result = (atmp->tm_year - btmp->tm_year)) == 0 &&
        (result = (atmp->tm_mon - btmp->tm_mon)) == 0 &&
        (result = (atmp->tm_mday - btmp->tm_mday)) == 0 &&
        (result = (atmp->tm_hour - btmp->tm_hour)) == 0 &&
        (result = (atmp->tm_min - btmp->tm_min)) == 0)
      result = atmp->tm_sec - btmp->tm_sec;
    return result;
  }
};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Gps, sensor_plugin::SensorBase);





