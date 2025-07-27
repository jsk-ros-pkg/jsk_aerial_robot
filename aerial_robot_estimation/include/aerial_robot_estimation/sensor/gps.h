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

#pragma once

#include <aerial_robot_estimation/sensor/base_plugin.h>
#include <geodesy/utm.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kalman_filter/kf_pos_vel_acc_plugin.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <spinal/Gps.h>
#include <spinal/GpsFull.h>
#include <time.h>

using namespace Eigen;
using namespace std;

/* TODO:
   1. gps redundant proccess to improce the accuracy of position estimation
   2. better way point control which is from sensor fusion but not only gps
   3. interaction between single gps and rtk gps
*/

namespace sensor_plugin
{

  class Gps :public sensor_plugin::SensorBase
    {
    public:
      void initialize(ros::NodeHandle nh,
                      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                      string sensor_name, int index) override;

      ~Gps() {}
      Gps();

      static tf::Vector3 wgs84ToNedLocalFrame(geographic_msgs::GeoPoint base_pos, geographic_msgs::GeoPoint target_pos);
      static geographic_msgs::GeoPoint NedLocalFrameToWgs84(tf::Vector3 diff_pos, geographic_msgs::GeoPoint base_pos);

      const bool isRtk() const {return is_rtk_gps_;}
      const geographic_msgs::GeoPoint getBasePoint() const { return base_wgs84_point_;}
      const geographic_msgs::GeoPoint getCurrentPoint() const { return curr_wgs84_point_;}

    private:
      /* ros */
      ros::Publisher gps_pub_;
      ros::Subscriber gps_sub_, gps_full_sub_; /* from spinal */
      ros::Subscriber gps_ros_sub_; /* from ros */
      ros::Subscriber rtk_gps_sub_; /* from rtk */

      /* ros param */
      double pos_noise_sigma_, vel_noise_sigma_;
      int min_est_sat_num_;

      bool ned_flag_;
      bool only_use_vel_;
      bool only_use_pos_;
      bool is_rtk_gps_; /* self is RTK GPS */
      bool rtk_offset_; /* set the takeoff point as zero point for RTK mode */

      aerial_robot_msgs::States gps_state_;

      geographic_msgs::GeoPoint base_wgs84_point_, curr_wgs84_point_;

      tf::Vector3 pos_, raw_pos_, prev_raw_pos_;
      tf::Vector3 vel_, raw_vel_;
      tf::Vector3 pos_offset_;

      void gpsCallback(const spinal::Gps::ConstPtr & gps_msg);
      void gpsFullCallback(const spinal::GpsFull::ConstPtr & gps_full_msg);
      void gpsRosCallback(const sensor_msgs::NavSatFix::ConstPtr & gps_msg);
      void rtkGpsCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & gps_msg);
      void estimateProcess();
      void rosParamInit();
      void activate();

      /* utc time */
      /* https://github.com/KumarRobotics/ublox/blob/master/ublox_gps/include/ublox_gps/mkgmtime.h */
      time_t mkgmtime(struct tm * const tmp);
      int tmcomp(const struct tm * const  atmp, const struct tm * const btmp);
    };
};





