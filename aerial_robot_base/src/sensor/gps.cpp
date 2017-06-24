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

/* gps convert */
#include <geodesy/utm.h>

/* ros msg */
#include <aerial_robot_msgs/Gps.h>
#include <aerial_robot_base/States.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <aerial_robot_base/FlightNav.h>
#include <mavros_msgs/WaypointList.h> /* temporarily */

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
      void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, string sensor_name)
      {
        SensorBase::initialize(nh, nhp, estimator, sensor_name);
        rosParamInit();

        /* ros publisher of aerial_robot_base::State */
        state_pub_ = nh_.advertise<aerial_robot_base::States>("data", 10);
        /* ros publisher of sensor_msgs::NavSatFix */
        gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/single_gps", 2);
        /* ros subscriber for single gps */
        gps_sub_ = nh_.subscribe(gps_sub_name_, 5, &Gps::gpsCallback, this);
        /* ros subscriber for rtk gps */
        rtk_gps_sub_ = nh_.subscribe(rtk_gps_sub_name_, 5, &Gps::rtkGpsCallback, this);
        /* ros publisher / subscriber for waypoint, the message type is not good rightnow */
        wp_sub_ = nh_.subscribe(wp_sub_name_, 5, &Gps::wpCallback, this);
        nav_pub_ = nh_.advertise<aerial_robot_base::FlightNav>(nav_pub_name_, 1);

        /* waypoint control timer init */
        if(wp_ctrl_rate_ > 0)
          wp_timer_ = nhp_.createTimer(ros::Duration(1.0 / wp_ctrl_rate_), &Gps::wpCtrlFunc, this);

      }
      ~Gps() {}

      Gps():
        pos_(0, 0, 0),
        raw_pos_(0, 0, 0),
        prev_raw_pos_(0, 0, 0),
        raw_rtk_pos_(0, 0, 0),
        prev_raw_rtk_pos_(0, 0, 0),
        vel_(0, 0, 0),
        raw_vel_(0, 0, 0),
        rtk_gps_activate_(false),
        wp_activate_(false)
      {
        gps_state_.states.resize(4);
        gps_state_.states[0].id = "x";
        gps_state_.states[0].state.resize(3);
        gps_state_.states[1].id = "y";
        gps_state_.states[1].state.resize(3);
        gps_state_.states[2].id = "rtk_x";
        gps_state_.states[2].state.resize(3);
        gps_state_.states[3].id = "rtk_y";
        gps_state_.states[3].state.resize(3);

        /* set health chan num */
        setHealthChanNum(2);
      }

    private:
      /* ros */
      ros::Publisher gps_pub_;
      ros::Publisher state_pub_;
      ros::Subscriber gps_sub_; /* single gps to get NavSatFix */
      ros::Subscriber rtk_gps_sub_; /* rtk gps */
      /* for gps way point control */
      ros::Subscriber wp_sub_; /* waypoint control */
      ros::Publisher nav_pub_; /* waypoint control */
      ros::Timer  wp_timer_;

      /* ros param */
      string gps_sub_name_;
      string rtk_gps_sub_name_;


      /* for gps way point control */
      string wp_sub_name_;
      string nav_pub_name_;
      double wp_ctrl_rate_;
      double wp_dist_thre_;
      double wp_alt_thre_;
      int min_wp_sat_num_;
      double nav_vel_limit_;
      double nav_vel_gain_;

      double pos_noise_sigma_, vel_noise_sigma_;
      int min_est_sat_num_;

      aerial_robot_base::States gps_state_;

      geodesy::UTMPoint home_utm_pos_, utm_pos_;
      tf::Vector3 pos_, raw_pos_, prev_raw_pos_;
      tf::Vector3 raw_rtk_pos_, prev_raw_rtk_pos_;
      tf::Vector3 vel_, raw_vel_;
      tf::Vector3 rtk_vel_, raw_rtk_vel_;
      bool rtk_gps_activate_;

      /* gps waypoint control */
      bool wp_activate_;
      tf::Vector3 target_pos_;


      void rosParamInit()
      {
        std::string ns = nhp_.getNamespace();

        nhp_.param("rtk_gps_sub_name", rtk_gps_sub_name_, string("/rtk_gps"));
        if(param_verbose_) cout << "rtk gps sub_name is:" << rtk_gps_sub_name_ << endl;
        nhp_.param("gps_sub_name", gps_sub_name_, string("/gps"));
        if(param_verbose_) cout << "rtk gps sub_name is:" << gps_sub_name_ << endl;
        nhp_.param("wp_sub_name", wp_sub_name_, string("/wp"));
        if(param_verbose_) cout << "rtk wp sub_name is:" << wp_sub_name_ << endl;
        nhp_.param("nav_pub_name", nav_pub_name_, string("/nav"));
        if(param_verbose_) cout << "rtk wp pub_name is:" << nav_pub_name_ << endl;

        nhp_.param("wp_ctrl_rate", wp_ctrl_rate_, 0.0);
        if(param_verbose_) cout << "wp ctrl rate is " << wp_ctrl_rate_ << endl;
        nhp_.param("wp_dist_thre", wp_dist_thre_, 500.0);
        if(param_verbose_) cout << "wp dist thre is " << wp_dist_thre_ << endl;
        nhp_.param("wp_alt_thre", wp_alt_thre_, 50.0);
        if(param_verbose_) cout << "wp alt thre is " << wp_alt_thre_ << endl;
        nhp_.param("min_est_sat_num", min_est_sat_num_, 4);
        if(param_verbose_) cout << "min est sat num is " << min_est_sat_num_ << endl;
        nhp_.param("min_wp_sat_num", min_wp_sat_num_, 7);
        if(param_verbose_) cout << "min wp sat num is " << min_wp_sat_num_ << endl;
        nhp_.param("nav_vel_limit", nav_vel_limit_, 2.0);
        if(param_verbose_) cout << "nav vel limit is " << nav_vel_limit_ << endl;
        nhp_.param("vel_nav_gain", nav_vel_gain_, 1.0);
        if(param_verbose_) cout << "vel nav gain is " << nav_vel_gain_ << endl;

        nhp_.param("pos_noise_sigma", pos_noise_sigma_, 0.001);
        if(param_verbose_) cout << "pos noise sigma is " << pos_noise_sigma_ << endl;
        nhp_.param("vel_noise_sigma", vel_noise_sigma_, 0.1);
        if(param_verbose_) cout << "vel noise sigma is " << vel_noise_sigma_ << endl;
      }

      void gpsCallback(const aerial_robot_msgs::Gps::ConstPtr & gps_msg)
      {
        static bool first_flag = true;
        double current_secs = gps_msg->stamp.toSec();


        /* frame conversion */
        /* UTM */
        /* raw data about alt/lon from gps is * 10e7 */
        geodesy::fromMsg(geodesy::toMsg(gps_msg->location[0] / 1e7, gps_msg->location[1] / 1e7), utm_pos_);

        raw_pos_ = baselink_transform_.getBasis() * tf::Vector3(utm_pos_.northing - home_utm_pos_.northing, utm_pos_.easting - home_utm_pos_.easting, 0);
        tf::Vector3 raw_vel_temp(gps_msg->velocity[0], gps_msg->velocity[1], 0);
        raw_vel_ = baselink_transform_.getBasis() * raw_vel_temp;

        gps_state_.header.stamp = gps_msg->stamp;

        /* fusion process */
        /* quit if the satellite number is too low */
        if(gps_msg->sat_num < min_est_sat_num_)
          {
            ROS_WARN_THROTTLE(1, "the satellite is not enough: %d", gps_msg->sat_num);
            if(!first_flag)
              {
                /* set the status */
                estimator_->setStateStatus(BasicEstimator::X_W, BasicEstimator::EGOMOTION_ESTIMATE, false);
                estimator_->setStateStatus(BasicEstimator::Y_W, BasicEstimator::EGOMOTION_ESTIMATE, false);
                first_flag = true;
              }
            return;
          }

        if(first_flag)
          {
            first_flag = false;
            ROS_WARN("GPS: start/restart");

            if(!estimator_->getStateStatus(BasicEstimator::X_W, BasicEstimator::EGOMOTION_ESTIMATE) ||
               !estimator_->getStateStatus(BasicEstimator::Y_W, BasicEstimator::EGOMOTION_ESTIMATE))
              {
                ROS_WARN("GPS: start/restart gps kalman filter");

                /* fuser for 0: egomotion, 1: experiment */
                for(int mode = 0; mode < 2; mode++)
                  {
                    if(!getFuserActivate(mode)) continue;

                    for(auto& fuser : estimator_->getFuser(mode))
                      {
                        boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
                        int id = kf->getId();

                        //if((id & (1 << BasicEstimator::X_W)) || id & (1 << BasicEstimator::Y_W)))
                        if(id <= 2) // equal to the previous condition
                          {
                            /* set velocity correction mode */
                            if(time_sync_) kf->setTimeSync(true);

                            kf->setInitState(raw_vel_[id >> 1], 1);
                            kf->setMeasureFlag();
                          }
                      }
                  }
              }

            /* set home position */
            home_utm_pos_ = utm_pos_;
            ROS_WARN("home position from gps: UTM zone: %d, [%f, %f]", home_utm_pos_.zone, home_utm_pos_.northing, home_utm_pos_.easting);

            /* set the status */
            estimator_->setStateStatus(BasicEstimator::X_W, BasicEstimator::EGOMOTION_ESTIMATE, true);
            estimator_->setStateStatus(BasicEstimator::Y_W, BasicEstimator::EGOMOTION_ESTIMATE, true);

          }
        else
          {
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

                    if(id <= 2)
                      {
                        if(plugin_name == "kalman_filter/kf_pos_vel_acc" ||
                           plugin_name == "kalman_filter/kf_pos_vel_acc_bias")
                          {
                            /* set noise sigma */
                            VectorXd measure_sigma(1);
                            measure_sigma << vel_noise_sigma_;
                            kf->setMeasureSigma(measure_sigma);

                            /* correction */
                            VectorXd meas(1); meas <<  raw_vel_[id >> 1];
                            vector<double> params = {kf_plugin::VEL};
                            /* time sync and delay process: get from kf timestamp */
                            if(time_sync_ && delay_ < 0)
                              {
                                current_secs = kf->getTimestamp() + delay_;
                                gps_state_.header.stamp.fromSec(current_secs);
                              }
                            kf->correction(meas, params, current_secs);
                          }

                        VectorXd state = kf->getEstimateState();
                        /* do not use the position data */
                        estimator_->setState(id >> 1, mode, 1, state(1));

                        gps_state_.states[id >> 1].state[mode + 1].x = state(0);
                        gps_state_.states[id >> 1].state[mode + 1].y = state(1);

                        gps_state_.states[id >> 1].state[0].x = raw_pos_[id >> 1];
                        gps_state_.states[id >> 1].state[0].y = raw_vel_[id >> 1];
                      }
                  }
              }
            state_pub_.publish(gps_state_);
          }

        /* update */
        prev_raw_pos_ = raw_pos_;
        updateHealthStamp(current_secs, 0); //channel: 1

        /* for wp control, not good */
        if(gps_msg->sat_num > min_wp_sat_num_)
          {
            pos_ = raw_pos_;
            //ROS_INFO("gps pos: [%f, %f]", pos_[0], pos_[1]);
          }
      }

      void rtkGpsCallback(const nav_msgs::Odometry::ConstPtr & rtk_gps_msg)
      {
        static bool rtk_first_flag = true;
        double current_secs = rtk_gps_msg->header.stamp.toSec();

        /* frame conversion */
        tf::Vector3 raw_rtk_pos_temp, raw_rtk_vel_temp;
        tf::pointMsgToTF(rtk_gps_msg->pose.pose.position, raw_rtk_pos_temp);
        tf::vector3MsgToTF(rtk_gps_msg->twist.twist.linear, raw_rtk_vel_temp);
        raw_rtk_pos_ = baselink_transform_.getBasis() * raw_rtk_pos_temp;
        raw_rtk_vel_ = baselink_transform_.getBasis() * raw_rtk_vel_temp;

        gps_state_.header.stamp = rtk_gps_msg->header.stamp;

        if(rtk_first_flag)
          {
            rtk_first_flag = false;

              /* fuser for 0: egomotion, 1: experiment */
              for(int mode = 0; mode < 2; mode++)
                {
                  if(!getFuserActivate(mode)) continue;

                  for(auto& fuser : estimator_->getFuser(mode))
                    {
                      boost::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
                      int id = kf->getId();

                      //if((id & (1 << BasicEstimator::X_W)) || id & (1 << BasicEstimator::Y_W)))
                      if(id <= 2) // equal to the previous condition
                        {
                          /* position correction mode */
                          if(time_sync_) kf->setTimeSync(true);

                          kf->setInitState(raw_rtk_pos_[id], 0);
                          kf->setMeasureFlag();
                        }
                    }
              }
          }
        else
          {
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

                    if(id <= 2)
                      {

                      if(plugin_name == "kalman_filter/kf_pos_vel_acc" ||
                         plugin_name == "kalman_filter/kf_pos_vel_acc_bias")
                        {
                          /* set noise sigma */
                          VectorXd measure_sigma(1);
                          measure_sigma << pos_noise_sigma_;
                          kf->setMeasureSigma(measure_sigma);

                          /* correction */
                          VectorXd meas(1); meas <<  raw_rtk_pos_[id >> 1];
                          vector<double> params = {kf_plugin::POS};
                          /* time sync and delay process: get from kf time stamp */
                          if(time_sync_ && delay_ < 0)
                            {
                              current_secs = kf->getTimestamp() + delay_;
                              gps_state_.header.stamp.fromSec(current_secs);
                            }
                          kf->correction(meas, params, current_secs);
                        }

                        VectorXd state = kf->getEstimateState();

                        estimator_->setState(id >> 1, mode, 0, state(0));
                        estimator_->setState(id >> 1, mode, 1, state(1));

                        gps_state_.states[id >> 1 + 2].state[mode + 1].x = state(0);
                        gps_state_.states[id >> 1 + 2].state[mode + 1].y = state(1);

                        gps_state_.states[id >> 1 + 2].state[0].x = raw_rtk_pos_[id >> 1];
                        gps_state_.states[id >> 1 + 2].state[0].y = raw_rtk_vel_[id >> 1];
                        /* for wp control, not good */
                        pos_[id >> 1] = state(0, 0);
                        //ROS_INFO("rtk gps pos%d: %d", id >> 1, pos_[id >> 1]);
                      }

                  }
              }
            state_pub_.publish(gps_state_);
          }

        /* update */
        prev_raw_rtk_pos_ = raw_rtk_pos_;
        updateHealthStamp(current_secs, 1); //channel: 1
      }

      void wpCallback(const mavros_msgs::WaypointList::ConstPtr & msg)
      {
        static int frame = -1;
        if(msg->waypoints.size() != 1)
          {
            ROS_WARN("GPS: too many waypoint in the list");
            return;
          }

        /* activate */
        if(!wp_activate_) wp_activate_ = true;

        /* if the command is old, quit */
        if(msg->waypoints[0].frame == frame) return;

        geodesy::UTMPoint target_utm_pos;
        geodesy::fromMsg(geodesy::toMsg(msg->waypoints[0].x_lat, msg->waypoints[0].y_long, msg->waypoints[0].z_alt), target_utm_pos);

        /* TODO: the process for different zone */
        tf::Vector3 target_pos = baselink_transform_.getBasis() * tf::Vector3(target_utm_pos.northing - home_utm_pos_.northing, target_utm_pos.easting - home_utm_pos_.easting, 0);

        /* threshold */
        if(target_pos.length() < wp_dist_thre_ && target_utm_pos.altitude < wp_alt_thre_)
          {
            target_pos_ = target_pos;
            target_pos_.setZ(target_utm_pos.altitude);
            ROS_WARN("GPS: get new waypoint, [%f, %f, %f]", target_pos_.x(), target_pos_.y(), target_pos_.z());
          }

        frame = msg->waypoints[0].frame;
      }

      /* simple wp function */
      void wpCtrlFunc(const ros::TimerEvent & e)
      {
        if(!wp_activate_) return;

        /* nav part */
        tf::Vector3 delta = target_pos_ - raw_pos_;
        tf::Vector3 nav_vel = delta * nav_vel_gain_;

        double speed = nav_vel.length();
        if(speed  > nav_vel_limit_) nav_vel *= (nav_vel_limit_ / speed);

        aerial_robot_base::FlightNav nav_msg;
        nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::VEL_MODE;
        nav_msg.target_vel_x = nav_vel.x();
        nav_msg.target_vel_y = nav_vel.y();
        nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
        nav_msg.target_pos_z = target_pos_.z();
        nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
        nav_pub_.publish(nav_msg);
      }

    };
};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Gps, sensor_plugin::SensorBase);





