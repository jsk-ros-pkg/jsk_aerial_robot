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

/* basic class */
#include <aerial_robot_base/control/flight_control.h>

/* ros msg */
#include <aerial_robot_msgs/FourAxisGain.h>
#include <aerial_robot_base/FlatnessPid.h>

//* for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <aerial_robot_msgs/DynamicReconfigureLevels.h>
#include <aerial_robot_base/XYPidControlConfig.h>

using boost::algorithm::clamp;
using namespace std;

namespace control_plugin
{
  class FlatnessPid : public control_plugin::ControlBase
  {
  public:
    FlatnessPid();
    ~FlatnessPid(){};

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    BasicEstimator* estimator, Navigator* navigator,
                    double ctrl_loop_rate);

    void activate()
    {
      static ros::Time activate_timestamp = ros::Time::now();
      if(ros::Time::now().toSec() - activate_timestamp.toSec()  > 0.1)
        {
          /* send motor info to uav, about 10Hz */
          aerial_robot_base::MotorInfo motor_info_msg;
          motor_info_msg.min_pwm = min_pwm_;
          motor_info_msg.max_pwm = max_pwm_;
          motor_info_msg.f_pwm_offset = f_pwm_offset_;
          motor_info_msg.f_pwm_rate = f_pwm_rate_;
          motor_info_msg.m_f_rate = m_f_rate_;
          motor_info_msg.pwm_rate = pwm_rate_;
          motor_info_msg.force_landing_pwm = force_landing_pwm_;
          motor_info_pub_.publish(motor_info_msg);
          activate_timestamp = ros::Time::now();
        }
      reset();
    }

    void reset()
    {
      control_timestamp_ = -1;
      start_rp_integration_ = false;
      alt_i_term_.assign(motor_num_, 0);
      yaw_i_term_.assign(motor_num_, 0);
      xy_i_term_.setValue(0,0,0);
    }

    virtual void update();

  protected:
    ros::Publisher  pid_pub_;
    ros::Publisher  motor_info_pub_;
    ros::Publisher  flight_cmd_pub_;
    ros::Subscriber four_axis_gain_sub_;
    ros::Subscriber xy_vel_weak_gain_sub_;

    /* basic var */
    tf::Vector3 state_pos_;
    tf::Vector3 state_vel_;
    tf::Vector3 target_vel_;
    tf::Vector3 target_pos_;
    tf::Vector3 target_acc_;
    tf::Vector3 pos_err_;
    tf::Vector3 vel_err_;
    double state_psi_;
    double target_psi_;
    double psi_err_;

    //**** altitude
    std::vector<tf::Vector3>  alt_gains_;
    double alt_err_thresh_;
    double alt_limit_;
    tf::Vector3 alt_terms_limit_;
    std::vector<double> alt_i_term_;
    double alt_landing_const_i_ctrl_thresh_;
    double alt_offset_;

    //**** xy
    tf::Vector3 xy_gains_;
    double xy_limit_;
    tf::Vector3 xy_terms_limits_;
    tf::Vector3 xy_i_term_;
    double xy_hovering_i_gain_;
    tf::Vector3 xy_offset_;
    std::string xy_vel_weak_gain_sub_name_;
    double xy_vel_weak_rate_;

    //**** yaw
    std::vector<tf::Vector3> yaw_gains_;
    double yaw_limit_;
    tf::Vector3 yaw_terms_limits_;
    std::vector<double>  yaw_i_term_;

    bool start_rp_integration_;

    dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>* xy_pid_server_;
    dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>::CallbackType dynamic_reconf_func_xy_pid_;

    void fourAxisGainCallback(const aerial_robot_msgs::FourAxisGainConstPtr & msg);
    void xyVelWeakGainCallback(const std_msgs::UInt8ConstPtr & msg);
    void cfgXYPidCallback(aerial_robot_base::XYPidControlConfig &config, uint32_t level);
    virtual void rosParamInit();


    tf::Vector3 clampV(tf::Vector3 input, double min, double max)
    {
      return tf::Vector3(clamp(input.x(), min, max),
                         clamp(input.y(), min, max),
                         clamp(input.z(), min, max));
    }

  };
};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(control_plugin::FlatnessPid, control_plugin::ControlBase);

