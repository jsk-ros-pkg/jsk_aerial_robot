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
#include <aerial_robot_base/flight_control.h>

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
    FlatnessPid():
      ControlBase(),
      xy_i_term_(0,0,0),
      start_rp_integration_(false) //xy(pitch/roll control) integration start
    {
      xy_offset_[2] = 0;
      alt_i_term_.resize(motor_num_);
      yaw_i_term_.resize(motor_num_);
    }

    ~FlatnessPid(){};

    void initialize(ros::NodeHandle nh,
              ros::NodeHandle nhp,
              BasicEstimator* estimator,
              Navigator* navigator,
              double ctrl_loop_rate)
    {
      ControlBase::initialize(nh, nhp, estimator, navigator, ctrl_loop_rate);

      rosParamInit();

      //publish
      flight_cmd_pub_ = nh_.advertise<aerial_robot_msgs::FourAxisCommand>("/aerial_robot_control_four_axis", 10);
      pid_pub_ = nh_.advertise<aerial_robot_base::FlatnessPid>("debug", 10);
      motor_info_pub_ = nh_.advertise<aerial_robot_base::MotorInfo>("/motor_info", 10);

      //subscriber
      four_axis_gain_sub_ = nh_.subscribe<aerial_robot_msgs::FourAxisGain>("/four_axis_gain", 1, &FlatnessPid::fourAxisGainCallback, this, ros::TransportHints().tcpNoDelay());
      /* for weak control for xy velocirt movement? necessary */
      xy_vel_weak_gain_sub_ = nh_.subscribe<std_msgs::UInt8>(xy_vel_weak_gain_sub_name_, 1, &FlatnessPid::xyVelWeakGainCallback, this, ros::TransportHints().tcpNoDelay());
      //dynamic reconfigure server
      xy_pid_server_ = new dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>(ros::NodeHandle(nhp_, "pitch"));
      dynamic_reconf_func_xy_pid_ = boost::bind(&FlatnessPid::cfgXYPidCallback, this, _1, _2);
      xy_pid_server_->setCallback(dynamic_reconf_func_xy_pid_);

    }

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

    void update()
    {
      //tf::Vector3 state_pos = estimator_->getPos(Frame::COG, estimate_mode_);
      tf::Vector3 state_pos(estimator_->getState(State::X_COG, estimate_mode_)[0],
                            estimator_->getState(State::Y_COG, estimate_mode_)[0],
                            estimator_->getState(State::Z_COG, estimate_mode_)[0]);
      tf::Vector3 state_vel(estimator_->getState(State::X_COG, estimate_mode_)[1],
                            estimator_->getState(State::Y_COG, estimate_mode_)[1],
                            estimator_->getState(State::Z_COG, estimate_mode_)[1]);
      //tf::Vector3 state_vel = estimator_->getVel(Frame::COG, estimate_mode_);
      tf::Vector3 target_vel = navigator_->getTargetVel();
      tf::Vector3 target_pos = navigator_->getTargetPos();
      tf::Vector3 target_acc = navigator_->getTargetAcc();

      double state_psi = estimator_->getState(State::YAW, estimate_mode_)[0];
      double target_psi = navigator_->getTargetPsi();

      aerial_robot_base::FlatnessPid pid_msg;
      pid_msg.header.stamp = ros::Time::now();

      /* motor related info */
      /* initialize setting */
      if(navigator_->getNaviState() == Navigator::ARM_ON_STATE) activate();
      if(navigator_->getNaviState() == Navigator::ARM_OFF_STATE && control_timestamp_ > 0)
        {
          reset();
        }

      if (control_timestamp_ < 0)
        {
          if (navigator_->getNaviState() == Navigator::TAKEOFF_STATE)
            {
              reset();
              control_timestamp_ = ros::Time::now().toSec();
            }
          else return;
        }


      /* roll/pitch integration flag */
      if(!start_rp_integration_)
        {
          if(state_pos.z() > 0.01)
            {
              start_rp_integration_ = true;
              std_msgs::UInt8 integration_cmd;
              integration_cmd.data = navigator_->ROS_INTEGRATE_CMD;
              navigator_->flight_config_pub_.publish(integration_cmd);
              ROS_WARN("start rp integration");
            }
        }

      /* time diff */
      double du = ros::Time::now().toSec() - control_timestamp_;

      /* xy */
      tf::Vector3 xy_p_term, xy_d_term;
      tf::Vector3 pos_err = tf::Matrix3x3(tf::createQuaternionFromYaw(-state_psi)) * (target_pos - state_pos);
      float pos_err_pitch = (target_pos - state_pos).x() * cos(state_psi) + (target_pos - state_pos).y() * sin(state_psi);
      float pos_err_roll = -(target_pos - state_pos).x() * sin(state_psi) + (target_pos - state_pos).y() * cos(state_psi);
      switch(navigator_->getXyControlMode())
        {
        case flight_nav::POS_CONTROL_MODE:
          {
            /* convert from world frame to CoG frame */
            tf::Vector3 vel_err = tf::Matrix3x3(tf::createQuaternionFromYaw(-state_psi)) * (-state_vel);

            /* P */
            xy_p_term = clampV(xy_gains_[0] * pos_err,  -xy_terms_limits_[0], xy_terms_limits_[0]);

            /* I */
            if(navigator_->getNaviState() == Navigator::TAKEOFF_STATE || navigator_->getNaviState() == Navigator::LAND_STATE) //takeoff or land
              xy_i_term_ += (pos_err * du * xy_gains_[1]);
            else
              xy_i_term_ += (pos_err * du * xy_hovering_i_gain_);
            xy_i_term_ = clampV(xy_i_term_, -xy_terms_limits_[1], xy_terms_limits_[1]);

            /* D */
            xy_d_term = clampV(xy_gains_[2] * vel_err,  -xy_terms_limits_[2], xy_terms_limits_[2]);
            break;
          }
        case flight_nav::VEL_CONTROL_MODE:
          {
            /* convert from world frame to CoG frame */
            tf::Vector3 vel_err = tf::Matrix3x3(tf::createQuaternionFromYaw(-state_psi)) * (target_vel - state_vel);

            /* P */
            xy_p_term = clampV(xy_gains_[2] * vel_err,  -xy_terms_limits_[0], xy_terms_limits_[0]);
            xy_d_term.setValue(0, 0, 0);
            break;
          }
        case flight_nav::ACC_CONTROL_MODE:
          {
            /* convert from world frame to CoG frame */
            xy_p_term = tf::Matrix3x3(tf::createQuaternionFromYaw(-state_psi)) * (target_acc / BasicEstimator::G);
            xy_i_term_.setValue(0, 0, 0);
            xy_d_term.setValue(0, 0, 0);
            break;
          }
        default:
          {
            break;
          }
        }

      tf::Vector3 xy_total_term = xy_p_term + xy_i_term_ + xy_d_term + xy_offset_;
      double target_pitch = clamp(xy_total_term[0], -xy_limit_, xy_limit_);
      double target_roll = clamp(-xy_total_term[1], -xy_limit_, xy_limit_);

      /* ros pub */
      pid_msg.pitch.total.push_back(target_pitch);
      pid_msg.roll.total.push_back(target_roll);
      pid_msg.pitch.p_term.push_back(xy_p_term[0]);
      pid_msg.pitch.i_term.push_back(xy_i_term_[0]);
      pid_msg.pitch.d_term.push_back(xy_d_term[0]);
      pid_msg.roll.p_term.push_back(xy_p_term[1]);
      pid_msg.roll.i_term.push_back(xy_i_term_[1]);
      pid_msg.roll.d_term.push_back(xy_d_term[1]);
      pid_msg.pitch.pos_err = pos_err[0];
      pid_msg.pitch.target_pos = target_pos[0];
      pid_msg.roll.pos_err = pos_err[1];
      pid_msg.roll.target_pos = target_pos[1];
      pid_msg.pitch.target_vel = target_vel[0];
      pid_msg.roll.target_vel = target_vel[1];


      /* yaw */
      std::vector<double> target_yaw(motor_num_);
      double psi_err = target_psi - state_psi;
      if(psi_err > M_PI)  psi_err -= 2 * M_PI;
      else if(psi_err < -M_PI)  psi_err += 2 * M_PI;

      for(int j = 0; j < motor_num_; j++)
        {
          //**** P term
          double yaw_p_term = clamp(-yaw_gains_[j][0] * psi_err, -yaw_terms_limits_[0], yaw_terms_limits_[0]);

          //**** I term:
          yaw_i_term_[j] += (psi_err * du * yaw_gains_[j][1]);
          yaw_i_term_[j] = clamp(yaw_i_term_[j], -yaw_terms_limits_[1], yaw_terms_limits_[1]);

          //***** D term: is in the flight board

          //*** each motor command value for log
          target_yaw[j] = clamp(yaw_p_term + yaw_i_term_[j], -yaw_limit_, yaw_limit_);

          pid_msg.yaw.total.push_back(target_yaw[j]);
          pid_msg.yaw.p_term.push_back(yaw_p_term);
          pid_msg.yaw.i_term.push_back(yaw_i_term_[j]);
        }

      //**** ros pub
      pid_msg.yaw.target_pos = target_psi;
      pid_msg.yaw.pos_err = psi_err;

      /* throttle */
      std::vector<double> target_throttle(motor_num_);
      double alt_err = clamp(target_pos.z() - state_pos.z(), -alt_err_thresh_, alt_err_thresh_);

      for(int j = 0; j < motor_num_; j++)
        {
          //**** P Term
          double alt_p_term = clamp(-alt_gains_[j][0] * alt_err, -alt_terms_limit_[0], alt_terms_limit_[0]);
          if(navigator_->getNaviState() == Navigator::LAND_STATE) alt_p_term = 0;

          //**** I Term
          if(navigator_->getNaviState() == Navigator::LAND_STATE) alt_err += alt_landing_const_i_ctrl_thresh_;
          /* two way to calculate the alt i term */
          alt_i_term_[j] +=  alt_err * du;
          double alt_i_term = clamp(alt_gains_[j][1] * alt_i_term_[j], -alt_terms_limit_[1], alt_terms_limit_[1]);
          //***** D Term
          double alt_d_term = clamp(alt_gains_[j][2] * state_vel.z(), -alt_terms_limit_[2], alt_terms_limit_[2]);

          //*** each motor command value for log
          target_throttle[j] = clamp(alt_p_term + + alt_i_term + alt_d_term + alt_offset_, -alt_limit_, alt_limit_);
          pid_msg.throttle.total.push_back(target_throttle[j]);
          pid_msg.throttle.p_term.push_back(alt_p_term);
          pid_msg.throttle.i_term.push_back(alt_i_term);
          pid_msg.throttle.d_term.push_back(alt_d_term);
        }

      pid_msg.throttle.target_pos = target_pos.z();
      pid_msg.throttle.pos_err = alt_err;
      pid_msg.throttle.vel_err = state_vel.z();

      /* ros publish */
      pid_pub_.publish(pid_msg);

      /* send flight command */
      aerial_robot_msgs::FourAxisCommand flight_command_data;
      flight_command_data.angles[0] =  target_roll;
      flight_command_data.angles[1] =  target_pitch;

      flight_command_data.base_throttle.resize(motor_num_);
      if(motor_num_ == 1)
        {
          /* Simple PID based attitude/altitude control */
          flight_command_data.angles[2] = target_yaw[0];
          flight_command_data.base_throttle[0] =  target_throttle[0];
          if(target_throttle[0] == 0) return; // do not publish the empty flight command => the force landing flag will be activate
        }
      else
        {
          /* LQI based attitude/altitude control */
          for(int i = 0; i < motor_num_; i++)
            flight_command_data.base_throttle[i] = target_throttle[i] + target_yaw[i];
        }
      flight_cmd_pub_.publish(flight_command_data);

      /* update */
      control_timestamp_ = ros::Time::now().toSec();
    }

  private:
    ros::Publisher  pid_pub_;
    ros::Publisher  motor_info_pub_;
    ros::Publisher  flight_cmd_pub_;
    ros::Subscriber four_axis_gain_sub_;
    ros::Subscriber xy_vel_weak_gain_sub_;

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

    void fourAxisGainCallback(const aerial_robot_msgs::FourAxisGainConstPtr & msg)
    {
      /* update the motor number */
      if(motor_num_ == 1)
        {
          motor_num_ = msg->motor_num;

          yaw_i_term_.resize(motor_num_);
          yaw_gains_.resize(motor_num_);
          alt_i_term_.resize(motor_num_);
          alt_gains_.resize(motor_num_);

          ROS_WARN("flight control: update the motor number: %d", motor_num_);
        }

      for(int i = 0; i < msg->motor_num; i++)
        {
          yaw_gains_[i].setValue(msg->pos_p_gain_yaw[i], msg->pos_i_gain_yaw[i], msg->pos_d_gain_yaw[i]);
          alt_gains_[i].setValue(msg->pos_p_gain_alt[i], msg->pos_i_gain_alt[i], msg->pos_d_gain_alt[i]);
        }
    }

    void xyVelWeakGainCallback(const std_msgs::UInt8ConstPtr & msg)
    {
      ROS_INFO("xyVelWeakGainCallback: gain from %f", xy_gains_[2]);
      if(msg->data == 1) xy_gains_[2] *= xy_vel_weak_rate_;
      else xy_gains_[2] /= xy_vel_weak_rate_;
      ROS_INFO("xyVelWeakGainCallback: gain to %f", xy_gains_[2]);
    }

    void cfgXYPidCallback(aerial_robot_base::XYPidControlConfig &config, uint32_t level)
    {
      if(config.xy_pid_control_flag)
        {
          printf("XY Pid Param:");
          switch(level)
            {
            case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_P_GAIN:
              xy_gains_[0] = config.p_gain;
              printf("change the p gain\n");
              break;
            case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_GAIN:
              xy_gains_[1] = config.i_gain;
              printf("change the i gain\n");
              break;
            case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_D_GAIN:
              xy_gains_[2] = config.d_gain;
              printf("change the d gain\n");
              break;
            case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_GAIN_HOVER:
              xy_hovering_i_gain_ = config.i_gain_hover;
              printf("change the i hovering gain\n");
              break;
            case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_LIMIT:
              xy_limit_ = config.limit;
              printf("change the limit\n");
              break;
            case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_P_LIMIT:
              xy_terms_limits_[0] = config.p_limit;
              printf("change the p limit\n");
              break;
            case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_LIMIT:
              xy_terms_limits_[1] = config.i_limit;
              printf("change the i limit\n");
              break;
            case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_D_LIMIT:
              xy_terms_limits_[2] = config.d_limit;
              printf("change the d limit\n");
              break;
            default :
              printf("\n");
              break;
            }
        }
    }

    void rosParamInit()
    {
      ros::NodeHandle alt_node(nhp_, "alt");
      ros::NodeHandle xy_node(nhp_, "xy");
      ros::NodeHandle yaw_node(nhp_, "yaw");

      string alt_ns = alt_node.getNamespace();
      string xy_ns = xy_node.getNamespace();
      string yaw_ns = yaw_node.getNamespace();

      /* altitude */
      alt_node.param("alt_landing_const_i_ctrl_thresh",  alt_landing_const_i_ctrl_thresh_, 0.0);
      if(verbose_) cout << alt_ns << ": alt_landing_const_i_ctrl_thresh_ is " << alt_landing_const_i_ctrl_thresh_ << endl;
      alt_node.param("offset", alt_offset_, 0.0);
      if(verbose_) cout << alt_ns << ": offset_ is " << alt_offset_ << endl;
      alt_node.param("limit", alt_limit_, 1.0e6);
      if(verbose_) cout << alt_ns << ": pos_limit_ is " << alt_limit_ << endl;
      alt_node.param("p_term_limit", alt_terms_limit_[0], 1.0e6);
      if(verbose_) cout << alt_ns << ": pos_p_limit_ is " << alt_terms_limit_[0] << endl;
      alt_node.param("i_term_limit", alt_terms_limit_[1], 1.0e6);
      if(verbose_) cout << alt_ns << ": pos_i_limit_ is " << alt_terms_limit_[1] << endl;
      alt_node.param("d_term_limit", alt_terms_limit_[2], 1.0e6);
      if(verbose_) cout << alt_ns << ": pos_d_limit_ is " << alt_terms_limit_[2] << endl;
      alt_node.param("err_thresh", alt_err_thresh_, 1.0);
      if(verbose_) cout << alt_ns << ": alt_err_thresh_ is " << alt_err_thresh_ << endl;
      alt_gains_.resize(1); /* default is for general multirotor */
      alt_node.param("p_gain", alt_gains_[0][0], 0.0);
      if(verbose_) cout << alt_ns << ": p_gain_ is " << alt_gains_[0][0] << endl;
      alt_node.param("i_gain", alt_gains_[0][1], 0.001);
      if(verbose_) cout << alt_ns << ": i_gain_ is " << alt_gains_[0][1] << endl;
      alt_node.param("d_gain", alt_gains_[0][2], 0.0);
      if(verbose_) cout << alt_ns << ": d_gain_ is " << alt_gains_[0][2] << endl;

      /* xy */
      xy_node.param("x_offset", xy_offset_[0], 0.0);
      if(verbose_) cout << xy_ns << ": x_offset_ is " <<  xy_offset_[0] << endl;
      xy_node.param("y_offset", xy_offset_[1], 0.0);
      if(verbose_) cout << xy_ns << ": y_offset_ is " <<  xy_offset_[1] << endl;
      xy_node.param("limit", xy_limit_, 1.0e6);
      if(verbose_) cout << xy_ns << ": pos_limit_ is " <<  xy_limit_ << endl;
      xy_node.param("p_term_limit", xy_terms_limits_[0], 1.0e6);
      if(verbose_) cout << xy_ns << ": pos_p_limit_ is " <<  xy_terms_limits_[0] << endl;
      xy_node.param("i_term_limit", xy_terms_limits_[1], 1.0e6);
      if(verbose_) cout << xy_ns << ": pos_i_limit_ is " <<  xy_terms_limits_[1] << endl;
      xy_node.param("d_term_limit", xy_terms_limits_[2], 1.0e6);
      if(verbose_) cout << xy_ns << ": pos_d_limit_ is " <<  xy_terms_limits_[2] << endl;

      xy_node.param("p_gain", xy_gains_[0], 0.0);
      if(verbose_) cout << xy_ns << ": p_gain_ is " << xy_gains_[0] << endl;
      xy_node.param("i_gain", xy_gains_[1], 0.0);
      if(verbose_) cout << xy_ns << ": i_gain_ is " << xy_gains_[1] << endl;
      xy_node.param("d_gain", xy_gains_[2], 0.0);
      if(verbose_) cout << xy_ns << ": d_gain_ is " << xy_gains_[2] << endl;
      xy_node.param("hovering_i_gain", xy_hovering_i_gain_, 0.0);
      if(verbose_) cout << xy_ns << ": pos_i_gain_hover_ is " <<  xy_hovering_i_gain_ << endl;
      xy_node.param("xy_vel_weak_gain_sub_name", xy_vel_weak_gain_sub_name_, string("xy_vel_weak_gain"));
      if(verbose_) cout << xy_ns << "xy_vel_weak_gain_sub_name_ is " << xy_vel_weak_gain_sub_name_ << endl;
      xy_node.param("xy_vel_weak_rate", xy_vel_weak_rate_, 0.2); // 20%
      if(verbose_) cout << xy_ns << "xy_vel_weak_rate_ is " << xy_vel_weak_rate_ << endl;

      /* yaw */
      yaw_node.param("limit", yaw_limit_, 1.0e6);
      if(verbose_) cout << yaw_ns << ": pos_limit_ is " << yaw_limit_ << endl;
      yaw_node.param("p_term_limit", yaw_terms_limits_[0], 1.0e6);
      if(verbose_) cout << yaw_ns << ": pos_p_limit_ is " << yaw_terms_limits_[0] << endl;
      yaw_node.param("i_term_limit", yaw_terms_limits_[1], 1.0e6);
      if(verbose_) cout << yaw_ns << ": pos_i_limit_ is " << yaw_terms_limits_[1] << endl;
      yaw_node.param("d_term_limit", yaw_terms_limits_[2], 1.0e6);
      if(verbose_) cout << yaw_ns << ": pos_d_limit_ is " << yaw_terms_limits_[2] << endl;
      yaw_gains_.resize(1); /* default is for general multirotor */
      yaw_node.param("p_gain", yaw_gains_[0][0], 0.0);
      if(verbose_) cout << yaw_ns << ": p_gain_ is " << yaw_gains_[0][0] << endl;
      yaw_node.param("i_gain", yaw_gains_[0][1], 0.0);
      if(verbose_) cout << yaw_ns << ": i_gain_ is " << yaw_gains_[0][1] << endl;
      yaw_node.param("d_gain", yaw_gains_[0][2], 0.0);
      if(verbose_) cout << yaw_ns << ": d_gain_ is " << yaw_gains_[0][2] << endl;
    }

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

