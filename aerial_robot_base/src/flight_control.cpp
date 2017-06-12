#include "aerial_robot_base/flight_control.h"

using boost::algorithm::clamp;

FlightController::FlightController(ros::NodeHandle nh,
                                   ros::NodeHandle nh_private,
                                   BasicEstimator* estimator, Navigator* navigator,
                                   FlightCtrlInput* flight_ctrl_input): nh_(nh, "controller"), nhp_(nh_private, "controller")
{
  estimator_ = estimator;
  navigator_ = navigator;
  flight_ctrl_input_ = flight_ctrl_input;

  estimate_mode_ = estimator_->getEstimateMode();

  ros::NodeHandle motor_info_node("motor_info");
  std::string ns = motor_info_node.getNamespace();
  //normal namespace
  motor_num_ = flight_ctrl_input_->getMotorNumber();
  printf("%s: motor_num_ is %d\n", ns.c_str(), motor_num_);
  //controller namespace
  if (!motor_info_node.getParam ("min_pwm", min_pwm_))
    min_pwm_ = 0.55; //0.55;
  printf("%s: min_pwm_ is %f\n", ns.c_str(), min_pwm_);

  if (!motor_info_node.getParam ("max_pwm", max_pwm_))
    max_pwm_ = 0.95; //0.55;
  printf("%s: max_pwm_ is %f\n", ns.c_str(), max_pwm_);

  if (!motor_info_node.getParam ("f_pwm_rate", f_pwm_rate_))
    f_pwm_rate_ = 1; //0.3029;
  printf("%s: f_pwm_rate_ is %f\n", ns.c_str(), f_pwm_rate_);

  if (!motor_info_node.getParam ("m_f_rate", m_f_rate_))
    m_f_rate_ = 0; 
  printf("%s: m_f_rate_ is %.3f\n", ns.c_str(), m_f_rate_); //-0.016837, the sgn is right?, should be nagative

  if (!motor_info_node.getParam ("f_pwm_offset", f_pwm_offset_))
    f_pwm_offset_ = 0; //-21.196;
  printf("%s: f_pwm_offset_ is %f\n", ns.c_str(), f_pwm_offset_);

  if (!motor_info_node.getParam ("pwm_rate", pwm_rate_))
    pwm_rate_ = 1; //1800/100.0;
  printf("%s: pwm_rate_ is %f\n", ns.c_str(), pwm_rate_);

  if (!motor_info_node.getParam ("force_landing_pwm", force_landing_pwm_))
    force_landing_pwm_ = 0.75; //1500;
  printf("%s: force_landing_pwm_ is %f\n", ns.c_str(), force_landing_pwm_);

  if (!nhp_.getParam ("feedforward_flag", feedforward_flag_))
    feedforward_flag_ = false;
  printf("%s: feedforward_flag_ is %s\n", nhp_.getNamespace().c_str(), (feedforward_flag_)?"true":"false");

  nhp_.param("motor_num_srv_name", motor_num_srv_name_, std::string("/get_motor_num"));
  motor_num_srv_ =  nh_.advertiseService(motor_num_srv_name_, &FlightController::motorNumCallback, this);
}

PidController::PidController(ros::NodeHandle nh,
                             ros::NodeHandle nh_private,
                             BasicEstimator* estimator,
                             Navigator* navigator,
                             FlightCtrlInput* flight_ctrl_input,
                             double ctrl_loop_rate)
  : FlightController(nh, nh_private, estimator, navigator, flight_ctrl_input)
{
  pid_ctrl_loop_rate_ = ctrl_loop_rate;

  rosParamInit(nhp_);

  d_err_pos_curr_pitch_ = 0;
  d_err_pos_curr_roll_ = 0;
  d_err_pos_curr_throttle_ = 0;
  d_err_pos_curr_yaw_ = 0;

  d_err_vel_curr_roll_ = 0;
  d_err_vel_curr_pitch_ = 0;
  d_err_vel_curr_yaw_ = 0;
  d_err_vel_curr_throttle_ = 0;
  d_err_vel_prev_roll_ = 0;
  d_err_vel_prev_pitch_ = 0;
  d_err_vel_prev_yaw_ = 0;
  d_err_vel_prev_throttle_ = 0;

  pos_i_term_roll_ = 0;
  pos_i_term_pitch_ = 0;
  pos_i_term_yaw_ = 0;
  pos_i_term_throttle_ = 0;
  pos_p_term_roll_ = 0;
  pos_p_term_pitch_ = 0;
  pos_p_term_yaw_ = 0;
  pos_p_term_throttle_ = 0;
  pos_d_term_roll_ = 0;
  pos_d_term_pitch_ = 0;
  pos_d_term_yaw_ = 0;
  pos_d_term_throttle_ = 0;

  error_i_throttle_ = 0;
  error_i_yaw_ = 0;


  feedforward_matrix_ = Eigen::MatrixXd::Zero(motor_num_, 2);

  //roll/pitch integration start
  start_rp_integration_ = false;

  //publish
  pid_pub_ = nh_.advertise<aerial_robot_base::FourAxisPid>("debug", 10);
  ff_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("ff", 10);
  motor_info_pub_ = nh_.advertise<aerial_robot_base::MotorInfo>("/motor_info", 10);

  //subscriber
  four_axis_gain_sub_ = nh_.subscribe<aerial_robot_msgs::FourAxisGain>("/four_axis_gain", 1, &PidController::fourAxisGainCallback, this, ros::TransportHints().tcpNoDelay());
  /* for weak control for xy velocirt movement? necessary */
  xy_vel_weak_gain_sub_ = nh_.subscribe<std_msgs::UInt8>(xy_vel_weak_gain_sub_name_, 1, &PidController::xyVelWeakGainCallback, this, ros::TransportHints().tcpNoDelay());
  //dynamic reconfigure server
  xy_pid_server_ = new dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>(ros::NodeHandle(nhp_, "pitch"));
  dynamic_reconf_func_xy_pid_ = boost::bind(&PidController::cfgXYPidCallback, this, _1, _2);
  xy_pid_server_->setCallback(dynamic_reconf_func_xy_pid_);

}


void PidController::fourAxisGainCallback(const aerial_robot_msgs::FourAxisGainConstPtr & msg)
{
  if(msg->motor_num != motor_num_)
    {
      ROS_FATAL("the motor number is not correct, msg->motor_num:%d, motor_num_:%d", msg->motor_num, motor_num_);
      return ;
    }

  for(int i = 0; i < msg->motor_num; i++)
    {
      pos_p_gain_yaw_[i] = msg->pos_p_gain_yaw[i];
      pos_i_gain_yaw_[i] = msg->pos_i_gain_yaw[i];
      pos_d_gain_yaw_[i] = msg->pos_d_gain_yaw[i];

      pos_p_gain_throttle_[i] = msg->pos_p_gain_throttle[i];
      pos_i_gain_throttle_[i] = msg->pos_i_gain_throttle[i];
      pos_d_gain_throttle_[i] = msg->pos_d_gain_throttle[i];

      feedforward_matrix_(i, 0) = msg->ff_roll_vec[i];
      feedforward_matrix_(i, 1) = msg->ff_pitch_vec[i];
    }
}

void PidController::xyVelWeakGainCallback(const std_msgs::UInt8ConstPtr & msg)
{
  ROS_INFO("xyVelWeakGainCallback: gain from %f", vel_p_gain_roll_);
  if(msg->data == 1)
    {/* weak control gain */
      vel_p_gain_roll_ *= xy_vel_weak_rate_;
      vel_p_gain_pitch_ *= xy_vel_weak_rate_;
    }
  else
    {/* return usual control gain*/
      vel_p_gain_roll_ /= xy_vel_weak_rate_;
      vel_p_gain_pitch_ /= xy_vel_weak_rate_;
    }
  ROS_INFO("xyVelWeakGainCallback: gain to %f", vel_p_gain_roll_);
}

void PidController::pidFunction()
{
  static bool first_flag = true;

  /* Reset */
  if (navigator_->getFlightMode() == Navigator::RESET_MODE)
    {
      //ROS_ERROR("ok");
      first_flag = true;

      pos_p_term_pitch_ = 0; pos_i_term_pitch_ = 0; pos_d_term_pitch_ = 0;
      pos_p_term_roll_ = 0; pos_i_term_roll_ = 0; pos_d_term_roll_ = 0;
      pos_p_term_yaw_ = 0; pos_i_term_yaw_ = 0; pos_d_term_yaw_ = 0;
      pos_p_term_throttle_ = 0; pos_i_term_throttle_ = 0; pos_d_term_throttle_ = 0;

      error_i_throttle_ = 0;  error_i_yaw_ = 0;

      start_rp_integration_ = false;

      flight_ctrl_input_->reset();

      navigator_->setFlightMode(Navigator::NO_CONTROL_MODE);

      return;
    }
  else if(navigator_->getFlightMode() == Navigator::NO_CONTROL_MODE)
    {
      if(navigator_->getNaviCommand() == Navigator::START_COMMAND)
        {
          /* send motor info to uav */
          aerial_robot_base::MotorInfo motor_info_msg;
          motor_info_msg.min_pwm = min_pwm_;
          motor_info_msg.max_pwm = max_pwm_;
          motor_info_msg.f_pwm_offset = f_pwm_offset_;
          motor_info_msg.f_pwm_rate = f_pwm_rate_;
          motor_info_msg.m_f_rate = m_f_rate_;
          motor_info_msg.pwm_rate = pwm_rate_;
          motor_info_msg.force_landing_pwm = force_landing_pwm_;
          motor_info_pub_.publish(motor_info_msg);
        }
      return;
    }
  else
    {
      float state_pos_x = 0;
      float state_vel_x = 0;
      float state_pos_y = 0;
      float state_vel_y = 0;

      float target_pos_x = 0;
      float target_pos_y = 0;
      float target_vel_x = 0;
      float target_vel_y = 0;

      float state_pos_z = 0;
      float state_vel_z = 0;
      float state_psi_cog = 0;
      float state_psi_board = 0;
      float state_vel_psi = 0;

      if(navigator_->getXyControlMode() == Navigator::POS_WORLD_BASED_CONTROL_MODE ||
         navigator_->getXyControlMode() == Navigator::VEL_WORLD_BASED_CONTROL_MODE)
        {
          state_pos_x = estimator_->getState(BasicEstimator::X_W, estimate_mode_)[0];
          state_vel_x = estimator_->getState(BasicEstimator::X_W, estimate_mode_)[1];
          state_pos_y = estimator_->getState(BasicEstimator::Y_W, estimate_mode_)[0];
          state_vel_y = estimator_->getState(BasicEstimator::Y_W, estimate_mode_)[1];
        }
      else if(navigator_->getXyControlMode() == navigator_->POS_LOCAL_BASED_CONTROL_MODE ||
              navigator_->getXyControlMode() == navigator_->VEL_LOCAL_BASED_CONTROL_MODE)
        {
          state_pos_x = estimator_->getState(BasicEstimator::X_B, estimate_mode_)[0];
          state_vel_x = estimator_->getState(BasicEstimator::X_B, estimate_mode_)[1];
          state_pos_y = estimator_->getState(BasicEstimator::Y_B, estimate_mode_)[0];
          state_vel_y = estimator_->getState(BasicEstimator::Y_B, estimate_mode_)[1];
        }

      target_vel_x = navigator_->getTargetVelX();
      target_vel_y = navigator_->getTargetVelY();
      target_pos_x = navigator_->getTargetPosX();
      target_pos_y = navigator_->getTargetPosY();

      state_pos_z = estimator_->getState(BasicEstimator::Z_W, estimate_mode_)[0];
      state_vel_z = estimator_->getState(BasicEstimator::Z_W, estimate_mode_)[1];

      state_psi_cog = estimator_->getState(BasicEstimator::YAW_W, estimate_mode_)[0];
      state_vel_psi = estimator_->getState(BasicEstimator::YAW_W, estimate_mode_)[1];
      state_psi_board = estimator_->getState(BasicEstimator::YAW_W_B, estimate_mode_)[0];

      float target_pos_z = navigator_->getTargetPosZ();
      float target_psi = navigator_->getTargetPsi();

      aerial_robot_base::FourAxisPid  four_axis_pid_debug;

      four_axis_pid_debug.header.stamp = ros::Time::now();

      if (first_flag)
        {
          first_flag = false;
        }
      else
        {
          float roll_value = 0, pitch_value = 0;

          //roll/pitch integration flag
          if(!start_rp_integration_)
            {
              if(state_pos_z > 0.01)
                {
                  start_rp_integration_ = true;
                  std_msgs::UInt8 integration_cmd;
                  integration_cmd.data = navigator_->ROS_INTEGRATE_CMD;
                  navigator_->flight_config_pub_.publish(integration_cmd);
                  ROS_WARN("start rp integration");
                }
            }

          //pitch
          if(navigator_->getXyControlMode() == Navigator::POS_WORLD_BASED_CONTROL_MODE)
            {
              //** 座標変換
              d_err_pos_curr_pitch_
                = (target_pos_x - state_pos_x) * cos(state_psi_cog)
                + (target_pos_y - state_pos_y) * sin(state_psi_cog);

              d_err_vel_curr_pitch_ = - (state_vel_x * cos(state_psi_cog) + state_vel_y * sin(state_psi_cog));

              //**** Pの項
              pos_p_term_pitch_ =
                limit(pos_p_gain_pitch_ * d_err_pos_curr_pitch_,  pos_p_limit_pitch_);

              //**** Iの項
              if(navigator_->getFlightMode() == Navigator::TAKEOFF_MODE || navigator_->getFlightMode() == Navigator::LAND_MODE) //takeoff or land
                pos_i_term_pitch_ += d_err_pos_curr_pitch_ * (1 / (float)pitch_ctrl_loop_rate_) * pos_i_gain_pitch_;
              //else if(navigator_->getFlightMode() == Navigator::FLIGHT_MODE && fabs(d_err_pos_curr_pitch_) < i_enable_limit_pitch_) //hover or land
              else if(navigator_->getFlightMode() == Navigator::FLIGHT_MODE) //hover or land
                pos_i_term_pitch_ += d_err_pos_curr_pitch_ * (1 / (float)pitch_ctrl_loop_rate_) * pos_i_gain_pitch_hover_;
              pos_i_term_pitch_ = limit(pos_i_term_pitch_, pos_i_limit_pitch_);

              //***** Dの項
              pos_d_term_pitch_ = limit(pos_d_gain_pitch_ * d_err_vel_curr_pitch_, pos_d_limit_pitch_);

            }
          else if(navigator_->getXyControlMode() == Navigator::POS_LOCAL_BASED_CONTROL_MODE)
            {
              //** 座標変換
              d_err_pos_curr_pitch_ = target_pos_x - state_pos_x;
              d_err_vel_curr_pitch_ = target_vel_x - state_vel_x;

              //**** Pの項
              pos_p_term_pitch_ =
                limit(pos_p_gain_pitch_ * d_err_pos_curr_pitch_, pos_p_limit_pitch_);
              //**** Dの項
              pos_d_term_pitch_ =
                limit(pos_d_gain_pitch_ * d_err_vel_curr_pitch_, pos_d_limit_pitch_);

            }
          else if(navigator_->getXyControlMode() == Navigator::VEL_WORLD_BASED_CONTROL_MODE)
            {
              //** 座標変換
              d_err_vel_curr_pitch_ 
                = (target_vel_x - state_vel_x) * cos(state_psi_cog)
                + (target_vel_y - state_vel_y) * sin(state_psi_cog);

              //**** Pの項
              pos_p_term_pitch_ = limit(vel_p_gain_pitch_ * d_err_vel_curr_pitch_, pos_d_limit_pitch_);
              pos_d_term_pitch_ =  0;
            }
          else if(navigator_->getXyControlMode() == Navigator::VEL_LOCAL_BASED_CONTROL_MODE)
            {
              if(navigator_->getFlightMode() == Navigator::TAKEOFF_MODE)
                {// special mode for velocity control
                  if(navigator_->getXyVelModePosCtrlTakeoff())
                    {
                      //**** Pの項
                      d_err_pos_curr_pitch_ = target_pos_x - state_pos_x;
                      d_err_vel_curr_pitch_ = target_vel_x - state_vel_x;

                      pos_p_term_pitch_ = limit(pos_p_gain_pitch_ * d_err_pos_curr_pitch_,  pos_p_limit_pitch_);

                      pos_i_term_pitch_ += d_err_pos_curr_pitch_ * (1 / (float)pitch_ctrl_loop_rate_) * pos_i_gain_pitch_;
                      pos_i_term_pitch_ = limit(pos_i_term_pitch_, pos_i_limit_pitch_);

                      pos_d_term_pitch_ = limit(pos_d_gain_pitch_ * d_err_vel_curr_pitch_, pos_d_limit_pitch_);
                    }
                  else
                    {
                      d_err_vel_curr_pitch_ = target_vel_x - state_vel_x;
                      //**** Pの項
                      pos_p_term_pitch_ = limit(vel_p_gain_pitch_ * d_err_vel_curr_pitch_, pos_d_limit_pitch_);
                      //**** Iの項
                      pos_i_term_pitch_ = 0;
                      //**** Dの項
                      pos_d_term_pitch_ = 0;
                    }
                }
              else
                { // other flight mode except takeoff
                  d_err_vel_curr_pitch_ = target_vel_x - state_vel_x;
                  //**** Pの項
                  pos_p_term_pitch_ = limit(vel_p_gain_pitch_ * d_err_vel_curr_pitch_, pos_d_limit_pitch_);

                  //**** Dの項
                  pos_d_term_pitch_ = 0;
                }
            }
          //*** 指令値算出
          pitch_value = limit(pos_p_term_pitch_ + pos_i_term_pitch_ + pos_d_term_pitch_ + offset_pitch_, pos_limit_pitch_);

	  //**** attitude control mode
	  if(navigator_->getXyControlMode() == Navigator::ATT_CONTROL_MODE)
            {
              pitch_value = navigator_->getTargetAnglePitch();
              pos_p_term_pitch_ =  0;
              pos_i_term_pitch_ =  0;
              pos_d_term_pitch_ =  0;
            }

          //*** 指令値代入
          flight_ctrl_input_->setPitchValue(pitch_value);

          //**** ros pub
          four_axis_pid_debug.pitch.total.push_back(pitch_value);
          four_axis_pid_debug.pitch.p_term.push_back(pos_p_term_pitch_);
          four_axis_pid_debug.pitch.i_term.push_back(pos_i_term_pitch_);
          four_axis_pid_debug.pitch.d_term.push_back(pos_d_term_pitch_);
          four_axis_pid_debug.pitch.pos_err_transform = d_err_pos_curr_pitch_;
          four_axis_pid_debug.pitch.pos_err_no_transform = target_pos_x - state_pos_x;
          four_axis_pid_debug.pitch.vel_err_transform = target_vel_x;
          four_axis_pid_debug.pitch.vel_err_no_transform = target_vel_x -state_vel_x;

          //roll
          if(navigator_->getXyControlMode() == Navigator::POS_WORLD_BASED_CONTROL_MODE)
            {
              //** 座標変換
              d_err_pos_curr_roll_ 
                = - (target_pos_x - state_pos_x) * sin(state_psi_cog)
                + (target_pos_y - state_pos_y) * cos(state_psi_cog);

              d_err_vel_curr_roll_ = - (- state_vel_x * sin(state_psi_cog) + state_vel_y * cos(state_psi_cog));

              //**** Pの項
              pos_p_term_roll_ = limit(pos_p_gain_roll_ * d_err_pos_curr_roll_, pos_p_limit_roll_);

              //**** Iの項
              if(navigator_->getFlightMode() == Navigator::TAKEOFF_MODE || navigator_->getFlightMode() == Navigator::LAND_MODE) //takeoff or land
                pos_i_term_roll_ += d_err_pos_curr_roll_ * (1 / (float)roll_ctrl_loop_rate_) * pos_i_gain_roll_;
              //else if(navigator_->getFlightMode() == Navigator::FLIGHT_MODE && fabs(d_err_pos_curr_roll_) < i_enable_limit_roll_) //hover
              else if(navigator_->getFlightMode() == Navigator::FLIGHT_MODE) //hover
                pos_i_term_roll_ += d_err_pos_curr_roll_ * (1 / (float)roll_ctrl_loop_rate_) * pos_i_gain_roll_hover_;
              pos_i_term_roll_ = limit(pos_i_term_roll_, pos_i_limit_roll_);

              //***** Dの項
              pos_d_term_roll_ = limit(pos_d_gain_roll_ * d_err_vel_curr_roll_, pos_d_limit_roll_);


            }
          else if(navigator_->getXyControlMode() == Navigator::POS_LOCAL_BASED_CONTROL_MODE)
            {
              //** 座標変換
              d_err_pos_curr_roll_ = target_pos_y - state_pos_y;
              d_err_vel_curr_roll_ = target_vel_y - state_vel_y;

              //**** Pの項
              pos_p_term_roll_ = limit(pos_p_gain_roll_ * d_err_pos_curr_roll_, pos_p_limit_roll_);

              //**** Dの項
              pos_d_term_roll_ = limit(pos_d_gain_roll_ * d_err_vel_curr_roll_, pos_d_limit_roll_);

            }
          else if(navigator_->getXyControlMode() == Navigator::VEL_WORLD_BASED_CONTROL_MODE)
            {
              d_err_vel_curr_roll_ 
                = -(target_vel_x - state_vel_x) * sin(state_psi_cog)
                + (target_vel_y - state_vel_y)  * cos(state_psi_cog);

              //**** Pの項
              pos_p_term_roll_ = limit(vel_p_gain_roll_ * d_err_vel_curr_roll_, pos_d_limit_roll_);
              pos_d_term_roll_ = 0;
            }
          else if(navigator_->getXyControlMode() == Navigator::VEL_LOCAL_BASED_CONTROL_MODE)
            {
              if(navigator_->getFlightMode() == Navigator::TAKEOFF_MODE)
                {// special mode for velocity control
                  if(navigator_->getXyVelModePosCtrlTakeoff())
                    {
                      //**** Pの項
                      d_err_pos_curr_roll_ = target_pos_y - state_pos_y;
                      d_err_vel_curr_roll_ = target_vel_y - state_vel_y;

                      pos_p_term_roll_ = limit(pos_p_gain_roll_ * d_err_pos_curr_roll_,  pos_p_limit_roll_);

                      pos_i_term_roll_ += d_err_pos_curr_roll_ * (1 / (float)roll_ctrl_loop_rate_) * pos_i_gain_roll_;
                      pos_i_term_roll_ = limit(pos_i_term_roll_, pos_i_limit_roll_);

                      pos_d_term_roll_ = limit(pos_d_gain_roll_ * d_err_vel_curr_roll_, pos_d_limit_roll_);
                    }
                  else
                    {
                      d_err_vel_curr_roll_ = target_vel_y - state_vel_y;
                      //**** Pの項
                      pos_p_term_roll_ = limit(vel_p_gain_roll_ * d_err_vel_curr_roll_, pos_d_limit_roll_);
                      //**** Iの項
                      pos_i_term_roll_ = 0;
                      //**** Dの項
                      pos_d_term_roll_ = 0;
                    }

                }
              else
                {
                  d_err_vel_curr_roll_ = target_vel_y - state_vel_y;
                  //**** Pの項
                  pos_p_term_roll_ = limit(vel_p_gain_roll_ * d_err_vel_curr_roll_, pos_d_limit_roll_);


                  //**** Dの項
                  pos_d_term_roll_ = 0;
                }
            }
          //*** 指令値算出
          roll_value = limit(pos_p_term_roll_ + pos_i_term_roll_ + pos_d_term_roll_ + offset_roll_, pos_limit_roll_);
          //**** 指令値反転
          roll_value = - roll_value;

          //**** attitude control mode
          if(navigator_->getXyControlMode() == Navigator::ATT_CONTROL_MODE)
            {
              roll_value = navigator_->getTargetAngleRoll();
              pos_p_term_roll_ =  0;
              pos_i_term_roll_ =  0;
              pos_d_term_roll_ =  0;
            }

          //*** 指令値代入
          flight_ctrl_input_->setRollValue(roll_value);

          //**** ros pub
          four_axis_pid_debug.roll.total.push_back(roll_value);
          four_axis_pid_debug.roll.p_term.push_back(pos_p_term_roll_);
          four_axis_pid_debug.roll.i_term.push_back(pos_i_term_roll_);
          four_axis_pid_debug.roll.d_term.push_back(pos_d_term_roll_);
          four_axis_pid_debug.roll.pos_err_transform = d_err_pos_curr_roll_;
          four_axis_pid_debug.roll.pos_err_no_transform = target_pos_y - state_pos_y;
          four_axis_pid_debug.roll.vel_err_transform = target_vel_y;
          four_axis_pid_debug.roll.vel_err_no_transform = target_vel_y - state_vel_y;

          //yaw => refer to the board frame angle(psi_board)
          //error p
          if(yaw_control_frame_ == Frame::BODY)
            d_err_pos_curr_yaw_ = target_psi - state_psi_board;
          else if(yaw_control_frame_ == Frame::COG)
            d_err_pos_curr_yaw_ = target_psi - state_psi_cog;

          if(d_err_pos_curr_yaw_ > M_PI)  d_err_pos_curr_yaw_ -= 2 * M_PI;
          else if(d_err_pos_curr_yaw_ < -M_PI)  d_err_pos_curr_yaw_ += 2 * M_PI;
          //error i
          error_i_yaw_ += d_err_pos_curr_yaw_ * (1 / (float)yaw_ctrl_loop_rate_);

          for(int j = 0; j < motor_num_; j++)
            {
              //**** P term
              pos_p_term_yaw_ = limit(pos_p_gain_yaw_[j] * (-d_err_pos_curr_yaw_), pos_p_limit_yaw_);
              // if(motor_num_ == 1)
              //   pos_p_term_yaw_ = limit(pos_p_gain_yaw_[j] * , pos_p_limit_yaw_);

              //**** I term: deprecated
              if(motor_num_ == 1)
                error_i_yaw_ = limit(error_i_yaw_, pos_i_limit_yaw_ / pos_i_gain_yaw_[j]);
              pos_i_term_yaw_ = limit(pos_i_gain_yaw_[j] * error_i_yaw_, pos_i_limit_yaw_);

              //***** D term: is in the flight board
              pos_d_term_yaw_ = 0;

              //*** each motor command value for log
              float yaw_value = limit(pos_p_term_yaw_ + pos_i_term_yaw_ + pos_d_term_yaw_, pos_limit_yaw_);

              four_axis_pid_debug.yaw.total.push_back(yaw_value);
              four_axis_pid_debug.yaw.p_term.push_back(pos_p_term_yaw_);
              four_axis_pid_debug.yaw.i_term.push_back(pos_i_term_yaw_);
              four_axis_pid_debug.yaw.d_term.push_back(pos_d_term_yaw_);

              flight_ctrl_input_->setYawValue(yaw_value, j); //f => pwm;
            }

          //**** ros pub
          four_axis_pid_debug.yaw.pos_err_transform = target_psi;
          four_axis_pid_debug.yaw.pos_err_no_transform = d_err_pos_curr_yaw_;
          four_axis_pid_debug.yaw.vel_err_transform = state_vel_psi;
          four_axis_pid_debug.yaw.vel_err_no_transform = state_vel_psi;

          //throttle
          d_err_pos_curr_throttle_ = clamp(target_pos_z - state_pos_z, -pos_err_thresh_, pos_err_thresh_);
          // I term is special in landing mode
          if(navigator_->getFlightMode() == Navigator::LAND_MODE)
            d_err_pos_curr_throttle_ += const_i_ctrl_thre_throttle_land_;
          error_i_throttle_ += d_err_pos_curr_throttle_ * (1 / (float)throttle_ctrl_loop_rate_);
          if(navigator_->getFlightMode() == Navigator::TAKEOFF_MODE ||
             navigator_->getFlightMode() == Navigator::LAND_MODE   ||
             navigator_->getFlightMode() == Navigator::FLIGHT_MODE)
            {
              for(int j = 0; j < motor_num_; j++)
                {
                  //**** P Term (+ feed forward term)
                  pos_p_term_throttle_ = limit(pos_p_gain_throttle_[j] * (-d_err_pos_curr_throttle_), pos_p_limit_throttle_); // the err is reversed
                  //**** I Term
                  pos_i_term_throttle_ = limit(pos_i_gain_throttle_[j] * error_i_throttle_, pos_i_limit_throttle_);
                  //***** D Term
                  pos_d_term_throttle_ = limit(pos_d_gain_throttle_[j] * state_vel_z, pos_d_limit_throttle_);

                  if(navigator_->getFlightMode() == Navigator::LAND_MODE)
                    pos_p_term_throttle_ = 0;

                  //*** each motor command value for log
                  float throttle_value = limit(pos_p_term_throttle_ + pos_i_term_throttle_ + pos_d_term_throttle_ + offset_throttle_, pos_limit_throttle_);
                  four_axis_pid_debug.throttle.total.push_back(throttle_value);
                  four_axis_pid_debug.throttle.p_term.push_back(pos_p_term_throttle_);
                  four_axis_pid_debug.throttle.i_term.push_back(pos_i_term_throttle_);
                  four_axis_pid_debug.throttle.d_term.push_back(pos_d_term_throttle_);

                  //uint16_t throttle_value_16bit = (throttle_value - f_pwm_offset_) / f_pwm_rate_ * pwm_rate_;
                  flight_ctrl_input_->setThrottleValue(throttle_value, j);
                }
            }
          four_axis_pid_debug.throttle.pos_err_transform = target_pos_z;
          four_axis_pid_debug.throttle.pos_err_no_transform = d_err_pos_curr_throttle_;
          four_axis_pid_debug.throttle.vel_err_transform = state_vel_z;
          four_axis_pid_debug.throttle.vel_err_no_transform = state_vel_z;

          pid_pub_.publish(four_axis_pid_debug);

          if(feedforward_flag_)
            {
              std_msgs::Float32MultiArray ff_msg;

              //roll & pitch
              // ref set
              Eigen::Vector2d r;
              r << roll_value, pitch_value;

              // feedfowd input
              Eigen::VectorXd u_ff = feedforward_matrix_ * r;

              //std::vector<int16_t> u_ff_pwm;
              std::vector<float> u_ff_vec;
              u_ff_vec.resize(0);
              for(int i = 0; i < u_ff.rows(); i++)
                {
                  u_ff_vec.push_back(u_ff(i));
                  ff_msg.data.push_back(u_ff(i));
                }
              //add to throttle
              bool force_add_flag = !start_rp_integration_; //takeoff sign
              flight_ctrl_input_->addFFValues(u_ff_vec, force_add_flag);
              ff_pub_.publish(ff_msg);
            }
        }

      //*** Update
      d_err_vel_prev_pitch_ =  d_err_vel_curr_pitch_;
      d_err_vel_prev_roll_ =  d_err_vel_curr_roll_;
      d_err_vel_prev_throttle_ = d_err_vel_curr_throttle_;
      d_err_vel_prev_yaw_ =  d_err_vel_curr_yaw_;

      return;
    }
}

void PidController::feedForwardFunction()
{
  //do nothing
}


void PidController::cfgXYPidCallback(aerial_robot_base::XYPidControlConfig &config, uint32_t level)
{
  if(config.xy_pid_control_flag)
    {
      printf("XY Pid Param:");
      switch(level)
        {
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_CONTROL_LOOP_RATE:
          roll_ctrl_loop_rate_ = config.ctrl_loop_rate;
          pitch_ctrl_loop_rate_ = config.ctrl_loop_rate;
          printf("change the control loop rate\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_P_GAIN:
          pos_p_gain_roll_ = config.pos_p_gain;
          pos_p_gain_pitch_ = config.pos_p_gain;
          printf("change the pos p gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_I_GAIN:
          pos_i_gain_roll_ = config.pos_i_gain;
          pos_i_gain_pitch_ = config.pos_i_gain;
          printf("change the pos i gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_I_GAIN_HOVER:
          pos_i_gain_roll_hover_ = config.pos_i_gain_hover;
          pos_i_gain_pitch_hover_ = config.pos_i_gain_hover;
          printf("change the pos i_hover gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_D_GAIN:
          pos_d_gain_roll_ = config.pos_d_gain;
          pos_d_gain_pitch_ = config.pos_d_gain;
          printf("change the pos d gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_VEL_P_GAIN:
          vel_p_gain_roll_ = config.vel_p_gain;
          vel_p_gain_pitch_ = config.vel_p_gain;
          printf("change the vel p gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_VEL_I_GAIN:
          vel_i_gain_roll_ = config.vel_i_gain;
          printf("change the vel i gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_OFFSET:
          //bad!!
          offset_roll_ = config.offset;
          offset_pitch_ = config.offset;
          printf("change the offset\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_LIMIT:
          pos_limit_roll_ = config.pos_limit;
          pos_limit_pitch_ = config.pos_limit;
          printf("change the limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_P_LIMIT:
          pos_p_limit_roll_ = config.pos_p_limit;
          pos_p_limit_pitch_ = config.pos_p_limit;
          printf("change the p limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_I_LIMIT:
          pos_i_limit_roll_ = config.pos_i_limit;
          pos_i_limit_pitch_ = config.pos_i_limit;
          printf("change the i limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_D_LIMIT:
          pos_d_limit_roll_ = config.pos_d_limit;
          pos_d_limit_pitch_ = config.pos_d_limit;
          printf("change the d limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_ENABLE_LIMIT_HOVER:
          i_enable_limit_roll_ = config.i_enable_limit;
          i_enable_limit_pitch_ = config.i_enable_limit;
          printf("change the i enable limit\n");
          break;
        default :
          printf("\n");
          break;
        }
    }
}


void PidController::rosParamInit(ros::NodeHandle nh)
{
  ros::NodeHandle pitch_node(nh, "pitch");
  ros::NodeHandle roll_node(nh, "roll");
  ros::NodeHandle throttle_node(nh, "throttle");
  ros::NodeHandle yaw_node(nh, "yaw");

  std::string pitch_ns = pitch_node.getNamespace();
  std::string roll_ns = roll_node.getNamespace();
  std::string throttle_ns = throttle_node.getNamespace();
  std::string yaw_ns = yaw_node.getNamespace();

  //**** throttle
  if (!throttle_node.getParam ("ctrl_loop_rate", throttle_ctrl_loop_rate_))
    throttle_ctrl_loop_rate_ = 0;
  printf("%s: ctrl_loop_rate_ is %d\n", throttle_ns.c_str(), throttle_ctrl_loop_rate_);

  if (!throttle_node.getParam ("const_i_ctrl_thre_land",  const_i_ctrl_thre_throttle_land_))
    const_i_ctrl_thre_throttle_land_ = 0;
  printf("%s: const_i_ctrl_thre_land_ is %.3f\n", throttle_ns.c_str(), const_i_ctrl_thre_throttle_land_);

  if (!throttle_node.getParam ("offset", offset_throttle_))
    offset_throttle_ = 0;
  printf("%s: offset_ is %f\n", throttle_ns.c_str(), offset_throttle_);

  if (!throttle_node.getParam ("pos_limit", pos_limit_throttle_))
    pos_limit_throttle_ = 0;
  printf("%s: pos_limit_ is %d\n", throttle_ns.c_str(), pos_limit_throttle_);

  if (!throttle_node.getParam ("pos_p_limit", pos_p_limit_throttle_))
    pos_p_limit_throttle_ = 0;
  printf("%s: pos_p_limit_ is %d\n", throttle_ns.c_str(), pos_p_limit_throttle_);

  if (!throttle_node.getParam ("pos_i_limit", pos_i_limit_throttle_))
    pos_i_limit_throttle_ = 0;
  printf("%s: pos_i_limit_ is %d\n", throttle_ns.c_str(), pos_i_limit_throttle_);

  if (!throttle_node.getParam ("pos_d_limit", pos_d_limit_throttle_))
    pos_d_limit_throttle_ = 0;
  printf("%s: pos_d_limit_ is %d\n", throttle_ns.c_str(), pos_d_limit_throttle_);

  if (!throttle_node.getParam ("pos_err_thresh", pos_err_thresh_))
    pos_err_thresh_ = 1.0;
  printf("%s: pos_err_thresh_ is %f\n", throttle_ns.c_str(), pos_err_thresh_);


  //**** pitch
  if (!pitch_node.getParam ("ctrl_loop_rate", pitch_ctrl_loop_rate_))
    pitch_ctrl_loop_rate_ = 0;
  printf("%s: ctrl_loop_rate_ is %d\n", pitch_ns.c_str(), pitch_ctrl_loop_rate_);

  if (!pitch_node.getParam ("pos_p_gain", pos_p_gain_pitch_))
    pos_p_gain_pitch_ = 0;
  printf("%s: pos_p_gain_ is %.3f\n", pitch_ns.c_str(), pos_p_gain_pitch_);

  if (!pitch_node.getParam ("pos_i_gain", pos_i_gain_pitch_))
    pos_i_gain_pitch_ = 0;
  printf("%s: pos_i_gain_ is %.3f\n", pitch_ns.c_str(), pos_i_gain_pitch_);

  if (!pitch_node.getParam ("pos_i_gain_hover", pos_i_gain_pitch_hover_))
    pos_i_gain_pitch_hover_ = 0;
  printf("%s: pos_i_gain_hover_ is %.3f\n", pitch_ns.c_str(), pos_i_gain_pitch_hover_);

  if (!pitch_node.getParam ("pos_d_gain", pos_d_gain_pitch_))
    pos_d_gain_pitch_ = 0;
  printf("%s: pos_d_gain_ is %.3f\n", pitch_ns.c_str(), pos_d_gain_pitch_);

  if (!pitch_node.getParam ("vel_p_gain", vel_p_gain_pitch_))
    vel_p_gain_pitch_ = 0;
  printf("%s: vel_p_gain_ is %.3f\n", pitch_ns.c_str(), vel_p_gain_pitch_);

  if (!pitch_node.getParam ("vel_i_gain", vel_i_gain_pitch_))
    vel_i_gain_pitch_ = 0;
  printf("%s: vel_i_gain_ is %.3f\n", pitch_ns.c_str(), vel_i_gain_pitch_);


  if (!pitch_node.getParam ("offset", offset_pitch_))
    offset_pitch_ = 0;
  printf("%s: offset_ is %f\n", pitch_ns.c_str(), offset_pitch_);

  if (!pitch_node.getParam ("pos_limit", pos_limit_pitch_))
    pos_limit_pitch_ = 0;
  printf("%s: pos_limit_ is %f\n", pitch_ns.c_str(), pos_limit_pitch_);

  if (!pitch_node.getParam ("pos_p_limit", pos_p_limit_pitch_))
    pos_p_limit_pitch_ = 0;
  printf("%s: pos_p_limit_ is %f\n", pitch_ns.c_str(), pos_p_limit_pitch_);

  if (!pitch_node.getParam ("pos_i_limit", pos_i_limit_pitch_))
    pos_i_limit_pitch_ = 0;
  printf("%s: pos_i_limit_ is %f\n", pitch_ns.c_str(), pos_i_limit_pitch_);

  if (!pitch_node.getParam ("pos_d_limit", pos_d_limit_pitch_))
    pos_d_limit_pitch_ = 0;
  printf("%s: pos_d_limit_ is %f\n", pitch_ns.c_str(), pos_d_limit_pitch_);

  //***** roll
  if (!roll_node.getParam ("ctrl_loop_rate", roll_ctrl_loop_rate_))
    roll_ctrl_loop_rate_ = 0;
  printf("%s: ctrl_loop_rate_ is %d\n", roll_ns.c_str(), roll_ctrl_loop_rate_);

  if (!roll_node.getParam ("pos_p_gain", pos_p_gain_roll_))
    pos_p_gain_roll_ = 0;
  printf("%s: pos_p_gain_ is %.3f\n", roll_ns.c_str(), pos_p_gain_roll_);

  if (!roll_node.getParam ("pos_i_gain", pos_i_gain_roll_))
    pos_i_gain_roll_ = 0;
  printf("%s: pos_i_gain_ is %.3f\n", roll_ns.c_str(), pos_i_gain_roll_);

  if (!roll_node.getParam ("pos_i_gain_hover", pos_i_gain_roll_hover_))
    pos_i_gain_roll_hover_ = 0;
  printf("%s: pos_i_gain_hover_ is %.3f\n", roll_ns.c_str(), pos_i_gain_roll_hover_);

  if (!roll_node.getParam ("pos_d_gain", pos_d_gain_roll_))
    pos_d_gain_roll_ = 0;
  printf("%s: pos_d_gain_ is %.3f\n", roll_ns.c_str(), pos_d_gain_roll_);

  if (!roll_node.getParam ("vel_p_gain", vel_p_gain_roll_))
    vel_p_gain_roll_ = 0;
  printf("%s: vel_p_gain_ is %.3f\n", roll_ns.c_str(), vel_p_gain_roll_);

  if (!roll_node.getParam ("vel_i_gain", vel_i_gain_roll_))
    vel_i_gain_roll_ = 0;
  printf("%s: vel_i_gain_ is %.3f\n", roll_ns.c_str(), vel_i_gain_roll_);

  if (!roll_node.getParam ("offset", offset_roll_))
    offset_roll_ = 0;
  printf("%s: offset_ is %f\n", roll_ns.c_str(), offset_roll_);

  if (!roll_node.getParam ("pos_limit", pos_limit_roll_))
    pos_limit_roll_ = 0;
  printf("%s: pos_limit_ is %f\n", roll_ns.c_str(), pos_limit_roll_);

  if (!roll_node.getParam ("pos_p_limit", pos_p_limit_roll_))
    pos_p_limit_roll_ = 0;
  printf("%s: pos_p_limit_ is %f\n", roll_ns.c_str(), pos_p_limit_roll_);

  if (!roll_node.getParam ("pos_i_limit", pos_i_limit_roll_))
    pos_i_limit_roll_ = 0;
  printf("%s: pos_i_limit_ is %f\n", roll_ns.c_str(), pos_i_limit_roll_);

  if (!roll_node.getParam ("pos_d_limit", pos_d_limit_roll_))
    pos_d_limit_roll_ = 0;
  printf("%s: pos_d_limit_ is %f\n", roll_ns.c_str(), pos_d_limit_roll_);

  //**** pitch & roll
  if (!nh.getParam ("xy_vel_weak_gain_sub_name", xy_vel_weak_gain_sub_name_))
    xy_vel_weak_gain_sub_name_ = std::string("xy_vel_weak_gain"); //20%
  printf("xy_vel_weak_gain_sub_name_ is %s\n", xy_vel_weak_gain_sub_name_.c_str());
  if (!nh.getParam ("xy_vel_weak_rate", xy_vel_weak_rate_))
    xy_vel_weak_rate_ = 0.2; //20%
  printf("xy_vel_weak_rate_ is %f\n", xy_vel_weak_rate_);

  //**** yaw
  if (!yaw_node.getParam ("ctrl_loop_rate", yaw_ctrl_loop_rate_))
    yaw_ctrl_loop_rate_ = 0;
  printf("%s: ctrl_loop_rate_ is %d\n", yaw_ns.c_str(), yaw_ctrl_loop_rate_);
  if (!yaw_node.getParam ("pos_limit", pos_limit_yaw_))
    pos_limit_yaw_ = 0;
  printf("%s: pos_limit_ is %d\n", yaw_ns.c_str(), pos_limit_yaw_);

  if (!yaw_node.getParam ("pos_p_limit", pos_p_limit_yaw_))
    pos_p_limit_yaw_ = 0;
  printf("%s: pos_p_limit_ is %d\n", yaw_ns.c_str(), pos_p_limit_yaw_);

  if (!yaw_node.getParam ("pos_i_limit", pos_i_limit_yaw_))
    pos_i_limit_yaw_ = 0;
  printf("%s: pos_i_limit_ is %d\n", yaw_ns.c_str(), pos_i_limit_yaw_);

  if (!yaw_node.getParam ("pos_d_limit", pos_d_limit_yaw_))
    pos_d_limit_yaw_ = 0;
  printf("%s: pos_d_limit_ is %d\n", yaw_ns.c_str(), pos_d_limit_yaw_);

  if (!yaw_node.getParam ("yaw_control_frame", yaw_control_frame_))
    yaw_control_frame_ = Frame::BODY;
  printf("%s: yaw_control_frame_ is %d\n", yaw_ns.c_str(), yaw_control_frame_);

  pos_p_gain_yaw_.resize(motor_num_);
  pos_i_gain_yaw_.resize(motor_num_);
  pos_d_gain_yaw_.resize(motor_num_);

  pos_p_gain_throttle_.resize(motor_num_);
  pos_i_gain_throttle_.resize(motor_num_);
  pos_d_gain_throttle_.resize(motor_num_);


  if(motor_num_ == 1)//general multirot
    {
      if (!throttle_node.getParam ("pos_p_gain", pos_p_gain_throttle_[0]))
        pos_p_gain_throttle_[0] = 0;
      printf("%s: pos_p_gain_ is %.3f\n", throttle_ns.c_str(), pos_p_gain_throttle_[0]);

      if (!throttle_node.getParam ("pos_i_gain", pos_i_gain_throttle_[0]))
        pos_i_gain_throttle_[0] = 0;
      printf("%s: pos_i_gain_ is %.3f\n", throttle_ns.c_str(), pos_i_gain_throttle_[0]);

      if (!throttle_node.getParam ("pos_d_gain", pos_d_gain_throttle_[0]))
        pos_d_gain_throttle_[0] = 0;
      printf("%s: pos_d_gain_ is %.3f\n", throttle_ns.c_str(), pos_d_gain_throttle_[0]);

      if (!yaw_node.getParam ("pos_p_gain", pos_p_gain_yaw_[0]))
        pos_p_gain_yaw_[0] = 0;
      printf("%s: pos_p_gain_ is %.3f\n", yaw_ns.c_str(), pos_p_gain_yaw_[0]);

      if (!yaw_node.getParam ("pos_i_gain", pos_i_gain_yaw_[0]))
        pos_i_gain_yaw_[0] = 0;
      printf("%s: pos_i_gain_ is %.3f\n", yaw_ns.c_str(), pos_i_gain_yaw_[0]);

      if (!yaw_node.getParam ("pos_d_gain", pos_d_gain_yaw_[0]))
        pos_d_gain_yaw_[0] = 0;
      printf("%s: pos_d_gain_ is %.3f\n", yaw_ns.c_str(), pos_d_gain_yaw_[0]);
    }
  else
    {//transformable
      for(int i = 0; i < motor_num_; i++)
        {
          pos_p_gain_yaw_[i] = 0;
          pos_i_gain_yaw_[i] = 0;
          pos_d_gain_yaw_[i] = 0;

          pos_p_gain_throttle_[i] = 0;
          pos_i_gain_throttle_[i] = 0;
          pos_d_gain_throttle_[i] = 0;
        }
    }
}
