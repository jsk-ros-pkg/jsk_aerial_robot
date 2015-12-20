/*
I_y * dot(w_y) = - x * F => Q (符号の話)
varphi_des = - 1/g * y_des => (roll軸制御　反転の話) 
 */

#include "aerial_robot_base/flight_control.h"

FlightController::FlightController(ros::NodeHandle nh,
                                   ros::NodeHandle nh_private,
                                   BasicEstimator* estimator, Navigator* navigator, 
                                   FlightCtrlInput* flight_ctrl_input): nh_(nh, "controller"), nhp_(nh_private, "controller")
{
  estimator_ = estimator;
  navigator_ = navigator;
  flight_ctrl_input_ = flight_ctrl_input;

  //normal namespace
  // if (!nh_private.getParam ("motor_num", motor_num_))
  //   motor_num_ = 4;
  motor_num_ = flight_ctrl_input_->getMotorNumber();
  printf("motor_num_ is %d\n", motor_num_);
  //controller namespace
  if (!nhp_.getParam ("f_pwm_rate", f_pwm_rate_))
    f_pwm_rate_ = 1; //0.3029;
  printf("f_pwm_rate_ is %f\n", f_pwm_rate_);

  if (!nhp_.getParam ("f_pwm_offset", f_pwm_offset_))
    f_pwm_offset_ = 0; //-21.196;
  printf("f_pwm_offset_ is %f\n", f_pwm_offset_);

  if (!nhp_.getParam ("pwm_rate", pwm_rate_))
    pwm_rate_ = 1; //1800/100.0;
  printf("pwm_rate_ is %f\n", pwm_rate_);

  if (!nhp_.getParam ("feedforward_flag", feedforward_flag_))
    feedforward_flag_ = true;
  printf("feedforward_flag_ is %s\n", (feedforward_flag_)?"true":"false");

}

FlightController::~FlightController()
{
}

float FlightController::limit(float value, int limit)
{
  if(value > (float)limit) { return (float)limit;}
  else if(value < - (float)limit) { return -(float)limit; }
  else return value;
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


  feedforward_matrix_ = Eigen::MatrixXd::Zero(motor_num_, 3);

  //roll/pitch integration start
  start_rp_integration_ = false;

  //publish
  pid_pub_ = nh_.advertise<aerial_robot_base::FourAxisPid>("debug", 10); 
  ff_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("ff", 10); 

  //subscriber
  four_axis_gain_sub_ = nh_.subscribe<aerial_robot_base::YawThrottleGain>("yaw_throttle_gain", 1, &PidController::yawThrottleGainCallback, this, ros::TransportHints().tcpNoDelay());


  //dynamic reconfigure server
  xy_pid_server_ = new dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>(ros::NodeHandle(nhp_, "pitch"));
  dynamic_reconf_func_xy_pid_ = boost::bind(&PidController::cfgXYPidCallback, this, _1, _2);
  xy_pid_server_->setCallback(dynamic_reconf_func_xy_pid_);

}

PidController::~PidController()
{
  printf(" deleted pid flight control input!\n");
}

void PidController::yawThrottleGainCallback(const aerial_robot_base::YawThrottleGainConstPtr & msg)
{
  if(msg->motor_num != motor_num_) 
    {
      ROS_ERROR("the motor number is not correct, msg->motor_num:%d, motor_num_:%d", msg->motor_num, motor_num_);
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

      feedforward_matrix_(i, 0) = msg->roll_vec[i];
      feedforward_matrix_(i, 1) = msg->pitch_vec[i];
      feedforward_matrix_(i, 2) = msg->yaw_vec[i];
    }
  //debug 
  //std::cout << "ff matrix :\n" << feedforward_matrix_ << std::endl;
}

void PidController::pidFunction()
{
  static bool first_flag = true;

  //*** リセット
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
      return;
    }
  else
    {
      float state_pos_x;
      float state_vel_x;
      float state_pos_y;
      float state_vel_y;

      float target_pos_x;
      float target_pos_y;
      float target_vel_x;
      float target_vel_y;

      if(navigator_->getXyControlMode() == Navigator::POS_WORLD_BASED_CONTROL_MODE ||
         navigator_->getXyControlMode() == Navigator::VEL_WORLD_BASED_CONTROL_MODE) 
        {
          state_pos_x = estimator_->getStatePosX();
          state_vel_x = estimator_->getStateVelX();
          state_pos_y = estimator_->getStatePosY();
          state_vel_y = estimator_->getStateVelY();
        }
      else if(navigator_->getXyControlMode() == navigator_->POS_LOCAL_BASED_CONTROL_MODE)
        {
          state_pos_x = estimator_->getStatePosX();
          state_pos_y = estimator_->getStatePosY();
          state_vel_x = estimator_->getStateVelXc();
          state_vel_y = estimator_->getStateVelYc();
        }
      else if(navigator_->getXyControlMode() == navigator_->VEL_LOCAL_BASED_CONTROL_MODE)
        {
          state_pos_x = estimator_->getStatePosXc();
          state_pos_y = estimator_->getStatePosYc();
          state_vel_x = estimator_->getStateVelXc();
          state_vel_y = estimator_->getStateVelYc();
        }
      else
        {
          ROS_ERROR("wrong xy control mode ");
          state_pos_x = 0;
          state_vel_x = 0;
          state_pos_y = 0;
          state_vel_y = 0;
        }

      target_vel_x = navigator_->getTargetVelX();
      target_vel_y = navigator_->getTargetVelY();
      target_pos_x = navigator_->getTargetPosX();
      target_pos_y = navigator_->getTargetPosY();

      float state_pos_z = estimator_->getStatePosZ();
      float state_vel_z = estimator_->getStateVelZ();

      float state_psi_cog = estimator_->getStatePsiCog();

      float state_psi_board = estimator_->getStatePsiBoard();
      float state_vel_psi_board = estimator_->getStateVelPsiBoard();

      float target_pos_z = navigator_->getTargetPosZ();
      float target_psi = navigator_->getTargetPsi();

      aerial_robot_base::FourAxisPid  four_axis_pid_debug;

      four_axis_pid_debug.header.stamp = ros::Time::now();

      if (first_flag) 
	first_flag = false;
      else
	{
          float roll_value = 0, pitch_value = 0;

          //roll/pitch integration flag
          if(!start_rp_integration_)
            {
              if(state_pos_z > 0.01) 
                {
                  start_rp_integration_ = true;
                  std_msgs::UInt16 integration_cmd;
                  integration_cmd.data = navigator_->ROS_INTEGRATE_CMD;
                  navigator_->config_cmd_pub_.publish(integration_cmd);
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
              d_err_pos_curr_pitch_ 
                = (target_pos_x - state_pos_x) * cos(state_psi_cog)
                + (target_pos_y - state_pos_y) * sin(state_psi_cog);

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
          else
            ROS_ERROR("wrong control mode");


          //*** 指令値算出
          pitch_value = limit(pos_p_term_pitch_ + pos_i_term_pitch_ + pos_d_term_pitch_ + offset_pitch_, pos_limit_pitch_);

          //**** attitude gain tunnign mode
          if(navigator_->getGainTunningMode() == Navigator::ATTITUDE_GAIN_TUNNING_MODE)
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
              d_err_pos_curr_roll_
                = - (target_pos_x - state_pos_x) * sin(state_psi_cog)
                + (target_pos_y - state_pos_y) * cos(state_psi_cog);

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
          else
            ROS_ERROR("wrong control mode");

          //*** 指令値算出
          roll_value = limit(pos_p_term_roll_ + pos_i_term_roll_ + pos_d_term_roll_ + offset_roll_, pos_limit_roll_);
          //**** 指令値反転
          roll_value = - roll_value;

          //**** attitude gain tunnign mode
          if(navigator_->getGainTunningMode() == Navigator::ATTITUDE_GAIN_TUNNING_MODE)
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
          d_err_pos_curr_yaw_ = target_psi - state_psi_board;
          if(d_err_pos_curr_yaw_ > M_PI)  d_err_pos_curr_yaw_ -= 2 * M_PI; 
          else if(d_err_pos_curr_yaw_ < -M_PI)  d_err_pos_curr_yaw_ += 2 * M_PI;
          //error i
          error_i_yaw_ += d_err_pos_curr_yaw_ * (1 / (float)yaw_ctrl_loop_rate_); 

          if(navigator_->getXyControlMode() == Navigator::POS_WORLD_BASED_CONTROL_MODE ||
             navigator_->getXyControlMode() == Navigator::POS_LOCAL_BASED_CONTROL_MODE || 
             navigator_->getXyControlMode() == Navigator::VEL_WORLD_BASED_CONTROL_MODE ||
             navigator_->getXyControlMode() == Navigator::VEL_LOCAL_BASED_CONTROL_MODE)
            {

              for(int j = 0; j < motor_num_; j++)
                {
                  //**** Pの項
                  pos_p_term_yaw_ = limit(pos_p_gain_yaw_[j] * state_psi_board, pos_p_limit_yaw_);
                  //**** Iの項 : deprecated
                  pos_i_term_yaw_ = limit(pos_i_gain_yaw_[j] * error_i_yaw_, pos_i_limit_yaw_);
                  //***** Dの項 // to think about d term, now in kduino controller
                  pos_d_term_yaw_ = 0;

                  if(motor_num_ == 1)
                    pos_p_term_yaw_ = limit(pos_p_gain_yaw_[j] * d_err_pos_curr_yaw_, pos_p_limit_yaw_);

                  //*** each motor command value for log
                  float yaw_value = limit(pos_p_term_yaw_ + pos_i_term_yaw_ + pos_d_term_yaw_, pos_limit_yaw_);

                  //**** attitude gain tunnign mode
                  if(navigator_->getGainTunningMode() == Navigator::ATTITUDE_GAIN_TUNNING_MODE)
                    {
                      yaw_value = limit(pos_p_gain_yaw_[j] * target_psi, pos_p_limit_yaw_);
                      pos_p_term_yaw_ =  0;
                      pos_i_term_yaw_ =  0;
                      pos_d_term_yaw_ =  0;
                    }


                  four_axis_pid_debug.yaw.total.push_back(yaw_value);
                  four_axis_pid_debug.yaw.p_term.push_back(pos_p_term_yaw_);
                  four_axis_pid_debug.yaw.i_term.push_back(pos_i_term_yaw_);
                  four_axis_pid_debug.yaw.d_term.push_back(pos_d_term_yaw_);

                  //*** 指令値代入(new*** )
                  //flight_ctrl_input_->setYawValue(yaw_value / f_pwm_rate_ * pwm_rate_ * 10000, j); //f => pwm; x10000 //2015.12.20
                  flight_ctrl_input_->setYawValue(yaw_value / f_pwm_rate_ * pwm_rate_, j); //f => pwm;
                }
            }

          //**** ros pub
          four_axis_pid_debug.yaw.pos_err_transform = target_psi;
          four_axis_pid_debug.yaw.pos_err_no_transform = d_err_pos_curr_yaw_;
          four_axis_pid_debug.yaw.vel_err_transform = state_vel_psi_board;
          four_axis_pid_debug.yaw.vel_err_no_transform = state_vel_psi_board;

          //throttle
          d_err_pos_curr_throttle_ = target_pos_z - state_pos_z;
          // land時,iterm は特殊
          if(navigator_->getFlightMode() == Navigator::LAND_MODE)
            d_err_pos_curr_throttle_ += const_i_ctrl_thre_throttle_land_;
          error_i_throttle_ += d_err_pos_curr_throttle_ * (1 / (float)throttle_ctrl_loop_rate_);
          if(navigator_->getFlightMode() == Navigator::TAKEOFF_MODE ||
             navigator_->getFlightMode() == Navigator::LAND_MODE   ||
             navigator_->getFlightMode() == Navigator::FLIGHT_MODE)
            {
              for(int j = 0; j < motor_num_; j++)
                {
                  //**** Pの項
                  pos_p_term_throttle_ = limit(pos_p_gain_throttle_[j] * state_pos_z, pos_p_limit_throttle_);
                  //**** Iの項
                  pos_i_term_throttle_ = limit(pos_i_gain_throttle_[j] * error_i_throttle_, pos_i_limit_throttle_);
                  //***** Dの項
                  pos_d_term_throttle_ = limit(pos_d_gain_throttle_[j] * state_vel_z, pos_d_limit_throttle_);

                  if(motor_num_ == 1)
                    {
                      pos_p_term_throttle_ = limit(pos_p_gain_throttle_[j] * d_err_pos_curr_throttle_, pos_p_limit_throttle_); //P term for pid
                      pos_d_term_throttle_ = limit(-pos_d_gain_throttle_[j] * state_vel_z, pos_d_limit_throttle_);

                      if(navigator_->getFlightMode() == Navigator::LAND_MODE)
                        {
                          //pos_p_term_throttle_ = limit(pos_p_gain_throttle_[j] * land_gain_slow_rate_ *  d_err_pos_curr_throttle_, pos_p_limit_throttle_); //half of the gain
                          //pos_d_term_throttle_ = limit(-pos_d_gain_throttle_[j] / land_gain_slow_rate_ * state_vel_z, pos_d_limit_throttle_); //twice
                          pos_p_term_throttle_ = 0;

                        }
                    }

                  //*** each motor command value for log
                  float throttle_value = limit(pos_p_term_throttle_ + pos_i_term_throttle_ + pos_d_term_throttle_ + offset_throttle_, pos_limit_throttle_);
                  four_axis_pid_debug.throttle.total.push_back(throttle_value);
                  four_axis_pid_debug.throttle.p_term.push_back(pos_p_term_throttle_);
                  four_axis_pid_debug.throttle.i_term.push_back(pos_i_term_throttle_);
                  four_axis_pid_debug.throttle.d_term.push_back(pos_d_term_throttle_);

                  //*** 指令値代入
                  uint16_t throttle_value_16bit = (throttle_value - f_pwm_offset_) / f_pwm_rate_ * pwm_rate_;
                  flight_ctrl_input_->setThrottleValue(throttle_value_16bit, j);
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

              //roll & pitch : 0.1deg, yaw: rad
              // ref set
              Eigen::Vector3d r;
              r << roll_value / 10 /  180  * M_PI, pitch_value / 10 / 180  * M_PI, target_psi;

              // feedfowd input
              Eigen::VectorXd u_ff = feedforward_matrix_  * r;
              //debug
              //std::cout << "u_ff :\n" << u_ff << std::endl;
              std::vector<int16_t> u_ff_pwm;
              u_ff_pwm.resize(0);
              for(int i = 0; i < u_ff.rows(); i++)
                {
                  u_ff_pwm.push_back( u_ff(i) / f_pwm_rate_ * pwm_rate_);
                  ff_msg.data.push_back(u_ff(i));
                }

              //add to throttle
              flight_ctrl_input_->addFFValues(u_ff_pwm);

              ff_pub_.publish(ff_msg);
            }
	}

      //*** 更新
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
  printf("%s: offset_ is %d\n", pitch_ns.c_str(), offset_pitch_);

  if (!pitch_node.getParam ("pos_limit", pos_limit_pitch_))
    pos_limit_pitch_ = 0;
  printf("%s: pos_limit_ is %d\n", pitch_ns.c_str(), pos_limit_pitch_);

  if (!pitch_node.getParam ("pos_p_limit", pos_p_limit_pitch_))
    pos_p_limit_pitch_ = 0;
  printf("%s: pos_p_limit_ is %d\n", pitch_ns.c_str(), pos_p_limit_pitch_);

  if (!pitch_node.getParam ("pos_i_limit", pos_i_limit_pitch_))
    pos_i_limit_pitch_ = 0;
  printf("%s: pos_i_limit_ is %d\n", pitch_ns.c_str(), pos_i_limit_pitch_);

  if (!pitch_node.getParam ("pos_d_limit", pos_d_limit_pitch_))
    pos_d_limit_pitch_ = 0;
  printf("%s: pos_d_limit_ is %d\n", pitch_ns.c_str(), pos_d_limit_pitch_);

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
  printf("%s: offset_ is %d\n", roll_ns.c_str(), offset_roll_);

  if (!roll_node.getParam ("pos_limit", pos_limit_roll_))
    pos_limit_roll_ = 0;
  printf("%s: pos_limit_ is %d\n", roll_ns.c_str(), pos_limit_roll_);

  if (!roll_node.getParam ("pos_p_limit", pos_p_limit_roll_))
    pos_p_limit_roll_ = 0;
  printf("%s: pos_p_limit_ is %d\n", roll_ns.c_str(), pos_p_limit_roll_);

  if (!roll_node.getParam ("pos_i_limit", pos_i_limit_roll_))
    pos_i_limit_roll_ = 0;
  printf("%s: pos_i_limit_ is %d\n", roll_ns.c_str(), pos_i_limit_roll_);

  if (!roll_node.getParam ("pos_d_limit", pos_d_limit_roll_))
    pos_d_limit_roll_ = 0;
  printf("%s: pos_d_limit_ is %d\n", roll_ns.c_str(), pos_d_limit_roll_);

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

  pos_p_gain_yaw_.resize(motor_num_);
  pos_i_gain_yaw_.resize(motor_num_);
  pos_d_gain_yaw_.resize(motor_num_);

  pos_p_gain_throttle_.resize(motor_num_);
  pos_i_gain_throttle_.resize(motor_num_);
  pos_d_gain_throttle_.resize(motor_num_);


  if(motor_num_ == 1)//general multirot
    {
      if (!nh.getParam ("land_gain_slow_rate", land_gain_slow_rate_))
        land_gain_slow_rate_ = 1.0;
      printf(" land_gain_slow_rate_ is %.3f\n", land_gain_slow_rate_);

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
