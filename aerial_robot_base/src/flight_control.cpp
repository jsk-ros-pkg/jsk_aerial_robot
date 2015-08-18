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
  flight_ctrl_input_= flight_ctrl_input;
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

  //publish
 pid_pub_ = nh_.advertise<aerial_robot_base::FourAxisPid>("debug", 10); 

  //subscribe
  motor_bias_flag_ = false;
  motor_bias_set_pub_ = nh_.advertise<aerial_robot_msgs::ITermBias>("/kduino/i_term_bias", 2);
  //deprecated
  motor_bias_set_sub_ = nh_.subscribe<std_msgs::Int8>("/teleop_command/motor_bais_set", 1, &PidController::motorBiasSetCallback, this, ros::TransportHints().tcpNoDelay());
  rpy_ctrl_offset_sub_ = nh_.subscribe<aerial_robot_msgs::RPYCtrlOffset>("/flight_control/rpy_ctrl_offset", 1, &PidController::rpyCtrlOffsetCallback, this, ros::TransportHints().tcpNoDelay());

  //dynamic reconfigure server
  pitch_server_ = new dynamic_reconfigure::Server<aerial_robot_base::PidPitchControlConfig>(ros::NodeHandle(nhp_, "pitch"));
  dynamic_reconf_func_pitch_ = boost::bind(&PidController::cfgPitchCallback, this, _1, _2);
  pitch_server_->setCallback(dynamic_reconf_func_pitch_);

  roll_server_ = new dynamic_reconfigure::Server<aerial_robot_base::PidRollControlConfig>(ros::NodeHandle(nhp_,"roll"));
  dynamic_reconf_func_roll_ = boost::bind(&PidController::cfgRollCallback, this, _1, _2);
  roll_server_->setCallback(dynamic_reconf_func_roll_);

  yaw_server_ = new dynamic_reconfigure::Server<aerial_robot_base::PidYawControlConfig>(ros::NodeHandle(nhp_, "yaw"));
  dynamic_reconf_func_yaw_ = boost::bind(&PidController::cfgYawCallback, this, _1, _2);
  yaw_server_->setCallback(dynamic_reconf_func_yaw_);

  throttle_server_ = new dynamic_reconfigure::Server<aerial_robot_base::PidThrottleControlConfig>(ros::NodeHandle(nhp_, "throttle"));
  dynamic_reconf_func_throttle_ = boost::bind(&PidController::cfgThrottleCallback, this, _1, _2);
  throttle_server_->setCallback(dynamic_reconf_func_throttle_);
}

PidController::~PidController()
{
  printf(" deleted pid flight control input!\n");
}


//deprecated
void PidController::motorBiasSetCallback(const std_msgs::Int8ConstPtr& set_flag)
{

  aerial_robot_msgs::ITermBias msg;
  msg.bias[0] = (short)(-pos_i_term_roll_ - offset_roll_); //反転
  msg.bias[1] = (short)(pos_i_term_pitch_ + offset_pitch_);
  motor_bias_set_pub_.publish(msg);

  motor_bias_flag_ = true;
  pos_i_term_roll_ = 0;
  pos_i_term_pitch_ = 0;

}

void PidController::rpyCtrlOffsetCallback(const aerial_robot_msgs::RPYCtrlOffsetConstPtr& offset_value)
{
  offset_roll_ = offset_value->roll_offset;
  offset_pitch_ = offset_value->pitch_offset;

  pos_i_term_roll_ = 0;
  pos_i_term_pitch_ = 0;
}


void PidController::pidFunction()
{
  static bool first_flag = true;
  static bool start_free_fall = false;

  //for rocket start
  static int rocket_start_value_tmp = rocket_start_init_value_ - offset_throttle_;
  static int restart_cnt = 0;

  //*** リセット
  if (navigator_->getFlightMode() == Navigator::RESET_MODE) 
    {
      first_flag = true;

      pos_p_term_pitch_ = 0; pos_i_term_pitch_ = 0; pos_d_term_pitch_ = 0;
      pos_p_term_roll_ = 0; pos_i_term_roll_ = 0; pos_d_term_roll_ = 0;
      pos_p_term_yaw_ = 0; pos_i_term_yaw_ = 0; pos_d_term_yaw_ = 0;
      pos_p_term_throttle_ = 0; pos_i_term_throttle_ = 0; pos_d_term_throttle_ = 0;

      flight_ctrl_input_->setPitchValue(0);
      flight_ctrl_input_->setRollValue(0);
      flight_ctrl_input_->setYawValue(0);
      flight_ctrl_input_->setThrottleValue(0);

      restart_cnt++;
      rocket_start_value_tmp
        = rocket_start_init_value_ - offset_throttle_ + rocket_start_init_increment_value_ * restart_cnt;

      start_free_fall = false;
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
          ROS_ERROR(" bad ");
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
                limit(1000 * pos_p_gain_pitch_ * d_err_pos_curr_pitch_,  pos_p_limit_pitch_);

              //**** Iの項
              if(navigator_->getFlightMode() == Navigator::TAKEOFF_MODE || navigator_->getFlightMode() == Navigator::LAND_MODE) //takeoff or land
                pos_i_term_pitch_ += 1000 * 
                  d_err_pos_curr_pitch_ * (1 / (float)pitch_ctrl_loop_rate_) * pos_i_gain_pitch_;
              else if(navigator_->getFlightMode() == Navigator::FLIGHT_MODE &&
                      fabs(d_err_pos_curr_pitch_) < i_enable_limit_pitch_) //hover or land
                pos_i_term_pitch_ += 1000 * 
                  d_err_pos_curr_pitch_ * (1 / (float)pitch_ctrl_loop_rate_) * pos_i_gain_pitch_hover_;
              pos_i_term_pitch_ = limit(pos_i_term_pitch_, pos_i_limit_pitch_);

              //***** Dの項
              pos_d_term_pitch_ = 
                limit(1000 * pos_d_gain_pitch_ * d_err_vel_curr_pitch_, pos_d_limit_pitch_);

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
                limit(1000 * pos_p_gain_pitch_ * d_err_pos_curr_pitch_, pos_p_limit_pitch_);
              //**** Dの項
              pos_d_term_pitch_ =
                limit(1000 * pos_d_gain_pitch_ * d_err_vel_curr_pitch_, pos_d_limit_pitch_);

            }
          else if(navigator_->getXyControlMode() == Navigator::VEL_WORLD_BASED_CONTROL_MODE)
            {
              //** 座標変換
              d_err_vel_curr_pitch_ 
                = (target_vel_x - state_vel_x) * cos(state_psi_cog) 
                + (target_vel_y - state_vel_y) * sin(state_psi_cog); 

              //**** Pの項
              pos_p_term_pitch_ =
                limit(1000 * vel_p_gain_pitch_ * d_err_vel_curr_pitch_, pos_d_limit_pitch_);
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

                      pos_p_term_pitch_ =
                        limit( 1000 * pos_p_gain_pitch_ * d_err_pos_curr_pitch_,  pos_p_limit_pitch_);

                      pos_i_term_pitch_ += 1000 * 
                        d_err_pos_curr_pitch_ * (1 / (float)pitch_ctrl_loop_rate_) * pos_i_gain_pitch_;
                      pos_i_term_pitch_ = limit(pos_i_term_pitch_, pos_i_limit_pitch_);

                      pos_d_term_pitch_ = 
                        limit(1000 * pos_d_gain_pitch_ * d_err_vel_curr_pitch_, pos_d_limit_pitch_);
                    }
                  else
                    {
                      d_err_vel_curr_pitch_ = target_vel_x - state_vel_x;
                      //**** Pの項
                      pos_p_term_pitch_ =
                        limit(1000 * vel_p_gain_pitch_ * d_err_vel_curr_pitch_, pos_d_limit_pitch_);
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
                  pos_p_term_pitch_ =
                    limit(1000 * vel_p_gain_pitch_ * d_err_vel_curr_pitch_, pos_d_limit_pitch_);

                  //**** Dの項
                  pos_d_term_pitch_ = 0;
                }

            }
          else
            ROS_ERROR("wrong control mode");

          if(navigator_->getFreeFallFlag())
            {// free fall mode
              pos_p_term_pitch_ =  0;
              pos_i_term_pitch_ =  0;
              pos_d_term_pitch_ =  0;
            }

          //*** motor bias
          if(motor_bias_flag_) pos_i_term_pitch_ = 0;

          //*** 指令値算出
          short pitch_value = limit(pos_p_term_pitch_ + pos_i_term_pitch_ + pos_d_term_pitch_ + offset_pitch_, pos_limit_pitch_);

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
          four_axis_pid_debug.pitch.total = pitch_value;
          four_axis_pid_debug.pitch.p_term = pos_p_term_pitch_;
          four_axis_pid_debug.pitch.i_term = pos_i_term_pitch_;
          four_axis_pid_debug.pitch.d_term = pos_d_term_pitch_;
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
              pos_p_term_roll_ =
                limit(1000 * pos_p_gain_roll_ * d_err_pos_curr_roll_, pos_p_limit_roll_);

              //**** Iの項
              if(navigator_->getFlightMode() == Navigator::TAKEOFF_MODE || navigator_->getFlightMode() == Navigator::LAND_MODE) //takeoff or land
                pos_i_term_roll_ += 1000 * 
                  d_err_pos_curr_roll_ * (1 / (float)roll_ctrl_loop_rate_) * pos_i_gain_roll_;
              else if(navigator_->getFlightMode() == Navigator::FLIGHT_MODE &&
                      fabs(d_err_pos_curr_roll_) < i_enable_limit_roll_) //hover
                pos_i_term_roll_ += 1000 * 
                  d_err_pos_curr_roll_ * (1 / (float)roll_ctrl_loop_rate_) * pos_i_gain_roll_hover_;
              pos_i_term_roll_ = limit(pos_i_term_roll_, pos_i_limit_roll_);

              //***** Dの項
              pos_d_term_roll_ = 
                limit(1000 * pos_d_gain_roll_ * d_err_vel_curr_roll_, pos_d_limit_roll_);
            }
          else if(navigator_->getXyControlMode() == Navigator::POS_LOCAL_BASED_CONTROL_MODE)
            {
              //** 座標変換
              d_err_pos_curr_roll_
                = - (target_pos_x - state_pos_x) * sin(state_psi_cog)
                + (target_pos_y - state_pos_y) * cos(state_psi_cog);

              d_err_vel_curr_roll_ = target_vel_y - state_vel_y;

              //**** Pの項
              pos_p_term_roll_ =  
                limit(1000 * pos_p_gain_roll_ * d_err_pos_curr_roll_, pos_p_limit_roll_);

              //**** Dの項
              pos_d_term_roll_ =
                limit(1000 * pos_d_gain_roll_ * d_err_vel_curr_roll_, pos_d_limit_roll_);

            }
          else if(navigator_->getXyControlMode() == Navigator::VEL_WORLD_BASED_CONTROL_MODE)
            {
              d_err_vel_curr_roll_ 
                = -(target_vel_x - state_vel_x) * sin(state_psi_cog)
                + (target_vel_y - state_vel_y)  * cos(state_psi_cog);

              //**** Pの項
              pos_p_term_roll_ =
                limit(1000 * vel_p_gain_roll_ * d_err_vel_curr_roll_, pos_d_limit_roll_);
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

                      pos_p_term_roll_ =
                        limit( 1000 * pos_p_gain_roll_ * d_err_pos_curr_roll_,  pos_p_limit_roll_);

                      pos_i_term_roll_ += 1000 * 
                        d_err_pos_curr_roll_ * (1 / (float)roll_ctrl_loop_rate_) * pos_i_gain_roll_;
                      pos_i_term_roll_ = limit(pos_i_term_roll_, pos_i_limit_roll_);

                      pos_d_term_roll_ = 
                        limit(1000 * pos_d_gain_roll_ * d_err_vel_curr_roll_, pos_d_limit_roll_);
                    }
                  else
                    {
                      d_err_vel_curr_roll_ = target_vel_y - state_vel_y;
                      //**** Pの項
                      pos_p_term_roll_ =
                        limit(1000 * vel_p_gain_roll_ * d_err_vel_curr_roll_, pos_d_limit_roll_);
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
                  pos_p_term_roll_ =
                    limit(1000 * vel_p_gain_roll_ * d_err_vel_curr_roll_, pos_d_limit_roll_);


                  //**** Dの項
                  pos_d_term_roll_ = 0;
                }
            }
          else
            ROS_ERROR("wrong control mode");

          if(navigator_->getFreeFallFlag())
            {// free fall mode
              pos_p_term_roll_ =  0;
              pos_i_term_roll_ =  0;
              pos_d_term_roll_ =  0;
            }

          //*** motor bias
          if(motor_bias_flag_) pos_i_term_roll_ = 0;

          //*** 指令値算出
          short roll_value = limit(pos_p_term_roll_ + pos_i_term_roll_ + pos_d_term_roll_ + offset_roll_, pos_limit_roll_);
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
          four_axis_pid_debug.roll.total = roll_value;
          four_axis_pid_debug.roll.p_term = pos_p_term_roll_;
          four_axis_pid_debug.roll.i_term = pos_i_term_roll_;
          four_axis_pid_debug.roll.d_term = pos_d_term_roll_;
          four_axis_pid_debug.roll.pos_err_transform = d_err_pos_curr_roll_;
          four_axis_pid_debug.roll.pos_err_no_transform = target_pos_y - state_pos_y;
          four_axis_pid_debug.roll.vel_err_transform = target_vel_y;
          four_axis_pid_debug.roll.vel_err_no_transform = target_vel_y - state_vel_y;


          //yaw => refer to the board frame angle(psi_board)
          if(navigator_->getXyControlMode() == Navigator::POS_WORLD_BASED_CONTROL_MODE ||
             navigator_->getXyControlMode() == Navigator::POS_LOCAL_BASED_CONTROL_MODE || 
             navigator_->getXyControlMode() == Navigator::VEL_WORLD_BASED_CONTROL_MODE)
            {
              d_err_pos_curr_yaw_ = target_psi - state_psi_board;
              if(d_err_pos_curr_yaw_ > M_PI)  d_err_pos_curr_yaw_ -= 2 * M_PI; 
              else if(d_err_pos_curr_yaw_ < -M_PI)  d_err_pos_curr_yaw_ += 2 * M_PI;

              //**** Pの項
              pos_p_term_yaw_ = 
                limit(1000 * pos_p_gain_yaw_ * d_err_pos_curr_yaw_, pos_p_limit_yaw_);

              //**** Iの項 : deprecated
#if 1 //bad for hydra control
              if(fabs(d_err_pos_curr_yaw_) < i_enable_limit_yaw_) 
                pos_i_term_yaw_ += 1000 *
                  d_err_pos_curr_yaw_ * ( 1 / (float)yaw_ctrl_loop_rate_ ) * pos_i_gain_yaw_;
              else{ pos_i_term_yaw_ = 0;} // right ?
#else 
              pos_i_term_yaw_ += 1000 *
                d_err_pos_curr_yaw_ * ( 1 / (float)yaw_ctrl_loop_rate_ ) * pos_i_gain_yaw_;
#endif

              pos_i_term_yaw_ = limit(pos_i_term_yaw_, pos_i_limit_yaw_);

              //***** Dの項
              pos_d_term_yaw_ = 
                limit(- 1000 * pos_d_gain_yaw_ * state_vel_psi_board, pos_d_limit_yaw_);
            }
          else if(navigator_->getXyControlMode() == Navigator::VEL_LOCAL_BASED_CONTROL_MODE)
            {//bad
              //fixed point  getStatePsi()
              //develop the p term yaw
              pos_p_term_yaw_ = limit(1000 * target_psi, pos_p_limit_yaw_);
              pos_i_term_yaw_ = 0;
              pos_d_term_yaw_ = 0;
            }

          //*** 指令値算出
          short yaw_value = limit(pos_p_term_yaw_ + pos_i_term_yaw_ + pos_d_term_yaw_,
                                  pos_limit_yaw_);
	
          //**** ros pub
          four_axis_pid_debug.yaw.total = yaw_value;
          four_axis_pid_debug.yaw.p_term = pos_p_term_yaw_;
          four_axis_pid_debug.yaw.i_term = pos_i_term_yaw_;
          four_axis_pid_debug.yaw.d_term = pos_d_term_yaw_;
          four_axis_pid_debug.yaw.pos_err_transform = target_psi;
          four_axis_pid_debug.yaw.pos_err_no_transform = d_err_pos_curr_yaw_;
          four_axis_pid_debug.yaw.vel_err_transform = state_vel_psi_board;
          four_axis_pid_debug.yaw.vel_err_no_transform = state_vel_psi_board;

          //*** 指令値代入
          flight_ctrl_input_->setYawValue(yaw_value);
          //flight_ctrl_input->set_yaw_value(0);

          //throttle
          d_err_pos_curr_throttle_ = target_pos_z - state_pos_z;

          if(navigator_->getFlightMode() == Navigator::TAKEOFF_MODE)
            {
              if(estimator_->getRocketStartFlag())
                {
                  ROS_INFO("rocket start");
                  pos_p_term_throttle_ = 0;
                  //increment rocket start
                  rocket_start_value_tmp += rocket_start_step_value_;
                  pos_i_term_throttle_ = rocket_start_value_tmp;
                  pos_d_term_throttle_ = 0;
                }
              else
                {
                  //ROS_INFO("normal takeoff");
                  //**** Pの項
                  pos_p_term_throttle_ = 
                    limit(1000 * pos_p_gain_throttle_ * d_err_pos_curr_throttle_, pos_p_limit_throttle_);
                  //**** Iの項
                  pos_i_term_throttle_ += 1000 *
                    d_err_pos_curr_throttle_ * ( 1 / (float)throttle_ctrl_loop_rate_) * pos_i_gain_throttle_;
                  pos_i_term_throttle_ = limit(pos_i_term_throttle_, pos_i_limit_throttle_);
                  //***** Dの項
                  pos_d_term_throttle_ = 
                    limit(-1000 * pos_d_gain_throttle_ * state_vel_z, pos_d_limit_throttle_);
                }
            }
          else if(navigator_->getFlightMode() == Navigator::FLIGHT_MODE) //hover
            {
              //**** Pの項
              pos_p_term_throttle_ = 
                limit(1000 * pos_p_gain_throttle_ * d_err_pos_curr_throttle_, pos_p_limit_throttle_hover_);
              //**** Iの項
              pos_i_term_throttle_ += 1000 *
                d_err_pos_curr_throttle_ * ( 1 / (float)throttle_ctrl_loop_rate_) * pos_i_gain_throttle_;
              pos_i_term_throttle_ = limit(pos_i_term_throttle_, pos_i_limit_throttle_);
              //***** Dの項
              pos_d_term_throttle_ = 
                limit(-1000 * pos_d_gain_throttle_ * state_vel_z, pos_d_limit_throttle_);
            }
          else if(navigator_->getFlightMode() == Navigator::LAND_MODE)
            {
              ROS_WARN(" land mode");

              if(navigator_->getFreeFallFlag())
                {//free fall mode from free fall threshold

                  if(!start_free_fall)
                    { //first time
                      short pos_i_term_throttle_tmp = - offset_throttle_ + limit(pos_p_term_throttle_ +
                                                                            pos_i_term_throttle_ + 
                                                                            pos_d_term_throttle_ + 
                                                                            offset_throttle_,
                                                                            pos_limit_throttle_);
                      pos_i_term_throttle_ = pos_i_term_throttle_tmp;
                      start_free_fall  = true;
                    }
                  pos_p_term_throttle_ = 0;
                  pos_i_term_throttle_ -= free_fall_step_value_;
                  pos_d_term_throttle_ = 0;

                  if(pos_i_term_throttle_ + offset_throttle_ < motor_stop_value_)
                    {
                      bool tmp = true;
                      navigator_->setMotorStopFlag(tmp);
                      ROS_WARN("  stop motor ");
                    }
                }
              else
                {
                  ROS_WARN(" no free fall mode");

                  //**** Pの項
                  //TODO: d_err_pos_curr_throttle < -0.1 or > -0.1 ; Original is < -0.1
                  //const_pcontrolThre_land : -0.1m
                  //const_p_term_lev1_value_land : -50
                  //const_p_term_lev2_value_land : -90
                  if(d_err_pos_curr_throttle_ < const_p_ctrl_thre_throttle_land_) 
                    pos_p_term_throttle_ = const_p_term_lev1_value_throttle_land_; 
                  else  pos_p_term_throttle_ = const_p_term_lev2_value_throttle_land_;

                  //**** Iの項
                  // const_icontrolThre_land : -0.25m
                  if(d_err_pos_curr_throttle_ > const_i_ctrl_thre_throttle_land_)
                    pos_i_term_throttle_ += 1000 * d_err_pos_curr_throttle_ *
                      (1 / (float)throttle_ctrl_loop_rate_) * pos_i_gain_throttle_land_;
                  else pos_i_term_throttle_ -= const_i_term_value_throttle_land_;

                  //***** Dの項
                  pos_d_term_throttle_ =
                    limit(-1000 * pos_d_gain_throttle_land_ * state_vel_z, pos_d_limit_throttle_);
                }
            }
          //throwing mode function
          throttleThrowingMode();

          //*** 指令値算出
          short throttle_value = limit(pos_p_term_throttle_ + pos_i_term_throttle_ 
                                       + pos_d_term_throttle_ + offset_throttle_, 
                                       pos_limit_throttle_);
          //**** 指令値反転
          //throttle_value = - throttle_value;
	  
          //*** 指令値代入
          flight_ctrl_input_->setThrottleValue(throttle_value);

          //**** ros pub
          four_axis_pid_debug.throttle.total = throttle_value;
          four_axis_pid_debug.throttle.p_term = pos_p_term_throttle_;
          four_axis_pid_debug.throttle.i_term = pos_i_term_throttle_;
          four_axis_pid_debug.throttle.d_term = pos_d_term_throttle_;
          four_axis_pid_debug.throttle.pos_err_transform = target_pos_z;
          four_axis_pid_debug.throttle.pos_err_no_transform = d_err_pos_curr_throttle_;
          four_axis_pid_debug.throttle.vel_err_transform = state_vel_z;
          four_axis_pid_debug.throttle.vel_err_no_transform = state_vel_z;

	  pid_pub_.publish(four_axis_pid_debug);
	}

      //*** 更新
      //previous_secs = current_secs;
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


void PidController::throttleThrowingMode()
{
  static bool first_flag = true;
  if(navigator_->getThrowingMode() == Navigator::THROWING_START_ALT_HOLD)
    {
      if(first_flag)
        {
          first_flag = false;
          pos_i_term_throttle_ += (rocket_start_init_value_ - offset_throttle_  + throwing_mode_init_value_from_rocket_start_); //param

          //debug
          ROS_ERROR("first time: pos_i_termThtottle init");
        }

      //**** Pの項
      pos_p_term_throttle_ = 
        limit(1000 * pos_p_gain_throttle_ * d_err_pos_curr_throttle_, pos_p_limit_throttle_);
      //**** Iの項
      pos_i_term_throttle_ += 1000 *
        d_err_pos_curr_throttle_ * ( 1 / (float)throttle_ctrl_loop_rate_) * pos_i_gain_throttle_;
      pos_i_term_throttle_ = limit(pos_i_term_throttle_, pos_i_limit_throttle_);
      //***** Dの項
      pos_d_term_throttle_ = 
        limit(-1000 * pos_d_gain_throttle_ * estimator_->getStateVelZ(), pos_d_limit_throttle_);

      ROS_ERROR("ok, d_err_pos_curr_throttle: %f",d_err_pos_curr_throttle_ );

    }
}

void PidController::cfgPitchCallback(aerial_robot_base::PidPitchControlConfig &config, uint32_t level)
{
  if(config.pidControlFlag)
    {
      printf("Pitch Dynamic:");
      switch(level)
        {
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_CONTROL_LOOP_RATE:
          pitch_ctrl_loop_rate_ = config.ctrlLoopRate;
          printf("change the control loop rate\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_P_GAIN:
          pos_p_gain_pitch_ = config.posPGain;
          printf("change the pos p gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_I_GAIN:
          pos_i_gain_pitch_ = config.posIGain;
          printf("change the pos i gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_I_GAIN_HOVER:
          pos_i_gain_pitch_hover_ = config.posIGainHover;
          printf("change the posi_hover gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_D_GAIN:
          pos_d_gain_pitch_ = config.posDGain;
          printf("change the pos d gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_VEL_P_GAIN:
          vel_p_gain_pitch_ = config.velPGain;
          printf("change the vel p gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_VEL_I_GAIN:
          vel_i_gain_pitch_ = config.velIGain;
          printf("change the vel i gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_OFFSET:
          offset_pitch_ = config.offset;
          printf("change the offset\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_LIMIT:
          pos_limit_pitch_ = config.posLimit;
          printf("change the limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_P_LIMIT:
          pos_p_limit_pitch_ = config.posPLimit;
          printf("change the p limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_I_LIMIT:
          pos_i_limit_pitch_ = config.posILimit;
          printf("change the i limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_D_LIMIT:
          pos_d_limit_pitch_ = config.posDLimit;
          printf("change the d limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_VEL_VALUE_LIMIT_HOVER:
          vel_value_limit_pitch_ = config.velValueLimit;
          printf("change the vel value limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_ENABLE_LIMIT_HOVER:
          i_enable_limit_pitch_ = config.iEnableLimit;
          printf("change the i enable limit\n");
          break;
        default :
          printf("\n");
          break;
        }
    }
}

void PidController::cfgRollCallback(aerial_robot_base::PidRollControlConfig &config, uint32_t level)
{

  if(config.pidControlFlag)
    {
      printf("Roll Dynamic:");
      switch(level)
        {
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_CONTROL_LOOP_RATE:
          roll_ctrl_loop_rate_ = config.ctrlLoopRate;
          printf("change the control loop rate\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_P_GAIN:
          pos_p_gain_roll_ = config.posPGain;
          printf("change the pos p gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_I_GAIN:
          pos_i_gain_roll_ = config.posIGain;
          printf("change the pos i gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_I_GAIN_HOVER:
          pos_i_gain_roll_hover_ = config.posIGainHover;
          printf("change the pos i_hover gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_D_GAIN:
          pos_d_gain_roll_ = config.posDGain;
          printf("change the pos d gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_VEL_P_GAIN:
          vel_p_gain_roll_ = config.velPGain;
          printf("change the vel p gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_VEL_I_GAIN:
          vel_i_gain_roll_ = config.velIGain;
          printf("change the vel i gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_OFFSET:
          offset_roll_ = config.offset;
          printf("change the offset\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_LIMIT:
          pos_limit_roll_ = config.posLimit;
          printf("change the limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_P_LIMIT:
          pos_p_limit_roll_ = config.posPLimit;
          printf("change the p limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_I_LIMIT:
          pos_i_limit_roll_ = config.posILimit;
          printf("change the i limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_D_LIMIT:
          pos_d_limit_roll_ = config.posDLimit;
          printf("change the d limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_VEL_VALUE_LIMIT_HOVER:
          vel_value_limit_roll_ = config.velValueLimit;
          printf("change the vel value limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_ENABLE_LIMIT_HOVER:
          i_enable_limit_roll_ = config.iEnableLimit;
          printf("change the i enable limit\n");
          break;
        default :   
          printf("\n");
          break;
        }
    }
}

void PidController::cfgYawCallback(aerial_robot_base::PidYawControlConfig &config, uint32_t level)
{
  if(config.pidControlFlag)
    {
      printf("_yaw Dynamic:");
      switch(level)
        {
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_CONTROL_LOOP_RATE:
          yaw_ctrl_loop_rate_ = config.ctrlLoopRate;
          printf("change the control loop rate\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_P_GAIN:
          pos_p_gain_yaw_ = config.posPGain;
          printf("change the p gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_I_GAIN:
          pos_i_gain_yaw_ = config.posIGain;
          printf("change the i gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_D_GAIN:
          pos_d_gain_yaw_ = config.posDGain;
          printf("change the d gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_LIMIT:
          pos_limit_yaw_ = config.posLimit;
          printf("change the limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_P_LIMIT:
          pos_p_limit_yaw_ = config.posPLimit;
          printf("change the p limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_I_LIMIT:
          pos_i_limit_yaw_ = config.posILimit;
          printf("change the i limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_D_LIMIT:
          pos_d_limit_yaw_ = config.posDLimit;
          printf("change the d limit\n");
          break;
        default :
          printf("\n");
          break;
        }
    }
}

void PidController::cfgThrottleCallback(aerial_robot_base::PidThrottleControlConfig &config, uint32_t level)
{

  if(config.pidControlFlag)
    {
      printf("Throttle Dynamic:");
      switch(level)
        {
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_CONTROL_LOOP_RATE:
          throttle_ctrl_loop_rate_ = config.ctrlLoopRate;
          printf("change the control loop rate\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_P_GAIN:
          pos_p_gain_throttle_ = config.posPGain;
          printf("change the p gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_I_GAIN:
          pos_i_gain_throttle_ = config.posIGain;
          printf("change the i gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_D_GAIN:
          pos_d_gain_throttle_ = config.posDGain;
          printf("change the d gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_P_GAIN_LAND:
          pos_p_gain_throttle_land_ = config.posPGainLand;
          printf("change the p gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_I_GAIN_LAND:
          pos_i_gain_throttle_land_ = config.posIGainLand;
          printf("change the i gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_D_GAIN_LAND:
          pos_d_gain_throttle_land_ = config.posDGainLand;
          printf("change the d gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_CONST_P_CONTROL_THRESHOLD_LAND:
          const_p_ctrl_thre_throttle_land_ = config.constPControlThreLand;
          printf("change const p control threshold throttle land\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_CONST_P_TERM_LEVEL1_VALUE_LAND:
          const_p_term_lev1_value_throttle_land_ = config.constPTermLev1ValueLand;
          printf("change const p term level1 value throttle land\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_CONST_P_TERM_LEVEL2_VALUE_LAND:
          const_p_term_lev2_value_throttle_land_ = config.constPTermLev2ValueLand;
          printf("change const p term level2 value throttle land\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_CONST_I_CONTROL_THRESHOLD_LAND:
          const_i_ctrl_thre_throttle_land_ = config.constIControlThreLand;
          printf("change const i control threshold throttle land\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_CONST_I_TERM_VALUE_LAND:
          const_i_term_value_throttle_land_ = config.constITermValueLand;
          printf("change const i term value throttle land\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_OFFSET:
          offset_throttle_ = config.offset;
          printf("change the offset\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_LIMIT:
          pos_limit_throttle_ = config.posLimit;
          printf("change the limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_P_LIMIT:
          pos_p_limit_throttle_ = config.posPLimit;
          printf("change the p limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_P_LIMIT_HOVER:
          pos_p_limit_throttle_hover_ = config.posPLimitHover;
          printf("change the p hover limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_I_LIMIT:
          pos_i_limit_throttle_ = config.posILimit;
          printf("change the i limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_POS_D_LIMIT:
          pos_d_limit_throttle_ = config.posDLimit;
          printf("change the d limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_VEL_VALUE_LIMIT_HOVER:
          vel_value_limit_throttle_hover_ = config.velValueLimitHover;
          printf("change the vel value limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_ENABLE_LIMIT_HOVER:
          i_enable_limit_throttle_hover_ = config.iEnableLimitHover;
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

  if (!throttle_node.getParam ("pos_p_gain", pos_p_gain_throttle_))
    pos_p_gain_throttle_ = 0;
  printf("%s: pos_p_gain_ is %.3f\n", throttle_ns.c_str(), pos_p_gain_throttle_);

  if (!throttle_node.getParam ("pos_i_gain", pos_i_gain_throttle_))
    pos_i_gain_throttle_ = 0;
  printf("%s: pos_i_gain_ is %.3f\n", throttle_ns.c_str(), pos_i_gain_throttle_);

  if (!throttle_node.getParam ("pos_d_gain", pos_d_gain_throttle_))
    pos_d_gain_throttle_ = 0;
  printf("%s: pos_d_gain_ is %.3f\n", throttle_ns.c_str(), pos_d_gain_throttle_);

  if (!throttle_node.getParam ("pos_p_gain_land", pos_p_gain_throttle_land_))
    pos_p_gain_throttle_land_ = 0;
  printf("%s: pos_p_gain_land_ is %.3f\n", throttle_ns.c_str(), pos_p_gain_throttle_land_);

  if (!throttle_node.getParam ("pos_i_gain_land", pos_i_gain_throttle_land_))
    pos_i_gain_throttle_land_ = 0;
  printf("%s: pos_i_gain_land_ is %.3f\n", throttle_ns.c_str(), pos_i_gain_throttle_land_);

  if (!throttle_node.getParam ("pos_d_gain_land", pos_d_gain_throttle_land_))
    pos_d_gain_throttle_land_ = 0;
  printf("%s: pos_d_gain_land_ is %.3f\n", throttle_ns.c_str(), pos_d_gain_throttle_land_);

  if (!throttle_node.getParam ("const_p_ctrl_thre_land",  const_p_ctrl_thre_throttle_land_))
    const_p_ctrl_thre_throttle_land_ = 0;
  printf("%s: const_p_ctrl_thre_land_ is %.3f\n", throttle_ns.c_str(), const_p_ctrl_thre_throttle_land_);

  if(!throttle_node.getParam("const_p_term_lev1_value_land",const_p_term_lev1_value_throttle_land_))
    const_p_term_lev1_value_throttle_land_ = 0;
  printf("%s: const_p_term_lev1_value_land_ is %.3f\n", throttle_ns.c_str(), const_p_term_lev1_value_throttle_land_);

  if(!throttle_node.getParam("const_p_term_lev2_value_land",const_p_term_lev2_value_throttle_land_))
    const_p_term_lev2_value_throttle_land_ = 0;
  printf("%s: const_p_term_lev2_value_land_ is %.3f\n", throttle_ns.c_str(), const_p_term_lev2_value_throttle_land_);

  if (!throttle_node.getParam ("const_i_ctrl_thre_land",  const_i_ctrl_thre_throttle_land_))
    const_i_ctrl_thre_throttle_land_ = 0;
  printf("%s: const_i_ctrl_thre_land_ is %.3f\n", throttle_ns.c_str(), const_i_ctrl_thre_throttle_land_);

  if (!throttle_node.getParam ("const_i_term_value_land",  const_i_term_value_throttle_land_))
    const_i_term_value_throttle_land_ = 0;
  printf("%s: const_i_term_value_land_ is %.3f\n", throttle_ns.c_str(), const_i_term_value_throttle_land_);

  if (!throttle_node.getParam ("offset", offset_throttle_))
    offset_throttle_ = 0;
  printf("%s: offset_ is %d\n", throttle_ns.c_str(), offset_throttle_);

  if (!throttle_node.getParam ("pos_limit", pos_limit_throttle_))
    pos_limit_throttle_ = 0;
  printf("%s: pos_limit_ is %d\n", throttle_ns.c_str(), pos_limit_throttle_);

  if (!throttle_node.getParam ("pos_p_limit", pos_p_limit_throttle_))
    pos_p_limit_throttle_ = 0;
  printf("%s: pos_p_limit_ is %d\n", throttle_ns.c_str(), pos_p_limit_throttle_);

  if (!throttle_node.getParam ("pos_p_limit_hover", pos_p_limit_throttle_hover_))
    pos_p_limit_throttle_hover_ = 0;
  printf("%s: pos_p_limit_hover_ is %d\n", throttle_ns.c_str(), pos_p_limit_throttle_hover_);

  if (!throttle_node.getParam ("pos_i_limit", pos_i_limit_throttle_))
    pos_i_limit_throttle_ = 0;
  printf("%s: pos_i_limit_ is %d\n", throttle_ns.c_str(), pos_i_limit_throttle_);

  if (!throttle_node.getParam ("pos_d_limit", pos_d_limit_throttle_))
    pos_d_limit_throttle_ = 0;
  printf("%s: pos_d_limit_ is %d\n", throttle_ns.c_str(), pos_d_limit_throttle_);

  if (!throttle_node.getParam ("vel_value_limit_hover", vel_value_limit_throttle_hover_))
    vel_value_limit_throttle_hover_ = 0;
  printf("%s: vel_value_limit_hover_ is %.3f\n", throttle_ns.c_str(), vel_value_limit_throttle_hover_);

  if (!throttle_node.getParam ("i_enable_limit_hover", i_enable_limit_throttle_hover_))
    i_enable_limit_throttle_hover_ = 0;
  printf("%s: i_enable_limit_hover_ is %.3f\n", throttle_ns.c_str(), i_enable_limit_throttle_hover_);

  if (!throttle_node.getParam ("rocket_start_init_value", rocket_start_init_value_))
    rocket_start_init_value_ = 0;
  printf("%s: rocket_start_init_value_ is %d\n", throttle_ns.c_str(), rocket_start_init_value_);

  if (!throttle_node.getParam ("rocket_start_init_increment_value", rocket_start_init_increment_value_))
    rocket_start_init_increment_value_ = 0;
  printf("%s: rocket_start_init_increment_value_ is %d\n", throttle_ns.c_str(), rocket_start_init_increment_value_);

  if (!throttle_node.getParam ("rocket_start_step_value", rocket_start_step_value_))
    rocket_start_step_value_ = 0;
  printf("%s: rocket_start_step_value_ is %d\n", throttle_ns.c_str(), rocket_start_step_value_);

  if (!throttle_node.getParam ("throwing_mode_init_value_from_rocket_start", throwing_mode_init_value_from_rocket_start_))
    throwing_mode_init_value_from_rocket_start_ = 0;
  printf("%s: throwing_mode_init_value_from_rocket_start_ is %d\n", throttle_ns.c_str(), throwing_mode_init_value_from_rocket_start_);

  if (!throttle_node.getParam ("free_fall_step_value", free_fall_step_value_))
    free_fall_step_value_ = 0;
  printf("%s: free_fall_step_value_ is %d\n", throttle_ns.c_str(), free_fall_step_value_);

  if (!throttle_node.getParam ("motor_stop_value", motor_stop_value_))
    motor_stop_value_ = 0;
  printf("%s: motor_stop_value_ is %d\n", throttle_ns.c_str(), motor_stop_value_);

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

  if (!pitch_node.getParam ("vel_value_limit", vel_value_limit_pitch_))
    vel_value_limit_pitch_ = 0;
  printf("%s: vel_value_limit_ is %.3f\n", pitch_ns.c_str(), vel_value_limit_pitch_);

  if (!pitch_node.getParam ("i_enable_limit", i_enable_limit_pitch_))
    i_enable_limit_pitch_ = 0;
  printf("%s: i_enable_limit_ is %.3f\n", pitch_ns.c_str(), i_enable_limit_pitch_);

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

  if (!roll_node.getParam ("vel_value_limit", vel_value_limit_roll_))
    vel_value_limit_roll_ = 0;
  printf("%s: vel_value_limit_ is %.3f\n", roll_ns.c_str(), vel_value_limit_roll_);

  if (!roll_node.getParam ("i_enable_limit", i_enable_limit_roll_))
    i_enable_limit_roll_ = 0;
  printf("%s: i_enable_limit_ is %.3f\n", roll_ns.c_str(), i_enable_limit_roll_);

  //**** yaw
  if (!yaw_node.getParam ("pos_p_gain", pos_p_gain_yaw_))
    pos_p_gain_yaw_ = 0;
  printf("%s: pos_p_gain_ is %.3f\n", yaw_ns.c_str(), pos_p_gain_yaw_);

  if (!yaw_node.getParam ("pos_i_gain", pos_i_gain_yaw_))
    pos_i_gain_yaw_ = 0;
  printf("%s: pos_i_gain_ is %.3f\n", yaw_ns.c_str(), pos_i_gain_yaw_);

  if (!yaw_node.getParam ("pos_d_gain", pos_d_gain_yaw_))
    pos_d_gain_yaw_ = 0;
  printf("%s: pos_d_gain_ is %.3f\n", yaw_ns.c_str(), pos_d_gain_yaw_);

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

  if (!yaw_node.getParam ("vel_value_limit", vel_value_limit_yaw_))
    vel_value_limit_yaw_ = 0;
  printf("%s: vel_value_limit_ is %.3f\n", yaw_ns.c_str(), vel_value_limit_yaw_);

  if (!yaw_node.getParam ("i_enable_limit", i_enable_limit_yaw_))
    i_enable_limit_yaw_ = 0;
  printf("%s: i_enable_limit_ is %.3f\n", yaw_ns.c_str(), i_enable_limit_yaw_);

  if (!yaw_node.getParam ("ctrl_loop_rate", yaw_ctrl_loop_rate_))
    yaw_ctrl_loop_rate_ = 0;
  printf("%s: ctrl_loop_rate_ is %d\n", yaw_ns.c_str(), yaw_ctrl_loop_rate_);

}
