#include "aerial_robot_base/flight_navigation.h"

Navigator::Navigator(ros::NodeHandle nh, ros::NodeHandle nh_private, 
                     BasicEstimator* estimator, FlightCtrlInput* flight_ctrl_input,
                     int ctrl_loop_rate)
  : nh_(nh, "navigator"),
    nhp_(nh_private, "navigator"),
    ctrl_loop_rate_(ctrl_loop_rate),
    target_pos_(0, 0, 0),
    target_vel_(0, 0, 0),
    target_acc_(0, 0, 0),
    target_psi_(0), target_vel_psi_(0),
    force_att_control_flag_(false),
    flight_mode_(NO_CONTROL_MODE),
    low_voltage_flag_(false),
    prev_xy_control_mode_(flight_nav::ACC_CONTROL_MODE),
    vel_control_flag_(false),
    pos_control_flag_(false),
    xy_control_flag_(false),
    alt_control_flag_(false),
    yaw_control_flag_(false),
    vel_based_waypoint_(false),
    joy_stick_heart_beat_(false),
    joy_stick_prev_time_(0)
{
  rosParamInit(nhp_);

  navi_sub_ = nh_.subscribe<aerial_robot_base::FlightNav>("/uav/nav", 1, &Navigator::naviCallback, this, ros::TransportHints().tcpNoDelay());

  battery_sub_ = nh_.subscribe<std_msgs::UInt8>("/battery_voltage_status", 1, &Navigator::batteryCheckCallback, this, ros::TransportHints().tcpNoDelay());

  arming_ack_sub_ = nh_.subscribe<std_msgs::UInt8>("/flight_config_ack", 1, &Navigator::armingAckCallback, this, ros::TransportHints().tcpNoDelay());
  takeoff_sub_ = nh_.subscribe<std_msgs::Empty>("/teleop_command/takeoff", 1, &Navigator::takeoffCallback, this, ros::TransportHints().tcpNoDelay());
  halt_sub_ = nh_.subscribe<std_msgs::Empty>("/teleop_command/halt", 1, &Navigator::haltCallback, this, ros::TransportHints().tcpNoDelay());
  force_landing_sub_ = nh_.subscribe<std_msgs::Empty>("/teleop_command/force_landing", 1, &Navigator::forceLandingCallback, this, ros::TransportHints().tcpNoDelay());
  force_landing_flag_ = false;
  land_sub_ = nh_.subscribe<std_msgs::Empty>("/teleop_command/land", 1, &Navigator::landCallback, this, ros::TransportHints().tcpNoDelay());
  start_sub_ = nh_.subscribe<std_msgs::Empty>("/teleop_command/start", 1,&Navigator::startCallback, this, ros::TransportHints().tcpNoDelay());
  ctrl_mode_sub_ = nh_.subscribe<std_msgs::Int8>("/teleop_command/ctrl_mode", 1, &Navigator::xyControlModeCallback, this, ros::TransportHints().tcpNoDelay());

  joy_stick_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &Navigator::joyStickControl, this, ros::TransportHints().udp());

  rc_cmd_pub_ = nh_.advertise<aerial_robot_msgs::FourAxisCommand>("/aerial_robot_control_four_axis", 10);

  stop_teleop_sub_ = nh_.subscribe<std_msgs::UInt8>("stop_teleop", 1, &Navigator::stopTeleopCallback, this, ros::TransportHints().tcpNoDelay());
  teleop_flag_ = true;

  flight_config_pub_ = nh_.advertise<std_msgs::UInt8>("/flight_config_cmd", 10);

  estimator_ = estimator;
  estimate_mode_ = estimator_->getEstimateMode();
  flight_ctrl_input_ = flight_ctrl_input;
  stopNavigation();
  setNaviCommand( IDLE_COMMAND );
}

Navigator::~Navigator()
{
  printf(" deleted navigator!\n");
}

void Navigator::naviCallback(const aerial_robot_base::FlightNavConstPtr & msg)
{
  /* yaw */
  if(msg->psi_nav_mode == aerial_robot_base::FlightNav::POS_MODE)
    {
      setTargetPsi(msg->target_psi);
    }

  /* xy control */
  switch(msg->pos_xy_nav_mode)
    {
    case aerial_robot_base::FlightNav::POS_MODE:
      {
        force_att_control_flag_ = false;

        tf::Vector3 target_cog_pos(msg->target_pos_x, msg->target_pos_y, 0);
        if(msg->target == aerial_robot_base::FlightNav::BASELINK)
          {
            /* check the transformation */
            target_cog_pos -= tf::Matrix3x3(tf::createQuaternionFromYaw(getTargetPsi()))
              * estimator_->getCog2Baselink().getOrigin();
          }

        tf::Vector3 target_delta = getTargetPos() - target_cog_pos;
        target_delta.setZ(0);

        if(target_delta.length() > vel_nav_threshold_)
          {
            ROS_WARN("start vel nav control for waypoint");
            vel_based_waypoint_ = true;
            xy_control_mode_ = flight_nav::VEL_CONTROL_MODE;
          }

        if(!vel_based_waypoint_)
          xy_control_mode_ = flight_nav::POS_CONTROL_MODE;

        setTargetPosX(target_cog_pos.x());
        setTargetPosY(target_cog_pos.y());

        break;
      }
    case aerial_robot_base::FlightNav::VEL_MODE:
      {
        if(msg->target == aerial_robot_base::FlightNav::BASELINK)
          {
            ROS_ERROR("[Flight nav] can not do vel nav for baselink");
            return;
          }
        /* should be in COG frame */
        force_att_control_flag_ = false;
        xy_control_mode_ = flight_nav::VEL_CONTROL_MODE;
        switch(msg->control_frame)
          {
          case flight_nav::WORLD_FRAME:
            {
              setTargetVelX(msg->target_vel_x);
              setTargetVelY(msg->target_vel_y);
              break;
            }
          case flight_nav::LOCAL_FRAME:
            {
              tf::Vector3 target_vel = frameConversion(tf::Vector3(msg->target_vel_x, msg->target_vel_y, 0),  estimator_->getState(State::YAW, estimate_mode_)[0]);
              setTargetVelX(target_vel.x());
              setTargetVelY(target_vel.y());
              break;
            }
          default:
            {
              break;
            }
          }
        break;
      }
    case aerial_robot_base::FlightNav::ACC_MODE:
      {
        /* should be in COG frame */
        force_att_control_flag_ = true;
        xy_control_mode_ = flight_nav::ACC_CONTROL_MODE;
        switch(msg->control_frame)
          {
          case flight_nav::WORLD_FRAME:
            {
              target_acc_.setValue(msg->target_acc_x, msg->target_acc_y, 0);
              break;
            }
          case flight_nav::LOCAL_FRAME:
            {
              tf::Vector3 target_acc = target_acc_;
              target_acc_ = frameConversion(target_acc,  estimator_->getState(State::YAW, estimate_mode_)[0]);
              break;
            }
          default:
            {
              break;
            }
          }
        break;
      }
    }

  /* z */
  if(msg->pos_z_nav_mode == aerial_robot_base::FlightNav::VEL_MODE)
    {
      addTargetPosZ(msg->target_pos_diff_z);
    }
  else if(msg->pos_z_nav_mode == aerial_robot_base::FlightNav::POS_MODE)
    {
      setTargetPosZ(msg->target_pos_z);
    }
}

void Navigator::joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
{
  /* ps3 joy bottons assignment: http://wiki.ros.org/ps3joy */
  if(!joy_stick_heart_beat_) joy_stick_heart_beat_ = true;
  joy_stick_prev_time_ = ros::Time::now().toSec();

  /* common command */
  /* start */
  if(joy_msg->buttons[3] == 1 && !getStartAble())
    {
      motorArming();
      return;
    }

  /* force landing && halt */
  static ros::Time force_landing_start_time_ = ros::Time::now();
  if(joy_msg->buttons[0] == 1)
    {
      /* Force Landing in inflight mode: TAKEOFF_COMMAND/LAND_COMMAND/HOVER_COMMAND */
      if(!force_landing_flag_ && (getNaviCommand() == TAKEOFF_COMMAND || getNaviCommand() == LAND_COMMAND || getNaviCommand() == HOVER_COMMAND))
        {
          ROS_WARN("Joy Control: force landing command");
          std_msgs::UInt8 force_landing_cmd;
          force_landing_cmd.data = FORCE_LANDING_CMD;
          flight_config_pub_.publish(force_landing_cmd);
          force_landing_flag_ = true;

          /* update the force landing stamp for the halt process*/
          force_landing_start_time_ = joy_msg->header.stamp;
        }

      /* Halt mode */
      if(joy_msg->header.stamp.toSec() - force_landing_start_time_.toSec() > force_landing_to_halt_du_ && getStartAble())
        {
          ROS_ERROR("Joy Control: Halt!");

          setNaviCommand(STOP_COMMAND);
          flight_mode_= RESET_MODE;

          /* update the target pos(maybe not necessary) */
          setTargetXyFromCurrentState();
          setTargetPsiFromCurrentState();

          /* several flag should be false */
          estimator_->setSensorFusionFlag(false);
          estimator_->setLandingMode(false);
          estimator_->setLandedFlag(false);
          estimator_->setFlyingFlag(false);

        }
      return;
    }
  else
    {
      /* update the halt process */
      force_landing_start_time_ = joy_msg->header.stamp;
    }

  /* takeoff */
  if(joy_msg->buttons[7] == 1 && joy_msg->buttons[13] == 1)
    {
      startTakeoff();
      return;
    }

  /* landing */
  if(joy_msg->buttons[5] == 1 && joy_msg->buttons[15] == 1)
    {
      if(getNaviCommand() == LAND_COMMAND) return;

      setNaviCommand(LAND_COMMAND);
      //update
      setTargetXyFromCurrentState();
      setTargetPsiFromCurrentState();
      setTargetPosZ(estimator_->getLandingHeight());
      ROS_INFO("Joy Control: Land command");

      return;
    }

  /* Motion: Up/Down */
  if(fabs(joy_msg->axes[3]) > 0.2)
    {
      if(getNaviCommand() == HOVER_COMMAND)
        {
          alt_control_flag_ = true;
          if(joy_msg->axes[3] >= 0)
            addTargetPosZ(joy_target_alt_interval_);
          else
            addTargetPosZ(-joy_target_alt_interval_);
          ROS_INFO("Joy Control: Thrust command");
        }
    }
  else
    {
      if(alt_control_flag_)
        {
          alt_control_flag_= false;
          setTargetZFromCurrentState();
          ROS_INFO("Joy Control: Fixed Alt command, targetPosz_is %f",target_pos_.z());
        }
    }

  /* Motion: Yaw */
  /* this is the yaw_angle control */
  if(joy_msg->buttons[2] == 1)
    {
      if(fabs(joy_msg->axes[2]) > 0.05)
        {
          float  state_psi = estimator_->getState(State::YAW, estimate_mode_)[0];
          target_psi_ = state_psi + joy_msg->axes[2] * max_target_yaw_rate_;
          if(target_psi_ > M_PI)  target_psi_ -= 2 * M_PI;
          else if(target_psi_ < -M_PI)  target_psi_ += 2 * M_PI;
          ROS_WARN("Joy Control: yaw control based on angle control only");
        }
      else
        setTargetPsiFromCurrentState();
    }

  /* turn to ACC_CONTROL_MODE */
  if(joy_msg->buttons[6] == 1)
    {
      ROS_WARN("Change to attitude control");
      force_att_control_flag_ = true;
      xy_control_mode_ = flight_nav::ACC_CONTROL_MODE;
    }

  /* change to vel control mode */
  if(joy_msg->buttons[12] == 1 && !vel_control_flag_)
    {
      ROS_INFO("change to vel pos-based control");
      vel_control_flag_ = true;
      force_att_control_flag_ = false;
      xy_control_mode_ = flight_nav::VEL_CONTROL_MODE;
      target_vel_.setValue(0, 0, 0);
    }
  if(joy_msg->buttons[12] == 0 && vel_control_flag_)
    vel_control_flag_ = false;

  /* change to pos control mode */
  if(joy_msg->buttons[14] == 1 && !pos_control_flag_)
    {
      ROS_INFO("change to pos control");
      pos_control_flag_ = true;
      force_att_control_flag_ = false;
      xy_control_mode_ = flight_nav::POS_CONTROL_MODE;
      setTargetXyFromCurrentState();
    }
  if(joy_msg->buttons[14] == 0 && pos_control_flag_)
    pos_control_flag_ = false;

  /* mode oriented command */
  switch (xy_control_mode_)
    {
    case flight_nav::ACC_CONTROL_MODE:
      {
        control_frame_ = flight_nav::WORLD_FRAME;
        if(joy_msg->buttons[8]) control_frame_ = flight_nav::LOCAL_FRAME;

        /* pitch && roll angle command */
        target_acc_.setValue(joy_msg->axes[1] * max_target_tilt_angle_ * BasicEstimator::G,
                             joy_msg->axes[0] * max_target_tilt_angle_ * BasicEstimator::G, 0);

        if(control_frame_ == flight_nav::LOCAL_FRAME)
          {
            tf::Vector3 target_acc = target_acc_;
            target_acc_ = frameConversion(target_acc,  estimator_->getState(State::YAW, estimate_mode_)[0]);
          }

        break;
      }
    case flight_nav::VEL_CONTROL_MODE:
      {
        if(teleop_flag_)
          {
            control_frame_ = flight_nav::WORLD_FRAME;
            if(joy_msg->buttons[8]) control_frame_ = flight_nav::LOCAL_FRAME;

            tf::Vector3 target_vel(joy_msg->axes[1] * fabs(joy_msg->axes[1]) * max_target_vel_,
                                 joy_msg->axes[0] * fabs(joy_msg->axes[0]) * max_target_vel_, 0);

            /* defualt: world frame control */
            /* L2 trigger: fc(cog/ baselink frame) frame control */
            if(control_frame_ == flight_nav::LOCAL_FRAME)
              {
                tf::Vector3 target_vel_tmp = target_vel;
                target_vel = frameConversion(target_vel_tmp,  estimator_->getState(State::YAW, estimate_mode_)[0]);
              }

            /* interpolation for vel target */
            if(target_vel.x() - target_vel_.x() > joy_target_vel_interval_)
              target_vel_ += tf::Vector3(joy_target_vel_interval_, 0, 0);
            else if (target_vel.x() - target_vel_.x() < - joy_target_vel_interval_)
              target_vel_ -= tf::Vector3(joy_target_vel_interval_, 0, 0);
            else
              target_vel_.setX(target_vel.x());

            if(target_vel.y() - target_vel_.y() > joy_target_vel_interval_)
              target_vel_ += tf::Vector3(0, joy_target_vel_interval_, 0);
            else if (target_vel.y() - target_vel_.y() < - joy_target_vel_interval_)
              target_vel_ -= tf::Vector3(0, joy_target_vel_interval_, 0);
            else
              target_vel_.setY(target_vel.y());
          }
        break;
      }
    default:
      {
        break;
      }
    }
}

void Navigator::navigation()
{
  static int convergence_cnt = 0;
  static int clock_cnt = 0; //mainly for velocity control takeoff

  tf::Vector3 delta = target_pos_ - estimator_->getPos(Frame::COG, estimate_mode_);

  /* check the xy estimation status, if not ready, change to att_control_mode */
  if(!force_att_control_flag_)
    {
      if(!estimator_->getStateStatus(State::X_BASE, estimate_mode_) || !estimator_->getStateStatus(State::Y_BASE, estimate_mode_))
        {
          if(xy_control_mode_ == flight_nav::VEL_CONTROL_MODE ||
             xy_control_mode_ == flight_nav::POS_CONTROL_MODE)
            {
              ROS_ERROR("No estimation for X, Y state, change to attitude control mode");
              prev_xy_control_mode_ = xy_control_mode_;
              xy_control_mode_ = flight_nav::ACC_CONTROL_MODE;
            }
        }
      else
        {
          if(xy_control_mode_ == flight_nav::ACC_CONTROL_MODE &&
             prev_xy_control_mode_ != flight_nav::ACC_CONTROL_MODE)
            {
              ROS_ERROR("Estimation for X, Y state is established, siwtch back to the xy control mode");
              xy_control_mode_ = prev_xy_control_mode_;
            }
        }
    }

  /* sensor health check */
  if(estimator_->getUnhealthLevel() == BasicEstimator::UNHEALTH_LEVEL3 && !force_landing_flag_)
    {
      if(getNaviCommand() == TAKEOFF_COMMAND || getNaviCommand() == HOVER_COMMAND  || getNaviCommand() == LAND_COMMAND)
        ROS_WARN("Sensor Unhealth Level%d: force landing command", estimator_->getUnhealthLevel());
      std_msgs::UInt8 force_landing_cmd;
      force_landing_cmd.data = FORCE_LANDING_CMD;
      flight_config_pub_.publish(force_landing_cmd);
      force_landing_flag_ = true;
    }

  if(getNaviCommand() == TAKEOFF_COMMAND || getNaviCommand() == HOVER_COMMAND)
    {
      bool normal_land = false;

      /* joystick heartbeat check */
      if(check_joy_stick_heart_beat_ && joy_stick_heart_beat_ &&
         ros::Time::now().toSec() - joy_stick_prev_time_ > joy_stick_heart_beat_du_)
        {
          normal_land = true;
          ROS_ERROR("Normal Landing: att control mode, because no joy control");
        }

      /* low voltage flag */
      if(low_voltage_flag_)
        {
          normal_land = true;
          ROS_ERROR("Normal Landing: low battery");
        }

      if(normal_land)
        {
          setNaviCommand(LAND_COMMAND);
          setTargetXyFromCurrentState();
          setTargetPsiFromCurrentState();
          setTargetPosZ(estimator_->getLandingHeight());
        }
    }

  if(getNaviCommand() == START_COMMAND)
    { //takeoff phase
      flight_mode_= NO_CONTROL_MODE;
      estimator_->setSensorFusionFlag(true);
      force_landing_flag_ = false; //is here good?

      /* low voltage */
      if(low_voltage_flag_) setNaviCommand(IDLE_COMMAND);

    }
  else if(getNaviCommand() == TAKEOFF_COMMAND)
    { //Takeoff Phase
      /* set flying flag to true once */
      if(!estimator_->getFlyingFlag())
        estimator_->setFlyingFlag(true);

      flight_mode_= TAKEOFF_MODE;

      if(xy_control_mode_ == flight_nav::POS_CONTROL_MODE)
        {
          if (fabs(delta.z() < POS_Z_THRE && fabs(delta.x()) < POS_X_THRE && fabs(delta.x()) < POS_Y_THRE))
              convergence_cnt++;
        }
      else
        {
          if (fabs(delta.z()) < POS_Z_THRE) convergence_cnt++;
        }

      if (convergence_cnt > ctrl_loop_rate_)
        {
          convergence_cnt = 0;
          setNaviCommand(HOVER_COMMAND);
          ROS_WARN("Hovering!");
        }
    }
  else if(getNaviCommand() == LAND_COMMAND)
    {
      //for estimator landing mode
      estimator_->setLandingMode(true);

      if (getStartAble())
        {
          flight_mode_= LAND_MODE; //--> for control

          if (fabs(delta.z()) < POS_Z_THRE) convergence_cnt++;

          if (convergence_cnt > ctrl_loop_rate_)
            {
              convergence_cnt = 0;

              ROS_ERROR("disarm motors");
              setNaviCommand(STOP_COMMAND);
              flight_mode_= RESET_MODE;

              setTargetXyFromCurrentState();
              setTargetPsiFromCurrentState();

              estimator_->setSensorFusionFlag(false);
              estimator_->setLandingMode(false);
              estimator_->setLandedFlag(false);
              estimator_->setFlyingFlag(false);
            }
        }
      else
        {
          flight_mode_= NO_CONTROL_MODE;
          setNaviCommand(IDLE_COMMAND);
        }
    }
  else if(getNaviCommand() == HOVER_COMMAND)
    {
      if(vel_based_waypoint_)
        {
          /* vel nav */
          delta.setZ(0); // we do not need z
          if(delta.length() > vel_nav_threshold_)
            {
              //ROS_INFO("debug: vel based waypoint");
              tf::Vector3 nav_vel = delta * vel_nav_gain_;

              double speed = nav_vel.length();
              if(speed  > nav_vel_limit_) nav_vel *= (nav_vel_limit_ / speed);

              setTargetVelX(nav_vel.x());
              setTargetVelY(nav_vel.y());
            }
          else
            {
              ROS_WARN("back to pos nav control for way point");
              xy_control_mode_ = flight_nav::POS_CONTROL_MODE;
              vel_based_waypoint_ = false;
            }
        }
      flight_mode_= FLIGHT_MODE;
    }
  else if(getNaviCommand() == IDLE_COMMAND)
    {
      flight_mode_= NO_CONTROL_MODE;
    }
  else if(getNaviCommand() == STOP_COMMAND)
    {
      estimator_->setSensorFusionFlag(false);
      estimator_->setLandingMode(false);
      estimator_->setLandedFlag(false);
      estimator_->setFlyingFlag(false);

      if(flight_mode_ != RESET_MODE)
        flight_mode_= NO_CONTROL_MODE;
    }
  else
    {
      flight_mode_= NO_CONTROL_MODE;
      ROS_WARN("ERROR COMMAND, CAN NOT BE SEND TO AERIAL ROBOT");
    }
}

void Navigator::sendAttCmd()
{
  if(getNaviCommand() == START_COMMAND)
    {
      std_msgs::UInt8 start_cmd;
      start_cmd.data = ARM_ON_CMD;
      flight_config_pub_.publish(start_cmd);
    }
  else if(getNaviCommand() == STOP_COMMAND)
    {
      std_msgs::UInt8 stop_cmd;
      stop_cmd.data = ARM_OFF_CMD;
      flight_config_pub_.publish(stop_cmd);
    }
  else if(getNaviCommand() == TAKEOFF_COMMAND ||
          getNaviCommand() == LAND_COMMAND ||
          getNaviCommand() == HOVER_COMMAND)
    {
      aerial_robot_msgs::FourAxisCommand rc_command_data;
      rc_command_data.angles[0]  =  flight_ctrl_input_->getRollValue();
      rc_command_data.angles[1] =  flight_ctrl_input_->getPitchValue();

      rc_command_data.base_throttle.resize(flight_ctrl_input_->getMotorNumber());
      if(flight_ctrl_input_->getMotorNumber() == 1)
        {
          /* Simple PID based attitude/altitude control */
          rc_command_data.angles[2] = (flight_ctrl_input_->getYawValue())[0];
          rc_command_data.base_throttle[0] =  (flight_ctrl_input_->getThrottleValue())[0];
          if((flight_ctrl_input_->getThrottleValue())[0] == 0) return; // do not publish the empty flight command => the force landing flag will be activate
        }
      else
        {
          /* LQI based attitude/altitude control */
          for(int i = 0; i < flight_ctrl_input_->getMotorNumber(); i++)
            rc_command_data.base_throttle[i] = (flight_ctrl_input_->getYawValue())[i] + (flight_ctrl_input_->getThrottleValue())[i];
        }
      rc_cmd_pub_.publish(rc_command_data);
    }
  else
    {
      //ROS_ERROR("ERROR PISITION COMMAND, CAN NOT BE SEND TO Quadcopter");
    }
}


void Navigator::rosParamInit(ros::NodeHandle nh)
{
  std::string ns = nh.getNamespace();

  if (!nh.getParam ("xy_control_mode", xy_control_mode_))
    xy_control_mode_ = 0;
  printf("%s: xy_control_mode_ is %d\n", ns.c_str(), xy_control_mode_);

  if (!nh.getParam ("low_voltage_thre", low_voltage_thre_))
    low_voltage_thre_ = 105; //[10 * voltage]
  printf("%s: low_voltage_thre_ is %d\n", ns.c_str(), low_voltage_thre_);

  if (!nh.getParam ("takeoff_height", takeoff_height_))
    takeoff_height_ = 0;
  printf("%s: takeoff_height_ is %.3f\n", ns.c_str(), takeoff_height_);

  if (!nh.getParam ("max_target_vel", max_target_vel_))
    max_target_vel_ = 0;
  printf("%s: max_target_vel_ is %.3f\n", ns.c_str(), max_target_vel_);

  if (!nh.getParam ("max_target_yaw_rate", max_target_yaw_rate_))
    max_target_yaw_rate_ = 0;
  printf("%s: max_target_yaw_rate_ is %.3f\n", ns.c_str(), max_target_yaw_rate_);

  if (!nh.getParam ("max_target_tilt_angle", max_target_tilt_angle_))
    max_target_tilt_angle_ = 1.0;
  printf("%s: max_target_tilt_angle_ is %f\n", ns.c_str(), max_target_tilt_angle_);

  //*** auto vel nav
  nh.param("nav_vel_limit", nav_vel_limit_, 0.2);
  printf("%s: nav_vel_limit_ is %.3f\n", ns.c_str(), nav_vel_limit_);
  nh.param("vel_nav_threshold", vel_nav_threshold_, 0.4);
  printf("%s: vel_nav_threshold_ is %.3f\n", ns.c_str(), vel_nav_threshold_);
  nh.param("vel_nav_gain", vel_nav_gain_, 1.0);
  printf("%s: vel_nav_gain_ is %.3f\n", ns.c_str(), vel_nav_gain_);

    //*** teleop navigation
  if (!nh.getParam ("joy_target_vel_interval", joy_target_vel_interval_))
    joy_target_vel_interval_ = 0;
  printf("%s: joy_target_vel_interval_ is %.3f\n", ns.c_str(), joy_target_vel_interval_);

  if (!nh.getParam ("joy_target_alt_interval", joy_target_alt_interval_))
    joy_target_alt_interval_ = 0;
  printf("%s: joy_target_alt_interval_ is %.3f\n", ns.c_str(), joy_target_alt_interval_);

  if (!nh.getParam ("navi_frame_int", navi_frame_int_))
    navi_frame_int_ = 0;
  printf("%s: navi_frame_int_ is %d\n", ns.c_str(), navi_frame_int_);
  navi_frame_ = navi_frame_int_;

  if (!nh.getParam ("joy_stick_heart_beat_du", joy_stick_heart_beat_du_))
    joy_stick_heart_beat_du_ = 2.0;
  printf("%s: joy_stick_heart_beat_du_ is %f\n", ns.c_str(), joy_stick_heart_beat_du_);

  if (!nh.getParam ("force_landing_to_halt_du", force_landing_to_halt_du_))
    force_landing_to_halt_du_ = 1.0;
  printf("%s: force_landing_to_halt_du_ is %f\n", ns.c_str(), force_landing_to_halt_du_);

  if (!nh.getParam ("check_joy_stick_heart_beat", check_joy_stick_heart_beat_))
    check_joy_stick_heart_beat_ = false;
  printf("%s: check_joy_stick_heart_beat_ is %s\n", ns.c_str(), check_joy_stick_heart_beat_?std::string("true").c_str():std::string("false").c_str());

}

