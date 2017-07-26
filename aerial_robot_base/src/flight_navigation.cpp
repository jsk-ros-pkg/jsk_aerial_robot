#include "aerial_robot_base/flight_navigation.h"

Navigator::Navigator(ros::NodeHandle nh, ros::NodeHandle nh_private, 
                     BasicEstimator* estimator, FlightCtrlInput* flight_ctrl_input,
                     int ctrl_loop_rate)
  : nh_(nh, "navigator"),
    nhp_(nh_private, "navigator"),
    ctrl_loop_rate_(ctrl_loop_rate)
{
  Navigator::rosParamInit(nhp_);

  navi_sub_ = nh_.subscribe<aerial_robot_base::FlightNav>("/uav/nav", 1, &Navigator::naviCallback, this, ros::TransportHints().tcpNoDelay());

  battery_sub_ = nh_.subscribe<std_msgs::UInt8>("/battery_voltage_status", 1, &Navigator::batteryCheckCallback, this, ros::TransportHints().tcpNoDelay());

  estimator_ = estimator;
  estimate_mode_ = estimator_->getEstimateMode();
  flight_ctrl_input_ = flight_ctrl_input;

  br_ =  new tf::TransformBroadcaster();

  target_pos_x_ = 0;
  target_vel_x_ = 0;
  target_acc_x_ = 0;
  target_pos_y_ = 0;
  target_vel_y_ = 0;
  target_acc_y_ = 0;
  target_pos_z_ = 0;
  target_vel_z_ = 0;
  target_psi_ = 0;
  target_vel_psi_ = 0;

  stopNavigation();
  setNaviCommand( IDLE_COMMAND );

  force_att_control_flag_ = false;

  //base navigation mode init
  flight_mode_ = NO_CONTROL_MODE;
  low_voltage_flag_ = false;
  prev_xy_control_mode_ = flight_nav::ACC_CONTROL_MODE;
}

Navigator::~Navigator()
{
  printf(" deleted navigator!\n");
  delete br_;
}

void Navigator::naviCallback(const aerial_robot_base::FlightNavConstPtr & msg)
{
  /* xy control */
  switch(msg->pos_xy_nav_mode)
    {
    case aerial_robot_base::FlightNav::POS_MODE:
      {
        // ROS_INFO("change to pos control");
        force_att_control_flag_ = false;
        xy_control_mode_ = flight_nav::POS_CONTROL_MODE;
        setTargetPosX(msg->target_pos_x);
        setTargetPosY(msg->target_pos_y);
        break;
      }
    case aerial_robot_base::FlightNav::VEL_MODE:
      {
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
              setTargetAccX(msg->target_acc_x);
              setTargetAccY(msg->target_acc_y);
              break;
            }
          case flight_nav::LOCAL_FRAME:
            {
              tf::Vector3 target_acc = frameConversion(tf::Vector3(msg->target_acc_x, msg->target_acc_y, 0),  estimator_->getState(State::YAW, estimate_mode_)[0]);
              setTargetAccX(target_acc.x());
              setTargetAccY(target_acc.y());
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

  /* yaw */
  if(msg->psi_nav_mode == aerial_robot_base::FlightNav::POS_MODE)
    {
      setTargetPsi(msg->target_psi);
    }
}

void Navigator::batteryCheckCallback(const std_msgs::UInt8ConstPtr &msg)
{
  static double low_voltage_start_time = ros::Time::now().toSec();
  if(msg->data < low_voltage_thre_)
    {
      if(ros::Time::now().toSec() - low_voltage_start_time > 1.0)//1sec
        {
          ROS_WARN("low voltage!");
          low_voltage_flag_  = true;
        }
    }
  else
    {
      low_voltage_start_time = ros::Time::now().toSec();
    }
}

void Navigator::tfPublish()
{
  //TODO mutex
  tf::Transform map_to_target;
  tf::Quaternion tmp;

  map_to_target.setOrigin(tf::Vector3(target_pos_x_, target_pos_y_, target_pos_z_));
  tmp.setRPY(0.0, 0.0, target_psi_);
  map_to_target.setRotation(tmp);
  ros::Time tm = ros::Time::now();
  br_->sendTransform(tf::StampedTransform(map_to_target, tm, map_frame_, target_frame_));
}

void Navigator::rosParamInit(ros::NodeHandle nh)
{
  std::string ns = nh.getNamespace();

  //*** teleop navigation

  if (!nh.getParam ("map_frame", map_frame_))
    map_frame_ = "unknown";
  printf("%s: map_frame_ is %s\n", ns.c_str(), map_frame_.c_str());

  if (!nh.getParam ("target_frame", target_frame_))
    target_frame_ = "unknown";
  printf("%s: target_frame_ is %s\n", ns.c_str(), target_frame_.c_str());

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

}

TeleopNavigator::TeleopNavigator(ros::NodeHandle nh, ros::NodeHandle nh_private,
                                 BasicEstimator* estimator, 
                                 FlightCtrlInput* flight_ctrl_input,
                                 int ctrl_loop_rate)
  :Navigator(nh, nh_private, estimator, flight_ctrl_input, ctrl_loop_rate)
{
  TeleopNavigator::rosParamInit(nhp_);

  //joystick init
  vel_control_flag_ = false;
  pos_control_flag_ = false;
  xy_control_flag_ = false;
  alt_control_flag_ = false;
  yaw_control_flag_ = false;

  arming_ack_sub_ = nh_.subscribe<std_msgs::UInt8>("/flight_config_ack", 1, &TeleopNavigator::armingAckCallback, this, ros::TransportHints().tcpNoDelay());
  takeoff_sub_ = nh_.subscribe<std_msgs::Empty>("/teleop_command/takeoff", 1, &TeleopNavigator::takeoffCallback, this, ros::TransportHints().tcpNoDelay());
  halt_sub_ = nh_.subscribe<std_msgs::Empty>("/teleop_command/halt", 1, &TeleopNavigator::haltCallback, this, ros::TransportHints().tcpNoDelay());
  force_landing_sub_ = nh_.subscribe<std_msgs::Empty>("/teleop_command/force_landing", 1, &TeleopNavigator::forceLandingCallback, this, ros::TransportHints().tcpNoDelay());
  force_landing_flag_ = false;
  land_sub_ = nh_.subscribe<std_msgs::Empty>("/teleop_command/land", 1, &TeleopNavigator::landCallback, this, ros::TransportHints().tcpNoDelay());
  start_sub_ = nh_.subscribe<std_msgs::Empty>("/teleop_command/start", 1,&TeleopNavigator::startCallback, this, ros::TransportHints().tcpNoDelay());
  ctrl_mode_sub_ = nh_.subscribe<std_msgs::Int8>("/teleop_command/ctrl_mode", 1, &TeleopNavigator::xyControlModeCallback, this, ros::TransportHints().tcpNoDelay());

  joy_stick_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &TeleopNavigator::joyStickControl, this, ros::TransportHints().udp());

  rc_cmd_pub_ = nh_.advertise<aerial_robot_msgs::FourAxisCommand>("/aerial_robot_control_four_axis", 10);

  stop_teleop_sub_ = nh_.subscribe<std_msgs::UInt8>("stop_teleop", 1, &TeleopNavigator::stopTeleopCallback, this, ros::TransportHints().tcpNoDelay());
  teleop_flag_ = true;

  flight_config_pub_ = nh_.advertise<std_msgs::UInt8>("/flight_config_cmd", 10);

  joy_stick_heart_beat_ = false;
  joy_stick_prev_time_ = 0;

}

TeleopNavigator::~TeleopNavigator()
{
  printf(" deleted teleop navigator input!\n");
}

void TeleopNavigator::armingAckCallback(const std_msgs::UInt8ConstPtr& ack_msg)
{
  if(ack_msg->data == ARM_OFF_CMD)
    {//  arming off
      ROS_INFO("STOP RES From AERIAL ROBOT");
      stopNavigation();
      setNaviCommand(IDLE_COMMAND);
    }

  if(ack_msg->data == ARM_ON_CMD)
    {//  arming on
      ROS_INFO("START RES From AERIAL ROBOT");
      startNavigation();
      setNaviCommand(IDLE_COMMAND);
    }
}

void TeleopNavigator::takeoffCallback(const std_msgs::EmptyConstPtr & msg)
{
  startTakeoff();
}

void TeleopNavigator::startCallback(const std_msgs::EmptyConstPtr & msg)
{
  motorArming();
}

void TeleopNavigator::landCallback(const std_msgs::EmptyConstPtr & msg)
{
  setNaviCommand(LAND_COMMAND);
  //更新
  target_pos_x_ = getStatePosX(Frame::COG);
  target_pos_y_ = getStatePosY(Frame::COG);
  target_psi_   = getStatePsi();
  target_pos_z_ = estimator_->getLandingHeight();
  ROS_INFO("Land command");
}

void TeleopNavigator::haltCallback(const std_msgs::EmptyConstPtr & msg)
{
  setNaviCommand(STOP_COMMAND);
  flight_mode_ = RESET_MODE;
  setTargetPosX(getStatePosX(Frame::COG));
  setTargetPosY(getStatePosY(Frame::COG));
  setTargetPsi(getStatePsi());
  setTargetPosZ(estimator_->getLandingHeight());

  estimator_->setSensorFusionFlag(false);
  estimator_->setLandingMode(false);
  estimator_->setLandedFlag(false);
  estimator_->setFlyingFlag(false);

  ROS_INFO("Halt command");
}

void TeleopNavigator::forceLandingCallback(const std_msgs::EmptyConstPtr & msg)
{
  std_msgs::UInt8 force_landing_cmd;
  force_landing_cmd.data = FORCE_LANDING_CMD;
  flight_config_pub_.publish(force_landing_cmd); 

  ROS_INFO("Force Landing command");
}

void TeleopNavigator::xyControlModeCallback(const std_msgs::Int8ConstPtr & msg)
{
  if(getStartAble())
    {
      if(msg->data == 0)
        {
          xy_control_mode_ = flight_nav::POS_CONTROL_MODE;
          ROS_INFO("x/y position control mode");
        }
      if(msg->data == 1)
        {
          xy_control_mode_ = flight_nav::VEL_CONTROL_MODE;
          ROS_INFO("x/y velocity control mode");
        }
    }
}

void TeleopNavigator::stopTeleopCallback(const std_msgs::UInt8ConstPtr & stop_msg)
{
  if(stop_msg->data == 1) 
    {
      ROS_WARN("stop teleop control");
      teleop_flag_ = false;
    }
  else if(stop_msg->data == 0)
    {
      ROS_WARN("start teleop control");
      teleop_flag_ = true;
    }
}

void TeleopNavigator::joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
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
          setTargetPosX(getStatePosX(Frame::COG));
          setTargetPosY(getStatePosY(Frame::COG));
          setTargetPsi(getStatePsi());

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
      target_pos_x_= getStatePosX(Frame::COG);
      target_pos_y_= getStatePosY(Frame::COG);
      target_pos_z_= estimator_->getLandingHeight();
      target_psi_  = getStatePsi();
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
            target_pos_z_+= joy_target_alt_interval_;
          else
            target_pos_z_-= joy_target_alt_interval_;
          ROS_INFO("Joy Control: Thrust command");
        }
    }
  else
    {
      if(alt_control_flag_)
        {
          alt_control_flag_= false;
          target_pos_z_= getStatePosZ(Frame::COG);
          ROS_INFO("Joy Control: Fixed Alt command, targetPosz_is %f",target_pos_z_);
        }
    }

  /* Motion: Yaw */
  /* this is the yaw_angle control */
  if(joy_msg->buttons[2] == 1)
    {
      if(fabs(joy_msg->axes[2]) > 0.05)
        {
          target_psi_ = getStatePsi() + joy_msg->axes[2] * max_target_yaw_rate_;
          if(target_psi_ > M_PI)  target_psi_ -= 2 * M_PI;
          else if(target_psi_ < -M_PI)  target_psi_ += 2 * M_PI;
          ROS_WARN("Joy Control: yaw control based on angle control only");
        }
      else
        target_psi_ = getStatePsi();
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
      target_vel_x_= 0;
      target_vel_y_= 0;
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
      target_pos_x_= getStatePosX(Frame::COG);
      target_pos_y_= getStatePosY(Frame::COG);
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
        target_acc_x_ = joy_msg->axes[1] * max_target_tilt_angle_ * BasicEstimator::G;
        target_acc_y_ = joy_msg->axes[0] * max_target_tilt_angle_ * BasicEstimator::G;


        if(control_frame_ == flight_nav::LOCAL_FRAME)
          {
            tf::Vector3 target_acc = frameConversion(tf::Vector3(target_acc_x_, target_acc_y_, 0),  estimator_->getState(State::YAW, estimate_mode_)[0]);
            target_acc_x_ = target_acc.x();
            target_acc_y_ = target_acc.y();
          }

        break;
      }
    case flight_nav::VEL_CONTROL_MODE:
      {
        if(teleop_flag_)
          {
            control_frame_ = flight_nav::WORLD_FRAME;
            if(joy_msg->buttons[8]) control_frame_ = flight_nav::LOCAL_FRAME;

            float joy_target_vel_x= joy_msg->axes[1] * fabs(joy_msg->axes[1]) * max_target_vel_;
            float joy_target_vel_y = joy_msg->axes[0] * fabs(joy_msg->axes[0]) * max_target_vel_;

            /* defualt: world frame control */
            /* L2 trigger: fc(cog/ baselink frame) frame control */
            if(control_frame_ == flight_nav::LOCAL_FRAME)
              {
                tf::Vector3 joy_target_vel = frameConversion(tf::Vector3(joy_target_vel_x, joy_target_vel_y, 0),  estimator_->getState(State::YAW, estimate_mode_)[0]);
                joy_target_vel_x = joy_target_vel.x();
                joy_target_vel_y = joy_target_vel.y();
              }

            /* interpolation for vel target */
            if(joy_target_vel_x - target_vel_x_> joy_target_vel_interval_)
              target_vel_x_ += joy_target_vel_interval_;
            else if (joy_target_vel_x - target_vel_x_ < - joy_target_vel_interval_)
              target_vel_x_ -= joy_target_vel_interval_;
            else
              target_vel_x_ = joy_target_vel_x;
            if(joy_target_vel_y - target_vel_y_ > joy_target_vel_interval_)
              target_vel_y_ += joy_target_vel_interval_;
            else if (joy_target_vel_y - target_vel_y_ < - joy_target_vel_interval_)
              target_vel_y_ -= joy_target_vel_interval_;
            else
              target_vel_y_ = joy_target_vel_y;
          }
        break;
      }
    default:
      {
        break;
      }
    }
}

void TeleopNavigator::sendAttCmd()
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

void TeleopNavigator::teleopNavigation()
{
  static int convergence_cnt = 0;
  static int clock_cnt = 0; //mainly for velocity control takeoff

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
          target_pos_x_ = getStatePosX(Frame::COG);
          target_pos_y_ = getStatePosY(Frame::COG);
          target_pos_z_ = estimator_->getLandingHeight();
          target_psi_   = getStatePsi();
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
          if (fabs(getTargetPosZ() - getStatePosZ(Frame::COG)) < POS_Z_THRE &&
              fabs(getTargetPosX() - getStatePosX(Frame::COG)) < POS_X_THRE &&
              fabs(getTargetPosY() - getStatePosY(Frame::COG)) < POS_Y_THRE)
            convergence_cnt++;
        }
      else
        {
          //TODO => check same as pos_world_based_control_mode
          if (fabs(getTargetPosZ() - getStatePosZ(Frame::COG)) < POS_Z_THRE)
            convergence_cnt++;
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

          if (fabs(getTargetPosZ() - getStatePosZ(Frame::COG)) < POS_Z_THRE)
            convergence_cnt++;

          if (convergence_cnt > ctrl_loop_rate_)
            {
              convergence_cnt = 0;

              ROS_ERROR("disarm motors");
              setNaviCommand(STOP_COMMAND);
              flight_mode_= RESET_MODE;

              setTargetPosX(getStatePosX(Frame::COG));
              setTargetPosY(getStatePosY(Frame::COG));
              setTargetPsi(getStatePsi());

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

void TeleopNavigator::rosParamInit(ros::NodeHandle nh)
{
  std::string ns = nh.getNamespace();
  //*** teleop navigation
  if (!nh.getParam ("even_move_distance", even_move_distance_))
    even_move_distance_ = 0;
  printf("%s: even_move_distance_ is %.3f\n", ns.c_str(), even_move_distance_);

  if (!nh.getParam ("up_down_distance", up_down_distance_))
    up_down_distance_ = 0;
  printf("%s: up_down_distance_ is %.3f\n", ns.c_str(), up_down_distance_);

  if (!nh.getParam ("forward_backward_distance", forward_backward_distance_))
    forward_backward_distance_ = 0;
  printf("%s: forward_backward_distance_ is %.3f\n", ns.c_str(), forward_backward_distance_);

  if (!nh.getParam ("left_right_distance", left_right_distance_))
    left_right_distance_ = 0;
  printf("%s: left_right_distance_ is %.3f\n", ns.c_str(), left_right_distance_);

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
