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
  state_mode_ = estimator_->getStateMode();
  flight_ctrl_input_ = flight_ctrl_input;

  br_ =  new tf::TransformBroadcaster();

  final_target_pos_x_ = 0;
  final_target_vel_x_ = 0;
  final_target_pos_y_ = 0;
  final_target_vel_y_ = 0;
  final_target_pos_z_ = 0;
  final_target_vel_z_ = 0;
  final_target_theta_ = 0;
  final_target_vel_theta_ = 0;
  final_target_phy_ = 0;
  final_target_vel_phy_ = 0;
  final_target_psi_ = 0;
  final_target_vel_psi_ = 0;

  //current target value
  current_target_pos_x_ = 0;
  current_target_vel_x_ = 0;
  current_target_pos_y_ = 0;
  current_target_vel_y_ = 0;
  current_target_pos_z_ = 0;
  current_target_vel_z_ = 0;
  current_target_theta_ = 0;
  current_target_vel_theta_ = 0;
  current_target_phy_ = 0;
  current_target_vel_phy_ = 0;
  current_target_psi_ = 0;
  current_target_vel_psi_ = 0;

  target_pitch_angle_ = 0;
  target_roll_angle_ = 0;


  stopNavigation();
  setNaviCommand( IDLE_COMMAND );

  //base navigation mode init
  flight_mode_ = NO_CONTROL_MODE;
  low_voltage_flag_ = false;
}

Navigator::~Navigator()
{
  printf(" deleted navigator!\n");
  delete br_;
}

void Navigator::naviCallback(const aerial_robot_base::FlightNavConstPtr & msg)
{
  //control mode change (pos/vel)
  if(msg->pos_xy_nav_mode == aerial_robot_base::FlightNav::VEL_MODE)
    {
      //only change mode in world based control (only optical flow in forbidden)
      if(xy_control_mode_ == POS_WORLD_BASED_CONTROL_MODE)
        {
          ROS_INFO("change to vel pos-based control");
          xy_control_mode_ = VEL_WORLD_BASED_CONTROL_MODE;
        }
    }
  if(msg->pos_xy_nav_mode == aerial_robot_base::FlightNav::POS_MODE)
    {
      //only change mode in world based control (only optical flow in forbidden)
      if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE)
        {
          ROS_INFO("change to pos control");
          xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
        }
    }

  //for x & y
  if(msg->pos_xy_nav_mode == aerial_robot_base::FlightNav::VEL_MODE)
    {
      setTargetVelX(msg->target_vel_x);
      setTargetVelY(msg->target_vel_y);
    }
  else if(msg->pos_xy_nav_mode == aerial_robot_base::FlightNav::POS_MODE)
    {
      setTargetPosX(msg->target_pos_x);
      setTargetPosY(msg->target_pos_y);
    }

  //for z
  if(msg->pos_z_nav_mode == aerial_robot_base::FlightNav::VEL_MODE)
    {
      addTargetPosZ(msg->target_pos_diff_z);
    }
  else if(msg->pos_z_nav_mode == aerial_robot_base::FlightNav::POS_MODE)
    {
      setTargetPosZ(msg->target_pos_z);
    }

  //for psi
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
      if(ros::Time::now().toSec() - low_voltage_start_time > 10.0)//10sec
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

  map_to_target.setOrigin(tf::Vector3(current_target_pos_x_, current_target_pos_y_, current_target_pos_z_));
  tmp.setRPY(0.0, 0.0, current_target_psi_);
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
    low_voltage_thre_ = 10; //10%
  printf("%s: low_voltage_thre_ is %d\n", ns.c_str(), low_voltage_thre_);

  //hidden variable
  if (!nh.getParam ("xy_vel_mode_pos_ctrl_takeoff", xy_vel_mode_pos_ctrl_takeoff_))
    xy_vel_mode_pos_ctrl_takeoff_ = true;
  printf("%s: xy_vel_mode_pos_ctrl_takeoff is %s\n", ns.c_str(), xy_vel_mode_pos_ctrl_takeoff_ ? ("true") : ("false"));
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

void TeleopNavigator::takeoffCallback(const std_msgs::EmptyConstPtr & msg){
  if(getStartAble())
    {
      if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE) xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
      setNaviCommand(TAKEOFF_COMMAND);
      ROS_INFO("Takeoff command");
    }
}

void TeleopNavigator::startCallback(const std_msgs::EmptyConstPtr & msg)
{//すべて軸に対して、初期化
  setNaviCommand(START_COMMAND);
  final_target_pos_x_ = getStatePosX();
  final_target_pos_y_ = getStatePosY();
  final_target_psi_   = getStatePsiBoard();
  final_target_pos_z_ = takeoff_height_;
  ROS_INFO("Start command");
}

void TeleopNavigator::landCallback(const std_msgs::EmptyConstPtr & msg)
{
  if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE) 
    xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
  setNaviCommand(LAND_COMMAND);
  //更新
  final_target_pos_x_ = getStatePosX();
  final_target_pos_y_ = getStatePosY();
  final_target_psi_   = getStatePsiBoard();
  final_target_pos_z_ = estimator_->getLandingHeight();
  ROS_INFO("Land command");
}

void TeleopNavigator::haltCallback(const std_msgs::EmptyConstPtr & msg)
{
  if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE) 
    xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;

  setNaviCommand(STOP_COMMAND);
  flight_mode_ = RESET_MODE;
  setTargetPosX(getStatePosX());
  setTargetPosY(getStatePosY());
  setTargetPsi(getStatePsiBoard());
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
          xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
          ROS_INFO("x/y position control mode");
        }
      if(msg->data == 1)
        {
          xy_control_mode_ = VEL_LOCAL_BASED_CONTROL_MODE;
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
  joy_stick_prev_time_ = ros::Time::now().toSec();

  /* common command */
  /* start */
  if(joy_msg->buttons[3] == 1 && !getStartAble())
    {
      setNaviCommand(START_COMMAND);
      final_target_pos_x_ = getStatePosX();
      final_target_pos_y_ = getStatePosY();
      final_target_pos_z_ = takeoff_height_ + getStatePosZ();
      final_target_psi_ = getStatePsiBoard();
      ROS_WARN("Joy Control: Start command,  final_target_pos_x_: %f, final_target_pos_y_: %f, final_target_pos_z_: %f, final_target_psi_: %f", final_target_pos_x_, final_target_pos_y_, final_target_pos_z_, final_target_psi_);
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
          setTargetPosX(getStatePosX());
          setTargetPosY(getStatePosY());
          setTargetPsi(getStatePsiBoard());

          /* several flag should be false */
          estimator_->setSensorFusionFlag(false);
          estimator_->setLandingMode(false);
          estimator_->setLandedFlag(false);
          estimator_->setFlyingFlag(false);

          /* the pos-vel mode update */
          if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE)
            xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
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
      if(getNaviCommand() == TAKEOFF_COMMAND) return;
      if(getStartAble())
        {
          if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE)
            xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;

          setNaviCommand(TAKEOFF_COMMAND);
          ROS_INFO("Joy Control: Takeoff command");
        }
      return;
    }

  /* landing */
  if(joy_msg->buttons[5] == 1 && joy_msg->buttons[15] == 1)
    {
      if(getNaviCommand() == LAND_COMMAND) return;

      if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE)
        xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;

      if(xy_control_mode_ == VEL_LOCAL_BASED_CONTROL_MODE ||
         xy_control_mode_ == POS_LOCAL_BASED_CONTROL_MODE)
        xy_control_mode_ = VEL_LOCAL_BASED_CONTROL_MODE;

      setNaviCommand(LAND_COMMAND);
      //update
      final_target_pos_x_= getStatePosX();
      final_target_pos_y_= getStatePosY();
      final_target_pos_z_= estimator_->getLandingHeight();
      final_target_psi_  = getStatePsiBoard();
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
            final_target_pos_z_+= target_alt_interval_;
          else
            final_target_pos_z_-= target_alt_interval_;
          ROS_INFO("Joy Control: Thrust command");
        }
    }
  else
    {
      if(alt_control_flag_)
        {
          alt_control_flag_= false;
          final_target_pos_z_= getStatePosZ();
          ROS_INFO("Joy Control: Fixed Alt command, targetPosz_is %f",final_target_pos_z_);
        }
    }

  /* Motion: Yaw */
  /* this is the yaw_angle control */
  if(getStatePsiBoard() != 0 ||
     xy_control_mode_ == POS_WORLD_BASED_CONTROL_MODE ||
     xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE)
    {
      if(joy_msg->buttons[2] == 1)
        {
          if(fabs(joy_msg->axes[2]) > 0.05)
            {
              final_target_psi_ = getStatePsiBoard() + joy_msg->axes[2] * target_yaw_rate_;
              if(final_target_psi_ > M_PI)  final_target_psi_ -= 2 * M_PI;
              else if(final_target_psi_ < -M_PI)  final_target_psi_ += 2 * M_PI;
              ROS_WARN("Joy Control: yaw control based on angle control only");
            }
          else
            final_target_psi_ = getStatePsiBoard();
        }
    }
  /* this is the non yaw_angle control(that is vel control) */
  if(getStatePsiBoard() == 0)
    {
      if(xy_control_mode_ == ATT_CONTROL_MODE ||
         xy_control_mode_ == VEL_LOCAL_BASED_CONTROL_MODE ||
         xy_control_mode_ == POS_LOCAL_BASED_CONTROL_MODE)
        {
          /*ROS_WARN("Joy Control: yaw control based on velocity(gyro) control only");*/
          final_target_psi_ = joy_msg->axes[2] * target_yaw_rate_;
        }
    }

  /* turn to ATT_CONTROL_MODE */
  static uint8_t prev_control_mode = 0;
  if(joy_msg->buttons[6] == 1)
    {
      if(xy_control_mode_ == POS_WORLD_BASED_CONTROL_MODE ||
         xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE ||
         xy_control_mode_ == VEL_LOCAL_BASED_CONTROL_MODE ||
         xy_control_mode_ == POS_LOCAL_BASED_CONTROL_MODE)
        {
          ROS_WARN("Joy Control: Change to ATT_CONTROL_MODE");
          prev_control_mode = xy_control_mode_;
          xy_control_mode_ = ATT_CONTROL_MODE;
        }
    }
  if(joy_msg->buttons[4] == 1)
    {
      if(xy_control_mode_ == ATT_CONTROL_MODE)
        {
          ROS_WARN("Joy Control: Change to NON_ATT_CONTROL_MODE");
          xy_control_mode_ = prev_control_mode;
        }
    }

  /* mode oriented command */
  if(xy_control_mode_ == ATT_CONTROL_MODE)
    {
      /* pitch && roll angle command */
      target_pitch_angle_ = joy_msg->axes[1] * target_angle_rate_;
      target_roll_angle_ = - joy_msg->axes[0]  * target_angle_rate_;
    }
  else if(xy_control_mode_ == POS_WORLD_BASED_CONTROL_MODE || xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE)
    {
      /* change to vel control mode */
      if(joy_msg->buttons[12] == 1 && !vel_control_flag_)
        {
          ROS_INFO("change to vel pos-based control");
          vel_control_flag_ = true;
          xy_control_mode_ = VEL_WORLD_BASED_CONTROL_MODE;
          final_target_vel_x_= 0; current_target_vel_x_= 0;
          final_target_vel_y_= 0; current_target_vel_y_= 0;
        }
      if(joy_msg->buttons[12] == 0 && vel_control_flag_)
        vel_control_flag_ = false;

      /* change to pos control mode */
      if(joy_msg->buttons[14] == 1 && !pos_control_flag_)
        {
          ROS_INFO("change to pos control");
          pos_control_flag_ = true;
          xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
          final_target_pos_x_= getStatePosX();
          final_target_pos_y_= getStatePosY();
        }
      if(joy_msg->buttons[14] == 0 && pos_control_flag_)
        pos_control_flag_ = false;

      if(getNaviCommand() == HOVER_COMMAND && teleop_flag_)
        {
          /* pitch && roll vel command for vel_mode */
          if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE && joy_msg->buttons[8] == 1)
            {//only push the left joysitck will be active
              final_target_vel_x_= joy_msg->axes[1] * fabs(joy_msg->axes[1]) * target_vel_rate_;
              final_target_vel_y_= joy_msg->axes[0] * fabs(joy_msg->axes[0]) * target_vel_rate_;
              xy_control_flag_ = true;
            }
          if(xy_control_flag_ && joy_msg->buttons[8] == 0)
            {
              final_target_vel_x_ = 0;
              final_target_vel_y_ = 0;
              xy_control_flag_ = false;
            }
        }
    }
  else if(xy_control_mode_ == VEL_LOCAL_BASED_CONTROL_MODE || xy_control_mode_ == POS_LOCAL_BASED_CONTROL_MODE)
    {
      if(getNaviCommand() == HOVER_COMMAND && teleop_flag_)
        {
          /* horizontal move */
          if(joy_msg->buttons[8] == 1)
            {
              final_target_vel_x_= joy_msg->axes[1] * fabs(joy_msg->axes[1]) * target_vel_rate_;
              final_target_vel_y_= joy_msg->axes[0] * fabs(joy_msg->axes[0]) * target_vel_rate_;
              xy_control_flag_ = true;
            }
          if(xy_control_flag_ && joy_msg->buttons[8] == 0)
            {
              final_target_vel_x_ = 0;
              final_target_vel_y_ = 0;
              xy_control_flag_ = false;
            }
        }
    }
}

void TeleopNavigator::targetValueCorrection()
{
  //no keystone correction
  current_target_pos_x_  = final_target_pos_x_;
  current_target_pos_y_= final_target_pos_y_;
  current_target_pos_z_  = final_target_pos_z_;
  current_target_vel_z_  = final_target_vel_z_;
  current_target_theta_ = final_target_theta_;
  current_target_vel_theta_ = final_target_vel_theta_;

  current_target_phy_ = final_target_phy_;
  current_target_vel_phy_ = final_target_vel_phy_;
  current_target_psi_ = final_target_psi_;
  current_target_vel_psi_ = final_target_vel_psi_;

#if 1 //keystone
  // for pitch and roll velocity control
  //pitch
  if(final_target_vel_x_ - current_target_vel_x_ > target_pitch_roll_interval_)
    current_target_vel_x_ += target_pitch_roll_interval_;
  else if (final_target_vel_x_ - current_target_vel_x_ < - target_pitch_roll_interval_)
    current_target_vel_x_ -= target_pitch_roll_interval_;
  else
    current_target_vel_x_ = final_target_vel_x_;
  //roll
  if(final_target_vel_y_ - current_target_vel_y_ > target_pitch_roll_interval_)
    current_target_vel_y_ += target_pitch_roll_interval_;
  else if (final_target_vel_y_ - current_target_vel_y_ < - target_pitch_roll_interval_)
    current_target_vel_y_ -= target_pitch_roll_interval_;
  else
    current_target_vel_y_ = final_target_vel_y_;
#else
  current_target_vel_x_ = final_target_vel_x_;
  current_target_vel_y_ = final_target_vel_y_;
#endif 
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
      for(int i = 0; i < flight_ctrl_input_->getMotorNumber(); i++)
        rc_command_data.base_throttle[i] = (flight_ctrl_input_->getYawValue())[i] + (flight_ctrl_input_->getThrottleValue())[i];
      rc_cmd_pub_.publish(rc_command_data);


#if 0 //old send method
      if(flight_ctrl_input_->getMotorNumber() > 1)
        {
          aerial_robot_msgs::FourAxisCommand flight_command_data;
          four_axis_command_data.angles[0]  =  flight_ctrl_input_->getRollValue();
          four_axis_command_data.angles[1] =  flight_ctrl_input_->getPitchValue();
          for(int i =0; i < flight_ctrl_input_->getMotorNumber(); i++)
            {
              four_axis_command_data.yaw_pi_term[i]   =  (flight_ctrl_input_->getYawValue())[i];
              four_axis_command_data.throttle_pid_term[i] = (flight_ctrl_input_->getThrottleValue())[i] ;
            }
          rc_cmd_pub_.publish(four_axis_command_data);
        }
      else
        {
          aerial_robot_msgs::RcData rc_data;
          rc_data.roll  =  flight_ctrl_input_->getRollValue();
          rc_data.pitch =  flight_ctrl_input_->getPitchValue();
          rc_data.yaw   =  (flight_ctrl_input_->getYawValue())[0];
          rc_data.throttle = (flight_ctrl_input_->getThrottleValue())[0];
          rc_cmd_pub_.publish(rc_data);
        }
#endif
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

  //keystron correction / target value process
  targetValueCorrection();

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

      //TODO convergenceFunction();
      if(xy_control_mode_ == POS_WORLD_BASED_CONTROL_MODE)
        {
          if (fabs(getTargetPosZ() - getStatePosZ()) < POS_Z_THRE &&
              fabs(getTargetPosX() - getStatePosX()) < POS_X_THRE &&
              fabs(getTargetPosY() - getStatePosY()) < POS_Y_THRE)
            convergence_cnt++;
        }
      else if(xy_control_mode_ == VEL_LOCAL_BASED_CONTROL_MODE ||
	      xy_control_mode_ == ATT_CONTROL_MODE)
        {
          //TODO => check same as pos_world_based_control_mode
          if (fabs(getTargetPosZ() - getStatePosZ()) < POS_Z_THRE)
            convergence_cnt++;
        }

      if (convergence_cnt > ctrl_loop_rate_) 
        { //*** 安定収束した 20 ~ 40
          if(xy_control_mode_ == POS_WORLD_BASED_CONTROL_MODE ||
	     xy_control_mode_ == ATT_CONTROL_MODE)
            {
              convergence_cnt = 0;
              setNaviCommand(HOVER_COMMAND); 
              ROS_WARN("Hovering!");
            }
          else if(xy_control_mode_ == VEL_LOCAL_BASED_CONTROL_MODE)
            {
              if(xy_vel_mode_pos_ctrl_takeoff_)
                {
                  clock_cnt++;
                  if(clock_cnt % 2 == 0) ROS_INFO("stable, clock_cnt: %.0f%% ", (float)clock_cnt / (ctrl_loop_rate_ * TAKEOFF_COUNT) * 100);
                  if(clock_cnt > (ctrl_loop_rate_ * TAKEOFF_COUNT))
                    {
                      clock_cnt = 0;
                      convergence_cnt = 0;
                      setNaviCommand(HOVER_COMMAND); 
                      ROS_WARN("Hovering!");
                    }
                }
              else
                {
                  convergence_cnt = 0;
                  setNaviCommand(HOVER_COMMAND); 
                  ROS_WARN("Hovering!");
                }
            }
        }

      /* check the joy stick heart beat */
      bool force_land = false;
      if(joy_stick_prev_time_ != 0 && ros::Time::now().toSec() - joy_stick_prev_time_ > joy_stick_heart_beat_du_ && xy_control_mode_ == ATT_CONTROL_MODE)
	{
	  ROS_ERROR("Force Landing: att control mode, because no joy control");
	  force_land = true;
	}

      if(low_voltage_flag_)
        {
	  ROS_ERROR("Force Landing: low battery");
	  force_land = true;
	}

      if(force_land)
	{
	  setNaviCommand(LAND_COMMAND);
	  final_target_pos_x_ = getStatePosX();
	  final_target_pos_y_ = getStatePosY();
	  final_target_pos_z_ = estimator_->getLandingHeight();
	  final_target_psi_   = getStatePsiBoard();
        }
    }
  else if(getNaviCommand() == LAND_COMMAND)
    {
      //for estimator landing mode
      estimator_->setLandingMode(true);

      if (getStartAble())
        {
          flight_mode_= LAND_MODE; //--> for control

          if (fabs(getTargetPosZ() - getStatePosZ()) < POS_Z_THRE)
            convergence_cnt++;

          if (convergence_cnt > ctrl_loop_rate_)
            {
              convergence_cnt = 0;

              ROS_ERROR("disarm motors");
              setNaviCommand(STOP_COMMAND);
              flight_mode_= RESET_MODE;

              setTargetPosX(getStatePosX());
              setTargetPosY(getStatePosY());
              setTargetPsi(getStatePsiBoard());

              estimator_->setSensorFusionFlag(false);
              estimator_->setLandingMode(false);
              estimator_->setLandedFlag(false);
              estimator_->setFlyingFlag(false);

              if(xy_control_mode_ == VEL_WORLD_BASED_CONTROL_MODE)
                xy_control_mode_ = POS_WORLD_BASED_CONTROL_MODE;
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


      /* check the joy stick heart beat */
      bool force_land = false;
      if(joy_stick_prev_time_ != 0 && ros::Time::now().toSec() - joy_stick_prev_time_ > joy_stick_heart_beat_du_ && xy_control_mode_ == ATT_CONTROL_MODE)
	{
	  ROS_ERROR("Force Landing: att control mode, because no joy control");
	  force_land = true;
	}

      if(low_voltage_flag_)
        {
	  ROS_ERROR("Force Landing: low battery");
	  force_land = true;
	}

      if(force_land)
	{
	  setNaviCommand(LAND_COMMAND);
	  final_target_pos_x_ = getStatePosX();
	  final_target_pos_y_ = getStatePosY();
	  final_target_pos_z_ = estimator_->getLandingHeight();
	  final_target_psi_   = getStatePsiBoard();
        }
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
  if (!nh.getParam ("takeoff_height", takeoff_height_))
    takeoff_height_ = 0;
  printf("%s: takeoff_height_ is %.3f\n", ns.c_str(), takeoff_height_);

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

  if (!nh.getParam ("target_vel_rate", target_vel_rate_))
    target_vel_rate_ = 0;
  printf("%s: target_vel_rate_ is %.3f\n", ns.c_str(), target_vel_rate_);

  if (!nh.getParam ("target_pitch_roll_interval", target_pitch_roll_interval_))
    target_pitch_roll_interval_ = 0;
  printf("%s: target_pitch_roll_interval_ is %.3f\n", ns.c_str(), target_pitch_roll_interval_);

  if (!nh.getParam ("target_alt_interval", target_alt_interval_))
    target_alt_interval_ = 0;
  printf("%s: target_alt_interval_ is %.3f\n", ns.c_str(), target_alt_interval_);

  if (!nh.getParam ("target_yaw_rate", target_yaw_rate_))
    target_yaw_rate_ = 0;
  printf("%s: target_yaw_rate_ is %.3f\n", ns.c_str(), target_yaw_rate_);

  if (!nh.getParam ("navi_frame_int", navi_frame_int_))
    navi_frame_int_ = 0;
  printf("%s: navi_frame_int_ is %d\n", ns.c_str(), navi_frame_int_);
  navi_frame_ = navi_frame_int_;

  if (!nh.getParam ("target_angle_rate", target_angle_rate_))
    target_angle_rate_ = 1.0;
  printf("%s: target_angle_rate_ is %f\n", ns.c_str(), target_angle_rate_);

  if (!nh.getParam ("joy_stick_heart_beat_du", joy_stick_heart_beat_du_))
    joy_stick_heart_beat_du_ = 1.0;
  printf("%s: joy_stick_heart_beat_du_ is %f\n", ns.c_str(), joy_stick_heart_beat_du_);

  if (!nh.getParam ("force_landing_to_halt_du", force_landing_to_halt_du_))
    force_landing_to_halt_du_ = 1.0;
  printf("%s: force_landing_to_halt_du_ is %f\n", ns.c_str(), force_landing_to_halt_du_);

}
