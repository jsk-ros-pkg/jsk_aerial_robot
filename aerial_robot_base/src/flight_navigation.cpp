#include "aerial_robot_base/flight_navigation.h"

using namespace std;

Navigator::Navigator(ros::NodeHandle nh, ros::NodeHandle nh_private,
                     BasicEstimator* estimator)
  : nh_(nh, "navigator"),
    nhp_(nh_private, "navigator"),
    target_pos_(0, 0, 0),
    target_vel_(0, 0, 0),
    target_acc_(0, 0, 0),
    target_psi_(0), target_vel_psi_(0),
    force_att_control_flag_(false),
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

  battery_sub_ = nh_.subscribe<std_msgs::Float32>("/battery_voltage_status", 1, &Navigator::batteryCheckCallback, this, ros::TransportHints().tcpNoDelay());

  arming_ack_sub_ = nh_.subscribe<std_msgs::UInt8>("/flight_config_ack", 1, &Navigator::armingAckCallback, this, ros::TransportHints().tcpNoDelay());
  takeoff_sub_ = nh_.subscribe<std_msgs::Empty>("/teleop_command/takeoff", 1, &Navigator::takeoffCallback, this, ros::TransportHints().tcpNoDelay());
  halt_sub_ = nh_.subscribe<std_msgs::Empty>("/teleop_command/halt", 1, &Navigator::haltCallback, this, ros::TransportHints().tcpNoDelay());
  force_landing_sub_ = nh_.subscribe<std_msgs::Empty>("/teleop_command/force_landing", 1, &Navigator::forceLandingCallback, this, ros::TransportHints().tcpNoDelay());
  force_landing_flag_ = false;
  land_sub_ = nh_.subscribe<std_msgs::Empty>("/teleop_command/land", 1, &Navigator::landCallback, this, ros::TransportHints().tcpNoDelay());
  start_sub_ = nh_.subscribe<std_msgs::Empty>("/teleop_command/start", 1,&Navigator::startCallback, this, ros::TransportHints().tcpNoDelay());
  ctrl_mode_sub_ = nh_.subscribe<std_msgs::Int8>("/teleop_command/ctrl_mode", 1, &Navigator::xyControlModeCallback, this, ros::TransportHints().tcpNoDelay());

  joy_stick_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &Navigator::joyStickControl, this, ros::TransportHints().udp());

  stop_teleop_sub_ = nh_.subscribe<std_msgs::UInt8>("stop_teleop", 1, &Navigator::stopTeleopCallback, this, ros::TransportHints().tcpNoDelay());
  teleop_flag_ = true;

  flight_config_pub_ = nh_.advertise<aerial_robot_base::FlightConfigCmd>("/flight_config_cmd", 10);
  power_info_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/uav_power", 10);

  estimator_ = estimator;
  estimate_mode_ = estimator_->getEstimateMode();
  setNaviState( ARM_OFF_STATE );
}

Navigator::~Navigator()
{
  printf(" deleted navigator!\n");
}

void Navigator::batteryCheckCallback(const std_msgs::Float32ConstPtr &msg)
{
  if(bat_cell_ == 0)
    {
      ROS_WARN("No correct battery information, cell is 0");
      return;
    }

  float voltage = msg->data;
  /* consider the voltage drop */
  if(getNaviState() == TAKEOFF_STATE || getNaviState() == HOVER_STATE)
    voltage += ( (bat_resistance_voltage_rate_ * voltage +  bat_resistance_) * hovering_current_);

  float average_voltage = voltage / bat_cell_;
  float percentage = 0;
  if(average_voltage  > VOLTAGE_90P) percentage = (average_voltage - VOLTAGE_90P) / (VOLTAGE_100P - VOLTAGE_90P) * 10 + 90;
  else if (average_voltage  > VOLTAGE_80P) percentage = (average_voltage - VOLTAGE_80P) / (VOLTAGE_90P - VOLTAGE_80P) * 10 + 80;
  else if (average_voltage  > VOLTAGE_70P) percentage = (average_voltage - VOLTAGE_70P) / (VOLTAGE_80P - VOLTAGE_70P) * 10 + 70;
  else if (average_voltage  > VOLTAGE_60P) percentage = (average_voltage - VOLTAGE_60P) / (VOLTAGE_70P - VOLTAGE_60P) * 10 + 60;
  else if (average_voltage  > VOLTAGE_50P) percentage = (average_voltage - VOLTAGE_50P) / (VOLTAGE_60P - VOLTAGE_50P) * 10 + 50;
  else if (average_voltage  > VOLTAGE_40P) percentage = (average_voltage - VOLTAGE_40P) / (VOLTAGE_50P - VOLTAGE_40P) * 10 + 40;
  else if (average_voltage  > VOLTAGE_30P) percentage = (average_voltage - VOLTAGE_30P) / (VOLTAGE_40P - VOLTAGE_30P) * 10 + 30;
  else if (average_voltage  > VOLTAGE_20P) percentage = (average_voltage - VOLTAGE_20P) / (VOLTAGE_30P - VOLTAGE_20P) * 10 + 20;
  else if (average_voltage  > VOLTAGE_10P) percentage = (average_voltage - VOLTAGE_10P) / (VOLTAGE_20P - VOLTAGE_10P) * 10 + 10;
  else percentage = (average_voltage - VOLTAGE_0P) / (VOLTAGE_10P - VOLTAGE_0P) * 10;

  if (percentage > 100) percentage = 100;
  if(percentage < 0)
    {
      /* can remove this information */
      ROS_WARN("no correct voltage information from spinal");
      return;
    }

  if(percentage < low_voltage_thre_)
    {
      low_voltage_flag_  = true;
      ROS_WARN_THROTTLE(1,"low voltage!");
    }

  geometry_msgs::Vector3Stamped power_info_msgs;
  power_info_msgs.header.stamp = ros::Time::now();
  power_info_msgs.vector.x = voltage;
  power_info_msgs.vector.y = percentage;
  power_info_pub_.publish(power_info_msgs);
}

void Navigator::naviCallback(const aerial_robot_base::FlightNavConstPtr & msg)
{
  if(getNaviState() == TAKEOFF_STATE || getNaviState() == LAND_STATE) return;

  /* yaw */
  if(msg->psi_nav_mode == aerial_robot_base::FlightNav::POS_MODE)
    {
      double target_psi = msg->target_psi;
      if(target_psi > M_PI)  target_psi -= (2 * M_PI);
      else if(target_psi < -M_PI)  target_psi += (2 * M_PI);

      setTargetPsi(target_psi);
    }
  if(msg->psi_nav_mode == aerial_robot_base::FlightNav::POS_VEL_MODE)
    {
      double target_psi = msg->target_psi;
      if(target_psi > M_PI)  target_psi -= (2 * M_PI);
      else if(target_psi < -M_PI)  target_psi += (2 * M_PI);

      setTargetPsi(target_psi);
      setTargetPsiVel(msg->target_vel_psi);
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
              tf::Vector3 target_vel = frameConversion(tf::Vector3(msg->target_vel_x, msg->target_vel_y, 0),  estimator_->getState(State::YAW_COG, estimate_mode_)[0]);
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
    case aerial_robot_base::FlightNav::POS_VEL_MODE:
      {
        force_att_control_flag_ = false;

        if(msg->target == aerial_robot_base::FlightNav::BASELINK)
          {
            ROS_ERROR("[Flight nav] can not do pos_vel nav for baselink");
            return;
          }

        xy_control_mode_ = flight_nav::POS_CONTROL_MODE;
        setTargetPosX(msg->target_pos_x);
        setTargetPosY(msg->target_pos_y);
        setTargetVelX(msg->target_vel_x);
        setTargetVelY(msg->target_vel_y);

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
              target_acc_ = frameConversion(target_acc,  estimator_->getState(State::YAW_COG, estimate_mode_)[0]);
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
      /* special */
      addTargetPosZ(msg->target_pos_diff_z);
    }
  else if(msg->pos_z_nav_mode == aerial_robot_base::FlightNav::POS_MODE)
    {
      setTargetPosZ(msg->target_pos_z);
    }
  else if(msg->pos_z_nav_mode == aerial_robot_base::FlightNav::POS_VEL_MODE)
    {
      setTargetPosZ(msg->target_pos_z);
      setTargetVelZ(msg->target_vel_z);
    }
}

void Navigator::joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
{
  /* ps3 joy bottons assignment: http://wiki.ros.org/ps3joy */
  if(!joy_stick_heart_beat_) joy_stick_heart_beat_ = true;
  joy_stick_prev_time_ = ros::Time::now().toSec();

  /* common command */
  /* start */
  if(joy_msg->buttons[3] == 1 && getNaviState() == ARM_OFF_STATE)
    {
      motorArming();
      return;
    }

  /* force landing && halt */
  static ros::Time force_landing_start_time_ = ros::Time::now();
  if(joy_msg->buttons[0] == 1)
    {
      /* Force Landing in inflight mode: TAKEOFF_STATE/LAND_STATE/HOVER_STATE */
      if(!force_landing_flag_ && (getNaviState() == TAKEOFF_STATE || getNaviState() == LAND_STATE || getNaviState() == HOVER_STATE))
        {
          ROS_WARN("Joy Control: force landing state");
          aerial_robot_base::FlightConfigCmd flight_config_cmd;
          flight_config_cmd.cmd = aerial_robot_base::FlightConfigCmd::FORCE_LANDING_CMD;
          flight_config_pub_.publish(flight_config_cmd);
          force_landing_flag_ = true;

          /* update the force landing stamp for the halt process*/
          force_landing_start_time_ = joy_msg->header.stamp;
        }

      /* Halt mode */
      if(joy_msg->header.stamp.toSec() - force_landing_start_time_.toSec() > force_landing_to_halt_du_ && getNaviState() > START_STATE)
        {
          //if(!teleop_flag_) return; /* can not do the process if other processs are running */

          ROS_ERROR("Joy Control: Halt!");

          setNaviState(STOP_STATE);

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
      if(getNaviState() == LAND_STATE) return;
      if(!teleop_flag_) return; /* can not do the process if other processs are running */

      setNaviState(LAND_STATE);
      //update
      setTargetXyFromCurrentState();
      setTargetPsiFromCurrentState();
      setTargetPosZ(estimator_->getLandingHeight());
      ROS_INFO("Joy Control: Land state");

      return;
    }

  /* Motion: Up/Down */
  if(fabs(joy_msg->axes[3]) > 0.2)
    {
      if(getNaviState() == HOVER_STATE)
        {
          alt_control_flag_ = true;
          if(joy_msg->axes[3] >= 0)
            addTargetPosZ(joy_target_alt_interval_);
          else
            addTargetPosZ(-joy_target_alt_interval_);
          ROS_INFO("Joy Control: Thrust state");
        }
    }
  else
    {
      if(alt_control_flag_)
        {
          alt_control_flag_= false;
          setTargetZFromCurrentState();
          ROS_INFO("Joy Control: Fixed Alt state, targetPosz_is %f",target_pos_.z());
        }
    }

  /* Motion: Yaw */
  /* this is the yaw_angle control */
  if(joy_msg->buttons[2] == 1)
    {
      if(fabs(joy_msg->axes[2]) > 0.05)
        {
          float  state_psi = estimator_->getState(State::YAW_COG, estimate_mode_)[0];
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

  /* mode oriented state */
  switch (xy_control_mode_)
    {
    case flight_nav::ACC_CONTROL_MODE:
      {
        if(teleop_flag_)
          {

            control_frame_ = flight_nav::WORLD_FRAME;
            if(joy_msg->buttons[8]) control_frame_ = flight_nav::LOCAL_FRAME;

            /* acc command */
            target_acc_.setValue(joy_msg->axes[1] * max_target_tilt_angle_ * BasicEstimator::G,
                                 joy_msg->axes[0] * max_target_tilt_angle_ * BasicEstimator::G, 0);

            if(control_frame_ == flight_nav::LOCAL_FRAME)
              {
                tf::Vector3 target_acc = target_acc_;
                target_acc_ = frameConversion(target_acc,  estimator_->getState(State::YAW_COG, estimate_mode_)[0]);
              }
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
                target_vel = frameConversion(target_vel_tmp,  estimator_->getState(State::YAW_COG, estimate_mode_)[0]);
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

void Navigator::update()
{
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
      if(getNaviState() == TAKEOFF_STATE || getNaviState() == HOVER_STATE  || getNaviState() == LAND_STATE)
        ROS_WARN("Sensor Unhealth Level%d: force landing state", estimator_->getUnhealthLevel());
      aerial_robot_base::FlightConfigCmd flight_config_cmd;
      flight_config_cmd.cmd = aerial_robot_base::FlightConfigCmd::FORCE_LANDING_CMD;
      flight_config_pub_.publish(flight_config_cmd);
      force_landing_flag_ = true;
    }

  if(getNaviState() == TAKEOFF_STATE || getNaviState() == HOVER_STATE)
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
          setNaviState(LAND_STATE);
          setTargetXyFromCurrentState();
          setTargetPsiFromCurrentState();
          setTargetPosZ(estimator_->getLandingHeight());
        }
    }

  tf::Vector3 delta = target_pos_ - estimator_->getPos(Frame::COG, estimate_mode_);

  switch(getNaviState())
    {
    case START_STATE:
      {
        /* low voltage */
        if(low_voltage_flag_)
          {
            setNaviState(ARM_OFF_STATE);
            break;
          }

        estimator_->setSensorFusionFlag(true);
        force_landing_flag_ = false; //is here good?

        aerial_robot_base::FlightConfigCmd flight_config_cmd;
        flight_config_cmd.cmd = aerial_robot_base::FlightConfigCmd::ARM_ON_CMD;
        flight_config_pub_.publish(flight_config_cmd);

        break;
      }
    case TAKEOFF_STATE:
      { //Takeoff Phase

        /* set flying flag to true once */
        if(!estimator_->getFlyingFlag())
          estimator_->setFlyingFlag(true);

        if(xy_control_mode_ == flight_nav::POS_CONTROL_MODE)
          {
            if (fabs(delta.z() > alt_convergent_thresh_ || fabs(delta.x()) > xy_convergent_thresh_ || fabs(delta.y()) > xy_convergent_thresh_))
              convergent_start_time_ = ros::Time::now().toSec();
          }
        else
          {
            if (fabs(delta.z()) > alt_convergent_thresh_) convergent_start_time_ = ros::Time::now().toSec();
          }
        if (ros::Time::now().toSec() - convergent_start_time_ > convergent_duration_)
          {
            convergent_start_time_ = ros::Time::now().toSec();
            setNaviState(HOVER_STATE);
            ROS_WARN("Hovering!");

          }
        break;
      }
    case LAND_STATE:
      {
        //for estimator landing mode
        estimator_->setLandingMode(true);

        if (getNaviState() > START_STATE)
          {
            if (fabs(delta.z()) > alt_convergent_thresh_) convergent_start_time_ = ros::Time::now().toSec();

            if (ros::Time::now().toSec() - convergent_start_time_ > convergent_duration_)
              {
                convergent_start_time_ = ros::Time::now().toSec();

                ROS_ERROR("disarm motors");
                setNaviState(STOP_STATE);

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
            setNaviState(ARM_OFF_STATE);
          }
        break;
      }
    case HOVER_STATE:
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
                setTargetVelX(0);
                setTargetVelY(0);
                setTargetVelZ(0);
              }
          }
        break;
      }
    case STOP_STATE:
      {
        estimator_->setSensorFusionFlag(false);
        estimator_->setLandingMode(false);
        estimator_->setLandedFlag(false);
        estimator_->setFlyingFlag(false);

        aerial_robot_base::FlightConfigCmd flight_config_cmd;
        flight_config_cmd.cmd = aerial_robot_base::FlightConfigCmd::ARM_OFF_CMD;
        flight_config_pub_.publish(flight_config_cmd);

        break;
      }
    default:
      {
        break;
      }
    }
}


void Navigator::rosParamInit(ros::NodeHandle nh)
{
  std::string ns = nh.getNamespace();

  if (!nh.getParam ("xy_control_mode", xy_control_mode_))
    xy_control_mode_ = 0;
  printf("%s: xy_control_mode_ is %d\n", ns.c_str(), xy_control_mode_);

  if (!nh.getParam ("takeoff_height", takeoff_height_))
    takeoff_height_ = 0;
  printf("%s: takeoff_height_ is %.3f\n", ns.c_str(), takeoff_height_);

  if (!nh.getParam ("convergent_duration", convergent_duration_))
    convergent_duration_ = 1.0; //sec
  printf("%s: convergent_duration_ is %.3f\n", ns.c_str(), convergent_duration_);

  if (!nh.getParam ("alt_convergent_thresh", alt_convergent_thresh_))
    alt_convergent_thresh_ = 0.05; //m
  printf("%s: alt_convergent_thresh_ is %.3f\n", ns.c_str(), alt_convergent_thresh_);

  if (!nh.getParam ("xy_convergent_thresh", xy_convergent_thresh_))
    xy_convergent_thresh_ = 0.15; //m
  printf("%s: xy_convergent_thresh_ is %.3f\n", ns.c_str(), xy_convergent_thresh_);

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

  ros::NodeHandle bat_info_node("bat_info");
  bat_info_node.param("bat_cell", bat_cell_, 0); // Lipo battery cell
  cout << ns  << ": bat_cell_ is "  <<  bat_cell_ << endl;
  bat_info_node.param("low_voltage_thre", low_voltage_thre_, 0.1); // Lipo battery cell
  cout << ns  << ": low_voltage_thre_ is "  <<  low_voltage_thre_ << endl;
  bat_info_node.param("bat_resistance", bat_resistance_, 0.0); //Battery internal resistance
  cout << ns  << ": bat_resistance_ is "  <<  bat_resistance_ << endl;
  bat_info_node.param("bat_resistance_voltage_rate", bat_resistance_voltage_rate_, 0.0); //Battery internal resistance_voltage_rate
  cout << ns  << ": bat_resistance_voltage_rate_ is "  <<  bat_resistance_voltage_rate_ << endl;
  bat_info_node.param("hovering_current", hovering_current_, 0.0); // current at hovering state
  cout << ns  << ": hovering_current_ is "  <<  hovering_current_ << endl;
}

