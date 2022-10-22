#include "aerial_robot_control/flight_navigation.h"

using namespace std;
using namespace aerial_robot_navigation;

BaseNavigator::BaseNavigator():
  target_pos_(0, 0, 0),
  target_vel_(0, 0, 0),
  target_acc_(0, 0, 0),
  target_rpy_(0, 0, 0),
  target_omega_(0, 0, 0),
  force_att_control_flag_(false),
  low_voltage_flag_(false),
  prev_xy_control_mode_(ACC_CONTROL_MODE),
  vel_control_flag_(false),
  pos_control_flag_(false),
  xy_control_flag_(false),
  z_control_flag_(false),
  yaw_control_flag_(false),
  vel_based_waypoint_(false),
  gps_waypoint_(false),
  gps_waypoint_time_(0),
  joy_stick_heart_beat_(false),
  joy_stick_prev_time_(0),
  teleop_flag_(true)
{
  setNaviState(ARM_OFF_STATE);
}

void BaseNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                               boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                               boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator)
{
  nh_ = nh;
  nhp_ = nhp;

  rosParamInit();

  robot_model_ = robot_model;
  estimator_ = estimator;

  navi_sub_ = nh_.subscribe("uav/nav", 1, &BaseNavigator::naviCallback, this, ros::TransportHints().tcpNoDelay());

  battery_sub_ = nh_.subscribe("battery_voltage_status", 1, &BaseNavigator::batteryCheckCallback, this);
  flight_status_ack_sub_ = nh_.subscribe("flight_config_ack", 1, &BaseNavigator::flightStatusAckCallback, this, ros::TransportHints().tcpNoDelay());

  ros::NodeHandle teleop_nh = ros::NodeHandle(nh_, "teleop_command");
  takeoff_sub_ = teleop_nh.subscribe("takeoff", 1, &BaseNavigator::takeoffCallback, this);
  halt_sub_ = teleop_nh.subscribe("halt", 1, &BaseNavigator::haltCallback, this);
  force_landing_sub_ = teleop_nh.subscribe("force_landing", 1, &BaseNavigator::forceLandingCallback, this);
  force_landing_flag_ = false;
  land_sub_ = teleop_nh.subscribe("land", 1, &BaseNavigator::landCallback, this);
  start_sub_ = teleop_nh.subscribe("start", 1,&BaseNavigator::startCallback, this);
  ctrl_mode_sub_ = teleop_nh.subscribe("ctrl_mode", 1, &BaseNavigator::xyControlModeCallback, this);

  ros::TransportHints joy_transport_hints;
#ifdef ARM_MELODIC // https://github.com/ros/ros_comm/issues/1404
  joy_udp_ = false;
#endif
  if(joy_udp_) joy_transport_hints = ros::TransportHints().udp();
  joy_stick_sub_ = nh_.subscribe("joy", 1, &BaseNavigator::joyStickControl, this, joy_transport_hints);

  stop_teleop_sub_ = nh_.subscribe("stop_teleop", 1, &BaseNavigator::stopTeleopCallback, this);

  flight_config_pub_ = nh_.advertise<spinal::FlightConfigCmd>("flight_config_cmd", 10);
  power_info_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("uav_power", 10);
  flight_state_pub_ = nh_.advertise<std_msgs::UInt8>("flight_state", 1);

  estimate_mode_ = estimator_->getEstimateMode();
  force_landing_start_time_ = ros::Time::now();
}

void BaseNavigator::batteryCheckCallback(const std_msgs::Float32ConstPtr &msg)
{
  if(std::isnan(msg->data))
    {
      throw std::runtime_error("the voltage from spinal is Nan, please re-calibrate the voltage scale using /set_adc_scale.");
    }

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
  else
    low_voltage_flag_  = false;

  if(power_info_pub_.getNumSubscribers() == 0) return;

  geometry_msgs::Vector3Stamped power_info_msgs;
  power_info_msgs.header.stamp = ros::Time::now();
  power_info_msgs.vector.x = voltage;
  power_info_msgs.vector.y = percentage;
  power_info_pub_.publish(power_info_msgs);

}

void BaseNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr & msg)
{
  if(getNaviState() == TAKEOFF_STATE || getNaviState() == LAND_STATE) return;

  gps_waypoint_ = false;

  if(force_att_control_flag_) return;

  /* yaw */
  if(msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE)
    {
      setTargetYaw(angles::normalize_angle(msg->target_yaw));
      setTargetOmegaZ(0);
    }
  if(msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::POS_VEL_MODE)
    {
      setTargetYaw(angles::normalize_angle(msg->target_yaw));
      setTargetOmegaZ(msg->target_omega_z);
    }

  /* xy control */
  switch(msg->pos_xy_nav_mode)
    {
    case aerial_robot_msgs::FlightNav::POS_MODE:
      {
        tf::Vector3 target_cog_pos(msg->target_pos_x, msg->target_pos_y, 0);
        if(msg->target == aerial_robot_msgs::FlightNav::BASELINK)
          {
            /* check the transformation */
            tf::Transform cog2baselink_tf;
            tf::transformKDLToTF(robot_model_->getCog2Baselink<KDL::Frame>(), cog2baselink_tf);
            target_cog_pos -= tf::Matrix3x3(tf::createQuaternionFromYaw(getTargetRPY().z()))
              * cog2baselink_tf.getOrigin();
          }

        tf::Vector3 target_delta = getTargetPos() - target_cog_pos;
        target_delta.setZ(0);

        if(target_delta.length() > vel_nav_threshold_)
          {
            ROS_WARN("start vel nav control for waypoint");
            vel_based_waypoint_ = true;
            xy_control_mode_ = VEL_CONTROL_MODE;
          }

        if(!vel_based_waypoint_)
          xy_control_mode_ = POS_CONTROL_MODE;

        setTargetPosX(target_cog_pos.x());
        setTargetPosY(target_cog_pos.y());
        setTargetVelX(0);
        setTargetVelY(0);

        break;
      }
    case aerial_robot_msgs::FlightNav::VEL_MODE:
      {
        if(msg->target == aerial_robot_msgs::FlightNav::BASELINK)
          {
            ROS_ERROR("[Flight nav] can not do vel nav for baselink");
            return;
          }
        /* should be in COG frame */
        xy_control_mode_ = VEL_CONTROL_MODE;
        switch(msg->control_frame)
          {
          case WORLD_FRAME:
            {
              setTargetVelX(msg->target_vel_x);
              setTargetVelY(msg->target_vel_y);
              break;
            }
          case LOCAL_FRAME:
            {
              tf::Vector3 target_vel = frameConversion(tf::Vector3(msg->target_vel_x, msg->target_vel_y, 0), estimator_->getEuler(Frame::COG, estimate_mode_).z());
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
    case aerial_robot_msgs::FlightNav::POS_VEL_MODE:
      {
        if(msg->target == aerial_robot_msgs::FlightNav::BASELINK)
          {
            ROS_ERROR("[Flight nav] can not do pos_vel nav for baselink");
            return;
          }

        xy_control_mode_ = POS_CONTROL_MODE;
        setTargetPosX(msg->target_pos_x);
        setTargetPosY(msg->target_pos_y);
        setTargetVelX(msg->target_vel_x);
        setTargetVelY(msg->target_vel_y);

        break;
      }
    case aerial_robot_msgs::FlightNav::ACC_MODE:
      {
        /* should be in COG frame */
        xy_control_mode_ = ACC_CONTROL_MODE;
        prev_xy_control_mode_ = ACC_CONTROL_MODE;

        switch(msg->control_frame)
          {
          case WORLD_FRAME:
            {
              target_acc_.setValue(msg->target_acc_x, msg->target_acc_y, 0);
              break;
            }
          case LOCAL_FRAME:
            {
              tf::Vector3 target_acc = frameConversion(tf::Vector3(msg->target_acc_x, msg->target_acc_y, 0), estimator_->getEuler(Frame::COG, estimate_mode_).z());
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
    case aerial_robot_msgs::FlightNav::GPS_WAYPOINT_MODE:
      {
        target_wp_ = geodesy::toMsg(msg->target_pos_x, msg->target_pos_y);
        gps_waypoint_ = true;

        break;
      }
    }
  if(msg->pos_xy_nav_mode != aerial_robot_msgs::FlightNav::ACC_MODE) target_acc_.setValue(0,0,0);

  /* z */
  if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::VEL_MODE)
    {
      /* special */
      addTargetPosZ(msg->target_pos_diff_z);
      setTargetVelZ(0);
    }
  else if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE)
    {
      setTargetPosZ(msg->target_pos_z);
      setTargetVelZ(0);
    }
  else if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::POS_VEL_MODE)
    {
      setTargetPosZ(msg->target_pos_z);
      setTargetVelZ(msg->target_vel_z);
    }
}

const sensor_msgs::Joy BaseNavigator::ps4joyToPs3joyConvert(const sensor_msgs::Joy& ps4_joy_msg)
{
  /* hard coding */
  sensor_msgs::Joy joy_cmd;
  joy_cmd.header = ps4_joy_msg.header;
  joy_cmd.axes.resize(PS3_AXES, 0);
  joy_cmd.buttons.resize(PS3_BUTTONS, 0);
  joy_cmd.buttons[PS3_BUTTON_SELECT] = ps4_joy_msg.buttons[PS4_BUTTON_SHARE];
  joy_cmd.buttons[PS3_BUTTON_STICK_LEFT] = ps4_joy_msg.buttons[PS4_BUTTON_STICK_LEFT];
  joy_cmd.buttons[PS3_BUTTON_STICK_RIGHT] = ps4_joy_msg.buttons[PS4_BUTTON_STICK_RIGHT];
  joy_cmd.buttons[PS3_BUTTON_START] = ps4_joy_msg.buttons[PS4_BUTTON_OPTIONS];
  if(ps4_joy_msg.axes[PS4_AXIS_BUTTON_CROSS_UP_DOWN] == 1)
    joy_cmd.buttons[PS3_BUTTON_CROSS_UP] = 1;
  if(ps4_joy_msg.axes[PS4_AXIS_BUTTON_CROSS_UP_DOWN] == -1)
    joy_cmd.buttons[PS3_BUTTON_CROSS_DOWN] = 1;
  if(ps4_joy_msg.axes[PS4_AXIS_BUTTON_CROSS_LEFT_RIGHT] == 1)
    joy_cmd.buttons[PS3_BUTTON_CROSS_LEFT] = 1;
  if(ps4_joy_msg.axes[PS4_AXIS_BUTTON_CROSS_LEFT_RIGHT] == -1)
    joy_cmd.buttons[PS3_BUTTON_CROSS_RIGHT] = 1;
  joy_cmd.buttons[PS3_BUTTON_REAR_LEFT_2] = ps4_joy_msg.buttons[PS4_BUTTON_REAR_LEFT_2];
  joy_cmd.buttons[PS3_BUTTON_REAR_RIGHT_2] = ps4_joy_msg.buttons[PS4_BUTTON_REAR_RIGHT_2];
  joy_cmd.buttons[PS3_BUTTON_REAR_LEFT_1] = ps4_joy_msg.buttons[PS4_BUTTON_REAR_LEFT_1];
  joy_cmd.buttons[PS3_BUTTON_REAR_RIGHT_1] = ps4_joy_msg.buttons[PS4_BUTTON_REAR_RIGHT_1];
  joy_cmd.buttons[PS3_BUTTON_ACTION_TRIANGLE] = ps4_joy_msg.buttons[PS4_BUTTON_ACTION_TRIANGLE];
  joy_cmd.buttons[PS3_BUTTON_ACTION_CIRCLE] = ps4_joy_msg.buttons[PS4_BUTTON_ACTION_CIRCLE];
  joy_cmd.buttons[PS3_BUTTON_ACTION_CROSS] = ps4_joy_msg.buttons[PS4_BUTTON_ACTION_CROSS];
  joy_cmd.buttons[PS3_BUTTON_ACTION_SQUARE] = ps4_joy_msg.buttons[PS4_BUTTON_ACTION_SQUARE];
  joy_cmd.buttons[PS3_BUTTON_PAIRING] = ps4_joy_msg.buttons[PS4_BUTTON_PAIRING];
  joy_cmd.axes[PS3_AXIS_STICK_LEFT_LEFTWARDS] = ps4_joy_msg.axes[PS4_AXIS_STICK_LEFT_LEFTWARDS];
  joy_cmd.axes[PS3_AXIS_STICK_LEFT_UPWARDS] = ps4_joy_msg.axes[PS4_AXIS_STICK_LEFT_UPWARDS];
  joy_cmd.axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS] = ps4_joy_msg.axes[PS4_AXIS_STICK_RIGHT_LEFTWARDS];
  joy_cmd.axes[PS3_AXIS_STICK_RIGHT_UPWARDS] = ps4_joy_msg.axes[PS4_AXIS_STICK_RIGHT_UPWARDS];
  joy_cmd.axes[PS3_AXIS_ACCELEROMETER_LEFT] = ps4_joy_msg.axes[PS4_AXIS_ACCELEROMETER_LEFT];
  joy_cmd.axes[PS3_AXIS_ACCELEROMETER_FORWARD] = ps4_joy_msg.axes[PS4_AXIS_ACCELEROMETER_FORWARD];
  joy_cmd.axes[PS3_AXIS_ACCELEROMETER_UP] = ps4_joy_msg.axes[PS4_AXIS_ACCELEROMETER_UP];
  joy_cmd.axes[PS3_AXIS_GYRO_YAW] = ps4_joy_msg.axes[PS4_AXIS_GYRO_YAW];
  return joy_cmd;
}


void BaseNavigator::joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
{
  sensor_msgs::Joy joy_cmd;
  if(joy_msg->axes.size() == PS3_AXES && joy_msg->buttons.size() == PS3_BUTTONS)
    {
      joy_cmd = (*joy_msg);
    }
  else if(joy_msg->axes.size() == PS4_AXES && joy_msg->buttons.size() == PS4_BUTTONS)
    {
      joy_cmd = ps4joyToPs3joyConvert(*joy_msg);
    }
  else
    {
      ROS_WARN("the joystick type is not supported (buttons: %d, axes: %d)", (int)joy_msg->buttons.size(), (int)joy_msg->axes.size());
      return;
    }

  /* ps3 joy bottons assignment: http://wiki.ros.org/ps3joy */
  if(!joy_stick_heart_beat_) joy_stick_heart_beat_ = true;
  joy_stick_prev_time_ = ros::Time::now().toSec();

  /* common command */
  /* start */
  if(joy_cmd.buttons[PS3_BUTTON_START] == 1 && getNaviState() == ARM_OFF_STATE)
    {
      motorArming();
      return;
    }

  /* force landing && halt */
  if(joy_cmd.buttons[PS3_BUTTON_SELECT] == 1)
    {
      /* Force Landing in inflight mode: TAKEOFF_STATE/LAND_STATE/HOVER_STATE */
      if(!force_landing_flag_ && (getNaviState() == TAKEOFF_STATE || getNaviState() == LAND_STATE || getNaviState() == HOVER_STATE))
        {
          ROS_WARN("Joy Control: force landing state");
          spinal::FlightConfigCmd flight_config_cmd;
          flight_config_cmd.cmd = spinal::FlightConfigCmd::FORCE_LANDING_CMD;
          flight_config_pub_.publish(flight_config_cmd);
          force_landing_flag_ = true;

          /* update the force landing stamp for the halt process*/
          force_landing_start_time_ = joy_cmd.header.stamp;
        }

      /* Halt mode */
      if(joy_cmd.header.stamp.toSec() - force_landing_start_time_.toSec() > force_landing_to_halt_du_ && getNaviState() > START_STATE)
        {
          //if(!teleop_flag_) return; /* can not do the process if other processs are running */

          ROS_ERROR("Joy Control: Halt!");

          setNaviState(STOP_STATE);

          /* update the target pos(maybe not necessary) */
          setTargetXyFromCurrentState();
          setTargetYawFromCurrentState();
        }
      return;
    }
  else
    {
      /* update the halt process */
      force_landing_start_time_ = joy_cmd.header.stamp;
    }

  /* takeoff */
  if(joy_cmd.buttons[PS3_BUTTON_CROSS_LEFT] == 1 && joy_cmd.buttons[PS3_BUTTON_ACTION_CIRCLE] == 1)
    {
      startTakeoff();
      return;
    }

  /* landing */
  if(joy_cmd.buttons[PS3_BUTTON_CROSS_RIGHT] == 1 || joy_cmd.buttons[PS3_BUTTON_ACTION_SQUARE] == 1)
    {
      if(force_att_control_flag_) return;

      if(getNaviState() == LAND_STATE) return;
      if(!teleop_flag_) return; /* can not do the process if other processs are running */

      setNaviState(LAND_STATE);
      //update
      setTargetXyFromCurrentState();
      setTargetYawFromCurrentState();
      setTargetPosZ(estimator_->getLandingHeight());
      ROS_INFO("Joy Control: Land state");

      return;
    }

  /* Motion: Up/Down */
  if(fabs(joy_cmd.axes[PS3_AXIS_STICK_RIGHT_UPWARDS]) > joy_z_deadzone_)
    {
      if(getNaviState() == HOVER_STATE)
        {
          z_control_flag_ = true;
          if(joy_cmd.axes[PS3_AXIS_STICK_RIGHT_UPWARDS] >= 0)
            addTargetPosZ(joy_target_z_interval_);
          else
            addTargetPosZ(-joy_target_z_interval_);
        }
    }
  else
    {
      if(z_control_flag_)
        {
          z_control_flag_= false;
          setTargetZFromCurrentState();
          ROS_INFO("Joy Control: fixed z state, target pos z is %f",target_pos_.z());
        }
    }

  /* Motion: Yaw */
  /* this is the yaw_angle control */
  if(fabs(joy_cmd.axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS]) > joy_yaw_deadzone_)
    {
      double target_yaw = estimator_->getEuler(Frame::COG, estimate_mode_).z()
        + joy_cmd.axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS] * max_target_yaw_rate_;
      setTargetYaw(angles::normalize_angle(target_yaw));
      setTargetOmegaZ(joy_cmd.axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS] * max_target_yaw_rate_);

      yaw_control_flag_ = true;
    }
  else
    {
      if(yaw_control_flag_)
        {
          yaw_control_flag_= false;
          setTargetYawFromCurrentState();
          setTargetOmegaZ(0);
          ROS_INFO("Joy Control: fixed yaw state, target yaw angle is %f", getTargetRPY().z());
        }
    }

  /* turn to ACC_CONTROL_MODE */
  if(joy_cmd.buttons[PS3_BUTTON_CROSS_DOWN] == 1)
    {
      ROS_WARN("Foce change to attitude control");
      force_att_control_flag_ = true;
      estimator_->setForceAttControlFlag(force_att_control_flag_);
      xy_control_mode_ = ACC_CONTROL_MODE;
    }

  /* change to vel control mode */
  if(joy_cmd.buttons[PS3_BUTTON_ACTION_TRIANGLE] == 1 && !vel_control_flag_)
    {
      ROS_INFO("change to vel pos-based control");
      vel_control_flag_ = true;
      force_att_control_flag_ = false;
      xy_control_mode_ = VEL_CONTROL_MODE;
      target_vel_.setValue(0, 0, 0);
      target_acc_.setValue(0, 0, 0);
    }
  if(joy_cmd.buttons[PS3_BUTTON_ACTION_TRIANGLE] == 0 && vel_control_flag_)
    vel_control_flag_ = false;

  /* change to pos control mode */
  if(joy_cmd.buttons[PS3_BUTTON_ACTION_CROSS] == 1 && !pos_control_flag_)
    {
      ROS_INFO("change to pos control");
      pos_control_flag_ = true;
      force_att_control_flag_ = false;
      xy_control_mode_ = POS_CONTROL_MODE;
      setTargetXyFromCurrentState();
      target_acc_.setValue(0, 0, 0);
    }
  if(joy_cmd.buttons[PS3_BUTTON_ACTION_CROSS] == 0 && pos_control_flag_)
    pos_control_flag_ = false;

  /* mode oriented state */
  switch (xy_control_mode_)
    {
    case ACC_CONTROL_MODE:
      {
        if(teleop_flag_)
          {
            control_frame_ = WORLD_FRAME;
            if(joy_cmd.buttons[PS3_BUTTON_REAR_LEFT_2]) control_frame_ = LOCAL_FRAME;

            /* acc command */
            target_acc_.setValue(joy_cmd.axes[PS3_AXIS_STICK_LEFT_UPWARDS] * max_target_tilt_angle_ * aerial_robot_estimation::G,
                                 joy_cmd.axes[PS3_AXIS_STICK_LEFT_LEFTWARDS] * max_target_tilt_angle_ * aerial_robot_estimation::G, 0);

            if(control_frame_ == LOCAL_FRAME)
              {
                tf::Vector3 target_acc = target_acc_;
                /* convert the frame */
                const auto segments_tf = robot_model_->getSegmentsTf();
                if(segments_tf.find(teleop_local_frame_) == segments_tf.end())
                  {
                    ROS_ERROR("can not find %s in kinematics model", teleop_local_frame_.c_str());
                    target_acc.setValue(0,0,0);
                  }
                tf::Transform teleop_local_frame_tf;
                tf::transformKDLToTF(segments_tf.at(robot_model_->getBaselinkName()).Inverse() * segments_tf.at(teleop_local_frame_), teleop_local_frame_tf);

                target_acc_ = frameConversion(target_acc,  tf::Matrix3x3(tf::createQuaternionFromYaw(estimator_->getEuler(Frame::COG, estimate_mode_).z())) * teleop_local_frame_tf.getBasis());
              }
          }
        break;
      }
    case VEL_CONTROL_MODE:
      {
        if(teleop_flag_)
          {
            control_frame_ = WORLD_FRAME;
            if(joy_cmd.buttons[PS3_BUTTON_REAR_LEFT_2]) control_frame_ = LOCAL_FRAME;

            tf::Vector3 target_vel(joy_cmd.axes[PS3_AXIS_STICK_LEFT_UPWARDS] * fabs(joy_cmd.axes[PS3_AXIS_STICK_LEFT_UPWARDS]) * max_target_vel_,
                                 joy_cmd.axes[PS3_AXIS_STICK_LEFT_LEFTWARDS] * fabs(joy_cmd.axes[PS3_AXIS_STICK_LEFT_LEFTWARDS]) * max_target_vel_, 0);

            /* defualt: world frame control */
            /* L2 trigger: fc(cog/ baselink frame) frame control */
            if(control_frame_ == LOCAL_FRAME)
              {
                tf::Vector3 target_vel_tmp = target_vel;
                target_vel = frameConversion(target_vel_tmp,  estimator_->getEuler(Frame::COG, estimate_mode_).z());
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

void BaseNavigator::update()
{
  if(force_att_control_flag_)
    {
      if(getNaviState() == LAND_STATE)
        {
          setTargetZFromCurrentState();
          setNaviState(HOVER_STATE);
        }
    }
  else
    {
      /* check the xy estimation status, if not ready, change to att_control_mode */
      if(!estimator_->getStateStatus(State::X_BASE, estimate_mode_) || !estimator_->getStateStatus(State::Y_BASE, estimate_mode_))
        {
          if(xy_control_mode_ == VEL_CONTROL_MODE ||
             xy_control_mode_ == POS_CONTROL_MODE)
            {
              ROS_ERROR("No estimation for X, Y state, change to attitude control mode");
              prev_xy_control_mode_ = xy_control_mode_;
              xy_control_mode_ = ACC_CONTROL_MODE;
            }
        }
      else
        {
          if(xy_control_mode_ == ACC_CONTROL_MODE &&
             prev_xy_control_mode_ != ACC_CONTROL_MODE)
            {
              ROS_ERROR("Estimation for X, Y state is established, siwtch back to the xy control mode");
              xy_control_mode_ = prev_xy_control_mode_;
            }
        }
    }

  /* sensor health check */
  if(estimator_->getUnhealthLevel() == Sensor::UNHEALTH_LEVEL3 && !force_landing_flag_)
    {
      if(getNaviState() == TAKEOFF_STATE || getNaviState() == HOVER_STATE  || getNaviState() == LAND_STATE)
        ROS_WARN("Sensor Unhealth Level%d: force landing state", estimator_->getUnhealthLevel());
      spinal::FlightConfigCmd flight_config_cmd;
      flight_config_cmd.cmd = spinal::FlightConfigCmd::FORCE_LANDING_CMD;
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

      if(normal_land && !force_att_control_flag_)
        {
          setNaviState(LAND_STATE);
          setTargetXyFromCurrentState();
          setTargetYawFromCurrentState();
          setTargetPosZ(estimator_->getLandingHeight());
        }
    }

  tf::Vector3 delta = target_pos_ - estimator_->getPos(Frame::COG, estimate_mode_);

  /* check the hard landing in force_landing model */
  if(force_landing_flag_)
    {
      if(!estimator_->getLandingMode()) estimator_->setLandingMode(true);
      if(estimator_->getLandedFlag() && force_landing_auto_stop_flag_)
        {
          ROS_WARN("hard touch to the ground in force landing mode, disarm motor");
          setNaviState(STOP_STATE);
        }
    }

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
        force_landing_flag_ = false;

        spinal::FlightConfigCmd flight_config_cmd;
        flight_config_cmd.cmd = spinal::FlightConfigCmd::ARM_ON_CMD;
        flight_config_pub_.publish(flight_config_cmd);

        break;
      }
    case TAKEOFF_STATE:
      { //Takeoff Phase

        /* set flying flag to true once */
        if(!estimator_->getFlyingFlag())
          estimator_->setFlyingFlag(true);

        if(xy_control_mode_ == POS_CONTROL_MODE)
          {
            if (fabs(delta.z()) > z_convergent_thresh_ || fabs(delta.x()) > xy_convergent_thresh_ || fabs(delta.y()) > xy_convergent_thresh_)
              convergent_start_time_ = ros::Time::now().toSec();
          }
        else
          {
            if (fabs(delta.z()) > z_convergent_thresh_) convergent_start_time_ = ros::Time::now().toSec();
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
            if (fabs(delta.z()) > z_convergent_thresh_) convergent_start_time_ = ros::Time::now().toSec();

            if (ros::Time::now().toSec() - convergent_start_time_ > convergent_duration_)
              {
                convergent_start_time_ = ros::Time::now().toSec();

                ROS_ERROR("disarm motors");
                setNaviState(STOP_STATE);

                setTargetXyFromCurrentState();
                setTargetYawFromCurrentState();
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
        if(force_att_control_flag_) break;

        if(gps_waypoint_)
          {
            if(ros::Time::now().toSec() - gps_waypoint_time_ > gps_waypoint_check_du_)
              {
                auto base_wp = estimator_->getCurrGpsPoint();
                tf::Matrix3x3 convert_frame; convert_frame.setRPY(M_PI, 0, 0); // NED -> XYZ
                tf::Vector3 gps_waypoint_delta =  convert_frame * sensor_plugin::Gps::wgs84ToNedLocalFrame(base_wp, target_wp_);


                if(gps_waypoint_delta.length() < gps_waypoint_threshold_)
                  gps_waypoint_ = false;

                xy_control_mode_ = POS_CONTROL_MODE;

                if(gps_waypoint_delta.length() > vel_nav_threshold_)
                  {
                    vel_based_waypoint_ = true;
                    xy_control_mode_ = VEL_CONTROL_MODE;
                  }

                //ROS_INFO("gps_waypoint_delta: %f, %f", gps_waypoint_delta.x(), gps_waypoint_delta.y());
                tf::Vector3 target_cog_pos = estimator_->getPos(Frame::COG, estimate_mode_) + gps_waypoint_delta;
                setTargetPosX(target_cog_pos.x());
                setTargetPosY(target_cog_pos.y());

                delta = gps_waypoint_delta;
                gps_waypoint_time_ = ros::Time::now().toSec();
              }
          }

        if(vel_based_waypoint_)
          {
            delta.setZ(0); // we do not need z
            /* vel nav */
            if(delta.length() > vel_nav_threshold_)
              {
                tf::Vector3 nav_vel = delta * vel_nav_gain_;

                double speed = nav_vel.length();
                if(speed  > nav_vel_limit_) nav_vel *= (nav_vel_limit_ / speed);

                setTargetVelX(nav_vel.x());
                setTargetVelY(nav_vel.y());
              }
            else
              {
                if(gps_waypoint_)
                  {
                    auto base_wp = estimator_->getCurrGpsPoint();
                    tf::Matrix3x3 convert_frame; convert_frame.setRPY(M_PI, 0, 0); // NED -> XYZ
                    tf::Vector3 gps_waypoint_delta =  convert_frame * sensor_plugin::Gps::wgs84ToNedLocalFrame(base_wp, target_wp_);

                    ROS_WARN("back to pos nav control for GPS way point, gps waypoint delta: %f, %f", gps_waypoint_delta.x(), gps_waypoint_delta.y());
                    gps_waypoint_  = false;
                  }
                else
                  {
                    ROS_WARN("back to pos nav control for way point");
                  }

                xy_control_mode_ = POS_CONTROL_MODE;
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
        reset();

        spinal::FlightConfigCmd flight_config_cmd;
        flight_config_cmd.cmd = spinal::FlightConfigCmd::ARM_OFF_CMD;
        flight_config_pub_.publish(flight_config_cmd);

        if(force_landing_flag_)
          {
            halt();
            force_landing_flag_ = false;
          }

        break;
      }
    default:
      {
        break;
      }
    }

  /* publish the state */
  std_msgs::UInt8 state_msg;
  state_msg.data = getNaviState();
  if(force_landing_flag_) state_msg.data = FORCE_LANDING_STATE;
  else if(low_voltage_flag_) state_msg.data = LOW_BATTERY_STATE;
  flight_state_pub_.publish(state_msg);
}


void BaseNavigator::rosParamInit()
{
  getParam<bool>(nhp_, "param_verbose", param_verbose_, false);

  ros::NodeHandle nh(nh_, "navigation");
  getParam<int>(nh, "xy_control_mode", xy_control_mode_, 0);
  getParam<double>(nh, "takeoff_height", takeoff_height_, 0.0);
  getParam<double>(nh, "convergent_duration", convergent_duration_, 1.0);
  getParam<double>(nh, "z_convergent_thresh", z_convergent_thresh_, 0.05);
  getParam<double>(nh, "xy_convergent_thresh", xy_convergent_thresh_, 0.15);
  getParam<double>(nh, "max_target_vel", max_target_vel_, 0.0);
  getParam<double>(nh, "max_target_yaw_rate", max_target_yaw_rate_, 0.0);
  getParam<double>(nh, "max_target_tilt_angle", max_target_tilt_angle_, 1.0);

  //*** auto vel nav
  getParam<double>(nh, "nav_vel_limit", nav_vel_limit_, 0.2);
  getParam<double>(nh, "vel_nav_threshold", vel_nav_threshold_, 0.4);
  getParam<double>(nh, "vel_nav_gain", vel_nav_gain_, 1.0);

  //*** gps waypoint
  getParam<double>(nh, "gps_waypoint_threshold", gps_waypoint_threshold_, 3.0);
  getParam<double>(nh, "gps_waypoint_check_du", gps_waypoint_check_du_, 1.0);

  //*** teleop navigation
  getParam<double>(nh, "joy_target_vel_interval", joy_target_vel_interval_, 0.0);
  getParam<double>(nh, "joy_target_z_interval", joy_target_z_interval_, 0.0);
  getParam<double>(nh, "joy_z_deadzone", joy_z_deadzone_, 0.2);
  getParam<double>(nh, "joy_yaw_deadzone", joy_yaw_deadzone_, 0.2);
  getParam<double>(nh, "joy_stick_heart_beat_du", joy_stick_heart_beat_du_, 2.0);
  getParam<double>(nh, "force_landing_to_halt_du", force_landing_to_halt_du_, 1.0);
  getParam<bool>(nh, "force_landing_auto_stop_flag", force_landing_auto_stop_flag_, true);
  getParam<bool>(nh, "joy_udp", joy_udp_, true);
  getParam<bool>(nh, "check_joy_stick_heart_beat", check_joy_stick_heart_beat_, false);
  getParam<std::string>(nh, "teleop_local_frame", teleop_local_frame_, std::string("root"));

  ros::NodeHandle bat_nh(nh_, "bat_info");
  getParam<int>(bat_nh, "bat_cell", bat_cell_, 0); // Lipo battery cell
  getParam<double>(bat_nh, "low_voltage_thre", low_voltage_thre_, 0.1); // Lipo battery cell
  getParam<double>(bat_nh, "bat_resistance", bat_resistance_, 0.0); //Battery internal resistance
  getParam<double>(bat_nh, "bat_resistance_voltage_rate", bat_resistance_voltage_rate_, 0.0); //Battery internal resistance_voltage_rate
  getParam<double>(bat_nh, "hovering_current", hovering_current_, 0.0); // current at hovering state
}

