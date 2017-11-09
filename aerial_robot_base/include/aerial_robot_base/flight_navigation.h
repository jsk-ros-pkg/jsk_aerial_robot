#ifndef FLIGHT_NAVIGATION_H
#define FLIGHT_NAVIGATION_H

/* ros */
#include <ros/ros.h>
#include <aerial_robot_base/basic_state_estimation.h>

/* ros msg */
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <aerial_robot_msgs/FourAxisCommand.h>
#include <tf/transform_broadcaster.h>
#include <aerial_robot_base/FlightNav.h>
#include <sensor_msgs/Joy.h>

namespace flight_nav
{
  /* control mode */
  enum control_mode
    {
      POS_CONTROL_MODE,
      VEL_CONTROL_MODE,
      ACC_CONTROL_MODE
    };
  /* control frame */
  enum control_frame
    {
      WORLD_FRAME, /* global frame, e.g. NEU, mocap */
      LOCAL_FRAME /* head frame which is identical with imu head direction */
    };
};

class Navigator
{
public:
  Navigator(ros::NodeHandle nh, ros::NodeHandle nh_private,
            BasicEstimator* estimator);
  virtual ~Navigator();

  ros::Publisher  flight_config_pub_;
  ros::Publisher  power_info_pub_;

  void update();

  inline uint8_t getNaviState(){  return navi_state_;}
  inline void setNaviState(const uint8_t  state){ navi_state_ = state;}

  inline uint8_t getXyControlMode(){  return (uint8_t)xy_control_mode_;}
  inline void setXyControlMode(uint8_t mode){  xy_control_mode_ = mode;}

  inline uint8_t getControlframe(){  return (uint8_t)control_frame_;}
  inline void setControlframe(uint8_t frame_type){  control_frame_ = frame_type;}

  inline bool getXyVelModePosCtrlTakeoff(){  return xy_vel_mode_pos_ctrl_takeoff_;}
  inline bool getForceLandingFlag() {return force_landing_flag_;}

  inline tf::Vector3 getTargetPos() {return target_pos_;}
  inline tf::Vector3 getTargetVel() {return target_vel_;}
  inline tf::Vector3 getTargetAcc() {return target_acc_;}
  inline float getTargetPsi() {return target_psi_;}

  inline void setTargetPsi(float value) { target_psi_ = value; }
  inline void setTargetVelPsi(float value) { target_vel_psi_ = value; }
  inline void setTargetPosX( float value){  target_pos_.setX(value);}
  inline void setTargetVelX( float value){  target_vel_.setX(value);}
  inline void setTargetAccX( float value){  target_acc_.setX(value);}
  inline void setTargetPosY( float value){  target_pos_.setY(value);}
  inline void setTargetVelY( float value){  target_vel_.setY(value);}
  inline void setTargetAccY( float value){  target_acc_.setY(value);}
  inline void setTargetPosZ( float value){  target_pos_.setZ(value);}
  inline void setTargetVelZ( float value){  target_vel_.setZ(value);}
  inline void setTargetAccZ( float value){  target_acc_.setZ(value);}
  inline void addTargetPosZ( float value){  target_pos_ += tf::Vector3(0, 0, value);}

  inline void setTeleopFlag(bool teleop_flag) { teleop_flag_ = teleop_flag; }
  inline bool getTeleopFlag() { return teleop_flag_; }

  void tfPublish();

  uint8_t getEstimateMode(){ return estimate_mode_;}
  void setEstimateMode(uint8_t estimate_mode){ estimate_mode_ = estimate_mode;}

  static constexpr uint8_t POS_CONTROL_COMMAND = 0;
  static constexpr uint8_t VEL_CONTROL_COMMAND = 1;

  // navi state
  static constexpr uint8_t ARM_OFF_STATE = 0x00;
  static constexpr uint8_t START_STATE = 0x01;
  static constexpr uint8_t ARM_ON_STATE = 0x02;
  static constexpr uint8_t TAKEOFF_STATE = 0x03;
  static constexpr uint8_t LAND_STATE = 0x04;
  static constexpr uint8_t HOVER_STATE= 0x05;
  static constexpr uint8_t STOP_STATE = 0x06;

  //for ros arm/disarm cmd
  static constexpr uint8_t ARM_ON_CMD = 0x00; //old: 150;
  static constexpr uint8_t ARM_OFF_CMD = 0x01; //old: 151;
  static constexpr uint8_t ROS_INTEGRATE_CMD = 160;
  static constexpr uint8_t FORCE_LANDING_CMD = 0x02; //force landing

  // battery check
  static constexpr float VOLTAGE_100P =  4.2;
  static constexpr float VOLTAGE_90P =  4.085;
  static constexpr float VOLTAGE_80P =  3.999;
  static constexpr float VOLTAGE_70P =  3.936;
  static constexpr float VOLTAGE_60P =  3.883;
  static constexpr float VOLTAGE_50P =  3.839;
  static constexpr float VOLTAGE_40P =  3.812;
  static constexpr float VOLTAGE_30P =  3.791;
  static constexpr float VOLTAGE_20P =  3.747;
  static constexpr float VOLTAGE_10P =  3.683;
  static constexpr float VOLTAGE_0P =  3.209;

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  ros::Subscriber navi_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber arming_ack_sub_;
  ros::Subscriber takeoff_sub_;
  ros::Subscriber land_sub_;
  ros::Subscriber start_sub_;
  ros::Subscriber halt_sub_;
  ros::Subscriber force_landing_sub_;
  ros::Subscriber ctrl_mode_sub_;
  ros::Subscriber joy_stick_sub_;
  ros::Subscriber flight_nav_sub_;
  ros::Subscriber stop_teleop_sub_;

  BasicEstimator* estimator_;

  bool start_able_;
  uint8_t navi_state_;

  int  xy_control_mode_;
  int  prev_xy_control_mode_;
  bool xy_vel_mode_pos_ctrl_takeoff_;

  int  control_frame_;

  int estimate_mode_;
  bool  force_att_control_flag_;
  bool lock_teleop_;

  double convergent_start_time_;
  double convergent_duration_;
  double alt_convergent_thresh_;
  double xy_convergent_thresh_;

  //target value
  tf::Vector3 target_pos_, target_vel_, target_acc_;
  float target_psi_, target_vel_psi_;

  double takeoff_height_;

  double max_target_vel_;
  double max_target_tilt_angle_;
  double max_target_yaw_rate_;


  /* auto vel nav */
  bool vel_based_waypoint_;
  double nav_vel_limit_; // the vel limitation
  double vel_nav_threshold_; // the range (board) to siwtch between vel_nav and pos_nav
  double vel_nav_gain_;

  /* teleop */
  double joy_target_vel_interval_;
  double joy_target_alt_interval_;

  int navi_frame_int_;
  uint8_t navi_frame_;

  bool  vel_control_flag_;
  bool  pos_control_flag_;
  bool  xy_control_flag_;
  bool  alt_control_flag_;
  bool  yaw_control_flag_;

  bool teleop_flag_;
  bool force_landing_flag_;

  bool check_joy_stick_heart_beat_;
  bool joy_stick_heart_beat_;
  double joy_stick_prev_time_;
  double joy_stick_heart_beat_du_;
  double force_landing_to_halt_du_;

  /* battery info */
  double low_voltage_thre_;
  bool low_voltage_flag_;
  int bat_cell_;
  double bat_resistance_;
  double hovering_current_;

  void rosParamInit(ros::NodeHandle nh);
  void naviCallback(const aerial_robot_base::FlightNavConstPtr & msg);
  void joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg);
  void batteryCheckCallback(const std_msgs::Float32ConstPtr &msg);

  void startTakeoff()
  {
    if(getNaviState() == TAKEOFF_STATE) return;

    if(getNaviState() == ARM_ON_STATE)
      {
        setNaviState(TAKEOFF_STATE);
        ROS_INFO("Takeoff state");
      }
  }

  void motorArming()
  {
    /* z(altitude) */
    /* check whether there is the fusion for the altitude */
    if(!estimator_->getStateStatus(State::Z_BASE, estimate_mode_))
      {
        ROS_ERROR("Flight Navigation: No correct sensor fusion for z(altitude), can not fly");
        return;
      }

    setNaviState(START_STATE);
    setTargetXyFromCurrentState();
    setTargetPosZ(takeoff_height_);
    setTargetPsiFromCurrentState();

    ROS_INFO("Start state");
  }

  tf::Vector3 frameConversion(tf::Vector3 origin_val, float yaw)
  {
    tf::Matrix3x3 orien;
    orien.setRPY(0, 0, yaw);
    return orien * origin_val;
  }

  void armingAckCallback(const std_msgs::UInt8ConstPtr& ack_msg)
  {
    if(ack_msg->data == ARM_OFF_CMD)
      {//  arming off
        ROS_INFO("STOP RES From AERIAL ROBOT");
        setNaviState(ARM_OFF_STATE);
      }

    if(ack_msg->data == ARM_ON_CMD)
      {//  arming on
        ROS_INFO("START RES From AERIAL ROBOT");
        setNaviState(ARM_ON_STATE);
      }
  }

  void takeoffCallback(const std_msgs::EmptyConstPtr & msg)
  {
    startTakeoff();
  }

  void startCallback(const std_msgs::EmptyConstPtr & msg)
  {
    motorArming();
  }

  void landCallback(const std_msgs::EmptyConstPtr & msg)
  {
    if(!teleop_flag_) return;

    setNaviState(LAND_STATE);
    //更新
    setTargetXyFromCurrentState();
    setTargetPsiFromCurrentState();
    setTargetPosZ(estimator_->getLandingHeight());
    ROS_INFO("Land state");
  }

  void haltCallback(const std_msgs::EmptyConstPtr & msg)
  {
    if(!teleop_flag_) return;

    setNaviState(STOP_STATE);
    setTargetXyFromCurrentState();
    setTargetPsiFromCurrentState();
    setTargetPosZ(estimator_->getLandingHeight());

    estimator_->setSensorFusionFlag(false);
    estimator_->setLandingMode(false);
    estimator_->setLandedFlag(false);
    estimator_->setFlyingFlag(false);

    ROS_INFO("Halt state");
  }

  void forceLandingCallback(const std_msgs::EmptyConstPtr & msg)
  {
    std_msgs::UInt8 force_landing_cmd;
    force_landing_cmd.data = FORCE_LANDING_CMD;
    flight_config_pub_.publish(force_landing_cmd);

    ROS_INFO("Force Landing state");
  }

  void xyControlModeCallback(const std_msgs::Int8ConstPtr & msg)
  {
    if(getNaviState() > START_STATE)
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

  void stopTeleopCallback(const std_msgs::UInt8ConstPtr & stop_msg)
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

  void setTargetXyFromCurrentState()
  {
    tf::Vector3 pos_cog = estimator_->getPos(Frame::COG, estimate_mode_);
    target_pos_.setX(pos_cog.x());
    target_pos_.setY(pos_cog.y());
  }

  void setTargetZFromCurrentState()
  {
    target_pos_.setZ(estimator_->getPos(Frame::COG, estimate_mode_).z());
  }

  void setTargetPsiFromCurrentState()
  {
    target_psi_ = estimator_->getState(State::YAW_COG, estimate_mode_)[0];
  }

};


#endif
