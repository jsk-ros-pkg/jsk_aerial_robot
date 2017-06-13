#ifndef FLIGHT_NAVIGATION_H
#define FLIGHT_NAVIGATION_H

//* ros
#include <ros/ros.h>
#include <aerial_robot_base/basic_state_estimation.h>
#include <aerial_robot_base/control_input_array.h>

#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <aerial_robot_msgs/FourAxisCommand.h>
#include <tf/transform_broadcaster.h>
#include <aerial_robot_base/FlightNav.h>
#include <sensor_msgs/Joy.h>

class Navigator
{
public:
  Navigator(ros::NodeHandle nh, ros::NodeHandle nh_private, 
            BasicEstimator* estimator, FlightCtrlInput* flight_ctrl_input,
            int ctrl_loop_rate);
  virtual ~Navigator();

  ros::Publisher  flight_config_pub_;

  inline bool getStartAble(){  return start_able_;}
  inline void startNavigation(){  start_able_ = true;}
  inline void stopNavigation() {  start_able_ = false;}

  inline uint8_t getFlightMode(){  return flight_mode_;}
  inline void setFlightMode(uint8_t flight_mode){ flight_mode_ = flight_mode;}

  inline uint8_t getNaviCommand(){  return navi_command_;}
  inline void setNaviCommand(const uint8_t  command){ navi_command_ = command;}

  inline uint8_t getXyControlMode(){  return (uint8_t)xy_control_mode_;}
  inline void setXyControlMode(uint8_t mode){  xy_control_mode_ = mode;}

  inline bool getXyVelModePosCtrlTakeoff(){  return xy_vel_mode_pos_ctrl_takeoff_;}


  /* temporary */
  inline float getStatePosX() { return estimator_->getState(BasicEstimator::X_W, estimate_mode_)[0]; }
  inline float getStateVelX() { return estimator_->getState(BasicEstimator::X_W, estimate_mode_)[1]; }
  inline float getStatePosY() { return estimator_->getState(BasicEstimator::Y_W, estimate_mode_)[0]; }
  inline float getStateVelY() { return estimator_->getState(BasicEstimator::Y_W, estimate_mode_)[1]; }
  inline float getStatePosZ() { return estimator_->getState(BasicEstimator::Z_W, estimate_mode_)[0]; }
  inline float getStateVelZ() { return estimator_->getState(BasicEstimator::Z_W, estimate_mode_)[1]; }
  inline float getStatePsiCog() { return estimator_->getState(BasicEstimator::YAW_W, estimate_mode_)[0]; }
  inline float getStateVelPsiCog() { return estimator_->getState(BasicEstimator::YAW_W, estimate_mode_)[1]; }
  inline float getStatePsiBoard() { return estimator_->getState(BasicEstimator::YAW_W_B, estimate_mode_)[0]; }
  inline float getStateVelPsiBoard() { return estimator_->getState(BasicEstimator::YAW_W_B, estimate_mode_)[1]; }

  inline float getTargetPosX(){  return current_target_pos_x_;}
  inline void setTargetPosX( float value){  final_target_pos_x_ = value;}
  inline void addTargetPosX( float value){  final_target_pos_x_ += value;}
  inline float getTargetVelX(){  return current_target_vel_x_;}
  inline void setTargetVelX( float value){  final_target_vel_x_= value;}
  inline float getTargetPosY(){  return current_target_pos_y_;}
  inline void setTargetPosY( float value){  final_target_pos_y_ = value;}
  inline void addTargetPosY( float value){  final_target_pos_y_ += value;}
  inline float getTargetVelY(){  return current_target_vel_y_;}
  inline void setTargetVelY( float value){  final_target_vel_y_ = value;}
  inline float getTargetPosZ(){  return current_target_pos_z_;}
  inline void setTargetPosZ( float value){  final_target_pos_z_ = value;}
  inline void addTargetPosZ( float value){  final_target_pos_z_ += value;}
  inline float getTargetVelZ(){  return current_target_vel_z_;}
  inline void setTargetVelZ( float value){  final_target_vel_z_ = value;}
  inline float getTargetTheta(){  return current_target_theta_;}
  inline void setTargetTheta( float value){  final_target_theta_ = value;}
  inline float getTargetVelTheta(){  return current_target_vel_theta_; }
  inline void setTargetVelTheta( float value){  final_target_vel_theta_ = value;}
  inline float getTargetPhy(){  return current_target_phy_;}
  inline void setTargetPhy( float value){  final_target_phy_ = value;}
  inline float getTargetVelPhy(){  return current_target_vel_phy_;}
  inline void setTargetVelPhy( float value){  final_target_vel_phy_ = value;}
  inline float getTargetPsi(){  return current_target_psi_;}
  inline void setTargetPsi( float value){  final_target_psi_ = value;}
  inline float getTargetVelPsi(){  return current_target_vel_psi_;}
  inline void setTargetVelPsi( float value){  final_target_vel_psi_ = value;}

  inline float getTargetAnglePitch(){  return target_pitch_angle_;}
  inline float getTargetAngleRoll(){  return target_roll_angle_;}

  void tfPublish();

  uint8_t getEstimateMode(){ return estimate_mode_;}
  void setEstimateMode(uint8_t estimate_mode){ estimate_mode_ = estimate_mode;}

  static constexpr uint8_t POS_CONTROL_COMMAND = 0;
  static constexpr uint8_t VEL_CONTROL_COMMAND = 1;

  // navi command
  static constexpr uint8_t START_COMMAND = 0x00;
  static constexpr uint8_t STOP_COMMAND = 0x01;
  static constexpr uint8_t IDLE_COMMAND = 0x02;
  static constexpr uint8_t TAKEOFF_COMMAND = 0x03;
  static constexpr uint8_t LAND_COMMAND = 0x04;
  static constexpr uint8_t HOVER_COMMAND= 0x05;

  //flight mode
  static constexpr uint8_t TAKEOFF_MODE = 0;
  static constexpr uint8_t FLIGHT_MODE = 1;
  static constexpr uint8_t LAND_MODE = 2;
  static constexpr uint8_t NO_CONTROL_MODE = 3; 
  static constexpr uint8_t RESET_MODE = 4;

  //for ros arm/disarm cmd
  static constexpr uint8_t ARM_ON_CMD = 0x00; //old: 150;
  static constexpr uint8_t ARM_OFF_CMD = 0x01; //old: 151;
  static constexpr uint8_t ROS_INTEGRATE_CMD = 160;
  static constexpr uint8_t FORCE_LANDING_CMD = 0x02; //force landing

  static constexpr uint8_t X_AXIS = 1;
  static constexpr uint8_t Y_AXIS = 2;
  static constexpr uint8_t Z_AXIS = 4;
  static constexpr uint8_t PITCH_AXIS = 8;
  static constexpr uint8_t ROLL_AXIS = 16;
  static constexpr uint8_t YAW_AXIS = 32;

  static constexpr uint8_t POS_WORLD_BASED_CONTROL_MODE = 0;
  static constexpr uint8_t POS_LOCAL_BASED_CONTROL_MODE = 1;
  static constexpr uint8_t VEL_WORLD_BASED_CONTROL_MODE = 2;
  static constexpr uint8_t VEL_LOCAL_BASED_CONTROL_MODE = 3;
  static constexpr uint8_t ATT_CONTROL_MODE = 4;

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Subscriber navi_sub_;
  ros::Subscriber battery_sub_;
  tf::TransformBroadcaster* br_ ; 

  BasicEstimator* estimator_;
  FlightCtrlInput* flight_ctrl_input_;

  bool start_able_;
  uint8_t navi_command_;
  uint8_t flight_mode_; //important

  int  xy_control_mode_;
  int  prev_xy_control_mode_;
  bool xy_vel_mode_pos_ctrl_takeoff_;

  int estimate_mode_;
  int low_voltage_thre_;
  bool low_voltage_flag_;

  // final target value
  double takeoff_height_;
  float final_target_pos_x_;
  float final_target_vel_x_;
  float final_target_pos_y_;
  float final_target_vel_y_;
  float final_target_pos_z_;
  float final_target_vel_z_;
  float final_target_theta_;
  float final_target_vel_theta_;
  float final_target_phy_;
  float final_target_vel_phy_;
  float final_target_psi_;
  float final_target_vel_psi_;

  //current target value
  float current_target_pos_x_;
  float current_target_vel_x_;
  float current_target_pos_y_;
  float current_target_vel_y_;
  float current_target_pos_z_;
  float current_target_vel_z_;
  float current_target_theta_;
  float current_target_vel_theta_;
  float current_target_phy_;
  float current_target_vel_phy_;
  float current_target_psi_;
  float current_target_vel_psi_;

  //att_control_mode
  double target_angle_rate_;
  double cmd_angle_lev2_gain_;
  float target_pitch_angle_;
  float target_roll_angle_;

  int ctrl_loop_rate_;
  std::string map_frame_;
  std::string target_frame_;

  void rosParamInit(ros::NodeHandle nh);

  void naviCallback(const aerial_robot_base::FlightNavConstPtr & msg);
  void batteryCheckCallback(const std_msgs::UInt8ConstPtr &msg);

  void startTakeoff()
  {
    if(getNaviCommand() == TAKEOFF_COMMAND) return;

    if(getStartAble())
      {
        setNaviCommand(TAKEOFF_COMMAND);
        ROS_INFO("Takeoff command");
      }
  }

  void motorArming()
  {
    /* z(altitude) */
    /* check whether there is the fusion for the altitude */
    if(!estimator_->getStateStatus(BasicEstimator::Z_W, estimate_mode_))
      {
        ROS_ERROR("Flight Navigation: No correct sensor fusion for z(altitude), can not fly");
        return;
      }

    setNaviCommand(START_COMMAND);
    final_target_pos_x_ = getStatePosX();
    final_target_pos_y_ = getStatePosY();
    final_target_psi_   = getStatePsiBoard();
    final_target_pos_z_ = takeoff_height_;
    ROS_INFO("Start command");
  }

};

class TeleopNavigator :public Navigator
{
public:
  TeleopNavigator(ros::NodeHandle nh,
                  ros::NodeHandle nh_private,
                  BasicEstimator* estimator,
                  FlightCtrlInput* flight_ctrl_input,
                  int ctrl_loop_rate);
  virtual ~TeleopNavigator();

  void takeoffCallback(const std_msgs::EmptyConstPtr & msg);
  void startCallback(const std_msgs::EmptyConstPtr & msg);
  void haltCallback(const std_msgs::EmptyConstPtr &  msg);
  void forceLandingCallback(const std_msgs::EmptyConstPtr &  msg);
  void landCallback(const std_msgs::EmptyConstPtr &  msg);

  void joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg);

  void xyControlModeCallback(const std_msgs::Int8ConstPtr & msg);
  void armingAckCallback(const std_msgs::UInt8ConstPtr& ack_msg);

  //for navigation => TODO
  void flightNavCallback(const aerial_robot_base::FlightNavConstPtr& msg);

  void stopTeleopCallback(const std_msgs::UInt8ConstPtr & stop_msg);

  void targetValueCorrection();
  void teleopNavigation();
  void sendAttCmd();


  static constexpr int TAKEOFF_COUNT = 8;

  //for hovering convergence
  static constexpr float POS_X_THRE = 0.15; //m
  static constexpr float POS_Y_THRE = 0.15; //m
  static constexpr float POS_Z_THRE = 0.05; //m


  static constexpr uint8_t MAP_FRAME = 0;
  static constexpr uint8_t BODY_FRAME = 1;


private:
  ros::Publisher  rc_cmd_pub_;
  //temporarily
  ros::Publisher  joints_ctrl_pub_;

  ros::Subscriber arming_ack_sub_;
  ros::Subscriber takeoff_sub_;
  ros::Subscriber land_sub_;
  ros::Subscriber start_sub_;
  ros::Subscriber halt_sub_;
  ros::Subscriber force_landing_sub_;
  ros::Subscriber roll_sub_;
  ros::Subscriber pitch_sub_;
  ros::Subscriber yaw_sub_;
  ros::Subscriber throttle_sub_;
  ros::Subscriber ctrl_mode_sub_;
  ros::Subscriber joy_stick_sub_;
  ros::Subscriber flight_nav_sub_;
  ros::Subscriber stop_teleop_sub_;

  //*** teleop navigation
  double even_move_distance_;
  double up_down_distance_;
  double forward_backward_distance_;
  double left_right_distance_;
  double target_vel_rate_;
  double target_pitch_roll_interval_;
  double target_alt_interval_;
  double target_yaw_rate_;

  int navi_frame_int_;
  uint8_t navi_frame_;

  bool  vel_control_flag_;
  bool  pos_control_flag_;
  bool  force_att_control_flag_;
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

  void rosParamInit(ros::NodeHandle nh);
};


#endif
