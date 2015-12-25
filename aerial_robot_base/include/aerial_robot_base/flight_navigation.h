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
#include <aerial_robot_msgs/RcData.h> //old
#include <aerial_robot_msgs/RcData2.h>
#include <tf/transform_broadcaster.h>
#include <aerial_robot_base/FlightNav.h>
#include <sensor_msgs/Joy.h>



class Navigator
{
 public:
  Navigator(ros::NodeHandle nh,	ros::NodeHandle nh_private, 
            BasicEstimator* estimator, FlightCtrlInput* flight_ctrl_input,
            int ctrl_loop_rate);
  virtual ~Navigator();

  ros::Publisher  config_cmd_pub_; //temporarily


  inline bool getStartAble(){  return start_able_;}
  inline void startNavigation(){  start_able_ = true;}
  inline void stopNavigation() {  start_able_ = false;}
  inline bool getFlightAble(){  return flight_able_;}
  inline void startFlight(){  flight_able_ = true;}
  inline void stopFlight(){  flight_able_ = false;}

  inline uint8_t getFlightMode(){  return flight_mode_;}
  inline void setFlightMode(uint8_t flight_mode){ flight_mode_ = flight_mode;}

  inline uint8_t getNaviCommand(){  return navi_command_;}
  inline void setNaviCommand(const uint8_t  command){ navi_command_ = command;}

  inline uint8_t getXyControlMode(){  return (uint8_t)xy_control_mode_;}
  inline void setXyControlMode(uint8_t mode){  xy_control_mode_ = mode;}

  inline uint8_t getGainTunningMode(){  return (uint8_t)gain_tunning_mode_;}
  inline void setGainTunningMode(uint8_t mode){  gain_tunning_mode_ = mode;}

  inline bool getXyVelModePosCtrlTakeoff(){  return xy_vel_mode_pos_ctrl_takeoff_;}


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

  void throwingModeNavi();

  void tfPublish();


  const static uint8_t POS_CONTROL_COMMAND = 0;
  const static uint8_t VEL_CONTROL_COMMAND = 1;

  //gain tunning mode
  static const uint8_t ATTITUDE_GAIN_TUNNING_MODE = 1;//also can operate in joystick

  // navi command
  static const uint8_t START_COMMAND = 0x00;
  static const uint8_t STOP_COMMAND = 0x01;
  static const uint8_t IDLE_COMMAND = 0x02;
  static const uint8_t TAKEOFF_COMMAND = 0x03;
  static const uint8_t LAND_COMMAND = 0x04;
  static const uint8_t HOVER_COMMAND= 0x05;

  //flight mode
  const static uint8_t TAKEOFF_MODE = 0;
  const static uint8_t FLIGHT_MODE = 1;
  const static uint8_t LAND_MODE = 2;
  const static uint8_t NO_CONTROL_MODE = 3; 
  const static uint8_t RESET_MODE = 4;




  //for throwing
  const static uint8_t THROWING_START_STANDBY = 0x31;
  const static uint8_t THROWING_START_ARMINGON = 0x32;
  const static uint8_t THROWING_START_ALT_HOLD = 0x33;

  const static uint8_t GOOD_XY_TRACKING = 0x01;
  const static uint8_t ZERO_XY_TRACKING = 0x02;
  const static uint8_t RECOVER_XY_TRACKING = 0x03;



  //for ros arm/disarm cmd
  const static uint8_t ARM_ON_CMD = 0x20; //old: 150;
  const static uint8_t ARM_OFF_CMD = 0x21; //old: 151;
  const static uint8_t ROS_INTEGRATE_CMD = 160;

  static const uint8_t X_AXIS = 1;
  static const uint8_t Y_AXIS = 2;
  static const uint8_t Z_AXIS = 4;
  static const uint8_t PITCH_AXIS = 8;
  static const uint8_t ROLL_AXIS = 16;
  static const uint8_t YAW_AXIS = 32;

  const static uint8_t POS_WORLD_BASED_CONTROL_MODE = 0;
  const static uint8_t POS_LOCAL_BASED_CONTROL_MODE = 1;
  const static uint8_t VEL_WORLD_BASED_CONTROL_MODE = 2;
  const static uint8_t VEL_LOCAL_BASED_CONTROL_MODE = 3;

  protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Subscriber navi_sub_;
    tf::TransformBroadcaster* br_ ; 

    BasicEstimator* estimator_;
    FlightCtrlInput* flight_ctrl_input_;

    bool start_able_;
    bool flight_able_;
    uint8_t navi_command_;
    uint8_t flight_mode_; //important

    int  xy_control_mode_;
    bool xy_vel_mode_pos_ctrl_takeoff_;


    //*** base navigation

    // final target value
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

    // gain tunning mode
    int gain_tunning_mode_; //for attitude gain
    double target_angle_rate_;
    double cmd_angle_lev2_gain_;
    float target_pitch_angle_;
    float target_roll_angle_;

    int ctrl_loop_rate_;
    std::string map_frame_;
    std::string target_frame_;

    void rosParamInit(ros::NodeHandle nh);

    void naviCallback(const aerial_robot_base::FlightNavConstPtr & msg);

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
  void landCallback(const std_msgs::EmptyConstPtr &  msg);
  void rollCallback(const std_msgs::Int8ConstPtr & msg);
  void pitchCallback(const std_msgs::Int8ConstPtr & ms);
  void yawCallback(const std_msgs::Int8ConstPtr & msg);
  void throttleCallback(const std_msgs::Int8ConstPtr & msg);

  void joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg);

  void xyControlModeCallback(const std_msgs::Int8ConstPtr & msg);
  void armingAckCallback(const std_msgs::Int8ConstPtr& ack_msg);

  //for navigation => TODO
  void flightNavCallback(const aerial_robot_base::FlightNavConstPtr& msg);

  void stopTeleopCallback(const std_msgs::UInt8ConstPtr & stop_msg);

  void targetValueCorrection();
  void teleopNavigation();
  void sendRcCmd();


  const static int TAKEOFF_COUNT = 8;

  //for hovering convergence
  const static float POS_X_THRE = 0.15; //m
  const static float POS_Y_THRE = 0.15; //m
  const static float POS_Z_THRE = 0.05; //m


  const static uint8_t MAP_FRAME = 0;
  const static uint8_t BODY_FRAME = 1;


 private:
  ros::Publisher  rc_cmd_pub_;
  ros::Publisher  rc_cmd_pub2_;

    //temporarily
  ros::Publisher  joints_ctrl_pub_;

    ros::Subscriber arming_ack_sub_;
    ros::Subscriber takeoff_sub_;
    ros::Subscriber land_sub_;
    ros::Subscriber start_sub_;
    ros::Subscriber halt_sub_;
    ros::Subscriber roll_sub_;
    ros::Subscriber pitch_sub_;
    ros::Subscriber yaw_sub_;
    ros::Subscriber throttle_sub_;
    ros::Subscriber ctrl_mode_sub_;
    ros::Subscriber joy_stick_sub_;
    ros::Subscriber flight_nav_sub_;
    ros::Subscriber stop_teleop_sub_;

    //*** teleop navigation
    double takeoff_height_;
    double even_move_distance_;
    double up_down_distance_;
    double forward_backward_distance_;
    double left_right_distance_;
    double target_vel_rate_;
    double target_pitch_roll_interval_;
    double target_alt_interval_;
    double target_yaw_rate_;

    double cmd_vel_lev2_gain_;
    int navi_frame_int_;
    uint8_t navi_frame_;

    bool  vel_control_flag_;
    bool  pos_control_flag_;
    bool  alt_control_flag_;
    bool  yaw_control_flag_;

    bool teleop_flag_;

    void rosParamInit(ros::NodeHandle nh);
};


#endif
