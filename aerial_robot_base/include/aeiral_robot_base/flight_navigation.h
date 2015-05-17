#ifndef FLIGHT_NAVIGATION_H
#define FLIGHT_NAVIGATION_H

//* ros
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <aerial_robot_msgs/VirtualRC.h>
#include <aerial_robot_msgs/RcData.h>
#include <tf/transform_broadcaster.h>
#include <aerial_robot_base/state_estimation.h>
#include <aerial_robot_base/control_input_array.h>
#include <aerial_robot_base/FlightNav.h>
#include <sensor_msgs/Joy.h>



class Navigator
{
 public:
  Navigator(ros::NodeHandle nh,	ros::NodeHandle nh_private, 
            Estimator* estimator, FlightCtrlInput* flight_ctrl_input,
            int ctrl_loop_rate);
  virtual ~Navigator();

  //*** startAble
  bool getStartAble();
  void startNavigation();
  void stopNavigation();

  //*** flightAble
  bool getFlightAble();
  void startFlight();
  void stopFlight();
  //*** command type
  uint8_t getNaviCommand();
  void setNaviCommand(const uint8_t  command);
  void tfPublish();

  float getTargetPosX();
  void setTargetPosX(float value);
  void addTargetPosX(float value);
  float getTargetVelX();
  void setTargetVelX(float value);
  float getTargetPosY();
  void setTargetPosY(float value);
  void addTargetPosY(float value);
  float getTargetVelY();
  void setTargetVelY(float value);
  float getTargetPosZ();
  void setTargetPosZ(float value);
  void addTargetPosZ(float value);
  float getTargetVelZ();
  void setTargetVelZ(float value);
  float getTargetTheta();
  void setTargetTheta(float value);
  float getTargetVelTheta();
  void setTargetVelTheta(float value);
  float getTargetPhy();
  void setTargetPhy(float value);
  float getTargetVelPhy();
  void setTargetVelPhy(float value);
  float getTargetPsi();
  void setTargetPsi(float value);
  float getTargetVelPsi();
  void setTargetVelPsi(float value);


  virtual uint8_t getFlightMode(); //for teleop navigator
  virtual void  setXyControlMode(uint8_t mode); //for teleop navigator
  virtual uint8_t getXyControlMode(); //for teleop navigator
  virtual bool getXyVelModePosCtrlTakeoff();
  virtual  bool getMotorStopFlag();
  virtual  void setMotorStopFlag(bool motor_stop_flag);
  virtual  bool getFreeFallFlag();
  virtual  void resetFreeFallFlag();
  virtual  uint8_t getThrowingMode();


  const static uint8_t POS_CONTROL_COMMAND = 0;
  const static uint8_t VEL_CONTROL_COMMAND = 1;

  static const uint8_t START_COMMAND = 0x00;
  static const uint8_t STOP_COMMAND = 0x01;
  static const uint8_t IDLE_COMMAND = 0x02;
  static const uint8_t TAKEOFF_COMMAND = 0x03;
  static const uint8_t LAND_COMMAND = 0x04;
  static const uint8_t HOVER_COMMAND= 0x05;


  //for throwing start, not good
  const static uint8_t THROWING_START_STANDBY = 0x31;
  const static uint8_t THROWING_START_ARMINGON = 0x32;
  const static uint8_t THROWING_START_ALTHOLD = 0x33;

  //for ros arm/disarm cmd
  const static uint8_t ARM_ON_CMD = 150;
  const static uint8_t ARM_OFF_CMD = 151;

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
    tf::TransformBroadcaster* tfB_ ; 

    Estimator* estimator_;
    FlightCtrlInput* flight_ctrl_input_;

    bool start_able_;
    bool flight_able_;
    uint8_t navi_command_;

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

    int ctrl_loop_rate;
    std::string map_frame_;
    std::string target_frame_;

};

class TeleopNavigator :public Navigator
{
 public:
  TeleopNavigator(ros::NodeHandle nh,
                  ros::NodeHandle nh_private,
                  Estimator* estimator,
                  FlightCtrlInput* flight_ctrl_input,
                  int ctrl_loop_rate);
  virtual ~TeleopNavigator();

  void teleopNavigation();
  void sendRcCmd();

  void takeoffCallback(const std_msgs::EmptyConstPtr & msg);
  void startCallback(const std_msgs::EmptyConstPtr & msg);
  void haltCallback(const std_msgs::EmptyConstPtr &  msg);
  void landCallback(const std_msgs::EmptyConstPtr &  msg);
  void rollCallback(const std_msgs::Int8ConstPtr & msg);
  void pitchCallback(const std_msgs::Int8ConstPtr & ms);
  void yawCallback(const std_msgs::Int8ConstPtr & msg);
  void throttleCallback(const std_msgs::Int8ConstPtr & msg);

  void joyStickControl(const sensor_msgs::JoyConstPtr joy_msg);
  void flightNavCallback(const aeiral_robot_base::FlightNavConstPtr& msg);

  void xyControlModeCallback(const std_msgs::Int8ConstPtr & msg);
  void armingAckCallback(const std_msgs::Int8ConstPtr& ack_msg);

  void targetValueCorrection();

  void throwingModeNavi(Estimator* estimator);
  uint8_t getThrowingMode(); 

  uint8_t getFlightMode();
  void  setXyControlMode(uint8_t mode); 
  uint8_t getXyControlMode(); 

  bool getMotorStopFlag();
  void setMotorStopFlag(bool motor_stop_flag); 

  bool getFreeFallFlag();
  void resetFreeFallFlag(); 

  bool getXyVelModePosCtrlTakeoff();

  const static uint8_t MAP_FRAME = 0;
  const static uint8_t BODY_FRAME = 1;

  //for pidFunction
  const static uint8_t TAKEOFF_MODE = 0;
  const static uint8_t FLIGHT_MODE = 1;
  const static uint8_t LAND_MODE = 2;
  const static uint8_t NO_CONTROL_MODE = 3; 
  const static uint8_t RESET_MODE = 4;

  const static int TAKEOFF_COUNT = 8;

  //for throwing
  const static uint8_t GOOD_XY_TRACKING = 0x01;
  const static uint8_t ZERO_XY_TRACKING = 0x02;
  const static uint8_t RECOVER_XY_TRACKING = 0x03;

  //for hovering convergence
  const static float POS_X_THRE = 0.15; //m
  const static float POS_Y_THRE = 0.15; //m
  const static float POS_Z_THRE = 0.05; //m


 private:
    ros::Publisher  rc_cmd_pub_;
    ros::Publisher  msp_cmd_pub_;
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




    //*** base navigation
    uint8_t flight_mode_; //important
    int  xy_control_mode_;
    bool xy_vel_mode_pos_ctrl_takeoff_;

    //*** free fall
    double free_fall_thre_;
    bool   use_free_fall_;
    bool   free_fall_flag_;
    bool   motor_stop_flag_;

    bool   use_throwing_mode_;
    uint8_t throwing_mode_;
    double throwing_mode_standby_alt_thre_;
    double throwing_mode_throw_acc_x_thre_;
    double throwing_mode_throw_acc_z_thre_;
    double throwing_mode_set_alt_acc_z_thre_;
    double throwing_mode_alt_hold_vel_z_thre_;
    int    throwing_mode_shift_step_cnt_thre_;

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
    

    void rosParamInit();
};


#endif
