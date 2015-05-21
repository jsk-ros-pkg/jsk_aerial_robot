#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

//* ros
#include <ros/ros.h>
#include <aerial_robot_base/state_estimation.h>
#include <aerial_robot_base/control_input_array.h>
#include <aerial_robot_base/flight_navigation.h>

#include <aerial_robot_base/FourAxisPidDebug.h>
#include <aerial_robot_msgs/ITermBias.h>
#include <aerial_robot_msgs/RPYCtrlOffset.h>

//* for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <aerial_robot_msgs/DynamicReconfigureLevels.h>
#include <aerial_robot_base/PidPitchControlConfig.h>
#include <aerial_robot_base/PidRollControlConfig.h>
#include <aerial_robot_base/PidYawControlConfig.h>
#include <aerial_robot_base/PidThrottleControlConfig.h>



class FlightController
{
public:
  FlightController(ros::NodeHandle nh,
                   ros::NodeHandle nh_private,
                   Estimator* estimator, Navigator* navigator, 
                   FlightCtrlInput* flight_ctrl_input);
  virtual ~FlightController();

  float limit(float value, int limit);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  Navigator* navigator_;
  Estimator* estimator_;
  FlightCtrlInput* flight_ctrl_input_;

};

class PidController : public FlightController
{
 public:
  PidController(ros::NodeHandle nh,
                ros::NodeHandle nh_private,
                Estimator* estimator, Navigator* navigator,
                FlightCtrlInput* flight_ctrl_input,
                double ctrl_loop_rate);
  ~PidController();


  void pidFunction();
  void feedForwardFunction();

  //dynamic reconfigure
  void cfgPitchCallback(aerial_robot_base::PidPitchControlConfig &config, uint32_t level);
  void cfgRollCallback(aerial_robot_base::PidRollControlConfig &config, uint32_t level);
  void cfgThrottleCallback(aerial_robot_base::PidThrottleControlConfig &config, uint32_t level);
  void cfgYawCallback(aerial_robot_base::PidYawControlConfig &config, uint32_t level);

 private:
  ros::Publisher  pid_pub_;
  ros::Publisher motor_bias_set_pub_;
  ros::Subscriber motor_bias_set_sub_;
  ros::Subscriber rpy_ctrl_offset_sub_;

  int pid_ctrl_loop_rate_;
  int pitch_ctrl_cnt;
  int pitch_ctrl_loop_rate_;
  int roll_ctrl_cnt;
  int roll_ctrl_loop_rate_;
  int throttle_ctrl_cnt;
  int throttle_ctrl_loop_rate_;
  int yaw_ctrl_cnt;
  int yaw_ctrl_loop_rate_;
  
 //**** throttle
 double pos_p_gain_throttle_;
 double pos_i_gain_throttle_;
 double pos_d_gain_throttle_;
 double pos_p_gain_throttle_land_;
 double pos_i_gain_throttle_land_;
 double pos_d_gain_throttle_land_;
 double const_p_ctrl_thre_throttle_land_; 
 double const_p_term_lev1_value_throttle_land_;
 double const_p_term_lev2_value_throttle_land_;
 double const_i_ctrl_thre_throttle_land_; 
 double const_i_term_value_throttle_land_; 
 int offset_throttle_;
 int pos_limit_throttle_;
 int pos_p_limit_throttle_;
 int pos_i_limit_throttle_;
 int pos_d_limit_throttle_;
 int pos_p_limit_throttle_hover_;  
 double vel_value_limit_throttle_hover_;
 double i_enable_limit_throttle_hover_; 
 int  rocket_start_init_value_;
 int  rocket_start_init_increment_value_;
 int  rocket_start_step_value_;
 int  free_fall_step_value_;
 int  motor_stop_value_;
 int throwing_mode_init_value_from_rocket_start_;

 //**** pitch
 double pos_p_gain_pitch_;
 double pos_i_gain_pitch_;
 double pos_d_gain_pitch_;
 double pos_i_gain_pitch_hover_;
 double vel_p_gain_pitch_; 
 double vel_i_gain_pitch_; 
 int offset_pitch_;
 int pos_limit_pitch_;
 int pos_p_limit_pitch_;
 int pos_i_limit_pitch_;
 int pos_d_limit_pitch_;
 double vel_value_limit_pitch_;
 double i_enable_limit_pitch_;

 //**** roll
 double pos_p_gain_roll_;
 double pos_i_gain_roll_;
 double pos_d_gain_roll_;
 double pos_i_gain_roll_hover_;
 double vel_p_gain_roll_; 
 double vel_i_gain_roll_; 
 int offset_roll_;
 int pos_limit_roll_;
 int pos_p_limit_roll_;
 int pos_i_limit_roll_;
 int pos_d_limit_roll_;
 double vel_value_limit_roll_;
 double i_enable_limit_roll_;

 //**** yaw
 double pos_p_gain_yaw_;
 double pos_i_gain_yaw_;
 double pos_d_gain_yaw_;
 int pos_limit_yaw_;
 int pos_p_limit_yaw_;
 int pos_i_limit_yaw_;
 int pos_d_limit_yaw_;
 double vel_value_limit_yaw_;
 double i_enable_limit_yaw_;

 float d_err_pos_curr_pitch_;
 float d_err_pos_curr_roll_;
 float d_err_pos_curr_throttle_;
 float d_err_pos_curr_yaw_;

 float d_err_vel_curr_roll_;
 float d_err_vel_curr_pitch_;
 float d_err_vel_curr_yaw_;
 float d_err_vel_curr_throttle_;
 float d_err_vel_prev_roll_;
 float d_err_vel_prev_pitch_;
 float d_err_vel_prev_yaw_;
 float d_err_vel_prev_throttle_;

 float pos_i_term_roll_;
 float pos_i_term_pitch_;
 float pos_i_term_yaw_;
 float pos_i_term_throttle_;
 float pos_p_term_roll_;
 float pos_p_term_pitch_;
 float pos_p_term_yaw_;
 float pos_p_term_throttle_;
 float pos_d_term_roll_;
 float pos_d_term_pitch_;
 float pos_d_term_yaw_;
 float pos_d_term_throttle_;

 //motor bias set
 bool motor_bias_flag_;
 void motorBiasSetCallback(const std_msgs::Int8ConstPtr& msg);
 void rpyCtrlOffsetCallback(const aerial_robot_msgs::RPYCtrlOffsetConstPtr& offset_value);

 //dynamic reconfigure
 dynamic_reconfigure::Server<aerial_robot_base::PidPitchControlConfig>* pitch_server_;
 dynamic_reconfigure::Server<aerial_robot_base::PidRollControlConfig>* roll_server_;
 dynamic_reconfigure::Server<aerial_robot_base::PidThrottleControlConfig>* throttle_server_;
 dynamic_reconfigure::Server<aerial_robot_base::PidYawControlConfig>* yaw_server_;

 dynamic_reconfigure::Server<aerial_robot_base::PidPitchControlConfig>::CallbackType dynamic_reconf_func_pitch_;
 dynamic_reconfigure::Server<aerial_robot_base::PidRollControlConfig>::CallbackType dynamic_reconf_func_roll_;
 dynamic_reconfigure::Server<aerial_robot_base::PidThrottleControlConfig>::CallbackType dynamic_reconf_func_throttle_;
 dynamic_reconfigure::Server<aerial_robot_base::PidYawControlConfig>::CallbackType dynamic_reconf_func_yaw_;

 void rosParamInit(ros::NodeHandle nh);
 void throttleThrowingMode();

};



#endif
