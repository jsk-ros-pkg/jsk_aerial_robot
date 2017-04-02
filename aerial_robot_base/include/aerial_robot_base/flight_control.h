#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

//* ros
#include <ros/ros.h>
#include <aerial_robot_base/basic_state_estimation.h>
#include <aerial_robot_base/control_input_array.h>
#include <aerial_robot_base/flight_navigation.h>

#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <aerial_robot_msgs/FourAxisGain.h>
#include <aerial_robot_base/FourAxisPid.h>
#include <aerial_robot_base/MotorInfo.h>

//* for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <aerial_robot_msgs/DynamicReconfigureLevels.h>
#include <aerial_robot_base/XYPidControlConfig.h>

class FlightController
{
public:
  FlightController(ros::NodeHandle nh,
                   ros::NodeHandle nh_private,
                   BasicEstimator* estimator, Navigator* navigator, 
                   FlightCtrlInput* flight_ctrl_input);
  virtual ~FlightController(){};

  inline float limit(float value, float limit)
  {
    if(value > limit) { return limit;}
    else if(value < - limit) { return -limit; }
    else return value;
  }


  inline int getMotorNumber(){return motor_num_;}



 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  Navigator* navigator_;
  BasicEstimator* estimator_;
  FlightCtrlInput* flight_ctrl_input_;

  int motor_num_;

  bool feedforward_flag_;
  Eigen::MatrixXd feedforward_matrix_;

  //new param
  double min_pwm_, max_pwm_;
  double f_pwm_rate_; //gain which convert f to pwm and also take the bit shift into account
  double f_pwm_offset_;
  double m_f_rate_;
  double pwm_rate_; //percentage
  double force_landing_pwm_; //pwm
  int estimate_mode_;
};

class PidController : public FlightController
{
 public:
  PidController(ros::NodeHandle nh,
                ros::NodeHandle nh_private,
                BasicEstimator* estimator, Navigator* navigator,
                FlightCtrlInput* flight_ctrl_input,
                double ctrl_loop_rate);
  ~PidController(){};

  void pidFunction();
  void feedForwardFunction();

  //dynamic reconfigure
  void cfgXYPidCallback(aerial_robot_base::XYPidControlConfig &config, uint32_t level);

private:
  ros::Publisher  pid_pub_;
  ros::Publisher  ff_pub_;
  ros::Publisher  motor_info_pub_;
  ros::Subscriber four_axis_gain_sub_;
  ros::Subscriber xy_vel_weak_gain_sub_;

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
  std::vector<double>  pos_p_gain_throttle_;
  std::vector<double>  pos_i_gain_throttle_;
  std::vector<double>  pos_d_gain_throttle_;
  /* std::vector<double>  pos_p_gain_throttle_land_; */
  /* std::vector<double>  pos_i_gain_throttle_land_; */
  /* std::vector<double>  pos_d_gain_throttle_land_; */
  //double const_p_ctrl_thre_throttle_land_; 
  //double const_p_term_lev1_value_throttle_land_;
  //double const_p_term_lev2_value_throttle_land_;
  double const_i_ctrl_thre_throttle_land_; 
  //double const_i_term_value_throttle_land_; 
  double offset_throttle_;
  int pos_limit_throttle_;
  double land_gain_slow_rate_;
  int pos_p_limit_throttle_;
  int pos_i_limit_throttle_;
  int pos_d_limit_throttle_;
 //int pos_p_limit_throttle_hover_;  
 //double vel_value_limit_throttle_hover_;
 //double i_enable_limit_throttle_hover_; 
 //int  free_fall_step_value_;
 // int  motor_stop_value_;
 //int throwing_mode_init_value_from_rocket_start_;

 //**** pitch
 double pos_p_gain_pitch_;
 double pos_i_gain_pitch_;
 double pos_d_gain_pitch_;
 double pos_i_gain_pitch_hover_;
 double vel_p_gain_pitch_;
 double vel_i_gain_pitch_;
 double offset_pitch_;
 double pos_limit_pitch_;
 double pos_p_limit_pitch_;
 double pos_i_limit_pitch_;
 double pos_d_limit_pitch_;
 //double vel_value_limit_pitch_;
 double i_enable_limit_pitch_;

 //**** roll
 double pos_p_gain_roll_;
 double pos_i_gain_roll_;
 double pos_d_gain_roll_;
 double pos_i_gain_roll_hover_;
 double vel_p_gain_roll_; 
 double vel_i_gain_roll_; 
 double offset_roll_;
 double pos_limit_roll_;
 double pos_p_limit_roll_;
 double pos_i_limit_roll_;
 double pos_d_limit_roll_;
 //double vel_value_limit_roll_;
 double i_enable_limit_roll_;

  //*** pitch and roll
 std::string xy_vel_weak_gain_sub_name_;
 double xy_vel_weak_rate_;

 //**** yaw
 std::vector<double> pos_p_gain_yaw_;
 std::vector<double> pos_i_gain_yaw_;
 std::vector<double>  pos_d_gain_yaw_;
 int pos_limit_yaw_;
 int pos_p_limit_yaw_;
 int pos_i_limit_yaw_;
 int pos_d_limit_yaw_;
 //double vel_value_limit_yaw_;
 //double i_enable_limit_yaw_;

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
 float error_i_throttle_, error_i_yaw_;

 bool start_rp_integration_;

 //callback
 void fourAxisGainCallback(const aerial_robot_msgs::FourAxisGainConstPtr & msg);
 void xyVelWeakGainCallback(const std_msgs::UInt8ConstPtr & msg);

 //dynamic reconfigure
 dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>* xy_pid_server_;

 dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>::CallbackType dynamic_reconf_func_xy_pid_;
 void rosParamInit(ros::NodeHandle nh);

};



#endif
