/*
******************************************************************************
* File Name          : attitude_control.h
* Description        : attitude control interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __ATTITUDE_CONTROL_H
#define __ATTITUDE_CONTROL_H

#ifndef SIMULATION
#include "stm32f7xx_hal.h"
#include "config.h"
#include <ros.h>
#else
#include <ros/ros.h>
#endif

#include <math/AP_Math.h>
#include <vector>

#ifndef SIMULATION
/* state estimate  */
#include <Spine/spine.h>
#include "state_estimate/state_estimate.h"
/* battery status */
#include "battery_status/battery_status.h"
#endif

#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <spinal/Pwms.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/PwmInfo.h>
#include <spinal/UavInfo.h>
#include <spinal/PMatrixPseudoInverseWithInertia.h>



#define MAX_PWM  54000
#define IDLE_DUTY 0.5f
#define FORCE_LANDING_INTEGRAL 0.0025f // 500Hz * 0.00005 = 0.025, 1600 -> 1500: 2sec

#define MAX_MOTOR_NUMBER 10

/* fail safe */
#define FLIGHT_COMMAND_TIMEOUT 500 //500ms
#define MAX_TILT_ANGLE 0.43 // 25d degree

#define CONTROL_PUB_INTERVAL 100 //40hz //100 //10ms

#define MOTOR_TEST 0

// anzai TODO: ros subscribe
#define P_TERM_LEVEL_LIMIT 500
#define I_TERM_LEVEL_LIMIT 25
#define I_TERM_YAW_LIMIT 90

#define LEVEL_P_GAIN 403.065f
#define LEVEL_I_GAIN 419.85f
#define LEVEL_D_GAIN 106.1913f

#define YAW_P_GAIN 0.0f
#define YAW_I_GAIN 300.0f //multiwii: 339.3897f
#define YAW_D_GAIN 300.0f //multiwii: 218.818f


enum AXIS {
  X = 0,
  Y = 1,
  Z = 2,
};

class AttitudeController
{
public:
  AttitudeController();
  ~AttitudeController(){}

#ifdef SIMULATION
  void init(ros::NodeHandle* nh);
#else
  void init(TIM_HandleTypeDef* htim1, TIM_HandleTypeDef* htim2, StateEstimate* estimator, BatteryStatus* bat, ros::NodeHandle* nh);
#endif

  void baseInit(); // common part in both pc and board
  void update();

  void setStartControlFlag(bool start_control_flag);
  void setUavModel(int8_t uav_model);
  inline uint8_t getMotorNumber(){return motor_number_;}
  void setMotorNumber(uint8_t motor_number);
  void setPwmTestMode(bool pwm_test_flag){pwm_test_flag_ = pwm_test_flag; }
  void setIntegrateFlag(bool integrate_flag){integrate_flag_ = integrate_flag; }
  void setForceLandingFlag(bool force_landing_flag){force_landing_flag_ = force_landing_flag;}
  float getPwm(uint8_t index) {return target_pwm_[index];}
  float getForce(uint8_t index) {return target_thrust_[index];}
  void levelPGain(float torque_p_gain) { torque_p_gain_[X] = torque_p_gain; torque_p_gain_[Y] = torque_p_gain;}
  void levelIGain(float torque_i_gain) { torque_i_gain_[X] = torque_i_gain; torque_i_gain_[Y] = torque_i_gain;}
  void levelDGain(float torque_d_gain) { torque_d_gain_[X] = torque_d_gain; torque_d_gain_[Y] = torque_d_gain;}
  void yawPGain(float torque_p_gain) { torque_p_gain_[Z] = torque_p_gain; }
  void yawIGain(float torque_i_gain) { torque_i_gain_[Z] = torque_i_gain; }
  void yawDGain(float torque_d_gain) { torque_d_gain_[Z] = torque_d_gain; }

  bool activated();

private:

#ifndef SIMULATION
  TIM_HandleTypeDef* pwm_htim1_;
  TIM_HandleTypeDef* pwm_htim2_;
#endif

  ros::NodeHandle* nh_;

  ros::Publisher pwms_pub_;
  ros::Publisher control_term_pub_;
  spinal::Pwms pwms_msg_;
  spinal::RollPitchYawTerms control_term_msg_;

#ifdef SIMULATION
  ros::Subscriber four_axis_cmd_sub_;
  ros::Subscriber pwm_info_sub_;
  ros::Subscriber rpy_gain_sub_;
  ros::Subscriber pwm_test_sub_;
  ros::Subscriber p_matrix_pseudo_inverse_inertia_sub_;
  ros::Publisher anti_gyro_pub_;
#else
  ros::Subscriber<spinal::FourAxisCommand, AttitudeController> four_axis_cmd_sub_;
  ros::Subscriber<spinal::PwmInfo, AttitudeController> pwm_info_sub_;
  ros::Subscriber<spinal::RollPitchYawTerms, AttitudeController> rpy_gain_sub_;
  ros::Subscriber<spinal::PMatrixPseudoInverseWithInertia, AttitudeController> p_matrix_pseudo_inverse_inertia_sub_;
  ros::Subscriber<std_msgs::Float32, AttitudeController> pwm_test_sub_;

  StateEstimate* estimator_;
  BatteryStatus* bat_;
#endif

  int8_t uav_model_;
  uint8_t motor_number_;

  //Control Flag
  bool start_control_flag_;
  bool pwm_test_flag_;
  bool integrate_flag_;
  bool force_landing_flag_;
  bool can_comm_flag_; //two types: pwm_direct_type OR can_comm_type

  // Control Input
  float target_angle_[3];
  float target_cog_force_[3];
  float target_cog_torque_[3];

  //Nonlinear Dynamics Inversion Control
  float torque_p_gain_[3];
  float torque_i_gain_[3];
  float torque_d_gain_[3];
  float torque_p_term_[3];
  float torque_i_term_[3];
  float torque_d_term_[3];
  float error_angle_i_[3];
  float error_angle_i_limit_[3];

  //LQI Control
  bool lqi_mode_;
  float p_lqi_gain_[MAX_MOTOR_NUMBER][3];
  float i_lqi_gain_[MAX_MOTOR_NUMBER][3];
  float d_lqi_gain_[MAX_MOTOR_NUMBER][3];
  float base_throttle_term_[MAX_MOTOR_NUMBER]; //[N]
  float motor_rpy_force_[MAX_MOTOR_NUMBER]; //[N]
  // Gyro Moment Compensation for LQI
  float p_matrix_pseudo_inverse_[MAX_MOTOR_NUMBER][4];
  Matrix3f inertia_;

  // Thrust and PWM
  float target_thrust_[MAX_MOTOR_NUMBER];
  float target_pwm_[MAX_MOTOR_NUMBER];
  float min_duty_;
  float max_duty_;
  float abs_max_duty_;
  float max_thrust_;
  float min_thrust_;
  float force_landing_thrust_;
  int8_t pwm_conversion_mode_;
  std::vector<spinal::MotorInfo> motor_info_;
  uint8_t motor_ref_index_;
  float v_factor_;
  uint32_t voltage_update_last_time_;

  //PWM Test
  float pwm_test_value_;

  // Failsafe
  bool failsafe_;
  uint32_t flight_command_last_stamp_;

  void fourAxisCommandCallback( const spinal::FourAxisCommand &cmd_msg);
  void pwmInfoCallback( const spinal::PwmInfo &info_msg);
  void rpyGainCallback( const spinal::RollPitchYawTerms &gain_msg);
  void pMatrixInertiaCallback(const spinal::PMatrixPseudoInverseWithInertia& msg);
  void pwmTestCallback(const std_msgs::Float32& pwm_msg);

  float pwmConversion(float thrust);
  void pwmsControl(void);

  void reset(void);

  void inversionMapping(void);


  float limit(float input, float limit)
  {
    if (input > limit) return limit;
    else if(input < -limit) return -limit;
    else return input;
  }

#ifdef SIMULATION
  uint32_t HAL_GetTick(){ return ros::Time::now().toSec() * 1000; }
public:
  void setRPY(float r, float p, float y) {angles_.x = r; angles_.y = p; angles_.z = y; }
  void setAngular(float wx, float wy, float wz) {vel_.x = wx; vel_.y = wy; vel_.z = wz; }
  Vector3f angles_;
  Vector3f vel_;
  float DELTA_T;
#endif
};
#endif
