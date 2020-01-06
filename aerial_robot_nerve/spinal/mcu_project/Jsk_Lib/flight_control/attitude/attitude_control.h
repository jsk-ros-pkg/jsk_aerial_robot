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
#include <std_srvs/SetBool.h>
#include <spinal/Pwms.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/PwmInfo.h>
#include <spinal/UavInfo.h>
#include <spinal/PMatrixPseudoInverseWithInertia.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <spinal/SetAttitudeGains.h>


#define MAX_PWM  54000
#define IDLE_DUTY 0.5f
#define FORCE_LANDING_INTEGRAL 0.0025f // 500Hz * 0.00005 = 0.025, 1600 -> 1500: 2sec

#define MAX_MOTOR_NUMBER 10

/* fail safe */
#define FLIGHT_COMMAND_TIMEOUT 500 //500ms
#define MAX_TILT_ANGLE 1.0f // rad

#define CONTROL_PUB_INTERVAL 100 //40hz //100 //10ms

#define MOTOR_TEST 0

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
  bool getForceLandingFlag() {return force_landing_flag_;}
  void setForceLandingFlag(bool force_landing_flag)
  {
    force_landing_flag_ = force_landing_flag;

#ifdef NERVE_COMM
    Spine::setServoControlFlag(!force_landing_flag);
#endif

  }
  float getPwm(uint8_t index) {return target_pwm_[index];}
  float getForce(uint8_t index) {return target_thrust_[index];}
  void levelPGain(float attitude_p_gain) { attitude_p_gain_[X] = attitude_p_gain; attitude_p_gain_[Y] = attitude_p_gain;}
  void levelIGain(float attitude_i_gain) { attitude_i_gain_[X] = attitude_i_gain; attitude_i_gain_[Y] = attitude_i_gain;}
  void levelDGain(float attitude_d_gain) { attitude_d_gain_[X] = attitude_d_gain; attitude_d_gain_[Y] = attitude_d_gain;}
  void yawPGain(float attitude_p_gain) { attitude_p_gain_[Z] = attitude_p_gain; }
  void yawIGain(float attitude_i_gain) { attitude_i_gain_[Z] = attitude_i_gain; }
  void yawDGain(float attitude_d_gain) { attitude_d_gain_[Z] = attitude_d_gain; }

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
  ros::Subscriber torque_allocation_matrix_inv_sub_;
  ros::Publisher anti_gyro_pub_;
  ros::ServiceServer att_control_srv_;
  ros::ServiceServer attitude_gains_srv_;

  bool setAttitudeControlCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool setAttitudeGainsCallback(spinal::SetAttitudeGains::Request& req, spinal::SetAttitudeGains::Response& res);
#else
  ros::Subscriber<spinal::FourAxisCommand, AttitudeController> four_axis_cmd_sub_;
  ros::Subscriber<spinal::PwmInfo, AttitudeController> pwm_info_sub_;
  ros::Subscriber<spinal::RollPitchYawTerms, AttitudeController> rpy_gain_sub_;
  ros::Subscriber<std_msgs::Float32, AttitudeController> pwm_test_sub_;
  ros::Subscriber<spinal::PMatrixPseudoInverseWithInertia, AttitudeController> p_matrix_pseudo_inverse_inertia_sub_;
  ros::Subscriber<spinal::TorqueAllocationMatrixInv, AttitudeController> torque_allocation_matrix_inv_sub_;
  ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response, AttitudeController> att_control_srv_;
  ros::ServiceServer<spinal::SetAttitudeGains::Request, spinal::SetAttitudeGains::Response, AttitudeController> attitude_gains_srv_;

  void setAttitudeControlCallback(const std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  void setAttitudeGainsCallback(const spinal::SetAttitudeGains::Request& req, spinal::SetAttitudeGains::Response& res);

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
  bool attitude_flag_;

  // Control Input
  float target_angle_[3];
  float target_cog_force_[3];
  float target_cog_torque_[3];

  //Nonlinear Dynamics Inversion Control
  float attitude_p_gain_[3];
  float attitude_i_gain_[3];
  float attitude_d_gain_[3];
  float attitude_term_limit_[3];
  float attitude_p_term_limit_[3];
  float attitude_i_term_limit_[3];
  float attitude_d_term_limit_[3];
  float attitude_yaw_p_i_term_;
  float error_angle_i_[3];
  float error_angle_i_limit_[3];
  bool attitude_gain_receive_flag_;
  float target_cog_angular_acc_[3];
  float torque_allocation_matrix_inv_[MAX_MOTOR_NUMBER][3];

  //LQI Control
  bool lqi_mode_;
  float p_lqi_gain_[MAX_MOTOR_NUMBER][3];
  float i_lqi_gain_[MAX_MOTOR_NUMBER][3];
  float d_lqi_gain_[MAX_MOTOR_NUMBER][3];
  float base_throttle_term_[MAX_MOTOR_NUMBER]; //[N]
  float yaw_pi_term_[MAX_MOTOR_NUMBER]; //[N]
  float yaw_term_[MAX_MOTOR_NUMBER]; //[N]
  float roll_pitch_term_[MAX_MOTOR_NUMBER]; //[N]
  int max_yaw_term_index_;
  // Gyro Moment Compensation for LQI
  float p_matrix_pseudo_inverse_[MAX_MOTOR_NUMBER][4];
  Matrix3f inertia_;

  // Thrust and PWM
  float target_thrust_[MAX_MOTOR_NUMBER];
  float target_pwm_[MAX_MOTOR_NUMBER];
  float min_duty_;
  float max_duty_;
  float min_thrust_; // max thrust is variant according to the voltage
  float force_landing_thrust_;
  int8_t rotor_devider_;
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
  void torqueAllocationMatrixInvCallback(const spinal::TorqueAllocationMatrixInv& msg);
  void pwmTestCallback(const std_msgs::Float32& pwm_msg);

  void pwmConversion(void);
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
