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
/* battery status */
#include "battery_status/battery_status.h"
#endif
#include "state_estimate/state_estimate.h"

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
#include <spinal/SetMRACParams.h>

#define MAX_PWM  54000
#define IDLE_DUTY 0.5f
#define FORCE_LANDING_INTEGRAL 0.0025f // 500Hz * 0.00005 = 0.025, 1600 -> 1500: 2sec

#define MAX_MOTOR_NUMBER 10

/* fail safe */
#define FLIGHT_COMMAND_TIMEOUT 500 //500ms
#define MAX_TILT_ANGLE 1.0f // rad

#define CONTROL_TERM_PUB_INTERVAL 100
#define CONTROL_FEEDBACK_STATE_PUB_INTERVAL 25
#define PWM_PUB_INTERVAL 100 //100ms

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
  void init(ros::NodeHandle* nh, StateEstimate* estimator);
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
  bool getIntegrateFlag(){return integrate_flag_; }
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

  bool activated();

private:

#ifndef SIMULATION
  TIM_HandleTypeDef* pwm_htim1_;
  TIM_HandleTypeDef* pwm_htim2_;
#endif

  ros::NodeHandle* nh_;

  ros::Publisher pwms_pub_;
  ros::Publisher control_term_pub_;
  ros::Publisher control_feedback_state_pub_;
  spinal::Pwms pwms_msg_;
  spinal::RollPitchYawTerms control_term_msg_;
  spinal::RollPitchYawTerm control_feedback_state_msg_;

#ifdef SIMULATION
  ros::Subscriber four_axis_cmd_sub_;
  ros::Subscriber pwm_info_sub_;
  ros::Subscriber rpy_gain_sub_;
  ros::Subscriber pwm_test_sub_;
  ros::Subscriber p_matrix_pseudo_inverse_inertia_sub_;
  ros::Subscriber torque_allocation_matrix_inv_sub_;
  ros::Subscriber sim_vol_sub_;
  ros::Publisher anti_gyro_pub_;
  ros::ServiceServer att_control_srv_;

  bool setAttitudeControlCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) { att_control_flag_ = req.data; }
  void setSimVolCallback(const std_msgs::Float32 vol_msg) { sim_voltage_ = vol_msg.data; }
  float sim_voltage_;

#else
  ros::Subscriber<spinal::FourAxisCommand, AttitudeController> four_axis_cmd_sub_;
  ros::Subscriber<spinal::PwmInfo, AttitudeController> pwm_info_sub_;
  ros::Subscriber<spinal::RollPitchYawTerms, AttitudeController> rpy_gain_sub_;
  ros::Subscriber<std_msgs::Float32, AttitudeController> pwm_test_sub_;
  ros::Subscriber<spinal::PMatrixPseudoInverseWithInertia, AttitudeController> p_matrix_pseudo_inverse_inertia_sub_;
  ros::Subscriber<spinal::TorqueAllocationMatrixInv, AttitudeController> torque_allocation_matrix_inv_sub_;
  ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response, AttitudeController> att_control_srv_;

  void setAttitudeControlCallback(const std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) { att_control_flag_ = req.data; }

  BatteryStatus* bat_;
#endif

  StateEstimate* estimator_;

  int8_t uav_model_;
  uint8_t motor_number_;
  bool start_control_flag_;
  bool pwm_test_flag_;
  bool integrate_flag_;
  bool force_landing_flag_;
  bool att_control_flag_;


  float target_angle_[3];
  float error_angle_i_[3];
  float error_angle_i_limit_[3];

  float torque_p_gain_[3];
  float torque_i_gain_[3];
  float torque_d_gain_[3];
  float thrust_p_gain_[MAX_MOTOR_NUMBER][3];
  float thrust_i_gain_[MAX_MOTOR_NUMBER][3];
  float thrust_d_gain_[MAX_MOTOR_NUMBER][3];
  float torque_allocation_matrix_inv_[MAX_MOTOR_NUMBER][3];
  float base_thrust_term_[MAX_MOTOR_NUMBER]; //[N]
  float roll_pitch_term_[MAX_MOTOR_NUMBER]; //[N]
  float yaw_term_[MAX_MOTOR_NUMBER]; //[N]
  float extra_yaw_pi_term_[MAX_MOTOR_NUMBER]; //[N]
  int max_yaw_term_index_;

  // Gyro Moment Compensation
  float p_matrix_pseudo_inverse_[MAX_MOTOR_NUMBER][4];
  ap::Matrix3f inertia_;

  // Failsafe
  bool failsafe_;
  uint32_t flight_command_last_stamp_;

  // Thrust PWM Conversion
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
  uint32_t control_term_pub_last_time_, control_feedback_state_pub_last_time_;
  uint32_t pwm_pub_last_time_;
  float pwm_test_value_; // PWM Test

  void fourAxisCommandCallback( const spinal::FourAxisCommand &cmd_msg);
  void pwmInfoCallback( const spinal::PwmInfo &info_msg);
  void rpyGainCallback( const spinal::RollPitchYawTerms &gain_msg);
  void pMatrixInertiaCallback(const spinal::PMatrixPseudoInverseWithInertia& msg);
  void torqueAllocationMatrixInvCallback(const spinal::TorqueAllocationMatrixInv& msg);
  void thrustGainMapping();
  void maxYawGainIndex();
  void pwmTestCallback(const std_msgs::Float32& pwm_msg);
  void pwmConversion(void);
  void pwmsControl(void);

  void reset(void);


  // MRAC control
#define MATH_PI 3.141592f
#define YAW_TORQUE_LIMIT 0.4f
  void MRACinit(void);
  void MRACReset(void);
  void MRACupdate(ap::Vector3f angles, ap::Vector3f vel);

  bool is_use_mrac_;
  int mrac_log_rate_;
  float mrac_target_cog_torque_[3];
  ap::Vector3f mrac_ratio_;
  ap::Vector3f rot_ref_; // reference model
  ap::Vector3f rot_ref_d_; // differential of ref model
  float gamma_[8]; // adaptive parameter
  ap::Matrix3f k1m_; // ref model param
  ap::Matrix3f k2m_; // ref model param
  ap::Matrix3f K1_; // mrac param
  ap::Matrix3f K2_; // mrac param
  float kappa_[8]; // mrac leaning rate
  float sigma_[8]; // constant sigma fix
  float pc_psi_;
  float attitude_p_;

  ros::Publisher mrac_gamma_pub_;
  ros::Publisher mrac_ref_model_pub_;
  std_msgs::Float32MultiArray mrac_gamma_msg_;
  std_msgs::Float32MultiArray mrac_ref_model_msg_;
  uint8_t mrac_param_pub_control_;

#ifdef SIMULATION
  ros::ServiceServer mrac_trigger_srv_;
  bool setMRACTriggerCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  ros::ServiceServer mrac_params_srv_;
  bool setMRACParamsCallback(spinal::SetMRACParams::Request& req, spinal::SetMRACParams::Response& res);
  // ros::Subscriber target_psi_sub_;
  // ros::Subscriber pc_psi_sub_;
#else
  ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response, AttitudeController> mrac_trigger_srv_;
  void setMRACTriggerCallback(const std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  ros::ServiceServer<spinal::SetMRACParams::Request, spinal::SetMRACParams::Response, AttitudeController> mrac_params_srv_;
  void setMRACParamsCallback(const spinal::SetMRACParams::Request& req, spinal::SetMRACParams::Response& res);
  // ros::Subscriber<std_msgs::Int16, AttitudeController> target_psi_sub_;
  // ros::Subscriber<std_msgs::Float32, AttitudeController> pc_psi_sub_;
#endif
  // void targetPsiCallback(const std_msgs::Int16& msg);
  // void pcPsiCallback(const std_msgs::Float32& msg);
  // bool target_psi_flag_;
  // bool pc_psi_flag_;
  // end of MRAC control


  float limit(float input, float limit)
  {
    if (input > limit) return limit;
    else if(input < -limit) return -limit;
    else return input;
  }

#ifdef SIMULATION
  bool use_ground_truth_;
  uint32_t HAL_GetTick(){ return ros::Time::now().toSec() * 1000; }

public:
  void useGroundTruth(bool flag) { use_ground_truth_ = flag; }
  void setTrueRPY(float r, float p, float y) {true_angles_.x = r; true_angles_.y = p; true_angles_.z = y; }
  void setTrueAngular(float wx, float wy, float wz) {true_vel_.x = wx; true_vel_.y = wy; true_vel_.z = wz; }
  ap::Vector3f true_angles_;
  ap::Vector3f true_vel_;
  float DELTA_T;
  double prev_time_;
#endif
};
#endif
