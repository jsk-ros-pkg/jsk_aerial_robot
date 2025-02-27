/*
******************************************************************************
* File Name          : attitude_control.h
* Description        : attitude control interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "flight_control/attitude/attitude_control.h"

#ifdef SIMULATION
AttitudeController::AttitudeController(): DELTA_T(0), prev_time_(-1), use_ground_truth_(false), sim_voltage_(0) {}

void AttitudeController::init(ros::NodeHandle* nh, StateEstimate* estimator)
{
  nh_ = nh;
  estimator_ = estimator;

  pwms_pub_ = nh_->advertise<spinal::Pwms>("motor_pwms", 1);
  control_term_pub_ = nh_->advertise<spinal::RollPitchYawTerms>("rpy/pid", 1);
  control_feedback_state_pub_ = nh_->advertise<spinal::RollPitchYawTerm>("rpy/feedback_state", 1);
  anti_gyro_pub_ = nh_->advertise<std_msgs::Float32MultiArray>("gyro_moment_compensation", 1);
  gimbal_control_pub_ = nh_->advertise<sensor_msgs::JointState>("gimbals_ctrl", 1); // only for simulation

  four_axis_cmd_sub_ = nh_->subscribe("four_axes/command", 1, &AttitudeController::fourAxisCommandCallback, this);
  target_angvel_sub_ = nh_->subscribe("target_angvel", 1, &AttitudeController::targetAngvelCallback, this);
  pwm_info_sub_ = nh_->subscribe("motor_info", 1, &AttitudeController::pwmInfoCallback, this);
  rpy_gain_sub_ = nh_->subscribe("rpy/gain", 1, &AttitudeController::rpyGainCallback, this);
  p_matrix_pseudo_inverse_inertia_sub_ = nh_->subscribe("p_matrix_pseudo_inverse_inertia", 1, &AttitudeController::pMatrixInertiaCallback, this);
  pwm_test_sub_ = nh_->subscribe("pwm_test", 1, &AttitudeController::pwmTestCallback, this);
  att_control_srv_ = nh_->advertiseService("set_attitude_control", &AttitudeController::setAttitudeControlCallback, this);
  torque_allocation_matrix_inv_sub_ = nh_->subscribe("torque_allocation_matrix_inv", 1, &AttitudeController::torqueAllocationMatrixInvCallback, this);
  sim_vol_sub_ = nh_->subscribe("set_sim_voltage", 1, &AttitudeController::setSimVolCallback, this);
  gimbal_dof_sub_ = nh_->subscribe("gimbal_dof", 1, &AttitudeController::gimbalDofCallback, this);
  gimbal_indices_sub_ = nh_->subscribe("gimbal_indices", 1, &AttitudeController::gimbalIndicesCallback, this);

  baseInit();
}

#else

AttitudeController::AttitudeController():
  pwms_pub_("motor_pwms", &pwms_msg_),
  control_term_pub_("rpy/pid", &control_term_msg_),
  control_feedback_state_pub_("rpy/feedback_state", &control_feedback_state_msg_),
  four_axis_cmd_sub_("four_axes/command", &AttitudeController::fourAxisCommandCallback, this ),
  target_angvel_sub_("target_angvel", &AttitudeController::targetAngvelCallback, this),
  pwm_info_sub_("motor_info", &AttitudeController::pwmInfoCallback, this),
  rpy_gain_sub_("rpy/gain", &AttitudeController::rpyGainCallback, this),
  p_matrix_pseudo_inverse_inertia_sub_("p_matrix_pseudo_inverse_inertia", &AttitudeController::pMatrixInertiaCallback, this),
  pwm_test_sub_("pwm_test", &AttitudeController::pwmTestCallback, this ),
  att_control_srv_("set_attitude_control", &AttitudeController::setAttitudeControlCallback, this),
  torque_allocation_matrix_inv_sub_("torque_allocation_matrix_inv", &AttitudeController::torqueAllocationMatrixInvCallback, this),
  gimbal_dof_sub_("gimbal_dof", &AttitudeController::gimbalDofCallback, this),
  gimbal_indices_sub_("gimbal_indices", &AttitudeController::gimbalIndicesCallback, this)
{
}

void AttitudeController::init(TIM_HandleTypeDef* htim1, TIM_HandleTypeDef* htim2, StateEstimate* estimator, BatteryStatus* bat, ros::NodeHandle* nh, osMutexId* mutex)
{

  pwm_htim1_ = htim1;
  pwm_htim2_ = htim2;
  nh_ = nh;
  estimator_ = estimator;
  bat_ = bat;
  mutex_ = mutex;

  HAL_TIM_PWM_Start(pwm_htim1_,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(pwm_htim1_,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(pwm_htim1_,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(pwm_htim1_,TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_4);

  nh_->advertise(pwms_pub_);
  nh_->advertise(control_term_pub_);
  nh_->advertise(control_feedback_state_pub_);

  nh_->subscribe(four_axis_cmd_sub_);
  nh_->subscribe(target_angvel_sub_);
  nh_->subscribe(pwm_info_sub_);
  nh_->subscribe(rpy_gain_sub_);
  nh_->subscribe(pwm_test_sub_);
  nh_->subscribe(p_matrix_pseudo_inverse_inertia_sub_);
  nh_->subscribe(torque_allocation_matrix_inv_sub_);
  nh_->subscribe(gimbal_dof_sub_);
  nh_->subscribe(gimbal_indices_sub_);

  nh_->advertiseService(att_control_srv_);

  baseInit();
}
#endif

void AttitudeController::baseInit()
{
  // base param for uav model
  motor_number_ = 0;
  uav_model_ = -1;
  rotor_devider_ = 1;

  start_control_flag_ = false;
  force_landing_flag_ = false;
  att_control_flag_ = true;
  gimbal_dof_ = 0;
  gimbal_number_ = 0;
  gimbal_indices_received_ = false;
  gain_msg_size_ = 0;
  called_subscribers_ = 0;
  motor_number_error_subscribers_ = 0;
  printed_motor_number_error_ = false;

  // pwm
  pwm_conversion_mode_ = -1;
  min_duty_ = IDLE_DUTY;
  max_duty_ = IDLE_DUTY; //should assign right value from PC(ros)
  min_thrust_ = 0;
  force_landing_thrust_ = 0;
  pwm_pub_last_time_ = 0;
  pwm_test_flag_ = false;

  // voltage
  motor_info_.resize(0);
  v_factor_ = 1;
  motor_ref_index_ = 0;
  voltage_update_last_time_ = 0;

  control_term_pub_last_time_ = 0;
  control_feedback_state_pub_last_time_ = 0;
  gimbal_control_pub_last_time_ = 0;

  reset();
}

void AttitudeController::pwmsControl(void)
{
  /* target thrust -> target pwm */
  pwmConversion();

#ifdef SIMULATION
  /* control result publish */
  if(HAL_GetTick() - control_term_pub_last_time_ > CONTROL_TERM_PUB_INTERVAL)
    {
      control_term_pub_last_time_ = HAL_GetTick();
      control_term_pub_.publish(control_term_msg_);
    }

  if(HAL_GetTick() - control_feedback_state_pub_last_time_ > CONTROL_FEEDBACK_STATE_PUB_INTERVAL)
    {
      control_feedback_state_pub_last_time_ = HAL_GetTick();
      control_feedback_state_pub_.publish(control_feedback_state_msg_);
    }

  if(HAL_GetTick() - pwm_pub_last_time_ > PWM_PUB_INTERVAL)
    {
      pwm_pub_last_time_ = HAL_GetTick();
      pwms_pub_.publish(pwms_msg_);
    }

#else
  /* control result publish */
  if(uav_model_ == spinal::UavInfo::DRAGON)
    {
      if(HAL_GetTick() - control_feedback_state_pub_last_time_ > CONTROL_FEEDBACK_STATE_PUB_INTERVAL)
        {
          control_feedback_state_pub_last_time_ = HAL_GetTick();
          control_feedback_state_pub_.publish(&control_feedback_state_msg_);
        }
    }
  else
    {
      if(HAL_GetTick() - control_term_pub_last_time_ > CONTROL_TERM_PUB_INTERVAL)
        {
          control_term_pub_last_time_ = HAL_GetTick();
          control_term_pub_.publish(&control_term_msg_);
        }
    }

  if(HAL_GetTick() - pwm_pub_last_time_ > PWM_PUB_INTERVAL)
    {
      pwm_pub_last_time_ = HAL_GetTick();
      pwms_pub_.publish(&pwms_msg_);
    }

  /* nerve comm type */
#if NERVE_COMM
  for(int i = 0; i < motor_number_ / (gimbal_dof_ + 1); i++) {
#if MOTOR_TEST

    if (i == (HAL_GetTick() / 2000) % motor_number_)
      Spine::setMotorPwm(200, i);
    else
      Spine::setMotorPwm(0, i);
#else
    Spine::setMotorPwm(target_pwm_[i] * 2000 - 1000, i);
#endif
  }
#endif

  /* direct pwm type */
  pwm_htim1_->Instance->CCR1 = (uint32_t)(target_pwm_[0] * pwm_htim1_->Init.Period);
  pwm_htim1_->Instance->CCR2 = (uint32_t)(target_pwm_[1] * pwm_htim1_->Init.Period);
  pwm_htim1_->Instance->CCR3 = (uint32_t)(target_pwm_[2] * pwm_htim1_->Init.Period);
  pwm_htim1_->Instance->CCR4 = (uint32_t)(target_pwm_[3] * pwm_htim1_->Init.Period);

  pwm_htim2_->Instance->CCR1 =   (uint32_t)(target_pwm_[4] * pwm_htim2_->Init.Period);
  pwm_htim2_->Instance->CCR2 =  (uint32_t)(target_pwm_[5] * pwm_htim2_->Init.Period);
  pwm_htim2_->Instance->CCR3 = (uint32_t)(target_pwm_[6] * pwm_htim2_->Init.Period);
  pwm_htim2_->Instance->CCR4 =  (uint32_t)(target_pwm_[7] * pwm_htim2_->Init.Period);

#endif
}

void AttitudeController::update(void)
{
#ifndef SIMULATION
  /* mutex to wait for the completion of update of ros callback function */
  if(mutex_ != NULL)
    {
      osMutexWait(*mutex_, osWaitForever);
      osMutexRelease(*mutex_);
    }
#endif

  if(start_control_flag_ && att_control_flag_)
    {
      /* check the motor number and each ros command size is same */
      if(motor_number_error_subscribers_)
        {
          if(!printed_motor_number_error_)
            {
#ifdef SIMULATION
              ROS_ERROR("[spinal] motor number error %d occuer! Please publish new command with size %d.", motor_number_error_subscribers_, motor_number_);
#else
              nh_->logerror("[spinal] motor number error occuer! Please publish new command with correct size.");
#endif
              printed_motor_number_error_ = true;
              return;
            }
        }

      /* failsafe 1: check the timeout of the flight command receive process */
      if(failsafe_ && !force_landing_flag_ &&
         (int32_t)(HAL_GetTick() - flight_command_last_stamp_) > FLIGHT_COMMAND_TIMEOUT)
        {
          /* timeout => start force landing */
#ifdef SIMULATION
          ROS_ERROR("failsafe: cannot connect with ROS. time now: %d, time last stamp: %d", HAL_GetTick(), flight_command_last_stamp_);
#else
          nh_->logerror("failsafe: cannot connect with ROS");
#endif
          setForceLandingFlag(true);
        }

      ap::Vector3f angles;
      ap::Vector3f vel;
#ifdef SIMULATION
      if(use_ground_truth_)
        {
          angles = true_angles_;
          vel = true_vel_;
        }
      else
        {
          angles = estimator_->getAttEstimator()->getAttitude(Frame::VIRTUAL);
          vel = estimator_->getAttEstimator()->getAngular(Frame::VIRTUAL);
        }

      ROS_DEBUG_THROTTLE(0.01, "true vs spinal: r [%f vs %f], p [%f vs %f], y [%f vs %f], ws [%f vs %f], wy [%f vs %f], wz [%f vs %f]", true_angles_.x, angles.x, true_angles_.y, angles.y, true_angles_.z, angles.z, true_vel_.x, vel.x, true_vel_.y, vel.y, true_vel_.z, vel.z);

      if(prev_time_ < 0) DELTA_T = 0;
      else DELTA_T = ros::Time::now().toSec() - prev_time_;
      prev_time_ = ros::Time::now().toSec();
#else
      angles = estimator_->getAttEstimator()->getAttitude(Frame::VIRTUAL);
      vel = estimator_->getAttEstimator()->getAngular(Frame::VIRTUAL);
#endif

      /* failsafe 3: too large tile angle */
      if(!force_landing_flag_  && (fabs(angles[X]) > MAX_TILT_ANGLE || fabs(angles[Y]) > MAX_TILT_ANGLE))
        {
#ifdef SIMULATION
          ROS_ERROR("failsafe: the roll pitch angles are too large, roll: %f (%f), pitch: %f (%f)",
                    angles[X], MAX_TILT_ANGLE, angles[Y], MAX_TILT_ANGLE);
#else
          nh_->logerror("failsafe: the roll pitch angles are too large");
#endif
          setForceLandingFlag(true);
          error_angle_i_[X] = 0;
          error_angle_i_[Y] = 0;
        }

      /* Force Landing Flag */
      if(force_landing_flag_)
        {
          target_angle_[X] = 0;
          target_angle_[Y] = 0;
          target_angle_[Z] = 0;
          target_angvel_[X] = 0;
          target_angvel_[Y] = 0;
          target_angvel_[Z] = 0;

          for(int i = 0; i < motor_number_; i++) extra_yaw_pi_term_[i] = 0;
        }

      // linear control method
      {
        /* gyro moment */
        ap::Vector3f gyro_moment = vel % (inertia_ * vel);
#ifdef SIMULATION
        std_msgs::Float32MultiArray anti_gyro_msg;
#endif

        float error_angle[3], error_angvel[3];
        for(int axis = 0; axis < 3; axis++)
          {
            error_angle[axis] = target_angle_[axis] - angles[axis];
            error_angvel[axis] = target_angvel_[axis] - vel[axis];
            if(integrate_flag_) error_angle_i_[axis] += error_angle[axis] * DELTA_T;

            if(axis == X)
              {
                control_feedback_state_msg_.roll_p = error_angle[axis] * 1000;
                control_feedback_state_msg_.roll_i = error_angle_i_[axis] * 1000;
                control_feedback_state_msg_.roll_d = error_angvel[axis]  * 1000;
              }
            if(axis == Y)
              {
                control_feedback_state_msg_.pitch_p = error_angle[axis] * 1000;
                control_feedback_state_msg_.pitch_i = error_angle_i_[axis] * 1000;
                control_feedback_state_msg_.pitch_d = error_angvel[axis]  * 1000;

              }
            if(axis == Z)
              {
                control_feedback_state_msg_.yaw_d = vel[axis] * 1000;
              }
          }

        float p_term = 0;
        float i_term = 0;
        float d_term = 0;
        for(int i = 0; i < motor_number_; i++)
          {
            for(int axis = 0; axis < 3; axis++)
              {
                p_term = error_angle[axis] * thrust_p_gain_[i][axis];
                i_term = error_angle_i_[axis] * thrust_i_gain_[i][axis];
                d_term = error_angvel[axis] * thrust_d_gain_[i][axis];
                if(axis == X)
                  {
                    roll_pitch_term_[i] = p_term + i_term + d_term; // [N]
                    control_term_msg_.motors[i].roll_p = p_term * 1000;
                    control_term_msg_.motors[i].roll_i= i_term * 1000;
                    control_term_msg_.motors[i].roll_d = d_term * 1000;
                  }
                if(axis == Y)
                  {
                    roll_pitch_term_[i] += (p_term + i_term + d_term); // [N]
                    control_term_msg_.motors[i].pitch_p = p_term * 1000;
                    control_term_msg_.motors[i].pitch_i = i_term * 1000;
                    control_term_msg_.motors[i].pitch_d = d_term * 1000;
                  }
                if(axis == Z)
                  {
                    yaw_term_[i] = extra_yaw_pi_term_[i] + d_term;
                    control_term_msg_.motors[i].yaw_d = d_term * 1000; //d_term;
                  }
              }

            /* gyro moment compensation */
            float gyro_moment_compensate =
              p_matrix_pseudo_inverse_[i][0] * gyro_moment.x +
              p_matrix_pseudo_inverse_[i][1] * gyro_moment.y +
              p_matrix_pseudo_inverse_[i][2] * gyro_moment.z;
            roll_pitch_term_[i] += gyro_moment_compensate;

#ifdef SIMULATION
            anti_gyro_msg.data.push_back(gyro_moment_compensate);
#endif
          }

#ifdef SIMULATION
        anti_gyro_pub_.publish(anti_gyro_msg);
#endif
      }

      if(force_landing_flag_)
        {
          float total_thrust = 0;
          /* sum */
          for(int i = 0; i < motor_number_; i++) total_thrust += base_thrust_term_[i];
          /* average */
          float average_thrust = total_thrust / motor_number_;

          if(average_thrust > force_landing_thrust_)
            {
              for(int i = 0; i < motor_number_; i++)
                base_thrust_term_[i] -= (base_thrust_term_[i] / average_thrust * FORCE_LANDING_INTEGRAL);
            }
        }
    }

  /* target thrust -> target pwm -> HAL */
  pwmsControl();
}


void AttitudeController::reset(void)
{
  for(int i = 0; i < MAX_MOTOR_NUMBER; i++)
    {
      target_thrust_[i] = 0;
      target_pwm_[i] = IDLE_DUTY;

      base_thrust_term_[i] = 0;
      roll_pitch_term_[i] = 0;
      yaw_term_[i] = 0;
      extra_yaw_pi_term_[i] = 0;

      for (int j = 0; j < 3; j++)
        {
          thrust_p_gain_[i][j] = 0;
          thrust_i_gain_[i][j] = 0;
          thrust_d_gain_[i][j] = 0;
          torque_allocation_matrix_inv_[i][j] = 0.0;
        }
    }

  for(int i = 0; i < 3; i++)
    {
      target_angle_[i] = 0;
      target_angvel_[i] = 0;

      error_angle_i_[i] = 0;

      torque_p_gain_[i] = 0;
      torque_i_gain_[i] = 0;
      torque_d_gain_[i] = 0;
    }

  max_yaw_term_index_ = -1;
  integrate_flag_ = false;

  /* failsafe */
  failsafe_ = false;
  flight_command_last_stamp_ = HAL_GetTick();

#ifdef SIMULATION
  prev_time_ = -1;
#endif
}

void AttitudeController::fourAxisCommandCallback( const spinal::FourAxisCommand &cmd_msg)
{
  if(!start_control_flag_) return; //do not receive command


  /* failsafe: if the pitch and roll angle is too big, start force landing */
  if(fabs(cmd_msg.angles[0]) > MAX_TILT_ANGLE || fabs(cmd_msg.angles[1]) > MAX_TILT_ANGLE )
    {
      setForceLandingFlag(true);
#ifdef SIMULATION
      ROS_ERROR("failsafe: target angles are too large, roll: %f (%f), pitch: %f ()%f",
                target_angle_[X], MAX_TILT_ANGLE, target_angle_[Y], MAX_TILT_ANGLE);
#else
      nh_->logerror("failsafe: target angles are too large");
#endif
    }

  /* check the number of motor which should be equal to the ros thrust */
#ifdef SIMULATION
  if(cmd_msg.base_thrust.size() != motor_number_)
    {
      ROS_ERROR("[spinal] four axis command: motor number is not identical between fc(%d) and pc(%ld)", motor_number_, cmd_msg.base_thrust.size());
      return;
    }
#else
  if(cmd_msg.base_thrust_length != motor_number_)
    {
      nh_->logerror("[spinal] four axis command: motor number is not identical between fc and pc");
      return;
    }
#endif

  if(force_landing_flag_)
    {
      float total_thrust = 0;
      for(int i = 0; i < motor_number_; i++) total_thrust += cmd_msg.base_thrust[i];
      float average_thrust = total_thrust / motor_number_;
      if(average_thrust < force_landing_thrust_) return;
    }


#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

  called_subscribers_ |= (1 << (FOUR_AXIS_COMMAND - 1));

  /* start failsafe func if not activate */
  if(!failsafe_) failsafe_ = true;
  flight_command_last_stamp_ = HAL_GetTick();

  target_angle_[X] = cmd_msg.angles[0];
  target_angle_[Y] = cmd_msg.angles[1];

  for(int i = 0; i < motor_number_; i++)
    {
      // base thrust is about the z control
      base_thrust_term_[i] = cmd_msg.base_thrust[i];

      // reconstruct the pi term for yaw (temporary measure for pwm saturation avoidance)
      if(max_yaw_term_index_ != -1)
        extra_yaw_pi_term_[i] = cmd_msg.angles[Z] * thrust_d_gain_[i][Z] / thrust_d_gain_[max_yaw_term_index_][Z];
    }

  motor_number_error_subscribers_ &= ~(1 << (FOUR_AXIS_COMMAND - 1));

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

void AttitudeController::targetAngvelCallback( const geometry_msgs::Twist &twist_msg)
{
  if(!start_control_flag_) return;

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

  target_angvel_[0] = twist_msg.angular.x;
  target_angvel_[1] = twist_msg.angular.y;

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

void AttitudeController::pwmInfoCallback( const spinal::PwmInfo &info_msg)
{
#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

  force_landing_thrust_ = info_msg.force_landing_thrust;

  min_duty_ = info_msg.min_pwm;
  max_duty_ = info_msg.max_pwm;
  pwm_conversion_mode_ = info_msg.pwm_conversion_mode;

  min_thrust_ = info_msg.min_thrust; // make a variant min_duty_

  motor_info_.resize(0);

#ifdef SIMULATION
  for(int i = 0; i < info_msg.motor_info.size(); i++)
#else
    for(int i = 0; i < info_msg.motor_info_length; i++)
#endif
      {
        motor_info_.push_back(info_msg.motor_info[i]);
      }

#ifdef SIMULATION
  if(sim_voltage_== 0) sim_voltage_ = motor_info_[0].voltage;
#endif

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

void AttitudeController::rpyGainCallback( const spinal::RollPitchYawTerms &gain_msg)
{
  if(motor_number_ == 0) return; //not be activated

  /* check the number of motor which should be equal to the ros thrust */
#ifdef SIMULATION
  if(gain_msg.motors.size() != motor_number_ && gain_msg.motors.size() != 1)
    {
      ROS_ERROR("rpy gain: motor number is not identical between fc:%d and pc:%d", motor_number_, (int)gain_msg.motors.size());
      return;
    }
#else
  if(gain_msg.motors_length != motor_number_ && gain_msg.motors_length != 1)
    {
      nh_->logerror("rpy gain: motor number is not identical between fc and pc");
      return;
    }
#endif

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

  called_subscribers_ |= (1 << (RPY_GAIN - 1));

#ifdef SIMULATION
  if(gain_msg.motors.size() == 1)
#else
  if(gain_msg.motors_length == 1)
#endif
    {
      torque_p_gain_[X] = gain_msg.motors[0].roll_p * 0.001f;
      torque_p_gain_[Y] = gain_msg.motors[0].pitch_p * 0.001f;
      torque_i_gain_[X] = gain_msg.motors[0].roll_i * 0.001f;
      torque_i_gain_[Y] = gain_msg.motors[0].pitch_i * 0.001f;
      torque_d_gain_[X] = gain_msg.motors[0].roll_d * 0.001f;
      torque_d_gain_[Y] = gain_msg.motors[0].pitch_d * 0.001f;
      torque_d_gain_[Z] = gain_msg.motors[0].yaw_d * 0.001f;

      thrustGainMapping(); // gain mapping
      gain_msg_size_ = 1;
    }
  else
    {
      for(int i = 0; i < motor_number_; i++)
        {
          thrust_p_gain_[i][X] = gain_msg.motors[i].roll_p * 0.001f;
          thrust_i_gain_[i][X] = gain_msg.motors[i].roll_i * 0.001f;
          thrust_d_gain_[i][X] = gain_msg.motors[i].roll_d * 0.001f;
          thrust_p_gain_[i][Y] = gain_msg.motors[i].pitch_p * 0.001f;
          thrust_i_gain_[i][Y] = gain_msg.motors[i].pitch_i * 0.001f;
          thrust_d_gain_[i][Y] = gain_msg.motors[i].pitch_d * 0.001f;
          thrust_d_gain_[i][Z] = gain_msg.motors[i].yaw_d * 0.001f;
#ifdef SIMULATION
          gain_msg_size_ = gain_msg.motors.size();
#else
          gain_msg_size_ = gain_msg.motors_length;
#endif
        }
    }
  maxYawGainIndex();

  motor_number_error_subscribers_ &= ~(1 << (RPY_GAIN - 1));

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

void AttitudeController::torqueAllocationMatrixInvCallback(const spinal::TorqueAllocationMatrixInv& msg)
{
  if(motor_number_ == 0) return;

#ifdef SIMULATION
  if(msg.rows.size() != motor_number_)
    {
      ROS_ERROR("[spinal] torqueAllocationMatrixInvCallback: motor number is not identical between fc(%d) and pc(%ld)", motor_number_, msg.rows.size());
      return;
    }
#else
  if(msg.rows_length != motor_number_)
    {
      nh_->logerror("[spinal] torqueAllocationMatrixInvCallback: motor number is not identical between fc and pc");
      return;
    }
#endif

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

  called_subscribers_ |= (1 << (TORQUE_ALLOCATION_MATRIX_INV - 1));

  for (int i = 0; i < motor_number_; i++)
    {
      torque_allocation_matrix_inv_[i][X] = msg.rows[i].x * 0.001f;
      torque_allocation_matrix_inv_[i][Y] = msg.rows[i].y * 0.001f;
      torque_allocation_matrix_inv_[i][Z] = msg.rows[i].z * 0.001f;
    }

  thrustGainMapping(); // gain mapping

  maxYawGainIndex();

  motor_number_error_subscribers_ &= ~(1 << (TORQUE_ALLOCATION_MATRIX_INV - 1));

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

void AttitudeController::gimbalDofCallback(const std_msgs::UInt8& msg)
{
#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

  gimbal_dof_ = msg.data;
  setMotorNumber((uint8_t)getInitialMotorNumber() * ((uint8_t)gimbal_dof_ + 1));

  if(called_subscribers_ & (1 << (FOUR_AXIS_COMMAND - 1))) motor_number_error_subscribers_ |= (1 << (FOUR_AXIS_COMMAND - 1));
  if((called_subscribers_ & (1 << (RPY_GAIN - 1))) && gain_msg_size_ != 1) motor_number_error_subscribers_ |= (1 << (RPY_GAIN - 1));
  if(called_subscribers_ & (1 << (P_MATRIX_INERTIA - 1))) motor_number_error_subscribers_ |= (1 << (P_MATRIX_INERTIA - 1));
  if(called_subscribers_ & (1 << (TORQUE_ALLOCATION_MATRIX_INV - 1))) motor_number_error_subscribers_ |= (1 << (TORQUE_ALLOCATION_MATRIX_INV - 1));
  if(motor_number_error_subscribers_) printed_motor_number_error_ = false;

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

void AttitudeController::gimbalIndicesCallback(const std_msgs::UInt8MultiArray& msg)
{
#ifndef SIMULATION
  if(msg.data_length == 0)
    {
      nh_->logerror("[spinal] gimbal indice is empty.");
    }
  else
    {
      for(int i = 0; i < msg.data_length; i++)
        {
          gimbal_indices_[i] = msg.data[i];
        }

      /* mutex to protect the completion of following update  */
      if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);

      gimbal_number_ = msg.data_length;
      gimbal_indices_received_ = true;

      /* mutex to protect the completion of following update  */
      if(mutex_ != NULL) osMutexRelease(*mutex_);

      nh_->logwarn("[spinal] receive gimbal indices.");
    }

#else
  ROS_WARN("[spinal] receive gimbal indices.");
  return;
#endif
}

void AttitudeController::thrustGainMapping()
{
  for(int i = 0; i < motor_number_; i++)
    {
      thrust_p_gain_[i][X] = torque_allocation_matrix_inv_[i][X] * torque_p_gain_[X];
      thrust_i_gain_[i][X] = torque_allocation_matrix_inv_[i][X] * torque_i_gain_[X];
      thrust_d_gain_[i][X] = torque_allocation_matrix_inv_[i][X] * torque_d_gain_[X];
      thrust_p_gain_[i][Y] = torque_allocation_matrix_inv_[i][Y] * torque_p_gain_[Y];
      thrust_i_gain_[i][Y] = torque_allocation_matrix_inv_[i][Y] * torque_i_gain_[Y];
      thrust_d_gain_[i][Y] = torque_allocation_matrix_inv_[i][Y] * torque_d_gain_[Y];
      thrust_d_gain_[i][Z] = torque_allocation_matrix_inv_[i][Z] * torque_d_gain_[Z];
    }
}

void AttitudeController::maxYawGainIndex()
{
  float max_yaw_gain = 0;
  max_yaw_term_index_ = -1;
  for(int i = 0; i < motor_number_; i++)
    {
      /* only find the maximum (positive) value */
      /* to avoid identical absolute value */
      if(thrust_d_gain_[i][Z] > max_yaw_gain)
        {
          max_yaw_gain = thrust_d_gain_[i][Z];
          max_yaw_term_index_ = i;
        }
    }
}

void AttitudeController::pwmTestCallback(const spinal::PwmTest& pwm_msg)
{
#ifndef SIMULATION  
  if(pwm_msg.pwms_length && !pwm_test_flag_)
    {
      pwm_test_flag_ = true;
      nh_->logwarn("Enter pwm test mode");
    }
  else if(!pwm_msg.pwms_length && pwm_test_flag_)
    {
      pwm_test_flag_ = false;
      nh_->logwarn("Escape from pwm test mode");
      return;
    }

  if(pwm_msg.motor_index_length)
    {
      /*Individual test mode*/
      if(pwm_msg.motor_index_length != pwm_msg.pwms_length)
        {
          nh_->logerror("The number of index does not match the number of pwms.");
          return;
        }
      for(int i = 0; i < pwm_msg.motor_index_length; i++){
        int motor_index = pwm_msg.motor_index[i];
        pwm_test_value_[motor_index] = pwm_msg.pwms[i];
      }
    }
  else
    {
      /*Simultaneous test mode*/
      for(int i = 0; i < MAX_MOTOR_NUMBER; i++){
        pwm_test_value_[i] = pwm_msg.pwms[0];
      }
    }
#endif  
}

void AttitudeController::setStartControlFlag(bool start_control_flag)
{
  start_control_flag_ = start_control_flag;

  if(!start_control_flag_) reset();
}

void AttitudeController::setMotorNumber(uint8_t motor_number)
{
  /* check the motor number which has spine system */
//   if(motor_number_ > 0)
//     {
//       if(motor_number_ != motor_number)
//         {
//           motor_number_ = 0;
// #ifdef SIMULATION
//           ROS_ERROR("ATTENTION: motor number is 0");
// #else
//           nh_->logerror("ATTENTION: motor number is 0");
// #endif
//         }
//     }
//   else
//     {

      if(motor_number == 0) return;

      size_t control_term_msg_size  = motor_number;

#ifdef SIMULATION
      pwms_msg_.motor_value.resize(motor_number);
      control_term_msg_.motors.resize(control_term_msg_size);
#else
      pwms_msg_.motor_value_length = motor_number;
      control_term_msg_.motors_length = control_term_msg_size;
      pwms_msg_.motor_value = new uint16_t[motor_number];
      control_term_msg_.motors = new spinal::RollPitchYawTerm[control_term_msg_size];
#endif
      for(int i = 0; i < motor_number; i++) pwms_msg_.motor_value[i] = 0;

      /* the initialize order is important */
      motor_number_ = motor_number ;

#ifdef SIMULATION
      ROS_ERROR("[spinal] motor number is set to %d.", motor_number_);
#else
      nh_->logerror("[spinal] motor number is set.");
#endif
      // }
}

void  AttitudeController::setUavModel(int8_t uav_model)
{
  /* check the uav model which has spine system */
  uav_model_ = uav_model;

  if(uav_model_ == spinal::UavInfo::DRAGON) {
    rotor_devider_ = 2; // dual-rotor
  }
}

void AttitudeController::pMatrixInertiaCallback(const spinal::PMatrixPseudoInverseWithInertia& msg)
{
  if(motor_number_ == 0) return;

#ifdef SIMULATION
  if(msg.pseudo_inverse.size() != motor_number_)
    {
      if(motor_number_ > 0) ROS_ERROR("p matrix pseudo inverse and inertia commnd: motor number is not identical between fc and pc");
      return;
    }
#else
  if(msg.pseudo_inverse_length != motor_number_)
    {
      nh_->logerror("p matrix pseudo inverse and inertia commnd: motor number is not identical between fc and pc");
      return;
    }
#endif

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

  called_subscribers_ |= (1 << P_MATRIX_INERTIA);

  for(int i = 0; i < motor_number_; i ++)
    {
      p_matrix_pseudo_inverse_[i][0] = msg.pseudo_inverse[i].r * 0.001f;
      p_matrix_pseudo_inverse_[i][1] = msg.pseudo_inverse[i].p * 0.001f;
      p_matrix_pseudo_inverse_[i][2] = msg.pseudo_inverse[i].y * 0.001f;
    }

  /* inertia */
  inertia_ = ap::Matrix3f(msg.inertia[0] * 0.001f, msg.inertia[3] * 0.001f, msg.inertia[5] * 0.001f,
                          msg.inertia[3] * 0.001f, msg.inertia[1] * 0.001f, msg.inertia[4] * 0.001f,
                          msg.inertia[5] * 0.001f, msg.inertia[4] * 0.001f, msg.inertia[2] * 0.001f);

  motor_number_error_subscribers_ &= ~(1 << P_MATRIX_INERTIA);

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}

bool AttitudeController::activated()
{
  /* uav model check and motor property */
  if(motor_number_ > 0 && uav_model_ >= spinal::UavInfo::DRONE && max_duty_ > min_duty_) return true;
  else return false;
}

void AttitudeController::pwmConversion()
{
  auto convert = [this](float target_thrust)
    {
      float scaled_thrust = v_factor_ * target_thrust / rotor_devider_;
      float target_pwm = 0;
      if (scaled_thrust < 0) scaled_thrust = 0;

      switch(pwm_conversion_mode_)
        {
        case spinal::MotorInfo::SQRT_MODE:
          {
            /* pwm = F_inv[(V_ref / V)^2 f] */
            float sqrt_tmp = motor_info_[motor_ref_index_].polynominal[1] * motor_info_[motor_ref_index_].polynominal[1] - 4 * 10 * motor_info_[motor_ref_index_].polynominal[2] * (motor_info_[motor_ref_index_].polynominal[0] - scaled_thrust); //special decimal order shift (x10)
            target_pwm = (-motor_info_[motor_ref_index_].polynominal[1] + sqrt_tmp * ap::inv_sqrt(sqrt_tmp)) / (2 * motor_info_[motor_ref_index_].polynominal[2]);
            break;
          }
        case spinal::MotorInfo::POLYNOMINAL_MODE:
          {
            /* pwm = F_inv[(V_ref / V)^1.5 f] */
            float tenth_scaled_thrust = scaled_thrust * 0.1f; //special decimal order shift (x0.1)
            /* 4 dimensional */
            int max_dimenstional = 4;
            target_pwm = motor_info_[motor_ref_index_].polynominal[max_dimenstional];
            for (int j = max_dimenstional - 1; j >= 0; j--)
              target_pwm = target_pwm * tenth_scaled_thrust + motor_info_[motor_ref_index_].polynominal[j];
            break;
          }
        default:
          {
            break;
          }
        }
      return target_pwm / 100; // target_pwm is [%]
    };

  if(pwm_test_flag_) /* motor pwm test */
    {
      for(int i = 0; i < MAX_MOTOR_NUMBER; i++)
        {
          target_pwm_[i] = pwm_test_value_[i];
        }
      return;
    }

  if(motor_info_.size() == 0) return;

  /* update the factor regarding the robot voltage */
  if(HAL_GetTick() - voltage_update_last_time_ > 500) //[500ms = 0.5s]
    {
#ifdef SIMULATION
      float voltage = sim_voltage_;
#else
      float voltage = bat_->getVoltage();
#endif

      /* find the best reference voltage */
      float min_voltage_diff = 1e6;
      for(int i = 0; i < motor_info_.size(); i++)
        {
          float voltage_diff = fabs(voltage - motor_info_[i].voltage);
          if(min_voltage_diff > voltage_diff)
            {
              motor_ref_index_ = i;
              min_voltage_diff = voltage_diff;
            }
        }

      switch(pwm_conversion_mode_)
        {
        case spinal::MotorInfo::SQRT_MODE:
          {
            /* pwm = F_inv[(V_ref / V)^2 f] */
            v_factor_ = (motor_info_[motor_ref_index_].voltage / voltage) *  (motor_info_[motor_ref_index_].voltage / voltage) ;
            break;
          }
        case spinal::MotorInfo::POLYNOMINAL_MODE:
          {
            /* pwm = F_inv[(V_ref / V)^1.5 f] */
            v_factor_ = motor_info_[motor_ref_index_].voltage / voltage * ap::inv_sqrt(voltage / motor_info_[motor_ref_index_].voltage);
            break;
          }
        default:
          {
            break;
          }
        }

      if(min_thrust_> 0) min_duty_ = convert(min_thrust_);

      voltage_update_last_time_ = HAL_GetTick();
    }

  /* pwm saturation avoidance */
  /* get the decreasing rate for the thrust to avoid the devergence because of the pwm saturation */
  float base_thrust_decreasing_rate = 0;
  float yaw_decreasing_rate = 0;
  float thrust_limit = motor_info_[motor_ref_index_].max_thrust / v_factor_;

  /* check saturation level 2: z control saturation */
  float max_thrust = 0;
  int max_thrust_index = 0;
  for(int i = 0; i < motor_number_ / (gimbal_dof_ + 1); i++)
    {
      float thrust;
      if(gimbal_dof_)
        {
          thrust = ap::pythagorous2(base_thrust_term_[2 * i] + roll_pitch_term_[2 * i], base_thrust_term_[2 * i + 1] + roll_pitch_term_[2 * i + 1]);
        }
      else
        {
          thrust = base_thrust_term_[i] + roll_pitch_term_[i];
        }
      if(max_thrust < thrust)
        {
          max_thrust = thrust;
          max_thrust_index = i;
        }
    }

  if(start_control_flag_)
    {
      float residual_term = thrust_limit - max_thrust / rotor_devider_;

      if(residual_term < 0 && base_thrust_term_[max_thrust_index] > 0)
        {
          base_thrust_decreasing_rate = residual_term / (base_thrust_term_[max_thrust_index] / rotor_devider_);
          yaw_decreasing_rate = -1; // also, we have to ignore the yaw control
        }
      else
        {
          if(max_yaw_term_index_ != -1 && fabs(base_thrust_term_[0]) > 0 )
            {
              /* check saturation level1: yaw control saturation */
              max_thrust = 0;
              float min_thrust = 10000;
              int min_thrust_index = 0;
              for(int i = 0; i < motor_number_ / (gimbal_dof_ + 1); i++)
                {
                  float thrust;
                  if(gimbal_dof_)
                    {
                      thrust = ap::pythagorous2(base_thrust_term_[2 * i] + roll_pitch_term_[2 * i], base_thrust_term_[2 * i + 1] + roll_pitch_term_[2 * i + 1]);
                    }
                  else
                    {
                      thrust = base_thrust_term_[i] + roll_pitch_term_[i];
                    }
                  if(max_thrust < thrust)
                    {
                      max_thrust = thrust;
                      max_thrust_index = i;
                    }
                  if(min_thrust > thrust)
                    {
                      min_thrust = thrust;
                      min_thrust_index = i;
                    }
                }

              float residual_term_max =  thrust_limit - max_thrust / rotor_devider_;
              float residual_term_min =  min_thrust / rotor_devider_ - min_thrust_;
              int thrust_index = 0;
              if (residual_term_min < residual_term_max)
                {
                  residual_term = residual_term_min;
                  thrust_index = min_thrust_index;
                }
              else
                {
                  residual_term = residual_term_max;
                  thrust_index = max_thrust_index;
                }

              if(residual_term < 0)
                {
                  yaw_decreasing_rate = residual_term / (fabs(yaw_term_[thrust_index]) / rotor_devider_);
                }

              if(yaw_decreasing_rate < -1) yaw_decreasing_rate = -1;
              if(yaw_decreasing_rate > 0) yaw_decreasing_rate = 0;
            }
          else
            {
              yaw_decreasing_rate = -1;
            }
        }
    }

  for(int i = 0; i < motor_number_; i++)
    target_thrust_[i] = roll_pitch_term_[i] + (1 + base_thrust_decreasing_rate) * base_thrust_term_[i] + (1 + yaw_decreasing_rate) * yaw_term_[i];

  /* convert to target pwm and gimbal angles */
  for(int i = 0; i < motor_number_ / (gimbal_dof_ + 1); i++)
    {
      if(start_control_flag_)
        {
          /* gimbal */
          if(gimbal_dof_)
            {
              ap::Vector3f f_i;
              f_i.y = target_thrust_[2 * i];
              f_i.z = target_thrust_[2 * i + 1];
              target_thrust_[i] = ap::pythagorous2(f_i.y, f_i.z);
              float gimbal_candidate = atan2f(-f_i.y, f_i.z);
              if(std::isfinite(gimbal_candidate))
                {
                  target_gimbal_angles_[i] = (target_gimbal_angles_[i] + gimbal_candidate) / 2;
                }
            }

          /* pwm */
          target_pwm_[i] = convert(target_thrust_[i]);

          /* constraint */
          if(target_pwm_[i] < min_duty_) target_pwm_[i]  = min_duty_;
          else if(target_pwm_[i]  > max_duty_) target_pwm_[i]  = max_duty_;
        }

      /* for ros */
      pwms_msg_.motor_value[i] = (target_pwm_[i] * 2000);
    }

  /* publish gimbal for gazebo */
#ifdef SIMULATION
  if(start_control_flag_)
    {
      if(gimbal_dof_)
        {
          sensor_msgs::JointState gimbal_control_msg;
          gimbal_control_msg.header.stamp = ros::Time::now();
          for(int i = 0; i < motor_number_ / (gimbal_dof_ + 1); i++){
            gimbal_control_msg.position.push_back(target_gimbal_angles_[i]);
          }
          if(HAL_GetTick() - gimbal_control_pub_last_time_ > GIMBAL_CONTROL_PUB_INTERVAL)
            {
              gimbal_control_pub_last_time_ = HAL_GetTick();
              gimbal_control_pub_.publish(gimbal_control_msg);
            }
        }
    }

#else

  if(start_control_flag_)
    {
      if(gimbal_dof_ && gimbal_indices_received_)
        {
          if(HAL_GetTick() - gimbal_control_pub_last_time_ > GIMBAL_CONTROL_SET_INTERVAL)
            {
              for(int i = 0; i < gimbal_number_; i++)
                {
                  int16_t target_servo_position = (float)ANGLE_SIGN * (float)SERVO_RESOLUTION / (float)M_PI * target_gimbal_angles_[i] + ZERO_POINT_OFFSET + 0.5;
                  Spine::setTargetServoPosition(gimbal_indices_[i], target_servo_position);
                }
              gimbal_control_pub_last_time_ = HAL_GetTick();
            }
        }
    }

#endif

}
