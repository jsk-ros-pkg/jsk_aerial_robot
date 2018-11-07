/*
******************************************************************************
* File Name          : flight_control.h
* Description        : flight control(attitude, position)  interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __EXTRA_SERVO_H
#define __EXTRA_SERVO_H

#include "stm32f7xx_hal.h"
#include "config.h"
#include <ros.h>

/* ros */
#include <spinal/ServoControlCmd.h>
#include <spinal/UavInfo.h>

#define MAX_PWM  54000
#define MAX_DUTY 20000.0f //ms
#define NEUTUAL_DUTY 1500.0f //ms


class ExtraServo
{
public:
  ~ExtraServo(){}

  ExtraServo(): extra_servo_control_sub_("/extra_servo_cmd", &ExtraServo::servoControlCallback, this )
  {
  }

  void  init(TIM_HandleTypeDef* htim, ros::NodeHandle* nh)
  {
    nh_ = nh;

    /* servo control sub */
    nh_->subscribe< ros::Subscriber<spinal::ServoControlCmd, ExtraServo> >(extra_servo_control_sub_);

    pwm_htim_ = htim;
    HAL_TIM_PWM_Start(pwm_htim_,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(pwm_htim_,TIM_CHANNEL_2);

    /* netual: TODO, use flash memory */
    pwm_htim_->Instance->CCR1 = (uint32_t)(NEUTUAL_DUTY / MAX_DUTY * MAX_PWM);
    pwm_htim_->Instance->CCR2 = (uint32_t)(NEUTUAL_DUTY / MAX_DUTY * MAX_PWM);
    pwm_htim_->Instance->CCR3 = (uint32_t)(NEUTUAL_DUTY / MAX_DUTY * MAX_PWM);
  }

private:

  ros::NodeHandle* nh_;
  ros::Subscriber<spinal::ServoControlCmd, ExtraServo> extra_servo_control_sub_;

  TIM_HandleTypeDef* pwm_htim_;

  void servoControlCallback(const spinal::ServoControlCmd& cmd_msg)
  {
    for(int i = 0; i < cmd_msg.index_length; i++)
      {
        switch(cmd_msg.index[i])
          {
          case 0:
            {
              pwm_htim_->Instance->CCR1 = (uint32_t)(cmd_msg.angles[i] / MAX_DUTY * MAX_PWM);
              break;
            }
          case 1:
            {
              pwm_htim_->Instance->CCR2 = (uint32_t)(cmd_msg.angles[i] / MAX_DUTY * MAX_PWM);
              break;
            }
          case 2:
            {
              pwm_htim_->Instance->CCR3 = (uint32_t)(cmd_msg.angles[i] / MAX_DUTY * MAX_PWM);
              break;
            }
          default:
            {
              nh_->logerror("wrong extra servo id");
            }
          }
      }
  }

};

#endif
