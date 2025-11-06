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

#include "config.h"
#include "flashmemory/flashmemory.h"
#include <ros.h>

/* ros */
#include <spinal/ServoControlCmd.h>
#include <spinal/ServoTorqueCmd.h>

#define MAX_PWM  54000
#define MAX_DUTY 20000.0f //conversion to [ms]

#define ANGLE_RANGE 180.0f

class ExtraServo
{
public:
  ~ExtraServo(){}

  ExtraServo():
    extra_servo_control_sub_("extra_servo_cmd", &ExtraServo::servoControlCallback, this ),
    extra_servo_torque_control_sub_("extra_servo_torque_enable", &ExtraServo::servoTorqueControlCallback, this ),
    extra_servo_init_duty_sub_("extra_servo_init_cmd", &ExtraServo::servoInitDutyCallback, this )
  {
  }

  void  init(TIM_HandleTypeDef* t1,TIM_HandleTypeDef* t2,ros::NodeHandle* nh)
  {
    nh_ = nh;

    /* servo control sub */
    nh_->subscribe(extra_servo_control_sub_);
    nh_->subscribe(extra_servo_torque_control_sub_);
    nh_->subscribe(extra_servo_init_duty_sub_);

    pwm_htim1_ = t1;
    pwm_htim2_ = t2;

    if(pwm_htim1_)
      {
	HAL_TIM_PWM_Start(pwm_htim1_,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(pwm_htim1_,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(pwm_htim1_,TIM_CHANNEL_3);
      }

    if(pwm_htim2_)
      {
	HAL_TIM_Base_Stop(pwm_htim2_);
	HAL_TIM_Base_DeInit(pwm_htim2_);

	pwm_htim2_->Init.Prescaler = 3;
	pwm_htim2_->Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
	pwm_htim2_->Init.Period = 50000;

	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	while(HAL_TIM_Base_Init(pwm_htim2_) != HAL_OK);
	while(HAL_TIM_PWM_Init(pwm_htim2_) != HAL_OK);

	if (pwm_htim2_->hdma[TIM_DMA_ID_UPDATE] != NULL) {
	  HAL_DMA_DeInit(pwm_htim2_->hdma[TIM_DMA_ID_UPDATE]);
	  pwm_htim2_->hdma[TIM_DMA_ID_UPDATE] = NULL;
	}

	HAL_TIM_Base_Start(pwm_htim2_);

	HAL_TIM_PWM_Start(pwm_htim2_, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(pwm_htim2_, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(pwm_htim2_, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(pwm_htim2_, TIM_CHANNEL_4);
      }

    for (int i = 0; i < 6; i++) {
		FlashMemory::addValue(&(init_duty_[i]), sizeof(float));
	}

    FlashMemory::read();
    if(pwm_htim1_)
      {
	pwm_htim1_->Instance->CCR1 = (uint32_t)(init_duty_[0] / MAX_DUTY * MAX_PWM);
	pwm_htim1_->Instance->CCR2 = (uint32_t)(init_duty_[1] / MAX_DUTY * MAX_PWM);
	pwm_htim1_->Instance->CCR3 = (uint32_t)(init_duty_[2] / MAX_DUTY * MAX_PWM);
      }
    if(pwm_htim2_)
      {
	pwm_htim2_->Instance->CCR1 = (uint32_t)(init_angle_[0] / ANGLE_RANGE * pwm_htim2_->Init.Period);
	pwm_htim2_->Instance->CCR2 = (uint32_t)(init_angle_[1] / ANGLE_RANGE * pwm_htim2_->Init.Period);
	pwm_htim2_->Instance->CCR3 = (uint32_t)(init_angle_[2] / ANGLE_RANGE * pwm_htim2_->Init.Period);
	pwm_htim2_->Instance->CCR4 = (uint32_t)(init_angle_[3] / ANGLE_RANGE * pwm_htim2_->Init.Period);
      }
  }

private:

  ros::NodeHandle* nh_;
  ros::Subscriber<spinal::ServoControlCmd, ExtraServo> extra_servo_control_sub_;
  ros::Subscriber<spinal::ServoTorqueCmd, ExtraServo> extra_servo_torque_control_sub_;
  ros::Subscriber<spinal::ServoControlCmd, ExtraServo> extra_servo_init_duty_sub_;

  float init_duty_[6] = {}; //[ms]
  float init_angle_[4] = {90, 90, 90, 90}; //[degree]
  TIM_HandleTypeDef* pwm_htim1_;
  TIM_HandleTypeDef* pwm_htim2_;


  void servoControlCallback(const spinal::ServoControlCmd& cmd_msg)
  {
    for(int i = 0; i < cmd_msg.index_length; i++)
      {
        switch(cmd_msg.index[i])
          {
          case 0:
            {
	      if(pwm_htim1_)
		pwm_htim1_->Instance->CCR1 = (uint32_t)(cmd_msg.angles[i] / ANGLE_RANGE * pwm_htim1_->Init.Period);
              break;
            }
          case 1:
            {
	      if(pwm_htim1_)
		pwm_htim1_->Instance->CCR2 = (uint32_t)(cmd_msg.angles[i] / ANGLE_RANGE * pwm_htim1_->Init.Period);
              break;
            }
          case 2:
            {
	      if(pwm_htim1_)
		pwm_htim1_->Instance->CCR3 = (uint32_t)(cmd_msg.angles[i] / ANGLE_RANGE * pwm_htim1_->Init.Period);
              break;
            }
	  case 3:
            {
	      if(pwm_htim1_)
		pwm_htim1_->Instance->CCR4 = (uint32_t)(cmd_msg.angles[i] / ANGLE_RANGE * pwm_htim1_->Init.Period);
              break;
            }
          case 4:
            {
	      if(pwm_htim2_)
		pwm_htim2_->Instance->CCR1 = (uint32_t)(cmd_msg.angles[i] / ANGLE_RANGE * pwm_htim2_->Init.Period);
              break;
            }
          case 5:
            {
	      if(pwm_htim2_)
		pwm_htim2_->Instance->CCR2 = (uint32_t)(cmd_msg.angles[i] / ANGLE_RANGE * pwm_htim2_->Init.Period);
              break;
            }
          case 6:
            {
	      if(pwm_htim2_)
		pwm_htim2_->Instance->CCR3 = (uint32_t)(cmd_msg.angles[i] / ANGLE_RANGE * pwm_htim2_->Init.Period);
              break;
            }
	   case 7:
            {
	      if(pwm_htim2_)
		pwm_htim2_->Instance->CCR4 = (uint32_t)(cmd_msg.angles[i] / ANGLE_RANGE * pwm_htim2_->Init.Period);
              break;
            }
          default:
            {
              nh_->logerror("wrong extra servo id");
            }
          }
      }
  }

  void servoTorqueControlCallback(const spinal::ServoTorqueCmd& cmd_msg)
  {
    for(int i = 0; i < cmd_msg.index_length; i++)
      {
        /* TODO: please implement the flag to enable the servo */
        if(cmd_msg.torque_enable[i])
          {
            nh_->logwarn("the flag to enable the (extra) servo is not implemented right now");
            continue;
          }

        switch(cmd_msg.index[i])
          {
          case 0:
            {
	      if(pwm_htim1_)
		pwm_htim1_->Instance->CCR1 = 0;
              break;
            }
          case 1:
            {
	      if(pwm_htim1_)
		pwm_htim1_->Instance->CCR2 = 0;
              break;
            }
          case 2:
            {
	      if(pwm_htim1_)
		pwm_htim1_->Instance->CCR3 = 0;
              break;
            }
	  case 3:
            {
	      if(pwm_htim1_)
		pwm_htim1_->Instance->CCR4 = 0;
              break;
            }
          case 4:
            {
	      if(pwm_htim2_)
		pwm_htim2_->Instance->CCR1 = 0;
              break;
            }
          case 5:
            {
	      if(pwm_htim2_)
		pwm_htim2_->Instance->CCR2 = 0;
              break;
            }
          case 6:
            {
	      if(pwm_htim2_)
		pwm_htim2_->Instance->CCR3 = 0;
              break;
            }
	  case 7:
            {
	      if(pwm_htim2_)
		pwm_htim2_->Instance->CCR4 = 0;
              break;
            }
          default:
            {
              nh_->logerror("wrong extra servo id");
            }
          }
      }
  }

  void servoInitDutyCallback(const spinal::ServoControlCmd& cmd_msg)
  {
    for(int i = 0; i < cmd_msg.index_length; i++)
    {
  	  init_duty_[cmd_msg.index[i]] = (float)(cmd_msg.angles[i]);
  	}
    FlashMemory::erase();
    FlashMemory::write();
    nh_->loginfo("servo duty initialize");
  }
};

#endif
