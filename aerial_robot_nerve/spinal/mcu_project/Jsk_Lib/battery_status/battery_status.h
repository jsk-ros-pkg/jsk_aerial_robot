/*
******************************************************************************
* File Name          : electromagnet.h
* Description        : electronic meganet device to pick up object in task3
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __BATTERY_STATUS_H
#define __BATTERY_STATUS_H

#include "stm32f7xx_hal.h"
#include "flashmemory/flashmemory.h"
#include "adc.h"

/* ros */
#include <ros.h>
#include <std_msgs/Float32.h>
#include <config.h>

#define VOLTAGE_CECK_INTERVAL 20 //500 //500ms
#define ROS_PUB_INTERVAL 100 //20[ms]; 500[ms]
/* https://www.sparkfun.com/datasheets/Sensors/DC%20Voltage%20and%20Current%20Sense%20PCB%20Spec%20Sheet.pdf */

class BatteryStatus
{
public:
  BatteryStatus():  voltage_status_pub_("/battery_voltage_status", &voltage_status_msg_),
                    adc_scale_sub_("/set_adc_scale", &BatteryStatus::adcScaleCallback, this)
  {
  }

  ~BatteryStatus(){}


  void init(ADC_HandleTypeDef *hadc, ros::NodeHandle* nh)
  {
    nh_ = nh;
    nh_->advertise(voltage_status_pub_);
    nh_->subscribe<ros::Subscriber<std_msgs::Float32, BatteryStatus> >(adc_scale_sub_);
    hadc_ = hadc;

    voltage_ = -1;

    FlashMemory::addValue(&adc_scale_, sizeof(float));

    HAL_ADC_Start(hadc_);
  }

  void adcScaleCallback(const std_msgs::Float32& cmd_msg)
  {
    adc_scale_ = cmd_msg.data;
    FlashMemory::erase();
    FlashMemory::write();
    nh_->loginfo("overwrite adc sacle");
  }

  void update()
  {
    static uint32_t last_time = HAL_GetTick();
    static uint32_t ros_pub_last_time = HAL_GetTick();

    if(HAL_GetTick() - last_time > VOLTAGE_CECK_INTERVAL)
      {
        last_time = HAL_GetTick();
        if(HAL_ADC_PollForConversion(hadc_,10) == HAL_OK)
          adc_value_ = HAL_ADC_GetValue(hadc_);
        HAL_ADC_Start(hadc_);

        float voltage =  adc_scale_ * adc_value_;
        if(voltage_ < 0) voltage_ = voltage;

        /* filtering */
        if(voltage  > 0) voltage_ = 0.99 * voltage_  + 0.01 * voltage;

      }

    if(HAL_GetTick() - ros_pub_last_time > ROS_PUB_INTERVAL)
      {
        ros_pub_last_time = HAL_GetTick();
        voltage_status_msg_.data = voltage_;
        voltage_status_pub_.publish(&voltage_status_msg_);
      }
  }

  ros::Publisher voltage_status_pub_;
  ros::Subscriber<std_msgs::Float32, BatteryStatus> adc_scale_sub_;
  std_msgs::Float32 voltage_status_msg_;

  inline float getVoltage() {return voltage_;}

private:
  ros::NodeHandle* nh_;
  ADC_HandleTypeDef *hadc_;

  float adc_value_;
  float adc_scale_;
  float voltage_;
};

#endif
