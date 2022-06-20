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

#include <config.h>
#include "flashmemory/flashmemory.h"

/* ros */
#include <ros.h>
#include <std_msgs/Float32.h>

#define VOLTAGE_CHECK_INTERVAL 20 // ms
#define ROS_PUB_INTERVAL 100 //ms

/* https://www.sparkfun.com/datasheets/Sensors/DC%20Voltage%20and%20Current%20Sense%20PCB%20Spec%20Sheet.pdf */

class BatteryStatus
{
public:
  BatteryStatus():  voltage_status_pub_("battery_voltage_status", &voltage_status_msg_),
                    adc_scale_sub_("set_adc_scale", &BatteryStatus::adcScaleCallback, this)
  {
  }

  ~BatteryStatus(){}


  void init(ADC_HandleTypeDef *hadc, ros::NodeHandle* nh)
  {
    nh_ = nh;
    nh_->advertise(voltage_status_pub_);
    nh_->subscribe(adc_scale_sub_);
    hadc_ = hadc;

    voltage_ = -1;
    ros_pub_last_time_ = HAL_GetTick();

    FlashMemory::addValue(&adc_scale_, sizeof(float));

    HAL_ADC_Start(hadc_);
  }

  void adcScaleCallback(const std_msgs::Float32& cmd_msg)
  {
    adc_scale_ = cmd_msg.data;
    voltage_ = -1; // reset
    FlashMemory::erase();
    FlashMemory::write();
    nh_->loginfo("overwrite adc sacle");
  }

  void update()
  {
    if(HAL_ADC_PollForConversion(hadc_,10) == HAL_OK)
      adc_value_ = HAL_ADC_GetValue(hadc_);

    HAL_ADC_Start(hadc_);

    float voltage =  adc_scale_ * adc_value_;
    if(voltage_ < 0) voltage_ = voltage;

    /* filtering */
    if(voltage  > 0) voltage_ = 0.99 * voltage_  + 0.01 * voltage;

    if(HAL_GetTick() - ros_pub_last_time_ > ROS_PUB_INTERVAL)
      {
        ros_pub_last_time_ = HAL_GetTick();
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

  uint32_t ros_pub_last_time_;
};

#endif
