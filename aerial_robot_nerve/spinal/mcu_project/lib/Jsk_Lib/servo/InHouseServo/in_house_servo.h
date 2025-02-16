/**
  ******************************************************************************
  * File Name          : in_house_servo.hpp
  * Description        : interface for in-house servo
 Includes ------------------------------------------------------------------*/

#ifndef __IN_HOUSE_SERVO_H
#define __IN_HOUSE_SERVO_H

#include <config.h>
#include <ros.h>
#include <CAN/can_device_manager.h>
#include <spinal/ServoExtendedState.h>
#include <spinal/ServoControlCmd.h>
#include <spinal/ServoTorqueCmd.h>

#include "math/AP_Math.h"

/* RTOS */
#include "cmsis_os.h"

#ifdef STM32F7
using CAN_GeranlHandleTypeDef = CAN_HandleTypeDef;
#endif

#ifdef STM32H7
using CAN_GeranlHandleTypeDef = FDCAN_HandleTypeDef;
#endif

namespace InHouse_Servo
{
  class Servo : public CANDirectDevice
  {
  public:
    Servo();
    ~Servo(){}

    void init(CAN_GeranlHandleTypeDef* hcan, osMailQId* handle, ros::NodeHandle* nh, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

    void update(void);
    void servoControlCallback(const spinal::ServoControlCmd& control_msg);
    void servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg);

    void sendData() override;
    void receiveDataCallback(uint32_t identifier, uint32_t dlc, uint8_t* data) override;

  private:
    ros::NodeHandle* nh_;
    ros::Publisher servo_state_pub_;
    ros::Subscriber<spinal::ServoControlCmd, Servo> servo_cmd_sub_;
    ros::Subscriber<spinal::ServoTorqueCmd, Servo> servo_torque_ctrl_sub_;
    spinal::ServoExtendedState servo_state_msg_;

    uint32_t servo_last_pub_time_;
    uint32_t servo_last_ctrl_time_;

    //motor states
    int16_t current_pos_i_;
    float current_pos_f_;
    int16_t current_rpm_i_;
    int16_t current_power_i_;
    float current_power_f_;
    uint8_t current_state_;

    uint8_t inst_data_[8];

    
    // temporary constants to communicate with in-house servo
    static const uint8_t servo_id_ = 1;
    static const uint32_t canTxId = 0x01;
    static constexpr int32_t SERVO_PUB_INTERVAL = 10; // [ms]
    static constexpr int32_t SERVO_CTRL_INTERVAL = 20; // [ms]
  };
}
#endif
