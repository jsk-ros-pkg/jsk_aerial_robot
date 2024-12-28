/**
  ******************************************************************************
  * File Name          : servo.h
  * Description        : interface for DJI M2006 with C610
 Includes ------------------------------------------------------------------*/

#ifndef __DJI_M2006_H
#define __DJI_M2006_H

#include <config.h>
#include <ros.h>
#include <CAN/can_device_manager.h>
#include <spinal/ServoExtendedState.h>
#include <spinal/ServoControlCmd.h>

#include "math/AP_Math.h"

/* RTOS */
#include "cmsis_os.h"

#ifdef STM32F7
using CAN_GeranlHandleTypeDef = CAN_HandleTypeDef;
#endif

#ifdef STM32H7
using CAN_GeranlHandleTypeDef = FDCAN_HandleTypeDef;
#endif

namespace DJI_M2006
{
  class Servo : public CANDirectDevice
  {
  public:
    Servo();
    ~Servo(){}

    void init(CAN_GeranlHandleTypeDef* hcan, osMailQId* handle, ros::NodeHandle* nh, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

    void update(void);
    void servoControlCallback(const spinal::ServoControlCmd& control_msg);

    void sendData() override;
    void receiveDataCallback(uint32_t identifier, uint32_t dlc, uint8_t* data) override;

  private:
    ros::NodeHandle* nh_;
    ros::Publisher servo_state_pub_;
    ros::Subscriber<spinal::ServoControlCmd, Servo> servo_cmd_sub_;
    spinal::ServoExtendedState servo_state_msg_;

    uint32_t last_connected_time_;
    uint32_t servo_last_pub_time_;
    uint32_t servo_last_ctrl_time_;

    uint8_t initialized_mechanical_angle_;
    int32_t rotations_;
    int32_t last_pos_measurement_;
    int32_t counts_;
    int32_t rpm_;
    int32_t m_current_;

    float output_ang_;
    float output_vel_;
    float current_;

    int16_t goal_current_;

    // Constants specific to the C610 + M2006 setup.
    static const uint32_t canTxId = 0x200;
    static const int32_t kCountsPerRev = 8192;
    static constexpr float kReduction = 36.0F;
    static constexpr float kCountsPerRad = kCountsPerRev * kReduction / (2 * M_PI);
    static constexpr float kRPMPerRadS = kReduction * 60 / (2.0F * M_PI);
    static constexpr float kMilliAmpPerAmp = 1000.0F;

    static constexpr float kResistance = 0.100;
    static constexpr float kVoltageConstant = 100.0;

    static constexpr int32_t SERVO_PUB_INTERVAL = 10; // [ms]
    static constexpr int32_t SERVO_CTRL_INTERVAL = 2; // [ms]
  };
}
#endif
