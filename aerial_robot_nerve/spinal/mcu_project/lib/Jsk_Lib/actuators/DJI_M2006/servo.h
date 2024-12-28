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
#include <spinal/ServoExtendedCmd.h>

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
    void servoControlCallback(const spinal::ServoExtendedCmd& control_msg);

    void sendData() override;
    void receiveDataCallback(uint32_t identifier, uint32_t dlc, uint8_t* data) override;

  private:
    ros::NodeHandle* nh_;
    ros::Publisher servo_state_pub_;
    ros::Subscriber<spinal::ServoExtendedCmd, Servo> servo_cmd_sub_;
    spinal::ServoExtendedState servo_state_msg_;

    uint32_t last_connected_time_;
    uint32_t servo_last_pub_time_;
    uint32_t servo_last_ctrl_time_;

    uint8_t initialized_mechanical_angle_;
    int32_t rotations_;
    int32_t last_pos_measurement_;
    int32_t counts_;
    int32_t rpm_;
    int32_t m_curr_;

    float output_pos_;
    float output_vel_;
    float curr_;

    float filter_vel_;
    float filter_vel_p_;

    int8_t control_mode_;
    float goal_value_;
    float goal_pos_;
    float goal_vel_;
    float goal_curr_;


    // PID velocity control
    float v_k_p_;
    float v_k_i_;
    float v_i_term_;

    // PID position control
    float p_k_p_;
    float p_k_i_;
    float p_k_d_;
    float p_i_term_;


    // Constants specific to the C610 + M2006 setup.
    static const uint32_t canTxId = 0x200;
    static const int32_t kCountsPerRev = 8192;
    static constexpr float kReduction = 36.0F;
    static constexpr float kCountsPerRad = kCountsPerRev * kReduction / (2 * M_PI);
    static constexpr float kRPMPerRadS = kReduction * 60 / (2.0F * M_PI);
    static constexpr float kMilliAmpPerAmp = 1000.0F;

    static constexpr float kResistance = 0.100;
    static constexpr float kVoltageConstant = 100.0;

    static const uint8_t POS_MODE = 0;
    static const uint8_t VEL_MODE = 1;
    static const uint8_t CUR_MODE = 2;

    static constexpr float MAX_CURRENT = 10.0; // [A]
    static constexpr int32_t SERVO_PUB_INTERVAL = 10; // [ms]
    static constexpr int32_t SERVO_CTRL_INTERVAL = 2; // [ms]

    static constexpr uint8_t GYRO_LPF_FACTOR = 20;

    void calcPosPid();
    void calcVelPid();
  };
}
#endif
