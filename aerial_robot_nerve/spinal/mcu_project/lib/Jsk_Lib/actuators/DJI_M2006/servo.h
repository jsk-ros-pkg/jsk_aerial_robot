/**
  ******************************************************************************
  * File Name          : servo.h
  * Description        : interface for DJI M2006 with C610
 Includes ------------------------------------------------------------------*/

#ifndef __DJI_M2006_H
#define __DJI_M2006_H

#include <config.h>
#include <map>
#include <ros.h>
#include <CAN/can_device_manager.h>
#include <spinal/ServoExtendedStates.h>
#include <spinal/ServoExtendedCmds.h>

#include "math/AP_Math.h"

/* RTOS */
#include "cmsis_os.h"

#ifdef STM32F7
using CAN_GeranlHandleTypeDef = CAN_HandleTypeDef;
#endif

#ifdef STM32H7
using CAN_GeranlHandleTypeDef = FDCAN_HandleTypeDef;
#endif

#define SERVO_CTRL_INTERVAL 2 // [ms]

namespace DJI_M2006
{
  class Servo
  {
  public:
    Servo();
    ~Servo() {}

    uint8_t getControlMode() {return control_mode_; }
    float getGoalValue() {return goal_value_; }
    float getGoalCurrent() {return goal_curr_; }

    float getAngle() {return output_pos_; }
    float getVelocity() {return output_vel_; }
    float getFilteredVelocity() {return filter_vel_; }
    float getCurrent() {return curr_; }


    void setControlMode(int mode) { control_mode_ = mode; }
    void setGoalValue(float cmd) { goal_value_ = cmd; }

    void update(uint16_t counts, int16_t rpm, int16_t m_curr);
    void control(void);

    static constexpr uint8_t GYRO_LPF_FACTOR = 20;
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

  private:

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

    void calcPosPid();
    void calcVelPid();
  };

  class Interface : public CANDirectDevice
  {
  public:
    Interface();
    ~Interface(){}

    void init(CAN_GeranlHandleTypeDef* hcan, osMailQId* handle, ros::NodeHandle* nh, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

    void servoControlCallback(const spinal::ServoExtendedCmds& msg);

    void update();

    void sendData() override;
    void receiveDataCallback(uint32_t identifier, uint32_t dlc, uint8_t* data) override;

    Servo& getServo(int id) {return servo_list_.at(id); }


  private:
    ros::NodeHandle* nh_;
    ros::Publisher servo_state_pub_;
    ros::Subscriber<spinal::ServoExtendedCmds, Interface> servo_cmd_sub_;
    spinal::ServoExtendedStates servo_states_msg_;


    uint32_t last_connected_time_;
    uint32_t servo_last_pub_time_;
    uint32_t servo_last_ctrl_time_;

    // Constants specific to the C610 + M2006 setup.
    static const uint32_t canTxId1 = 0x200;
    static const uint32_t canTxId2 = 0x1FF;

    // Constants specific to ros
    static constexpr int32_t SERVO_PUB_INTERVAL = 10; // [ms]

    std::map<int, Servo> servo_list_;

    int init_cnt_;
  };
}
#endif
