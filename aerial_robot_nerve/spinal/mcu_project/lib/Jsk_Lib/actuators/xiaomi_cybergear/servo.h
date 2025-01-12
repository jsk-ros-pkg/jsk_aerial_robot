#ifndef __XIAOMI_CYBERGEAR_H
#define __XIAOMI_CYBERGEAR_H

#include <config.h>
#include <ros.h>
#include <CAN/can_device_manager.h>
#include <spinal/XiaomiCybergearCmd.h>
#include <spinal/XiaomiCybergearState.h>

#include "math/AP_Math.h"

/* RTOS */
#include "cmsis_os.h"

#ifdef STM32F7
using CAN_GeranlHandleTypeDef = CAN_HandleTypeDef;
#endif

#ifdef STM32H7
using CAN_GeranlHandleTypeDef = FDCAN_HandleTypeDef;
#endif

namespace Xiaomi_Cybergear
{
  struct MotorComand
  {
    float target_position;
    float target_speed;
    float target_torque;
    float target_kp;
    float target_kd;
  };

  struct MotorStatus
  {
    uint8_t motor_id;          //!< motor id
    float position;            //!< encoder position (-4pi to 4pi)
    float velocity;            //!< motor velocity (-30rad/s to 30rad/s)
    float effort;              //!< motor effort (-12Nm - 12Nm)
    float temperature;         //!< temperature
    uint16_t raw_position;     //!< raw position (for sync data)
    uint16_t raw_velocity;     //!< raw velocity (for sync data)
    uint16_t raw_effort;       //!< raw effort (for sync data)
    uint16_t raw_temperature;  //!< raw temperature (for sync data)
  };

  class Servo : public CANDirectDevice
  {
  public:
    Servo();
    ~Servo(){}

    void init(CAN_GeranlHandleTypeDef* hcan, osMailQId* handle, ros::NodeHandle* nh, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

    void update(void);
    void servoControlCallback(const spinal::XiaomiCybergearCmd& control_msg);

    void sendData() override;
    void receiveDataCallback(uint32_t identifier, uint32_t dlc, uint8_t* data) override;

  private:
    ros::NodeHandle* nh_;
    ros::Publisher servo_state_pub_;
    ros::Subscriber<spinal::XiaomiCybergearCmd, Servo> servo_cmd_sub_;
    spinal::XiaomiCybergearState servo_state_msg_;

    bool activated_;
    uint32_t servo_last_pub_time_;
    uint32_t servo_last_ctrl_time_;

    uint8_t master_can_id_ = 0x00;
    uint8_t motor_can_id_ = 0x7F;

    MotorComand motor_command_;
    MotorStatus motor_status_;

    static constexpr int32_t SERVO_PUB_INTERVAL = 10; // [ms]
    static constexpr int32_t SERVO_CTRL_INTERVAL = 2; // [ms]

    // cybergear driver
    uint32_t makeIdentifier(uint8_t can_id, uint8_t cmd_id, uint16_t option);
    void init_motor(uint8_t run_mode);
    void enable_motor();
    void reset_motor();
    void set_run_mode(uint8_t run_mode);
    void motor_control(float position, float speed, float torque, float kp, float kd);
    void set_limit_speed(float speed);
    void set_limit_current(float current);
    void set_current_kp(float kp);
    void set_current_ki(float ki);
    void set_current_filter_gain(float gain);
    void set_limit_torque(float torque);
    void set_position_kp(float kp);
    void set_velocity_kp(float kp);
    void set_velocity_ki(float ki);
    void get_mech_position();
    void get_mech_velocity();
    void get_vbus();
    void get_rotation();
    void dump_motor_param();
    void set_position_ref(float position);
    void set_speed_ref(float speed);
    void set_current_ref(float current);
    void set_mech_position_to_zero();
    void change_motor_can_id(uint8_t can_id);
    void read_ram_data(uint16_t index);
    uint8_t get_run_mode() const;
    uint8_t get_motor_id() const;
    bool process_packet();
    bool update_motor_status(unsigned long id, const uint8_t * data, unsigned long len);
    MotorStatus get_motor_status() const { return motor_status_; }
    // MotorParameter get_motor_param() const { return motor_param_; }
    // unsigned long send_count() const { return send_count_; }
    void write_float_data(uint8_t can_id, uint16_t addr, float value, float min, float max);
    int float_to_uint(float x, float x_min, float x_max, int bits);
    float uint_to_float(uint16_t x, float x_min, float x_max);
    void send_command(uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t * data);
    bool receive_motor_data(MotorStatus & mot);
    void process_motor_packet(const uint8_t * data);
    void process_read_parameter_packet(const uint8_t * data, unsigned long len);
    void print_can_packet(uint32_t id, const uint8_t * data, uint8_t len);
  };
}

#endif
