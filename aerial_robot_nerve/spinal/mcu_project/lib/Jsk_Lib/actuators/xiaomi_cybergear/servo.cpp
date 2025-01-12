#include "servo.h"
#include "cybergear_driver_defs.hh"

using namespace Xiaomi_Cybergear;

Servo::Servo():
  servo_state_pub_("servo/extended_states", &servo_state_msg_),
  servo_cmd_sub_("servo/extended_cmd", &Servo::servoControlCallback, this)
{
  servo_last_pub_time_ = 0;
  servo_last_ctrl_time_ = 0;

  motor_command_.target_position = 0.0;
  motor_command_.target_speed = 0.0;
  motor_command_.target_torque = 0.0;
  motor_command_.target_kp = 0.0;
  motor_command_.target_kd = 0.0;

  activated_ = false;
}

void Servo::init(CAN_GeranlHandleTypeDef* hcan, osMailQId* handle, ros::NodeHandle* nh, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  /* CAN */
  CANDeviceManager::init(hcan, GPIOx, GPIO_Pin);
  CANDeviceManager::useRTOS(handle);
  CANDeviceManager::addDirectDevice(this);

  /* ros */
  nh_ = nh;
  nh_->advertise(servo_state_pub_);
  nh_->subscribe(servo_cmd_sub_);

  CANDeviceManager::CAN_START();
}

void Servo::sendData()
{
  if(!activated_)
    {
      reset_motor();
      set_mech_position_to_zero();
      enable_motor();
      set_run_mode(CMD_CONTROL);
      activated_ = true;
    }
  else
    motor_control(motor_command_.target_position,
                  motor_command_.target_speed,
                  motor_command_.target_torque,
                  motor_command_.target_kp,
                  motor_command_.target_kd);
}

void Servo::send_command(uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t * data)
{
  uint32_t id = makeIdentifier(can_id, cmd_id, option);
  sendMessage(id, 8, data, 0, true); // send data with extended id (last parameter)
}

void Servo::receiveDataCallback(uint32_t identifier, uint32_t dlc, uint8_t* data)
{
  process_motor_packet(data);
}

void Servo::update(void)
{
  CANDeviceManager::tick(1);
  uint32_t now_time = HAL_GetTick();

  /* control */
  if( now_time - servo_last_ctrl_time_ >= SERVO_CTRL_INTERVAL)
    {
      sendData();

      servo_last_ctrl_time_ = now_time;
    }

  /* ros publish */
  if( now_time - servo_last_pub_time_ >= SERVO_PUB_INTERVAL)
    {
      servo_state_msg_.position = motor_status_.position;
      servo_state_msg_.velocity = motor_status_.velocity;
      servo_state_msg_.effort = motor_status_.effort;
      servo_state_msg_.temperature = motor_status_.temperature;

      servo_state_pub_.publish(&servo_state_msg_);

      servo_last_pub_time_ = now_time;
    }
}

void Servo::servoControlCallback(const spinal::XiaomiCybergearCmd& control_msg)
{
  motor_command_.target_position = control_msg.position;
  motor_command_.target_speed = control_msg.speed;
  motor_command_.target_torque = control_msg.torque;
  motor_command_.target_kp = control_msg.kp;
  motor_command_.target_kd = control_msg.kd;
}

