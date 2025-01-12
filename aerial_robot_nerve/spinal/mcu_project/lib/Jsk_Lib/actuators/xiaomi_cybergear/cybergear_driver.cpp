#include "servo.h"
#include "cybergear_driver_defs.hh"

using namespace Xiaomi_Cybergear;

void Servo::init_motor(uint8_t run_mode)
{
  reset_motor();
  set_run_mode(run_mode);
}

void Servo::enable_motor()
{
  uint8_t data[8] = {0x00};
  send_command(motor_can_id_, CMD_ENABLE, master_can_id_, 8, data);
}

void Servo::reset_motor()
{
  uint8_t data[8] = {0x00};
  send_command(motor_can_id_, CMD_RESET, master_can_id_, 8, data);
}

void Servo::set_run_mode(uint8_t run_mode)
{
  // set class variable
  uint8_t data[8] = {0x00};
  data[0] = ADDR_RUN_MODE & 0x00FF;
  data[1] = ADDR_RUN_MODE >> 8;
  data[4] = run_mode;
  send_command(motor_can_id_, CMD_RAM_WRITE, master_can_id_, 8, data);
}

void Servo::motor_control(float position, float speed, float torque, float kp, float kd)
{
  uint8_t data[8] = {0x00};
  data[0] = float_to_uint(position, P_MIN, P_MAX, 16) >> 8;
  data[1] = float_to_uint(position, P_MIN, P_MAX, 16);
  data[2] = float_to_uint(speed, V_MIN, V_MAX, 16) >> 8;
  data[3] = float_to_uint(speed, V_MIN, V_MAX, 16);
  data[4] = float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8;
  data[5] = float_to_uint(kp, KP_MIN, KP_MAX, 16);
  data[6] = float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8;
  data[7] = float_to_uint(kd, KD_MIN, KD_MAX, 16);

  uint16_t data_torque = float_to_uint(torque, T_MIN, T_MAX, 16);
  send_command(motor_can_id_, CMD_POSITION, data_torque, 8, data);
}

void Servo::set_mech_position_to_zero()
{
  uint8_t data[8] = {0x00};
  data[0] = 0x01;
  send_command(motor_can_id_, CMD_SET_MECH_POSITION_TO_ZERO, master_can_id_, 8, data);
}

void Servo::read_ram_data(uint16_t index)
{
  uint8_t data[8] = {0x00};
  memcpy(&data[0], &index, 2);
  send_command(motor_can_id_, CMD_RAM_READ, master_can_id_, 8, data);
}

void Servo::write_float_data(uint8_t can_id, uint16_t addr, float value, float min, float max)
{
  uint8_t data[8] = {0x00};
  data[0] = addr & 0x00FF;
  data[1] = addr >> 8;

  float val = (max < value) ? max : value;
  val = (min > value) ? min : value;
  memcpy(&data[4], &value, 4);
  send_command(can_id, CMD_RAM_WRITE, master_can_id_, 8, data);
}

int Servo::float_to_uint(float x, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  if (x > x_max)
    x = x_max;
  else if (x < x_min)
    x = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float Servo::uint_to_float(uint16_t x, float x_min, float x_max)
{
  uint16_t type_max = 0xFFFF;
  float span = x_max - x_min;
  return (float)x / type_max * span + x_min;
}

uint32_t Servo::makeIdentifier(uint8_t can_id, uint8_t cmd_id, uint16_t option)
{
  uint32_t id = cmd_id << 24 | option << 8 | can_id;
  return id;
}

void Servo::process_motor_packet(const uint8_t * data)
{
  motor_status_.raw_position = data[1] | data[0] << 8;
  motor_status_.raw_velocity = data[3] | data[2] << 8;
  motor_status_.raw_effort = data[5] | data[4] << 8;
  motor_status_.raw_temperature = data[7] | data[6] << 8;

  // convert motor data
  motor_status_.motor_id = motor_can_id_;
  motor_status_.position = uint_to_float(motor_status_.raw_position, P_MIN, P_MAX);
  motor_status_.velocity = uint_to_float(motor_status_.raw_velocity, V_MIN, V_MAX);
  motor_status_.effort = uint_to_float(motor_status_.raw_effort, T_MIN, T_MAX);
  motor_status_.temperature = motor_status_.raw_temperature / 10.0;
}

