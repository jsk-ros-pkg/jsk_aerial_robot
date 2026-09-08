/*
 * can_servo.cpp
 *
 *  Created on: 2016/11/01
 *      Author: anzai
 */

#include "can_servo.h"

#ifdef STM32H7
namespace
{
constexpr uint8_t SERVO_TARGET_HEADER_SIZE = 4;
constexpr uint8_t SERVO_TARGET_RECORD_SIZE = 12;
constexpr uint8_t SERVO_CONTROL_MODE_CBP = 6;

void writeInt16(uint8_t* data, int16_t value)
{
  const uint16_t raw = static_cast<uint16_t>(value);
  data[0] = raw & 0xFF;
  data[1] = (raw >> 8) & 0xFF;
}

void writeInt32(uint8_t* data, int32_t value)
{
  const uint32_t raw = static_cast<uint32_t>(value);
  data[0] = raw & 0xFF;
  data[1] = (raw >> 8) & 0xFF;
  data[2] = (raw >> 16) & 0xFF;
  data[3] = (raw >> 24) & 0xFF;
}

int16_t readInt16(const uint8_t* data)
{
  return static_cast<int16_t>(
    static_cast<uint16_t>(data[0]) |
    (static_cast<uint16_t>(data[1]) << 8));
}

int32_t readInt32(const uint8_t* data)
{
  return static_cast<int32_t>(
    static_cast<uint32_t>(data[0]) |
    (static_cast<uint32_t>(data[1]) << 8) |
    (static_cast<uint32_t>(data[2]) << 16) |
    (static_cast<uint32_t>(data[3]) << 24));
}

uint16_t readUint16(const uint8_t* data)
{
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}
}
#endif

void Servo::setTorqueEnable(bool torque_enable)
{
  // A safety fault may reject an enable request, but an explicit disable must
  // always be accepted.
  if (torque_enable && force_servo_off_) return;

  torque_command_ = torque_enable;
}

uint8_t CANServo::requestConfig(uint8_t command, const int32_t* request_data,
                                uint8_t data_length, uint32_t timeout_ms)
{
#ifdef STM32H7
  if (request_data == nullptr || data_length > 5 || config_pending_) {
    return data_length > 5 ? CAN::SERVO_CONFIG_RESULT_INVALID
                           : CAN::SERVO_CONFIG_RESULT_BUSY;
  }

  uint8_t data[CAN::CANFD_SERVO_CONFIG_DLC] = {};
  const uint8_t sequence = config_sequence_++;
  data[0] = CAN::CANFD_PROTOCOL_VERSION;
  data[1] = sequence;
  data[2] = command;
  data[3] = data_length;
  for (uint8_t i = 0; i < data_length; ++i) {
    writeInt32(data + 4 + i * 4, request_data[i]);
  }

  pending_config_sequence_ = sequence;
  pending_config_command_ = command;
  config_result_ = CAN::SERVO_CONFIG_RESULT_BUSY;
  config_pending_ = true;
  if (!sendMessage(CAN::MESSAGEID_RECEIVE_SERVO_CONFIG, m_slave_id,
                   CAN::CANFD_SERVO_CONFIG_DLC, data, 1)) {
    config_pending_ = false;
    return CAN::SERVO_CONFIG_RESULT_BUSY;
  }

  const uint32_t start = HAL_GetTick();
  while (config_pending_ && HAL_GetTick() - start < timeout_ms) {
    osDelay(1);
  }
  if (config_pending_) {
    config_pending_ = false;
    return CAN::SERVO_CONFIG_RESULT_BUSY;
  }
  return config_result_;
#else
  (void)command;
  (void)request_data;
  (void)data_length;
  (void)timeout_ms;
  return CAN::SERVO_CONFIG_RESULT_INVALID;
#endif
}

uint8_t CANServo::requestConfigByIndex(
  uint8_t servo_index, uint8_t command,
  const int32_t* data, uint8_t data_length, uint32_t timeout_ms)
{
  if (servo_index >= servo_.size() || data_length >= 5 ||
      (data_length > 0 && data == nullptr)) {
    return CAN::SERVO_CONFIG_RESULT_INVALID;
  }

  int32_t request_data[5] = {};
  request_data[0] = servo_[servo_index].id_;
  for (uint8_t i = 0; i < data_length; ++i) {
    request_data[i + 1] = data[i];
  }
  return requestConfig(
    command, request_data, data_length + 1, timeout_ms);
}

void CANServo::sendData()
{
#ifdef STM32H7
  uint8_t data[CAN::CANFD_SERVO_TARGET_DLC] = {};
  const uint8_t servo_count = std::min(
    static_cast<size_t>(CAN::CANFD_MAX_SERVO_NUM), servo_.size());

  data[0] = CAN::CANFD_PROTOCOL_VERSION;
  data[1] = target_sequence_++;
  data[2] = (1U << servo_count) - 1U;
  data[3] = servo_count;

  for (uint8_t i = 0; i < servo_count; ++i) {
    Servo& servo = servo_[i];
    uint8_t* record =
      data + SERVO_TARGET_HEADER_SIZE + i * SERVO_TARGET_RECORD_SIZE;
    record[0] = CAN::SERVO_TARGET_POSITION_VALID |
                CAN::SERVO_TARGET_VELOCITY_VALID |
                CAN::SERVO_TARGET_CURRENT_VALID |
                (servo.torque_command_ && !servo.force_servo_off_
                   ? CAN::SERVO_TARGET_TORQUE_ENABLE : 0) |
                (servo.goal_position_ack_
                   ? CAN::SERVO_TARGET_POSITION_ACK : 0);
    writeInt32(record + 2, servo.goal_position_);
    writeInt32(record + 6, servo.goal_velocity_);
    writeInt16(record + 10, servo.goal_current_);
    servo.goal_position_ack_ = false;
  }

  sendMessage(CAN::MESSAGEID_RECEIVE_SERVO_TARGET, m_slave_id,
              CAN::CANFD_SERVO_TARGET_DLC, data, 0);
#else
  uint16_t target_angle[4];
  for (unsigned int i = 0 ; i < servo_.size(); i++) {
    target_angle[i] = ((servo_[i].goal_position_ack_ ? 1 : 0) << 15) | (servo_[i].goal_position_ & 0x7FFF);
    servo_[i].goal_position_ack_ = false;
  }
  sendMessage(CAN::MESSAGEID_RECEIVE_SERVO_ANGLE, m_slave_id, servo_.size() * 2, reinterpret_cast<uint8_t*>(target_angle), 0);

  uint16_t target_current[4];
  for (unsigned int i = 0 ; i < servo_.size(); i++) {
    target_current[i] = (((servo_[i].torque_command_ &&
                           !servo_[i].force_servo_off_) ? 1 : 0) << 15) |
                        (servo_[i].goal_current_ & 0x7FFF);
  }
  sendMessage(CAN::MESSAGEID_RECEIVE_SERVO_CURRENT, m_slave_id, servo_.size() * 2, reinterpret_cast<uint8_t*>(target_current), 0);
#endif
}

void CANServo::receiveDataCallback(uint8_t slave_id, uint8_t message_id,
                                   uint32_t DLC, uint8_t* data)
{
  (void)slave_id;
#ifdef STM32H7
  if (message_id == CAN::MESSAGEID_RECEIVE_SERVO_CONFIG) {
    if (DLC != CAN::CANFD_SERVO_CONFIG_DLC ||
        data[0] != CAN::CANFD_PROTOCOL_VERSION ||
        !config_pending_ || data[1] != pending_config_sequence_ ||
        data[2] != pending_config_command_) return;

    servo_feedback_frequency_hz_ = readUint16(data + 10);
    target_watchdog_timeout_ms_ =
      static_cast<uint32_t>(readInt32(data + 12));
    dynamixel_ttl_rs485_mixed_ = data[38] != 0;
    if (data[4] != 0xFF) {
      for (Servo& servo : servo_) {
        if (servo.id_ != data[4]) continue;
        servo.mode_ = data[5];
        servo.bus_type_ = data[6];
        servo.configured_ = (data[7] & 1U) != 0;
        servo.send_data_flag_ = (data[7] & (1U << 1)) != 0;
        servo.external_encoder_flag_ = (data[7] & (1U << 2)) != 0;
        servo.operating_mode_ = data[8];
        servo.p_gain_ = readUint16(data + 16);
        servo.i_gain_ = readUint16(data + 18);
        servo.d_gain_ = readUint16(data + 20);
        servo.profile_velocity_ = readUint16(data + 22);
        servo.current_limit_ = readUint16(data + 24);
        servo.joint_resolution_ = readUint16(data + 26);
        servo.servo_resolution_ = readUint16(data + 28);
        servo.kp_integer_ = readInt16(data + 30);
        servo.kp_decimal_ = readInt16(data + 32);
        servo.kd_integer_ = readInt16(data + 34);
        servo.kd_decimal_ = readInt16(data + 36);
        if ((pending_config_command_ ==
               CAN::SERVO_CONFIG_SET_CONTROL_MODE ||
             pending_config_command_ ==
               CAN::SERVO_CONFIG_SET_CURRENT_LIMIT) &&
            servo.mode_ == SERVO_CONTROL_MODE_CBP) {
          servo.goal_current_ = static_cast<int16_t>(
            servo.current_limit_ > INT16_MAX
              ? INT16_MAX : servo.current_limit_);
        }
        break;
      }
    }
    config_result_ = data[3];
    config_pending_ = false;
    return;
  }

  if (message_id >= servo_.size() ||
      message_id >= CAN::CANFD_MAX_SERVO_NUM ||
      DLC != CAN::CANFD_SERVO_STATE_DLC ||
      data[0] != CAN::CANFD_PROTOCOL_VERSION) {
    return;
  }

  Servo& servo = servo_[message_id];
  const uint8_t flags = data[3];
  const int32_t position = readInt32(data + 4);
  if ((flags & CAN::SERVO_STATE_POSITION_SYNC_REQUEST) != 0) {
    servo.goal_position_ = position;
    servo.goal_position_ack_ = true;
  } else {
    servo.present_position_ = position;
    servo.goal_position_ack_ = false;
  }

  servo.present_velocity_ = readInt32(data + 8);
  servo.present_current_ = readInt16(data + 12);
  servo.present_temperature_ = data[14];
  servo.error_ = data[15];
  servo.moving_ = (flags & CAN::SERVO_STATE_MOVING) != 0;
  servo.force_servo_off_ = (flags & CAN::SERVO_STATE_FORCE_OFF) != 0;
  servo.torque_feedback_ =
    !servo.force_servo_off_ &&
    ((flags & CAN::SERVO_STATE_TORQUE_ENABLE) != 0);
  if (servo.force_servo_off_) servo.torque_command_ = false;
  servo.watchdog_timed_out_ =
    (flags & CAN::SERVO_STATE_WATCHDOG_TIMEOUT) != 0;
  servo.configured_ = (flags & CAN::SERVO_STATE_CONFIGURED) != 0;
#else
  if (message_id >= servo_.size() || DLC < sizeof(CANServoData)) return;
  CANServoData servo_data = *(reinterpret_cast<CANServoData*>(data));
  servo_[message_id].present_temperature_ = servo_data.temperature;
  servo_[message_id].moving_ = (servo_data.status >> 1) & 0x01;
  servo_[message_id].force_servo_off_ = ((servo_data.status & 0x01) != 0)? true: false;
  bool receive_target_position = (servo_data.status >> 2) & 0x01;
  if (receive_target_position) {
    servo_[message_id].goal_position_ = servo_data.angle;
    servo_[message_id].goal_position_ack_ = true;
  } else {
    servo_[message_id].present_position_ = servo_data.angle;
    servo_[message_id].goal_position_ack_ = false;
  }

  servo_[message_id].present_current_ = servo_data.current;
  servo_[message_id].error_ = servo_data.error;

  servo_[message_id].torque_feedback_ =
    !servo_[message_id].force_servo_off_ &&
    servo_[message_id].torque_command_;
  if (servo_[message_id].force_servo_off_)
    servo_[message_id].torque_command_ = false;
#endif
}
