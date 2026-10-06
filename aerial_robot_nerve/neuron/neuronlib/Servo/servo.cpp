/*
 * servo.cpp
 *
 *  Created on: 2016/10/28
 *      Author: anzai
 *  Maintainer: bakui chou(2016/11/20)
 */

#include "servo.h"

#ifdef STM32G4
namespace
{
constexpr uint8_t SERVO_TARGET_HEADER_SIZE = 4;
constexpr uint8_t SERVO_TARGET_RECORD_SIZE = 12;

void writeInt16(uint8_t* data, int16_t value)
{
  const uint16_t raw = static_cast<uint16_t>(value);
  data[0] = raw & 0xFF;
  data[1] = (raw >> 8) & 0xFF;
}

void writeUint16(uint8_t* data, uint16_t value)
{
  data[0] = value & 0xFF;
  data[1] = (value >> 8) & 0xFF;
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
}
#endif

void Servo::init(UART_HandleTypeDef* huart, I2C_HandleTypeDef* hi2c,
                 osMutexId* mutex = NULL)
{
  servo_handler_.init(huart, hi2c, mutex);
  connect_ = false;
  config_request_busy_ = false;
  config_request_pending_ = false;
  config_response_required_ = false;
}

bool Servo::queueConfigRequest(uint8_t command, const int32_t* data,
                               uint8_t data_length, bool response_required,
                               uint8_t sequence)
{
  if (data_length > config_data_.size() ||
      (data_length != 0 && data == nullptr)) return false;

  taskENTER_CRITICAL();
  if (config_request_busy_) {
    taskEXIT_CRITICAL();
    return false;
  }
  config_request_busy_ = true;
  config_request_pending_ = true;
  config_response_required_ = response_required;
  config_sequence_ = sequence;
  config_command_ = command;
  config_data_length_ = data_length;
  for (uint8_t i = 0; i < data_length; ++i) config_data_[i] = data[i];
  taskEXIT_CRITICAL();
  return true;
}

void Servo::sendConfigResponse(uint8_t sequence, uint8_t command,
                               uint8_t result, const int32_t* request_data,
                               uint8_t data_length)
{
#ifdef STM32G4
  uint8_t response[CAN::CANFD_SERVO_CONFIG_DLC] = {};
  response[0] = CAN::CANFD_PROTOCOL_VERSION;
  response[1] = sequence;
  response[2] = command;
  response[3] = result;
  response[4] = 0xFF;
  writeUint16(response + 10, servo_handler_.getServoFeedbackFrequency());
  writeInt32(response + 12,
             servo_handler_.getTargetWatchdogTimeoutMs());

  const bool board_command =
    command == CAN::SERVO_CONFIG_SET_TTL_RS485_MIXED ||
    command == CAN::SERVO_CONFIG_SET_FEEDBACK_FREQUENCY ||
    command == CAN::SERVO_CONFIG_SET_TARGET_WATCHDOG;
  const ServoData* selected = nullptr;
  if (!board_command && data_length != 0 && request_data != nullptr) {
    for (uint8_t i = 0; i < servo_handler_.getServoNum(); ++i) {
      const ServoData& candidate = servo_handler_.getServo()[i];
      if (candidate.id_ == static_cast<uint8_t>(request_data[0])) {
        selected = &candidate;
        break;
      }
    }
  }
  if (selected != nullptr) {
    response[4] = selected->id_;
    response[5] = selected->mode_;
    response[6] = selected->bus_type_;
    response[7] = (selected->configured_ ? 1U : 0U) |
                  (selected->send_data_flag_ ? 1U << 1 : 0U) |
                  (selected->external_encoder_flag_ ? 1U << 2 : 0U);
    response[8] = selected->operating_mode_;
    writeUint16(response + 16, selected->p_gain_);
    writeUint16(response + 18, selected->i_gain_);
    writeUint16(response + 20, selected->d_gain_);
    writeUint16(response + 22, selected->profile_velocity_);
    writeUint16(response + 24, selected->current_limit_);
    writeUint16(response + 26, selected->joint_resolution_);
    writeUint16(response + 28, selected->servo_resolution_);
    writeInt16(response + 30, selected->mit_kp_integer_);
    writeInt16(response + 32, selected->mit_kp_decimal_);
    writeInt16(response + 34, selected->mit_kd_integer_);
    writeInt16(response + 36, selected->mit_kd_decimal_);
    writeUint16(response + 40, selected->mit_current_limit_);
  }
  response[38] = servo_handler_.getTTLRS485Mixed() ? 1 : 0;
  sendMessage(CAN::MESSAGEID_RECEIVE_SERVO_CONFIG, m_slave_id,
              CAN::CANFD_SERVO_CONFIG_DLC, response, 1);
#else
  (void)sequence;
  (void)command;
  (void)result;
  (void)request_data;
  (void)data_length;
#endif
}

void Servo::processConfigRequest()
{
  uint8_t sequence = 0;
  uint8_t command = 0;
  uint8_t data_length = 0;
  bool response_required = false;
  std::array<int32_t, 6> data = {};

#ifdef STM32G4
  taskENTER_CRITICAL();
  const bool pending = config_request_pending_;
  sequence = config_sequence_;
  command = config_command_;
  data_length = config_data_length_;
  response_required = config_response_required_;
  data = config_data_;
  taskEXIT_CRITICAL();

  if (pending)
  {
    if (servo_handler_.queueConfigRequest(command, data.data(), data_length))
    {
      taskENTER_CRITICAL();
      config_request_pending_ = false;
      taskEXIT_CRITICAL();
    }
    else
    {
      if (response_required)
      {
        sendConfigResponse(sequence, command, CAN::SERVO_CONFIG_RESULT_BUSY, nullptr, 0);
      }
      taskENTER_CRITICAL();
      config_request_pending_ = false;
      config_request_busy_ = false;
      taskEXIT_CRITICAL();
      return;
    }
  }

  uint8_t result = 0;
  if (!servo_handler_.takeConfigResult(result))
    return;
  if (response_required)
  {
    sendConfigResponse(sequence, command, result, data.data(), data_length);
  }
  taskENTER_CRITICAL();
  config_request_busy_ = false;
  taskEXIT_CRITICAL();
#else
  taskENTER_CRITICAL();
  if (!config_request_pending_) {
    taskEXIT_CRITICAL();
    return;
  }
  config_request_pending_ = false;
  sequence = config_sequence_;
  command = config_command_;
  data_length = config_data_length_;
  response_required = config_response_required_;
  data = config_data_;
  taskEXIT_CRITICAL();

  const uint8_t result = servo_handler_.configure(
    command, data.data(), data_length);
  if (response_required) {
    sendConfigResponse(sequence, command, result, data.data(), data_length);
  }
  taskENTER_CRITICAL();
  config_request_busy_ = false;
  taskEXIT_CRITICAL();
#endif
}

bool Servo::update()
{
  processConfigRequest();
  servo_handler_.update();
#ifdef STM32G4
  processConfigRequest();
  return servo_handler_.takeFeedbackCycleEvent();
#else
  return false;
#endif
}

void Servo::sendData()
{
#ifdef STM32G4
  // The Dynamixel PVC/control cycle stays at 200 Hz. DynamixelSerial only
  // raises feedback_fresh_ every N PVC cycles (servo_feedback_frequency), so
  // the cadence does not depend on which event woke canTxTask.
  if (!servo_handler_.takeFeedbackFresh())
    return;

  const unsigned int servo_count = std::min<unsigned int>(
    CAN::CANFD_MAX_SERVO_NUM, servo_handler_.getServoNum());
  const uint8_t sequence = state_sequence_++;
  bool success = true;

  for (unsigned int i = 0; i < servo_count; ++i) {
    const ServoData& servo = servo_handler_.getServo()[i];
    if (!servo.send_data_flag_) continue;

    uint8_t data[CAN::CANFD_SERVO_STATE_DLC] = {};
    uint8_t flags = (servo.moving_ ? CAN::SERVO_STATE_MOVING : 0) |
      ((!connect_ || servo.force_servo_off_)
         ? CAN::SERVO_STATE_FORCE_OFF : 0) |
      (servo.send_goal_position_
         ? CAN::SERVO_STATE_POSITION_SYNC_REQUEST : 0) |
      (servo.torque_enable_ ? CAN::SERVO_STATE_TORQUE_ENABLE : 0) |
      (servo.watchdog_timed_out_
         ? CAN::SERVO_STATE_WATCHDOG_TIMEOUT : 0) |
      (servo.configured_ ? CAN::SERVO_STATE_CONFIGURED : 0);

    const bool pvc_fresh = servo.pvc_update_count_ != last_sent_pvc_update_count_[i];
    if (pvc_fresh)
    {
      flags |= CAN::SERVO_STATE_PVC_FRESH;
    }

    data[0] = CAN::CANFD_PROTOCOL_VERSION;
    data[1] = sequence;
    data[2] = last_target_sequence_;
    data[3] = flags;
    writeInt32(data + 4, servo.send_goal_position_
                             ? servo.getGoalPosition()
                             : servo.getPresentPosition());
    writeInt32(data + 8, servo.getPresentVelocity());
    writeInt16(data + 12, servo.present_current_);
    data[14] = servo.present_temp_;
    data[15] = servo.hardware_error_status_;

    if (sendMessage(CAN::MESSAGEID_SEND_SERVO_LIST[i], m_slave_id, CAN::CANFD_SERVO_STATE_DLC, data, 1))
    {
      __atomic_add_fetch(&feedback_count_, 1, __ATOMIC_RELAXED);
      if (pvc_fresh)
      {
        last_sent_pvc_update_count_[i] = servo.pvc_update_count_;
      }
    }
    else
    {
      success = false;
    }
  }
  if (!success)
    servo_handler_.restoreFeedbackFresh();
#else
  for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
    const ServoData& s = servo_handler_.getServo()[i];

    if (!s.send_data_flag_) {
      continue;
    }

    int32_t position = s.getPresentPosition();
    if (s.send_goal_position_) {
      position = s.getGoalPosition();
    }

    CANServoData data(static_cast<int16_t>(position),
                      s.present_temp_,
                      s.moving_,
                      !connect_ || s.force_servo_off_,
                      s.send_goal_position_,
                      s.present_current_,
                      s.hardware_error_status_);
    sendMessage(CAN::MESSAGEID_SEND_SERVO_LIST[i], m_slave_id, 8,
                reinterpret_cast<uint8_t*>(&data), 1);
  }
#endif
}

void Servo::receiveDataCallback(uint8_t message_id, uint32_t DLC,
                                uint8_t* data)
{
  if (!connect_) return;

#ifdef STM32G4
  if (message_id == CAN::MESSAGEID_RECEIVE_SERVO_CONFIG) {
    if (DLC != CAN::CANFD_SERVO_CONFIG_DLC ||
        data[0] != CAN::CANFD_PROTOCOL_VERSION ||
        data[3] > 6) return;

    int32_t request_data[6] = {};
    for (uint8_t i = 0; i < data[3]; ++i) {
      request_data[i] = readInt32(data + 4 + i * 4);
    }
    if (!queueConfigRequest(
          data[2], request_data, data[3], true, data[1])) {
      sendConfigResponse(data[1], data[2], CAN::SERVO_CONFIG_RESULT_BUSY,
                         nullptr, 0);
    }
    return;
  }

  if (message_id != CAN::MESSAGEID_RECEIVE_SERVO_TARGET ||
      DLC != CAN::CANFD_SERVO_TARGET_DLC ||
      data[0] != CAN::CANFD_PROTOCOL_VERSION ||
      data[3] > CAN::CANFD_MAX_SERVO_NUM ||
      data[3] > servo_handler_.getServoNum()) {
    return;
  }

  __atomic_add_fetch(&target_count_, 1, __ATOMIC_RELAXED);
  last_target_sequence_ = data[1];
  const uint8_t valid_mask = data[2];
  std::array<DynamixelSerial::ServoTarget, MAX_SERVO_NUM> targets{};
  for (uint8_t i = 0; i < data[3]; ++i) {
    if ((valid_mask & (1U << i)) == 0) continue;

    const uint8_t* record =
      data + SERVO_TARGET_HEADER_SIZE + i * SERVO_TARGET_RECORD_SIZE;
    const uint8_t flags = record[0];
    DynamixelSerial::ServoTarget& target = targets[i];
    target.valid = true;
    target.torque_enable = (flags & CAN::SERVO_TARGET_TORQUE_ENABLE) != 0;
    target.position_valid = (flags & CAN::SERVO_TARGET_POSITION_VALID) != 0;
    target.position_ack = (flags & CAN::SERVO_TARGET_POSITION_ACK) != 0;
    target.velocity_valid = (flags & CAN::SERVO_TARGET_VELOCITY_VALID) != 0;
    target.current_valid = (flags & CAN::SERVO_TARGET_CURRENT_VALID) != 0;
    target.position = readInt32(record + 2);
    target.velocity = readInt32(record + 6);
    target.current = readInt16(record + 10);
  }
  servo_handler_.submitTargetFrame(targets, HAL_GetTick());
#else
  switch (message_id) {
  case CAN::MESSAGEID_RECEIVE_SERVO_ANGLE:
    {
      for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
        ServoData& s = servo_handler_.getServo()[i];
        //convert int15 to int32
        int sign = data[i * 2 + 1] & 0x40;
        int32_t goal_pos =
          ((data[i * 2 + 1] & 0x7F) << 8) | data[i * 2];
        if (sign != 0) {
          goal_pos = 0xFFFF8000 | goal_pos;
        }

        if (!s.torque_enable_) continue;

        if (s.send_goal_position_) {
          bool ack = ((data[i * 2 + 1] >> 7) & 0x01);
          if (!ack) continue;
          s.send_goal_position_ = false;
        }

        s.setGoalPosition(goal_pos);
      }
      break;
    }
  case CAN::MESSAGEID_RECEIVE_SERVO_CURRENT:
    {
      for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
        ServoData& s = servo_handler_.getServo()[i];
        //convert int15 to int32
        int sign = data[i * 2 + 1] & 0x40;
        int16_t goal_current = static_cast<int16_t>(
          (((data[i * 2 + 1] & 0x7F) << 8) & 0xFF00) |
          (data[i * 2] & 0xFF));
        if (sign != 0) {
          goal_current = 0x8000 | goal_current;
        }
        s.setGoalCurrent(goal_current);
        bool torque_enable =
          ((data[i * 2 + 1] >> 7) & 0x01) != 0;
        servo_handler_.setTorque(i, torque_enable);
      }
      break;
    }
  }
#endif
}

uint32_t Servo::takeTargetCount()
{
  return __atomic_exchange_n(&target_count_, 0, __ATOMIC_ACQ_REL);
}

uint32_t Servo::takeFeedbackCount()
{
  return __atomic_exchange_n(&feedback_count_, 0, __ATOMIC_ACQ_REL);
}
