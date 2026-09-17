#include "dynamixel_serial.h"
// #include "flashmemory/flashmemory.h"

namespace
{
bool appendStuffedByte(uint8_t value, uint8_t* output,
                       size_t capacity, size_t& output_size,
                       size_t window_begin = 0)
{
  if (output == nullptr || output_size >= capacity) return false;
  output[output_size++] = value;

  if (output_size >= window_begin + 3 &&
      output[output_size - 3] == 0xFF &&
      output[output_size - 2] == 0xFF &&
      output[output_size - 1] == 0xFD) {
    if (output_size >= capacity) return false;
    output[output_size++] = 0xFD;
  }
  return true;
}

class UnstuffState
{
public:
  UnstuffState() : raw_window_{0, 0, 0}, raw_count_(0) {}

  bool append(uint8_t value, uint8_t* output, size_t capacity,
              size_t& output_size)
  {
    const bool stuffing_prefix =
      raw_count_ >= 3 &&
      raw_window_[0] == 0xFF &&
      raw_window_[1] == 0xFF &&
      raw_window_[2] == 0xFD;
    if (stuffing_prefix && value != 0xFD) return false;
    const bool stuffing_byte = stuffing_prefix;

    if (raw_count_ < 3) {
      raw_window_[raw_count_++] = value;
    } else {
      raw_window_[0] = raw_window_[1];
      raw_window_[1] = raw_window_[2];
      raw_window_[2] = value;
    }

    if (stuffing_byte) return true;
    if (output == nullptr || output_size >= capacity) return false;
    output[output_size++] = value;
    return true;
  }

private:
  uint8_t raw_window_[3];
  uint8_t raw_count_;
};

  constexpr uint8_t CONFIG_SET_HOMING_OFFSET = 1;
  constexpr uint8_t CONFIG_SET_PID_GAIN = 2;
  constexpr uint8_t CONFIG_SET_PROFILE_VELOCITY = 3;
  constexpr uint8_t CONFIG_SET_SEND_DATA_FLAG = 4;
  constexpr uint8_t CONFIG_SET_CURRENT_LIMIT = 5;
  constexpr uint8_t CONFIG_SET_TTL_RS485_MIXED = 6;
  constexpr uint8_t CONFIG_SET_EXTERNAL_ENCODER_FLAG = 7;
  constexpr uint8_t CONFIG_SET_RESOLUTION_RATIO = 8;
  constexpr uint8_t CONFIG_SET_CONTROL_MODE = 9;
  constexpr uint8_t CONFIG_SET_MIT_PD = 10;
  constexpr uint8_t CONFIG_SET_BUS_TYPE = 12;
  constexpr uint8_t CONFIG_SET_FEEDBACK_FREQUENCY = 13;
  constexpr uint8_t CONFIG_SET_TARGET_WATCHDOG = 15;


  int16_t clampGoalCurrent(int64_t current, uint16_t configured_limit)
  {
    const int32_t limit =
      configured_limit > INT16_MAX ? INT16_MAX : configured_limit;
    if (current > limit) return static_cast<int16_t>(limit);
    if (current < -limit) return static_cast<int16_t>(-limit);
    return static_cast<int16_t>(current);
  }

  int64_t symmetricDivide(int64_t value, int64_t divisor)
  {
    return value >= 0
      ? (value + divisor / 2) / divisor
      : (value - divisor / 2) / divisor;
  }

  osMutexDef(DynamixelInstructionQueueMutex);
  osSemaphoreDef(DynamixelServiceCompletion);

#if STM32H7_V2
  uint8_t rx_buf_[RX_BUFFER_SIZE] __attribute__((section(".ServoRxBufferSection")));
#else
#ifdef STM32H7
  uint8_t rx_buf_[RX_BUFFER_SIZE] __attribute__((section(".GpsRxBufferSection")));
#else
  uint8_t rx_buf_[RX_BUFFER_SIZE];
#endif
#endif
}
void DynamixelSerial::init(UART_HandleTypeDef* huart, osMutexId* mutex)
{
	huart_ = huart;
  mutex_ = mutex;  // Kept for API compatibility; runtime UART ownership is task-local.
  pending_transaction_ = PendingTransaction();
  service_request_ = ServoServiceRequest();
  torque_off_pending_mask_ = 0;
  next_service_generation_ = 0;
  last_tx_status_ = HAL_OK;
  feedback_received_mask_ = 0;
  servo_feedback_frequency_hz_ = DEFAULT_SERVO_FEEDBACK_FREQUENCY_HZ;
  target_watchdog_timeout_ms_ = DEFAULT_SERVO_TARGET_WATCHDOG_MS;

	servo_num_ = 0;
	get_pos_tick_ = 0;
	get_temp_tick_ = 0;
	get_move_tick_ = 0;
	get_error_tick_ = 0;

  direct_ttl_mode_ = false;

  /* rx */
  rd_ptr_ = 0;
  memset(rx_buf_, 0, sizeof(rx_buf_));

	std::fill(servo_.begin(), servo_.end(), ServoData(255));
  servo_flash_configs_.fill(ServoFlashConfig());

  // scan to decide the comm (UART/ Half-duplex) mode
  pinReconfig();

	//initialize servo motors
  cmdReboot(DX_BROADCAST_ID);
	HAL_Delay(3000);

	setStatusReturnLevel();
	//Successfully detected servo's led will be turned on 1 seconds

	for (unsigned int i = 0; i < servo_num_; i++) {
		servo_[i].led_ = true;
	}
	cmdSyncWriteLed();

	HAL_Delay(1000);
	for (unsigned int i = 0; i < servo_num_; i++) {
		servo_[i].led_ = false;
	}
	cmdSyncWriteLed();

	for (int i = 0; i < MAX_SERVO_NUM; i++) {
		FlashMemory::addValue(&(servo_flash_configs_[i]),
		                      sizeof(ServoFlashConfig));
	}
	FlashMemory::addValue(&(ttl_rs485_mixed_), 2);

	FlashMemory::read();

	getHomingOffset(); // This operation always fails after the servo motor is ignited first
	getHomingOffset(); // So this line is necessary
	getCurrentLimit();
	getOperatingMode();
  for (uint8_t servo_index = 0; servo_index < servo_num_; ++servo_index) {
    ServoData& servo = servo_[servo_index];
    servo.bus_type_ = SERVO_BUS_UNKNOWN;
    servo.mode_ = DIRECT_SERVO_MODE_UNKNOWN;
    servo.send_data_flag_ = 1;
    servo.external_encoder_flag_ = 0;
    servo.configured_ = false;

    ServoFlashConfig* config = findServoFlashConfig(servo.id_);
    if (config != nullptr) {
      copyFlashConfigToServo(*config, servo);
      if (!validPersistentMode(config->mode)) {
        continue;
      }
      ServoResult result = applyOperatingMode(servo_index, config->mode);
      if (result == SERVO_RESULT_OK) result = applyPositionGains(servo_index);
      if (result == SERVO_RESULT_OK) result = applyProfileVelocity(servo_index);
      servo.configured_ = result == SERVO_RESULT_OK;
      if (result != SERVO_RESULT_OK) {
        servo.configured_ = false;
      }
    } else {
      (void)readOneServoRegister(INST_GET_POSITION_GAINS, servo_index);
      (void)readOneServoRegister(INST_GET_PROFILE_VELOCITY, servo_index);
    }
  }

  // initialize encoder: only can support one encoder
#ifndef SPINAL
  encoder_handler_.init(hi2c);  // Magnetic Encoder
#endif
  for (int i = 0; i < MAX_SERVO_NUM; i++)
  {
    servo_[i].hardware_error_status_ = 0;
    if (servo_[i].servo_resolution_ == 65535 || servo_[i].joint_resolution_ == 65535)
    {
#ifndef SPINAL
      if (servo_[i].external_encoder_flag_)
        servo_[i].hardware_error_status_ = 1 << RESOLUTION_RATIO_ERROR;  // 0b01000000;
#endif
      servo_[i].resolution_ratio_ = 1;
    }
    else
    {
      servo_[i].resolution_ratio_ = (float)servo_[i].servo_resolution_ / (float)servo_[i].joint_resolution_;
#ifndef SPINAL
      if (servo_[i].external_encoder_flag_)
        encoder_handler_.setOffset(servo_[i].joint_offset_);
#endif
    }
  }

  /*set angle scale values*/
  for (int i = 0; i < MAX_SERVO_NUM; i++)
  {
    servo_[i].zero_point_offset_ = 2047;
    servo_[i].angle_scale_ = 3.1416 / 2047;
  }

  enforceConfigurationSafety();
}

bool DynamixelSerial::createRTOSObjects()
{
  if (queue_mutex_ != nullptr || service_completion_ != nullptr) {
    return queue_mutex_ != nullptr && service_completion_ != nullptr;
  }

  queue_mutex_ = osMutexCreate(osMutex(DynamixelInstructionQueueMutex));
  service_completion_ =
    osSemaphoreCreate(osSemaphore(DynamixelServiceCompletion), 1);
  return queue_mutex_ != nullptr && service_completion_ != nullptr;
}

void DynamixelSerial::pinReconfig()
{
  // scan uart mode
  while(HAL_UART_DeInit(huart_) != HAL_OK);
  /*Change baud rate*/
  huart_->Init.BaudRate = 1000000;
  huart_->Init.WordLength = UART_WORDLENGTH_8B;
  huart_->Init.Parity = UART_PARITY_NONE;
  huart_->Init.Mode = UART_MODE_TX_RX;
  while(HAL_UART_Init(huart_) != HAL_OK);

  __HAL_UART_DISABLE_IT(huart_, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart_, UART_IT_ERR);
  HAL_UART_Receive_DMA(huart_, rx_buf_, RX_BUFFER_SIZE);

  HAL_Delay(1000);

  ping();

  // find more than one servo
  if (servo_num_ > 0) return;


  direct_ttl_mode_ = true;

  // scan with TTL mode
  while(HAL_UART_DeInit(huart_) != HAL_OK);
  /*Initialize as halfduplex mode*/
  while(HAL_HalfDuplex_Init(huart_) != HAL_OK);

  // change the pull mode for gpio
  GPIO_InitTypeDef GPIO_InitStruct = {0};
#if STM32H7_V2
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5);
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
#else
#ifdef STM32H7
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8);
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
#endif
#endif

  __HAL_UART_DISABLE_IT(huart_, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart_, UART_IT_ERR);
  HAL_UART_Receive_DMA(huart_, rx_buf_, RX_BUFFER_SIZE);

  HAL_Delay(1000);

  ping();

  // find more than one servo
  if (servo_num_ > 0) return;

  direct_ttl_mode_ = false;

  // change back to the no-push-pull mode for gpio
#if STM32H7_V2
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5);
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
#else
#ifdef STM32H7
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8);
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
#endif
#endif
}

void DynamixelSerial::ping()
{
  cmdPing(DX_BROADCAST_ID);
  const uint32_t deadline_ms =
    HAL_GetTick() + SERVO_BROADCAST_PING_TIMEOUT_MS;

  while (servo_num_ < MAX_SERVO_NUM &&
         !deadlineExpired(HAL_GetTick(), deadline_ms)) {
    const ServoResult result =
      readStatusPacket(INST_PING, deadline_ms);

    if (result == SERVO_RESULT_TIMEOUT ||
        result == SERVO_RESULT_UART_ERROR) {
      break;
    }
  }

  // Broadcast Ping response order is not deterministic.
  std::sort(servo_.begin(), servo_.begin() + servo_num_,
            [](const ServoData& lhs, const ServoData& rhs) {
              return lhs.id_ < rhs.id_;
            });
}


void DynamixelSerial::reboot(uint8_t servo_index)
{
	enqueueInstruction(INST_REBOOT, servoBit(servo_index));
}

void DynamixelSerial::setTorque(uint8_t servo_index)
{
  if (servo_index >= servo_num_) return;
  if (!servo_[servo_index].torque_enable_) {
    __atomic_fetch_or(&torque_off_pending_mask_, 1UL << servo_index,
                      __ATOMIC_RELEASE);
    return;
  }
  enqueueInstruction(INST_SET_TORQUE, servoBit(servo_index));
}

void DynamixelSerial::setTorqueFromPresetnPos(uint8_t servo_index)
{
  if (servo_index >= servo_num_) return;
  ServoData& s = servo_[servo_index];
  if (s.torque_enable_) s.resetGoalTargets();
  if (!s.torque_enable_) {
    __atomic_fetch_or(&torque_off_pending_mask_, 1UL << servo_index,
                      __ATOMIC_RELEASE);
    return;
  }
  enqueueInstruction(INST_SET_TORQUE, servoBit(servo_index));
}

void DynamixelSerial::setRoundOffset(uint8_t servo_index, int32_t ref_value)
{
  ServoData& s = servo_[servo_index];
  if (s.torque_enable_) return; // cannot process if torque is enable

  // workaround to update the internal offset for pulley type in the case that has round gap
  int32_t diff = ref_value - s.present_position_;
  if (diff > 2047) s.internal_offset_ += 4096;
  if (diff < -2047) s.internal_offset_ -= 4096;
}

void DynamixelSerial::setHomingOffset(uint8_t servo_index)
{
  enqueueInstruction(INST_SET_HOMING_OFFSET, servoBit(servo_index));
}

void DynamixelSerial::setPositionGains(uint8_t servo_index)
{
  enqueueInstruction(INST_SET_POSITION_GAINS, servoBit(servo_index));
}

void DynamixelSerial::setProfileVelocity(uint8_t servo_index)
{
  enqueueInstruction(INST_SET_PROFILE_VELOCITY, servoBit(servo_index));
}

void DynamixelSerial::setCurrentLimit(uint8_t servo_index)
{
  enqueueInstruction(INST_SET_CURRENT_LIMIT, servoBit(servo_index));
}

bool DynamixelSerial::setGoalTargets(uint8_t servo_index,
                                     bool has_position, int32_t position,
                                     bool has_velocity, int32_t velocity,
                                     bool has_current, int16_t current)
{
  if (!ServoBase::setGoalTargets(
        servo_index, has_position, position, has_velocity, velocity,
        has_current, current)) {
    return false;
  }

  ServoData& servo = servo_[servo_index];
  if (servo.mode_ == DIRECT_SERVO_MODE_MIT) {
    if (!has_velocity) servo.setGoalVelocity(0);
    if (!has_current) servo.setGoalCurrent(0);
  }

  bool relevant_target = false;
  switch (servo.mode_) {
  case DIRECT_SERVO_MODE_DP:
  case DIRECT_SERVO_MODE_DEP:
    relevant_target = has_position;
    break;
  case DIRECT_SERVO_MODE_CBP:
    relevant_target = has_position || has_current;
    break;
  case DIRECT_SERVO_MODE_DV:
    relevant_target = has_velocity;
    break;
  case DIRECT_SERVO_MODE_DC:
    relevant_target = has_current;
    break;
  case DIRECT_SERVO_MODE_MIT:
    relevant_target = has_position || has_velocity || has_current;
    break;
  default:
    break;
  }
  if (relevant_target) {
    servo.last_target_tick_ = HAL_GetTick();
    servo.target_received_ = true;
    servo.watchdog_timed_out_ = false;
  }
  return true;
}

bool DynamixelSerial::enqueueInstruction(uint8_t command, ServoMask servo_mask)
{
  const ServoInstruction instruction = {command, servo_mask, 0};
  return enqueueInstructionBatch(&instruction, 1);
}

bool DynamixelSerial::enqueueInstructionBatch(
  const ServoInstruction* instructions, uint8_t instruction_count)
{
  if (instructions == nullptr || instruction_count == 0) return false;

  const ServoMask invalid_mask = static_cast<ServoMask>(~allServoMask());
  for (uint8_t i = 0; i < instruction_count; ++i) {
    const ServoMask servo_mask = instructions[i].servo_mask;
    if (servo_mask == 0 || (servo_mask & invalid_mask) != 0) {
      return false;
    }
  }
  if (queue_mutex_ == nullptr ||
      osMutexWait(queue_mutex_, SERVO_QUEUE_LOCK_TIMEOUT_MS) != osOK) {
    return false;
  }

  const uint16_t available_slots = static_cast<uint16_t>(
    instruction_buffer_.capacity() - instruction_buffer_.length());
  if (available_slots < instruction_count) {
    osMutexRelease(queue_mutex_);
    return false;
  }

  bool queued = true;
  for (uint8_t i = 0; i < instruction_count; ++i) {
    queued = instruction_buffer_.push(instructions[i]) && queued;
  }
  osMutexRelease(queue_mutex_);
  return queued;
}

bool DynamixelSerial::popInstruction(ServoInstruction& instruction)
{
  if (queue_mutex_ == nullptr ||
      osMutexWait(queue_mutex_, SERVO_QUEUE_LOCK_TIMEOUT_MS) != osOK) {
    return false;
  }

  const bool popped = instruction_buffer_.pop(instruction);
  osMutexRelease(queue_mutex_);
  return popped;
}


DynamixelSerial::ServoMask DynamixelSerial::servoBit(uint8_t servo_index)
{
  return servo_index < MAX_SERVO_NUM
    ? static_cast<ServoMask>(1U << servo_index) : 0;
}

DynamixelSerial::ServoMask DynamixelSerial::allServoMask() const
{
  return servo_num_ == 0 ? 0
    : static_cast<ServoMask>((1U << servo_num_) - 1U);
}

DynamixelSerial::ServoMask DynamixelSerial::activeServoMask() const
{
  ServoMask servo_mask = 0;
  for (unsigned int i = 0; i < servo_num_; ++i) {
    // Every actively controlled servo needs fresh local feedback for fault
    // detection. MIT additionally consumes it in the local current loop.
    if (!servo_[i].send_data_flag_ && !servo_[i].first_get_pos_flag_ &&
        servo_[i].mode_ == DIRECT_SERVO_MODE_EM) {
      continue;
    }
    servo_mask |= servoBit(i);
  }
  return servo_mask;
}

uint8_t DynamixelSerial::servoCount(ServoMask servo_mask) const
{
  uint8_t count = 0;
  for (uint8_t i = 0; i < servo_num_; ++i) {
    if ((servo_mask & servoBit(i)) != 0) ++count;
  }
  return count;
}

bool DynamixelSerial::singleServoIndex(ServoMask servo_mask,
                                        uint8_t& servo_index) const
{
  const ServoMask invalid_mask = static_cast<ServoMask>(~allServoMask());
  if (servo_mask == 0 || (servo_mask & invalid_mask) != 0 ||
      (servo_mask & static_cast<ServoMask>(servo_mask - 1U)) != 0) {
    return false;
  }

  for (uint8_t i = 0; i < servo_num_; ++i) {
    if ((servo_mask & servoBit(i)) != 0) {
      servo_index = i;
      return true;
    }
  }
  return false;
}

bool DynamixelSerial::findServoIndex(uint8_t servo_id,
                                     uint8_t& servo_index) const
{
  for (uint8_t i = 0; i < servo_num_; ++i) {
    if (servo_[i].id_ == servo_id) {
      servo_index = i;
      return true;
    }
  }
  return false;
}

DynamixelSerial::ServoGroupMasks DynamixelSerial::configuredServoMasks(
  ServoMask selected_mask) const
{
  ServoGroupMasks groups;
  const ServoMask invalid_mask = static_cast<ServoMask>(~allServoMask());
  if (selected_mask == 0 || (selected_mask & invalid_mask) != 0) {
    return groups;
  }

  for (uint8_t i = 0; i < servo_num_; ++i) {
    const ServoMask bit = servoBit(i);
    if ((selected_mask & bit) == 0) continue;
    if (servo_[i].bus_type_ == SERVO_BUS_RS485) {
      groups.rs485_mask |= bit;
    } else if (servo_[i].bus_type_ == SERVO_BUS_TTL) {
      groups.ttl_mask |= bit;
    } else {
      return groups;
    }
  }
  groups.valid =
    static_cast<ServoMask>(groups.rs485_mask | groups.ttl_mask) ==
    selected_mask;
  return groups;
}

bool DynamixelSerial::validLogicalMode(uint8_t mode)
{
  return validPersistentMode(mode) || mode == DIRECT_SERVO_MODE_EM;
}

bool DynamixelSerial::validPersistentMode(uint8_t mode)
{
  return mode == DIRECT_SERVO_MODE_DP ||
         mode == DIRECT_SERVO_MODE_DEP ||
         mode == DIRECT_SERVO_MODE_CBP ||
         mode == DIRECT_SERVO_MODE_DV ||
         mode == DIRECT_SERVO_MODE_DC ||
         mode == DIRECT_SERVO_MODE_MIT;
}

uint8_t DynamixelSerial::expectedOperatingMode(uint8_t mode)
{
  switch (mode) {
  case DIRECT_SERVO_MODE_DP:
    return DYNAMIXEL_POSITION_CONTROL_MODE;
  case DIRECT_SERVO_MODE_DEP:
    return DYNAMIXEL_EXTENDED_POSITION_CONTROL_MODE;
  case DIRECT_SERVO_MODE_CBP:
    return DYNAMIXEL_CURRENT_BASED_POSITION_CONTROL_MODE;
  case DIRECT_SERVO_MODE_DV:
    return DYNAMIXEL_VELOCITY_CONTROL_MODE;
  case DIRECT_SERVO_MODE_DC:
  case DIRECT_SERVO_MODE_MIT:
    return DYNAMIXEL_CURRENT_CONTROL_MODE;
  default:
    return DYNAMIXEL_OPERATING_MODE_UNKNOWN;
  }
}

bool DynamixelSerial::validFlashConfig(const ServoFlashConfig& config)
{
  return config.id <= 252 &&
         (config.bus_type == SERVO_BUS_RS485 ||
          config.bus_type == SERVO_BUS_TTL) &&
         (validPersistentMode(config.mode) ||
          config.mode == DIRECT_SERVO_MODE_UNKNOWN) &&
         (config.flags & static_cast<uint8_t>(~SERVO_FLASH_FLAG_MASK)) == 0;
}

bool DynamixelSerial::busMapValid() const
{
  if (servo_num_ == 0) return false;
  for (uint8_t i = 0; i < servo_num_; ++i) {
    if (!servo_[i].configured()) return false;
  }
  return true;
}

bool DynamixelSerial::canSendControl(ServoMask servo_mask) const
{
  const ServoMask invalid_mask = static_cast<ServoMask>(~allServoMask());
  if (servo_mask == 0 || (servo_mask & invalid_mask) != 0 ||
      !busMapValid()) {
    return false;
  }
  for (uint8_t i = 0; i < servo_num_; ++i) {
    if ((servo_mask & servoBit(i)) == 0) continue;
    const ServoData& servo = servo_[i];
    if (servo.mode_ == DIRECT_SERVO_MODE_EM ||
        !validLogicalMode(servo.mode_) ||
        servo.operating_mode_ != expectedOperatingMode(servo.mode_)) {
      return false;
    }
  }
  return true;
}

void DynamixelSerial::enforceConfigurationSafety()
{
  if (busMapValid()) return;
  uint32_t torque_mask = 0;
  for (uint8_t i = 0; i < servo_num_; ++i) {
    if (!servo_[i].torque_enable_) continue;
    servo_[i].torque_enable_ = false;
    torque_mask |= 1UL << i;
  }
  if (torque_mask != 0) {
    __atomic_fetch_or(&torque_off_pending_mask_, torque_mask,
                      __ATOMIC_RELEASE);
  }
}

void DynamixelSerial::updateTargetWatchdogs(uint32_t now_ms)
{
  if (target_watchdog_timeout_ms_ == 0) return;

  for (uint8_t i = 0; i < servo_num_; ++i) {
    ServoData& servo = servo_[i];
    const bool watched_mode =
      servo.mode_ == DIRECT_SERVO_MODE_DV ||
      servo.mode_ == DIRECT_SERVO_MODE_DC ||
      servo.mode_ == DIRECT_SERVO_MODE_MIT;
    if (!watched_mode || !servo.target_received_ ||
        !deadlineExpired(now_ms,
                         servo.last_target_tick_ +
                         target_watchdog_timeout_ms_)) {
      continue;
    }
    if (!servo.watchdog_timed_out_) {
      servo.watchdog_timed_out_ = true;
      servo.goal_velocity_ = 0;
      servo.goal_current_ = 0;
    }
  }
}

bool DynamixelSerial::validateServiceRequest(uint8_t command, const int32_t* data,
                                             uint8_t data_length) const
{
  if (data == nullptr || data_length > SERVO_SERVICE_DATA_CAPACITY) {
    return false;
  }

  uint8_t expected_length = 0;
  switch (command) {
  case CONFIG_SET_TTL_RS485_MIXED:
  case CONFIG_SET_FEEDBACK_FREQUENCY:
  case CONFIG_SET_TARGET_WATCHDOG:
    expected_length = 1;
    break;
  case CONFIG_SET_HOMING_OFFSET:
  case CONFIG_SET_PROFILE_VELOCITY:
  case CONFIG_SET_SEND_DATA_FLAG:
  case CONFIG_SET_CURRENT_LIMIT:
  case CONFIG_SET_EXTERNAL_ENCODER_FLAG:
  case CONFIG_SET_CONTROL_MODE:
  case CONFIG_SET_BUS_TYPE:
    expected_length = 2;
    break;
  case CONFIG_SET_RESOLUTION_RATIO:
    expected_length = 3;
    break;
  case CONFIG_SET_PID_GAIN:
    expected_length = 4;
    break;
  case CONFIG_SET_MIT_PD:
    expected_length = 5;
    break;
  default:
    return false;
  }

  if (data_length != expected_length) {
    return false;
  }

  if (command == CONFIG_SET_TTL_RS485_MIXED) {
    return data[0] == 0 || data[0] == 1;
  }
  if (command == CONFIG_SET_FEEDBACK_FREQUENCY) {
    return data[0] > 0 && data[0] <= 200;
  }
  if (command == CONFIG_SET_TARGET_WATCHDOG) {
    return data[0] >= 0 && data[0] <= 60000;
  }

  uint8_t servo_index = 0;
  if (data[0] < 0 || data[0] > UINT8_MAX ||
      !findServoIndex(static_cast<uint8_t>(data[0]), servo_index)) {
    return false;
  }
  if (command == CONFIG_SET_CONTROL_MODE) {
    return data[1] >= 0 && data[1] <= UINT8_MAX &&
           validPersistentMode(static_cast<uint8_t>(data[1]));
  }
  if (command == CONFIG_SET_BUS_TYPE) {
    return data[1] == SERVO_BUS_RS485 || data[1] == SERVO_BUS_TTL;
  }
  if (command == CONFIG_SET_MIT_PD) {
    return data[1] >= 0 && data[1] <= INT16_MAX &&
           data[2] >= 0 && data[2] < MIT_GAIN_DECIMAL_SCALE &&
           data[3] >= 0 && data[3] <= INT16_MAX &&
           data[4] >= 0 && data[4] < MIT_GAIN_DECIMAL_SCALE;
  }
  switch (command) {
  case CONFIG_SET_HOMING_OFFSET:
    return data[1] >= -1044479 && data[1] <= 1044479;
  case CONFIG_SET_PID_GAIN:
    return data[1] >= 0 && data[1] <= UINT16_MAX &&
           data[2] >= 0 && data[2] <= UINT16_MAX &&
           data[3] >= 0 && data[3] <= UINT16_MAX;
  case CONFIG_SET_PROFILE_VELOCITY:
  case CONFIG_SET_CURRENT_LIMIT:
    return data[1] >= 0 && data[1] <= UINT16_MAX;
  case CONFIG_SET_SEND_DATA_FLAG:
  case CONFIG_SET_EXTERNAL_ENCODER_FLAG:
    return data[1] == 0 || data[1] == 1;
  case CONFIG_SET_RESOLUTION_RATIO:
    return data[1] > 0 && data[1] < UINT16_MAX &&
           data[2] > 0 && data[2] < UINT16_MAX;
  default:
    return false;
  }
}

ServoResult DynamixelSerial::requestConfig(uint8_t command, const int32_t* data,
                                           uint8_t data_length, uint32_t timeout_ms)
{
  if (!validateServiceRequest(command, data, data_length)) {
    return SERVO_RESULT_INVALID_ARGUMENT;
  }
  if (queue_mutex_ == nullptr || service_completion_ == nullptr) {
    return SERVO_RESULT_BUSY;
  }

  const uint32_t wait_timeout_ms = timeout_ms == 0 ? SERVO_SERVICE_TIMEOUT_MS : timeout_ms;
  if (osMutexWait(queue_mutex_, SERVO_QUEUE_LOCK_TIMEOUT_MS) != osOK) {
    return SERVO_RESULT_BUSY;
  }
  if (service_request_.active) {
    osMutexRelease(queue_mutex_);
    return SERVO_RESULT_BUSY;
  }

  while (osSemaphoreWait(service_completion_, 0) == osOK) {}

  ++next_service_generation_;
  if (next_service_generation_ == 0) {
    ++next_service_generation_;
  }

  service_request_ = ServoServiceRequest();
  service_request_.command = command;
  service_request_.data_length = data_length;
  for (uint8_t i = 0; i < data_length; ++i) {
    service_request_.data[i] = data[i];
  }
  service_request_.generation = next_service_generation_;
  service_request_.deadline_ms = HAL_GetTick() + wait_timeout_ms;
  service_request_.result = SERVO_RESULT_BUSY;
  service_request_.active = true;
  service_request_.callback_waiting = true;

  const uint32_t generation = service_request_.generation;
  const bool board_command =
    command == CONFIG_SET_TTL_RS485_MIXED ||
    command == CONFIG_SET_FEEDBACK_FREQUENCY ||
    command == CONFIG_SET_TARGET_WATCHDOG;
  uint8_t servo_index = 0;
  if (!board_command &&
      !findServoIndex(static_cast<uint8_t>(data[0]), servo_index)) {
    service_request_.active = false;
    service_request_.callback_waiting = false;
    osMutexRelease(queue_mutex_);
    return SERVO_RESULT_INVALID_ARGUMENT;
  }
  const ServoMask service_mask =
    board_command ? allServoMask() : servoBit(servo_index);
  const bool queued =
    instruction_buffer_.push({INST_SERVICE_REQUEST, service_mask, 0});
  if (!queued) {
    service_request_.active = false;
    service_request_.callback_waiting = false;
    osMutexRelease(queue_mutex_);
    return SERVO_RESULT_QUEUE_FULL;
  }
  osMutexRelease(queue_mutex_);

  const int32_t wait_result = osSemaphoreWait(service_completion_, wait_timeout_ms);
  if (osMutexWait(queue_mutex_, SERVO_QUEUE_LOCK_TIMEOUT_MS) != osOK) {
    return SERVO_RESULT_BUSY;
  }

  ServoResult result = SERVO_RESULT_TIMEOUT;
  if (service_request_.generation == generation &&
      wait_result == osOK && !service_request_.active) {
    result = service_request_.result;
  } else if (service_request_.active) {
    service_request_.callback_waiting = false;
  }
  osMutexRelease(queue_mutex_);
  return result;
}

void DynamixelSerial::completeServiceRequest(uint32_t generation, ServoResult result)
{
  if (queue_mutex_ == nullptr ||
      osMutexWait(queue_mutex_, SERVO_QUEUE_LOCK_TIMEOUT_MS) != osOK) {
    return;
  }

  bool notify_callback = false;
  if (service_request_.active && service_request_.generation == generation) {
    service_request_.result = result;
    service_request_.active = false;
    notify_callback = service_request_.callback_waiting;
    service_request_.callback_waiting = false;
  }
  osMutexRelease(queue_mutex_);

  if (notify_callback && service_completion_ != nullptr) {
    osSemaphoreRelease(service_completion_);
  }
}

bool DynamixelSerial::deadlineExpired(uint32_t now_ms, uint32_t deadline_ms)
{
  return static_cast<int32_t>(now_ms - deadline_ms) >= 0;
}

bool DynamixelSerial::startPendingTransaction(const ServoInstruction& instruction,
                                              bool fast_sync,
                                              uint32_t timeout_ms)
{
  if (pending_transaction_.active) {
    return false;
  }
  const ServoMask invalid_mask = static_cast<ServoMask>(~allServoMask());
  if (instruction.servo_mask == 0 ||
      (instruction.servo_mask & invalid_mask) != 0) {
    return false;
  }

  pending_transaction_ = PendingTransaction();
  pending_transaction_.active = true;
  pending_transaction_.instruction = instruction;
  pending_transaction_.fast_sync = fast_sync;
  pending_transaction_.deadline_ms = HAL_GetTick() + timeout_ms;
  return true;
}

ServoResult DynamixelSerial::completePendingTransaction()
{
  if (!pending_transaction_.active) {
    return SERVO_RESULT_BUSY;
  }

  const PendingTransaction transaction = pending_transaction_;
  ServoResult result = SERVO_RESULT_OK;
  if (deadlineExpired(HAL_GetTick(), transaction.deadline_ms)) {
    result = SERVO_RESULT_TIMEOUT;
  } else if (transaction.fast_sync) {
    result = readFastSyncStatusPacket(transaction.instruction.command,
                                      transaction.deadline_ms,
                                      transaction.instruction.servo_mask);
  } else {
    result = readStatusPacket(transaction.instruction.command,
                              transaction.deadline_ms);
  }

  pending_transaction_ = PendingTransaction();
  if (result != SERVO_RESULT_OK) {
    recoverRx();
  }
  return result;
}

ServoResult DynamixelSerial::executeBlockingRead(const ServoInstruction& instruction,
                                                 bool fast_sync,
                                                 uint32_t timeout_ms)
{
  ServoResult result = lastTransmitResult();
  if (result != SERVO_RESULT_OK) {
    return result;
  }
  if (!startPendingTransaction(instruction, fast_sync, timeout_ms)) {
    return SERVO_RESULT_BUSY;
  }
  return completePendingTransaction();
}

ServoResult DynamixelSerial::lastTransmitResult()
{
  if (last_tx_status_ == HAL_OK) {
    return SERVO_RESULT_OK;
  }

  const ServoResult result = last_tx_status_ == HAL_TIMEOUT
    ? SERVO_RESULT_TIMEOUT : SERVO_RESULT_UART_ERROR;
  recoverRx();
  return result;
}

void DynamixelSerial::recoverRx()
{
  HAL_UART_AbortReceive(huart_);
  __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                UART_CLEAR_NEF | UART_CLEAR_OREF);
  rd_ptr_ = 0;
  memset(rx_buf_, 0, sizeof(rx_buf_));

  if (direct_ttl_mode_) {
    HAL_HalfDuplex_EnableReceiver(huart_);
  }
  if (HAL_UART_Receive_DMA(huart_, rx_buf_, RX_BUFFER_SIZE) != HAL_OK) {
    HAL_UART_DMAStop(huart_);
    HAL_UART_Receive_DMA(huart_, rx_buf_, RX_BUFFER_SIZE);
  }
  rd_ptr_ = (RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) %
            RX_BUFFER_SIZE;
}

void DynamixelSerial::schedulePeriodicCommands()
{
  const uint32_t current_time = HAL_GetTick();

  if (get_pos_tick_ == 0U) {
    get_pos_tick_ = current_time + CONTROL_CYCLE_OFFSET;
  }
  if (get_temp_tick_ == 0U) get_temp_tick_ = current_time + GET_TEMP_OFFSET;
  if (get_move_tick_ == 0U) get_move_tick_ = current_time + GET_MOVE_OFFSET;
  if (get_error_tick_ == 0U) {
    get_error_tick_ = current_time + GET_HARDWARE_ERROR_STATUS_OFFSET;
  }

  auto advance_deadline = [current_time](uint32_t& deadline, uint32_t period) {
    do {
      deadline += period;
    } while (deadlineExpired(current_time, deadline));
  };

  auto enqueue_high_rate_cycle = [this]() {
    const ServoMask feedback_mask = activeServoMask();
    ServoMask goal_mask = 0;
    for (uint8_t i = 0; i < servo_num_; ++i) {
      if (servo_[i].mode_ != DIRECT_SERVO_MODE_EM) {
        goal_mask |= servoBit(i);
      }
    }
    ServoInstruction instructions[4];
    uint8_t instruction_count = 0;
    auto append = [&instructions, &instruction_count](
      uint8_t command, ServoMask servo_mask,
      ServoMask feedback_cycle_mask) {
      if (servo_mask == 0) return;
      instructions[instruction_count++] = {
        command, servo_mask, feedback_cycle_mask};
    };
    auto append_read = [&append, feedback_mask](
      ServoMask read_mask, bool last_feedback_group) {
      append(INST_GET_CUR_VEL_POS, read_mask,
             last_feedback_group ? feedback_mask : 0);
    };

    if (ttl_rs485_mixed_ == 0) {
      append_read(feedback_mask, true);
      append(INST_SET_GOAL_STATE, goal_mask, 0);
      const bool queued =
        enqueueInstructionBatch(instructions, instruction_count);
      if (queued) {
        feedback_received_mask_ = 0;
      }
      return queued;
    }

    const ServoGroupMasks feedback_groups =
      configuredServoMasks(feedback_mask);
    const ServoGroupMasks goal_groups =
      goal_mask == 0 ? ServoGroupMasks() : configuredServoMasks(goal_mask);
    if (!feedback_groups.valid ||
        (goal_mask != 0 && !goal_groups.valid)) {
      return false;
    }

    const bool ttl_feedback_expected = feedback_groups.ttl_mask != 0;
    append_read(feedback_groups.rs485_mask, !ttl_feedback_expected);
    append_read(feedback_groups.ttl_mask, ttl_feedback_expected);
    if (goal_mask != 0) {
      append(INST_SET_GOAL_STATE, goal_groups.rs485_mask, 0);
      append(INST_SET_GOAL_STATE, goal_groups.ttl_mask, 0);
    }
    const bool queued =
      enqueueInstructionBatch(instructions, instruction_count);
    if (queued) {
      feedback_received_mask_ = 0;
    }
    return queued;
  };

  auto enqueue_periodic_read = [this](uint8_t command) {
    const ServoMask servo_mask = activeServoMask();
    if (servo_mask == 0) return true;

    ServoInstruction instructions[2];
    uint8_t instruction_count = 0;
    if (ttl_rs485_mixed_ == 0) {
      instructions[instruction_count++] = {command, servo_mask, 0};
      return enqueueInstructionBatch(instructions, instruction_count);
    }

    const ServoGroupMasks groups = configuredServoMasks(servo_mask);
    if (!groups.valid) return false;
    if (groups.rs485_mask != 0) {
      instructions[instruction_count++] = {command, groups.rs485_mask, 0};
    }
    if (groups.ttl_mask != 0) {
      instructions[instruction_count++] = {command, groups.ttl_mask, 0};
    }
    return enqueueInstructionBatch(instructions, instruction_count);
  };

  // Feedback and goal writes form one ordered 200 Hz control cycle.
  if (CONTROL_CYCLE_DU > 0 &&
      deadlineExpired(current_time, get_pos_tick_)) {
    (void)enqueue_high_rate_cycle();
    advance_deadline(get_pos_tick_, CONTROL_CYCLE_DU);
    return;
  }

  if (GET_TEMP_DU > 0 && deadlineExpired(current_time, get_temp_tick_)) {
    (void)enqueue_periodic_read(INST_GET_PRESENT_TEMPERATURE);
    advance_deadline(get_temp_tick_, GET_TEMP_DU);
    return;
  }

  if (GET_MOVE_DU > 0 && deadlineExpired(current_time, get_move_tick_)) {
    (void)enqueue_periodic_read(INST_GET_PRESENT_MOVING);
    advance_deadline(get_move_tick_, GET_MOVE_DU);
    return;
  }

  if (GET_HARDWARE_ERROR_STATUS_DU > 0 &&
      deadlineExpired(current_time, get_error_tick_)) {
    (void)enqueue_periodic_read(INST_GET_HARDWARE_ERROR_STATUS);
    advance_deadline(get_error_tick_, GET_HARDWARE_ERROR_STATUS_DU);
  }
}

void DynamixelSerial::processInstructionQueue()
{
  ServoInstruction instruction;
  while (popInstruction(instruction)) {
    uint8_t servo_index = 0;
    const bool single_target =
      singleServoIndex(instruction.servo_mask, servo_index);

    switch (instruction.command) {
    case INST_SET_GOAL_STATE:
      if (!canSendControl(instruction.servo_mask)) continue;
      cmdBulkWriteGoalStates(instruction.servo_mask);
      (void)lastTransmitResult();
      return;
    case INST_SET_GOAL_POS:
      if (!canSendControl(instruction.servo_mask)) continue;
      cmdSyncWriteGoalPosition(instruction.servo_mask);
      (void)lastTransmitResult();
      return;
    case INST_SET_GOAL_VEL:
      if (!canSendControl(instruction.servo_mask)) continue;
      cmdSyncWriteGoalVelocity(instruction.servo_mask);
      (void)lastTransmitResult();
      return;
    case INST_SET_GOAL_CUR:
      if (!canSendControl(instruction.servo_mask)) continue;
      cmdSyncWriteGoalCurrent(instruction.servo_mask);
      (void)lastTransmitResult();
      return;
    case INST_SET_TORQUE:
    case INST_REBOOT:
    case INST_SET_HOMING_OFFSET:
    case INST_SET_POSITION_GAINS:
    case INST_SET_CURRENT_LIMIT:
    case INST_SET_PROFILE_VELOCITY:
      if (!single_target) {
        continue;
      }
      switch (instruction.command) {
      case INST_SET_TORQUE:
        if (servo_[servo_index].torque_enable_ &&
            !canSendControl(instruction.servo_mask)) {
          servo_[servo_index].torque_enable_ = false;
          continue;
        }
        cmdWriteTorqueEnableWithLED(servo_index,
                                    servo_[servo_index].torque_enable_);
        (void)lastTransmitResult();
        break;
      case INST_REBOOT:
        cmdReboot(servo_[servo_index].id_);
        (void)lastTransmitResult();
        break;
      case INST_SET_HOMING_OFFSET:
        (void)applyHomingOffset(servo_index);
        break;
      case INST_SET_POSITION_GAINS:
        (void)applyPositionGains(servo_index);
        break;
      case INST_SET_CURRENT_LIMIT:
        (void)applyCurrentLimit(servo_index);
        break;
      case INST_SET_PROFILE_VELOCITY:
        (void)applyProfileVelocity(servo_index);
        break;
      }
      return;
    case INST_SERVICE_REQUEST:
    {
      ServoServiceRequest request;
      bool request_valid = false;
      if (queue_mutex_ != nullptr &&
          osMutexWait(queue_mutex_, SERVO_QUEUE_LOCK_TIMEOUT_MS) == osOK) {
        if (service_request_.active) {
          request = service_request_;
          request_valid = true;
        }
        osMutexRelease(queue_mutex_);
      }

      if (!request_valid) {
        continue;
      }

      ServoResult result = SERVO_RESULT_TIMEOUT;
      if (!deadlineExpired(HAL_GetTick(), request.deadline_ms)) {
        result = executeServiceRequest(request);
      }
      completeServiceRequest(request.generation, result);
      return;
    }
    case INST_GET_CUR_VEL_POS:
    case INST_GET_PRESENT_TEMPERATURE:
    case INST_GET_PRESENT_MOVING:
    case INST_GET_HARDWARE_ERROR_STATUS:
      if (servoCount(instruction.servo_mask) == 0) {
        continue;
      }

      switch (instruction.command) {
      case INST_GET_CUR_VEL_POS:
        cmdFastSyncReadPresentCurVelPos(instruction.servo_mask);
        break;
      case INST_GET_PRESENT_TEMPERATURE:
        cmdFastSyncReadPresentTemperature(instruction.servo_mask);
        break;
      case INST_GET_PRESENT_MOVING:
        cmdFastSyncReadMoving(instruction.servo_mask);
        break;
      case INST_GET_HARDWARE_ERROR_STATUS:
        cmdFastSyncReadHardwareErrorStatus(instruction.servo_mask);
        break;
      }

      if (lastTransmitResult() != SERVO_RESULT_OK) {
        if (instruction.command == INST_GET_CUR_VEL_POS &&
            instruction.feedback_cycle_mask != 0) {
          feedback_received_mask_ = 0;
        }
        return;
      }
      if (!startPendingTransaction(instruction, true,
                                   SERVO_RESPONSE_TIMEOUT_MS)) {
        if (instruction.command == INST_GET_CUR_VEL_POS &&
            instruction.feedback_cycle_mask != 0) {
          feedback_received_mask_ = 0;
        }
        recoverRx();
      }
      return;
    default:
      continue;
    }
  }
}

void DynamixelSerial::update()
{
  if (servo_num_ == 0) return;

  updateTargetWatchdogs(HAL_GetTick());

  if (pending_transaction_.active) {
    const ServoInstruction completed_instruction = pending_transaction_.instruction;
    const ServoResult result = completePendingTransaction();
    if (completed_instruction.command == INST_GET_CUR_VEL_POS) {
      if (result == SERVO_RESULT_OK) {
        feedback_received_mask_ |= completed_instruction.servo_mask;
      }
      const ServoMask expected_mask =
        completed_instruction.feedback_cycle_mask;
      if (expected_mask != 0) {
        if ((feedback_received_mask_ & expected_mask) == expected_mask) {
          setROSCommFlag(true);
        }
        feedback_received_mask_ = 0;
      }
    }
  }

  const uint32_t torque_off_mask =
    __atomic_load_n(&torque_off_pending_mask_, __ATOMIC_ACQUIRE);
  if (torque_off_mask != 0U) {
    for (uint8_t servo_index = 0; servo_index < servo_num_; ++servo_index) {
      const uint32_t servo_mask = 1UL << servo_index;
      if ((torque_off_mask & servo_mask) == 0U) continue;

      __atomic_fetch_and(&torque_off_pending_mask_, ~servo_mask,
                         __ATOMIC_ACQ_REL);
      cmdWriteTorqueEnableWithLED(servo_index, false);
      if (lastTransmitResult() != SERVO_RESULT_OK) {
        __atomic_fetch_or(&torque_off_pending_mask_, servo_mask,
                          __ATOMIC_RELEASE);
      }
      return;
    }
  }

  schedulePeriodicCommands();
  processInstructionQueue();
}

/* Transmit instruction packet to Dynamixel */
HAL_StatusTypeDef DynamixelSerial::transmitInstructionPacket(
  uint8_t id, uint16_t len, uint8_t instruction, uint8_t* parameters)
{
  uint8_t transmit_data[INSTRUCTION_PACKET_SIZE];
  if (len < 3) {
    last_tx_status_ = HAL_ERROR;
    return last_tx_status_;
  }
  /* headers */
  transmit_data[0] = HEADER0;
  transmit_data[1] = HEADER1;
  transmit_data[2] = HEADER2;
  transmit_data[3] = HEADER3;
  /* servo id */
  transmit_data[4] = id;
  /* instruction len */
  transmit_data[5] = len & 0xFF; //LEN_L
  transmit_data[6] = (len >> 8) & 0xFF; //LEN_H
  /* instruction */
  transmit_data[7] = instruction;
  /* parameters */
  const size_t parameter_count = len - 3U;
  size_t transmit_data_index = 8;
  for (size_t i = 0; i < parameter_count; ++i) {
    if (parameters == nullptr ||
        !appendStuffedByte(
          parameters[i], transmit_data, INSTRUCTION_PACKET_SIZE - 2U,
          transmit_data_index, 8U)) {
      last_tx_status_ = HAL_ERROR;
      return last_tx_status_;
    }
  }
  const uint16_t stuffed_len = static_cast<uint16_t>(
    len + (transmit_data_index - 8U - parameter_count));
  transmit_data[5] = stuffed_len & 0xFF;
  transmit_data[6] = (stuffed_len >> 8) & 0xFF;

  /* checksum */
  uint16_t chksum = calcCRC16(
    0, transmit_data, static_cast<int>(transmit_data_index));

  transmit_data[transmit_data_index] = chksum & 0xFF; //CRC_L
  transmit_data_index++;
  transmit_data[transmit_data_index] = (chksum >> 8) & 0xFF; //CRC_H
  transmit_data_index++;

  /* send data */
  if(direct_ttl_mode_)  HAL_HalfDuplex_EnableTransmitter(huart_);

  last_tx_status_ = HAL_UART_Transmit(huart_, transmit_data,
                                      static_cast<uint16_t>(transmit_data_index),
                                      10);
  return last_tx_status_;
}

ServoResult DynamixelSerial::readFastSyncStatusPacket(
  uint8_t status_packet_instruction, uint32_t deadline_ms,
  ServoMask expected_servo_mask)
{
  const ServoMask invalid_mask = static_cast<ServoMask>(~allServoMask());
  const uint8_t expected_servo_count = servoCount(expected_servo_mask);
  if (expected_servo_count == 0 ||
      (expected_servo_mask & invalid_mask) != 0) {
    return SERVO_RESULT_INVALID_ARGUMENT;
  }

  int status_stage = READ_HEADER0;
  uint8_t rx_data;
  uint16_t parameter_len = 0;
  uint8_t parameters[STATUS_PACKET_SIZE] = {0};
  int parameter_index = 0;
  int parameter_loop_count = 0;
  uint16_t checksum = 0;
  uint8_t receive_data[STATUS_PACKET_SIZE] = {0};
  bool read_end_flag = false;
  int loop_count = 0;

  if(direct_ttl_mode_) {
    while (__HAL_UART_GET_FLAG(huart_, UART_FLAG_TC) == RESET) {
      if (deadlineExpired(HAL_GetTick(), deadline_ms)) {
        return SERVO_RESULT_TIMEOUT;
      }
    }
    HAL_HalfDuplex_EnableReceiver(huart_);
  }

  while(!read_end_flag) {
    HAL_StatusTypeDef receive_status = readUntil(&rx_data, deadline_ms);
    if (receive_status != HAL_OK) {
      return receive_status == HAL_TIMEOUT
        ? SERVO_RESULT_TIMEOUT : SERVO_RESULT_UART_ERROR;
    }

    switch (status_stage) {
    case READ_HEADER0:
      if (rx_data == HEADER0) status_stage++;
      break;
    case READ_HEADER1:
      if (rx_data == HEADER1) status_stage++;
      else return SERVO_RESULT_INVALID_RESPONSE;
      break;
    case READ_HEADER2:
      if (rx_data == HEADER2) status_stage++;
      else return SERVO_RESULT_INVALID_RESPONSE;
      break;
    case READ_HEADER3:
      if (rx_data == HEADER3) status_stage++;
      else return SERVO_RESULT_INVALID_RESPONSE;
      break;
    case READ_SERVOID:
      if (rx_data != DX_BROADCAST_ID) return SERVO_RESULT_INVALID_RESPONSE;
      status_stage++;
      break;
    case READ_LENL:
      parameter_len = rx_data;
      status_stage++;
      break;
    case READ_LENH:
      parameter_len |= ((rx_data << 8) & 0xFF00);
      if (parameter_len < 4 || parameter_len > STATUS_PACKET_SIZE) {
        return SERVO_RESULT_INVALID_RESPONSE;
      }
      status_stage++;
      break;
    case READ_INSTRUCTION:
      if (rx_data == STATUS_PACKET_INSTRUCTION) status_stage++;
      else return SERVO_RESULT_INVALID_RESPONSE;
      break;
    case READ_ERROR:
      if ((rx_data & 0x7F) == ERROR_NO_ERROR) status_stage++;
      else return SERVO_RESULT_INVALID_RESPONSE;
      break;
    case READ_PARAMETER:
      if (parameter_index >= STATUS_PACKET_SIZE) return SERVO_RESULT_INVALID_RESPONSE;
      parameters[parameter_index++] = rx_data;
      parameter_loop_count++;
      if (parameter_loop_count == parameter_len - 4) status_stage++;
      break;
    case READ_CHECKSUML:
      checksum = rx_data;
      status_stage++;
      break;
    case READ_CHECKSUMH:
      checksum |= ((rx_data << 8) & 0xFF00);
      if (checksum == calcCRC16(0, receive_data, loop_count)) read_end_flag = true;
      else return SERVO_RESULT_CRC_ERROR;
      break;
    default:
      return SERVO_RESULT_INVALID_RESPONSE;
    }

    if (status_stage > READ_HEADER0 && status_stage != READ_CHECKSUMH) {
      if (loop_count >= STATUS_PACKET_SIZE) return SERVO_RESULT_INVALID_RESPONSE;
      receive_data[loop_count++] = rx_data;
    }
  }

  __HAL_UART_CLEAR_FLAG(huart_, UART_FLAG_RXNE);
  __HAL_UART_CLEAR_PEFLAG(huart_);
  __HAL_UART_CLEAR_OREFLAG(huart_);
  __HAL_UART_CLEAR_FEFLAG(huart_);

  int servo_data_byte_len = 0;
  switch (status_packet_instruction) {
  case INST_GET_CUR_VEL_POS:
    servo_data_byte_len = PRESENT_CUR_VEL_POS_BYTE_LEN;
    break;
  case INST_GET_PRESENT_TEMPERATURE:
    servo_data_byte_len = PRESENT_TEMPERATURE_BYTE_LEN;
    break;
  case INST_GET_PRESENT_MOVING:
    servo_data_byte_len = MOVING_BYTE_LEN;
    break;
  case INST_GET_HARDWARE_ERROR_STATUS:
    servo_data_byte_len = HARDWARE_ERROR_STATUS_BYTE_LEN;
    break;
  default:
    return SERVO_RESULT_INVALID_RESPONSE;
  }

  const int expected_parameter_bytes =
    expected_servo_count * (servo_data_byte_len + 4) - 3;
  if (parameter_index != expected_parameter_bytes) return SERVO_RESULT_INVALID_RESPONSE;

  int offset = 0;
  uint8_t parsed_servo_count = 0;
  for (uint8_t i = 0; i < servo_num_; ++i) {
    if ((expected_servo_mask & servoBit(i)) == 0) continue;

    if (parsed_servo_count > 0) {
      if ((parameters[offset] & 0x7F) != ERROR_NO_ERROR) return SERVO_RESULT_INVALID_RESPONSE;
      offset++;
    }

    const uint8_t expected_id = servo_[i].id_;
    if (parameters[offset] != expected_id) return SERVO_RESULT_INVALID_RESPONSE;
    offset++;

    switch (status_packet_instruction) {
    case INST_GET_CUR_VEL_POS: {
      int16_t present_current = ((parameters[offset + 1] << 8) & 0xFF00) | (parameters[offset] & 0xFF);
      int32_t present_velocity = ((parameters[offset + 5] << 24) & 0xFF000000) |
                                 ((parameters[offset + 4] << 16) & 0xFF0000) |
                                 ((parameters[offset + 3] << 8) & 0xFF00) |
                                 (parameters[offset + 2] & 0xFF);
      int32_t present_position = ((parameters[offset + 9] << 24) & 0xFF000000) |
                                 ((parameters[offset + 8] << 16) & 0xFF0000) |
                                 ((parameters[offset + 7] << 8) & 0xFF00) |
                                 (parameters[offset + 6] & 0xFF);
      servo_[i].present_current_ = present_current;
      servo_[i].present_velocity = present_velocity;
      servo_[i].hardware_error_status_ &= ((1 << ENCODER_CONNECT_ERROR) - 1);
      if (servo_[i].external_encoder_flag_) {
#ifndef SPINAL
        encoder_handler_.update();
        if(encoder_handler_.connected()) {
          servo_[i].present_position_ = (int32_t)(encoder_handler_.getValue());
          if (servo_[i].first_get_pos_flag_) {
            servo_[i].internal_offset_ = servo_[i].resolution_ratio_ * servo_[i].present_position_ - present_position;
            servo_[i].first_get_pos_flag_ = false;
          }
        } else {
          servo_[i].hardware_error_status_ |= 1 << ENCODER_CONNECT_ERROR;
        }
#endif
      } else {
        if (servo_[i].first_get_pos_flag_) {
          servo_[i].internal_offset_ = std::floor(present_position / 4096.0) * -4096;
          servo_[i].first_get_pos_flag_ = false;
        }
        servo_[i].setPresentPosition(present_position);
      }
      break;
    }
    case INST_GET_PRESENT_TEMPERATURE:
      servo_[i].present_temp_ = parameters[offset];
      break;
    case INST_GET_PRESENT_MOVING:
      servo_[i].moving_ = parameters[offset];
      break;
    case INST_GET_HARDWARE_ERROR_STATUS:
      servo_[i].hardware_error_status_ &=  ((1 << RESOLUTION_RATIO_ERROR) + (1 << ENCODER_CONNECT_ERROR));
      servo_[i].hardware_error_status_ |= parameters[offset];
      break;
    default:
      return SERVO_RESULT_INVALID_RESPONSE;
    }

    offset += servo_data_byte_len;
    ++parsed_servo_count;
    if (parsed_servo_count < expected_servo_count) offset += 2;
  }

  return (offset == parameter_index)
    ? SERVO_RESULT_OK : SERVO_RESULT_INVALID_RESPONSE;
}

/* Receive status packet to Dynamixel */
ServoResult DynamixelSerial::readStatusPacket(
  uint8_t status_packet_instruction, uint32_t deadline_ms)
{
		int status_stage = READ_HEADER0;
		uint8_t rx_data;
		uint16_t parameter_len = 0;
		uint8_t parameters[STATUS_PACKET_SIZE] = {0};
		size_t parameter_index = 0;
		uint16_t parameter_loop_count = 0;
		UnstuffState unstuff_state;
		uint16_t checksum = 0;
		uint8_t receive_data[STATUS_PACKET_SIZE] = {0};
		bool read_end_flag = false;
		size_t loop_count = 0;
	uint8_t servo_id;

  if(direct_ttl_mode_) {
    while (__HAL_UART_GET_FLAG(huart_, UART_FLAG_TC) == RESET) {
      if (deadlineExpired(HAL_GetTick(), deadline_ms)) {
        return SERVO_RESULT_TIMEOUT;
      }
    }
    // After transmitting, enable the receiver
    HAL_HalfDuplex_EnableReceiver(huart_);
  }

	while(!read_end_flag) {
		HAL_StatusTypeDef receive_status = readUntil(&rx_data, deadline_ms);
			if (receive_status != HAL_OK) {
				return receive_status == HAL_TIMEOUT
				  ? SERVO_RESULT_TIMEOUT : SERVO_RESULT_UART_ERROR;
			}

		switch (status_stage) {
		case READ_HEADER0:
			if (rx_data == HEADER0) {
				status_stage++;
			}
			break;
		case READ_HEADER1:
			if (rx_data == HEADER1) {
				status_stage++;
			} else {
				return SERVO_RESULT_INVALID_RESPONSE;
			}
			break;
		case READ_HEADER2:
			if (rx_data == HEADER2) {
				status_stage++;
			} else {
				return SERVO_RESULT_INVALID_RESPONSE;
			}
			break;
		case READ_HEADER3:
			if (rx_data == HEADER3) {
				status_stage++;
			} else {
				return SERVO_RESULT_INVALID_RESPONSE;
			}
			break;
		case READ_SERVOID:
			servo_id = rx_data;
			status_stage++;
			break;
		case READ_LENL:
			parameter_len = rx_data;
			status_stage++;
			break;
		case READ_LENH:
			parameter_len |= ((rx_data << 8) & 0xFF00);
				if (parameter_len < 4 ||
				    parameter_len > STATUS_PACKET_SIZE - 5) {
					return SERVO_RESULT_INVALID_RESPONSE;
				}
				status_stage++;
			break;
		case READ_INSTRUCTION:
			if (rx_data == STATUS_PACKET_INSTRUCTION) {
				status_stage++;
			} else {
				return SERVO_RESULT_INVALID_RESPONSE;
			}
			break;
		case READ_ERROR:
			if ((rx_data & 0x7F) == ERROR_NO_ERROR) {
				status_stage++;
			} else {
				return SERVO_RESULT_INVALID_RESPONSE;
			}
			break;
		case READ_PARAMETER:
			if (!unstuff_state.append(rx_data, parameters,
			                          STATUS_PACKET_SIZE,
			                          parameter_index)) {
				return SERVO_RESULT_INVALID_RESPONSE;
			}
			parameter_loop_count++;
			if (parameter_loop_count == parameter_len - 4) {
				status_stage++;
			}
			break;
		case READ_CHECKSUML:
			checksum = rx_data;
			status_stage++;
			break;
		case READ_CHECKSUMH:
			checksum |= ((rx_data << 8) & 0xFF00);
			if (checksum == calcCRC16(0, receive_data, loop_count)) {
				read_end_flag = true;
			} else {
					return SERVO_RESULT_CRC_ERROR;
				}
				break;
			default:
			return SERVO_RESULT_INVALID_RESPONSE;
		}
		if (status_stage > READ_HEADER0 && status_stage != READ_CHECKSUMH) {
			if (loop_count >= STATUS_PACKET_SIZE) {
				return SERVO_RESULT_INVALID_RESPONSE;
			}
			receive_data[loop_count] = rx_data;
			loop_count++;
		}
	}

	auto s = std::find(servo_.begin(), servo_.end(), ServoData(servo_id));
	if (status_packet_instruction != INST_PING && s == servo_.end()) {
		return SERVO_RESULT_INVALID_RESPONSE;
	}

        /* clear UART RX */
        __HAL_UART_CLEAR_FLAG(huart_, UART_FLAG_RXNE);
        __HAL_UART_CLEAR_PEFLAG(huart_);
        __HAL_UART_CLEAR_OREFLAG(huart_);
        __HAL_UART_CLEAR_FEFLAG(huart_);

	/* read success */
	auto update_present_position = [this](ServoData* servo, int32_t present_position) {
		if (servo == nullptr) return;

		servo->hardware_error_status_ &= ((1 << ENCODER_CONNECT_ERROR) - 1);
		if(servo->external_encoder_flag_) {
#ifndef SPINAL
			encoder_handler_.update();
			if(encoder_handler_.connected()) {
				servo->present_position_ = (int32_t)(encoder_handler_.getValue());
				if (servo->first_get_pos_flag_) {
					servo->internal_offset_ = servo->resolution_ratio_ * servo->present_position_ - present_position;
					servo->first_get_pos_flag_ = false;
				}
			}
			else {
				servo->hardware_error_status_ |= 1 << ENCODER_CONNECT_ERROR;
			}
#endif
		}
		else {
			if (servo->first_get_pos_flag_) {
				servo->internal_offset_ = std::floor(present_position / 4096.0) * -4096;
				servo->first_get_pos_flag_ = false;
			}
			servo->setPresentPosition(present_position);
		}
	};

	switch (status_packet_instruction) {
	case INST_PING:
		if (parameter_index != PING_LEN) {
			return SERVO_RESULT_INVALID_RESPONSE;
		}
		if (servo_num_ >= MAX_SERVO_NUM) {
				return SERVO_RESULT_INVALID_RESPONSE;
			}
			servo_[servo_num_++].id_ = servo_id;
	    return SERVO_RESULT_OK;
	case INST_GET_PRESENT_POS:
	{
		if (parameter_index != PRESENT_POSITION_BYTE_LEN) {
			return SERVO_RESULT_INVALID_RESPONSE;
		}
		int32_t present_position = ((parameters[3] << 24) & 0xFF000000) | ((parameters[2] << 16) & 0xFF0000) | ((parameters[1] << 8) & 0xFF00) | (parameters[0] & 0xFF);
		if (s != servo_.end()) {
			update_present_position(&(*s), present_position);
		}
                return SERVO_RESULT_OK;
	}
	case INST_GET_HOMING_OFFSET:
		if (parameter_index != HOMING_OFFSET_BYTE_LEN) {
			return SERVO_RESULT_INVALID_RESPONSE;
		}
		if (s != servo_.end()) {
			s->homing_offset_ = ((parameters[3] << 24) & 0xFF000000) | ((parameters[2] << 16) & 0xFF0000) | ((parameters[1] << 8) & 0xFF00) | (parameters[0] & 0xFF);
		}
		return SERVO_RESULT_OK;
	case INST_GET_CURRENT_LIMIT:
		if (parameter_index != CURRENT_LIMIT_BYTE_LEN) {
			return SERVO_RESULT_INVALID_RESPONSE;
		}
		if (s != servo_.end()) {
			s->current_limit_ = ((parameters[1] << 8) & 0xFF00) | (parameters[0] & 0xFF);
		}
		return SERVO_RESULT_OK;
	case INST_GET_OPERATING_MODE:
		if (parameter_index != OPERATING_MODE_BYTE_LEN) {
			return SERVO_RESULT_INVALID_RESPONSE;
		}
		if (s != servo_.end()) {
			s->operating_mode_ = parameters[0];
		}
		return SERVO_RESULT_OK;
	case INST_GET_POSITION_GAINS:
		if (parameter_index != POSITION_GAINS_BYTE_LEN) {
			return SERVO_RESULT_INVALID_RESPONSE;
		}
		if (s != servo_.end()) {
			s->d_gain_ = ((parameters[1] << 8) & 0xFF00) | (parameters[0] & 0xFF);
			s->i_gain_ = ((parameters[3] << 8) & 0xFF00) | (parameters[2] & 0xFF);
			s->p_gain_ = ((parameters[5] << 8) & 0xFF00) | (parameters[4] & 0xFF);
		}
		return SERVO_RESULT_OK;
	case INST_GET_PROFILE_VELOCITY:
		if (parameter_index != PROFILE_VELOCITY_BYTE_LEN) {
			return SERVO_RESULT_INVALID_RESPONSE;
		}
		if (s != servo_.end()) {
			s->profile_velocity_ = ((parameters[3] << 24) & 0xFF000000) | ((parameters[2] << 16) & 0xFF0000) | ((parameters[1] << 8) & 0xFF00) | (parameters[0] & 0xFF);
		}
		return SERVO_RESULT_OK;
	default:
		return SERVO_RESULT_INVALID_RESPONSE;
	}
}

void DynamixelSerial::cmdPing(uint8_t id)
{
  transmitInstructionPacket(id, PING_LEN, COMMAND_PING, nullptr);
}

void DynamixelSerial::cmdReboot(uint8_t id)
{
	transmitInstructionPacket(id, REBOOT_LEN, COMMAND_REBOOT, nullptr);
}

void DynamixelSerial::cmdRead(uint8_t id, uint16_t address, uint16_t byte_size)
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];
	parameters[0] = address & 0xFF;
	parameters[1] = (address >> 8) & 0xFF;
	parameters[2] = byte_size & 0xFF;
	parameters[3] = (byte_size >> 8) & 0xFF;
	transmitInstructionPacket(id, READ_INST_LEN, COMMAND_READ, parameters);
}

void DynamixelSerial::cmdWrite(uint8_t id, uint16_t address, uint8_t* param, int param_len)
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];
	parameters[0] = address & 0xFF;
	parameters[1] = (address >> 8) & 0xFF;
	for (int i = 0; i < param_len; i++) {
		parameters[i + 2] = param[i];
	}
	transmitInstructionPacket(id, param_len + 5, COMMAND_WRITE, parameters);
}

void DynamixelSerial::cmdFastSyncRead(uint16_t address, uint16_t byte_size,
                                      ServoMask servo_mask)
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];
	parameters[0] = address & 0xFF;
	parameters[1] = (address >> 8) & 0xFF;
	parameters[2] = byte_size & 0xFF;
	parameters[3] = (byte_size >> 8) & 0xFF;
	int param_idx = 4;
	for (unsigned int i = 0; i < servo_num_; ++i) {
		if ((servo_mask & servoBit(i)) == 0) continue;
		parameters[param_idx++] = servo_[i].id_;
	}
	transmitInstructionPacket(DX_BROADCAST_ID, param_idx + 3, COMMAND_FAST_SYNC_READ, parameters);
}

void DynamixelSerial::cmdSyncWrite(uint16_t address, uint8_t* param,
                                   int param_len, ServoMask servo_mask)
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];
	parameters[0] = address & 0xFF;
	parameters[1] = (address >> 8) & 0xFF;
	parameters[2] = param_len & 0xFF;
	parameters[3] = (param_len >> 8) & 0xFF;
	int all_param_idx = 4;
	for (unsigned int i = 0; i < servo_num_; i++) {
		if ((servo_mask & servoBit(i)) == 0) continue;
		parameters[all_param_idx++] = servo_[i].id_;
		for (int j = 0; j < param_len; j++) {
			parameters[all_param_idx++] = param[i * param_len + j];
		}
	}
	transmitInstructionPacket(DX_BROADCAST_ID, all_param_idx + 3, COMMAND_SYNC_WRITE, parameters);
}

void DynamixelSerial::cmdReadCurrentLimit(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_CURRENT_LIMIT, CURRENT_LIMIT_BYTE_LEN);
}

void DynamixelSerial::cmdReadHomingOffset(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_HOMING_OFFSET, HOMING_OFFSET_BYTE_LEN);
}

void DynamixelSerial::cmdReadPositionGains(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_POSITION_D_GAIN, POSITION_GAINS_BYTE_LEN);
}

void DynamixelSerial::cmdReadPresentPosition(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_PRESENT_POSITION, PRESENT_POSITION_BYTE_LEN);
}

void DynamixelSerial::cmdReadProfileVelocity(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_PROFILE_VELOCITY, PROFILE_VELOCITY_BYTE_LEN);
}

void DynamixelSerial::cmdReadOperatingMode(uint8_t servo_index)
{
	cmdRead(servo_[servo_index].id_, CTRL_OPERATING_MODE,
	        OPERATING_MODE_BYTE_LEN);
}

void DynamixelSerial::cmdWriteCurrentLimit(uint8_t servo_index)
{
	uint16_t current_limit = servo_[servo_index].current_limit_;
	uint8_t parameters[CURRENT_LIMIT_BYTE_LEN];
	parameters[0] = (uint8_t)(current_limit & 0xFF);
	parameters[1] = (uint8_t)((current_limit >> 8) & 0xFF);

	cmdWrite(servo_[servo_index].id_, CTRL_CURRENT_LIMIT, parameters, CURRENT_LIMIT_BYTE_LEN);
}

void DynamixelSerial::cmdWriteHomingOffset(uint8_t servo_index)
{
	int32_t homing_offset = servo_[servo_index].homing_offset_;
	uint8_t parameters[4];
	parameters[0] = (uint8_t)(homing_offset & 0xFF);
	parameters[1] = (uint8_t)((homing_offset >> 8) & 0xFF);
	parameters[2] = (uint8_t)((homing_offset >> 16) & 0xFF);
	parameters[3] = (uint8_t)((homing_offset >> 24) & 0xFF);
	cmdWrite(servo_[servo_index].id_, CTRL_HOMING_OFFSET, parameters, HOMING_OFFSET_BYTE_LEN);
}

void DynamixelSerial::cmdWritePositionGains(uint8_t servo_index)
{
	uint8_t parameters[POSITION_GAINS_BYTE_LEN];
	parameters[0] = servo_[servo_index].d_gain_ & 0xFF;
	parameters[1] = (servo_[servo_index].d_gain_ >> 8) & 0xFF;
	parameters[2] = servo_[servo_index].i_gain_ & 0xFF;
	parameters[3] = (servo_[servo_index].i_gain_ >> 8) & 0xFF;
	parameters[4] = servo_[servo_index].p_gain_ & 0xFF;
	parameters[5] = (servo_[servo_index].p_gain_ >> 8) & 0xFF;

	cmdWrite(servo_[servo_index].id_, CTRL_POSITION_D_GAIN, parameters, POSITION_GAINS_BYTE_LEN);
}

void DynamixelSerial::cmdWriteProfileVelocity(uint8_t servo_index)
{
	uint16_t vel = servo_[servo_index].profile_velocity_;
	uint8_t parameters[PROFILE_VELOCITY_BYTE_LEN];
	parameters[0] = (uint8_t)(vel & 0xFF);
	parameters[1] = (uint8_t)((vel >> 8) & 0xFF);
	parameters[2] = (uint8_t)((vel >> 16) & 0xFF);
	parameters[3] = (uint8_t)((vel >> 24) & 0xFF);
	cmdWrite(servo_[servo_index].id_, CTRL_PROFILE_VELOCITY, parameters, PROFILE_VELOCITY_BYTE_LEN);
}

void DynamixelSerial::cmdWriteOperatingMode(uint8_t servo_index,
                                            uint8_t operating_mode)
{
	cmdWrite(servo_[servo_index].id_, CTRL_OPERATING_MODE, &operating_mode,
	         OPERATING_MODE_BYTE_LEN);
}

void DynamixelSerial::cmdWriteStatusReturnLevel(uint8_t id, uint8_t set)
{
	uint8_t param = set;
	cmdWrite(id, CTRL_STATUS_RETURN_LEVEL, &param, STATUS_RETURN_LEVEL_BYTE_LEN);
}

void DynamixelSerial::cmdWriteTorqueEnable(uint8_t servo_index)
{
	uint8_t parameter = servo_[servo_index].torque_enable_ ? 1U : 0U;
	cmdWrite(servo_[servo_index].id_, CTRL_TORQUE_ENABLE, &parameter,
	         TORQUE_ENABLE_BYTE_LEN);
}

void DynamixelSerial::cmdWriteTorqueEnableWithLED(uint8_t servo_index,
                                                  bool torque_enable)
{
	const uint8_t enable_value = torque_enable ? 1U : 0U;
	uint8_t parameters[TORQUE_LED_BYTE_LEN] = {enable_value, enable_value};
	servo_[servo_index].led_ = torque_enable;
	cmdWrite(servo_[servo_index].id_, CTRL_TORQUE_ENABLE, parameters,
	         TORQUE_LED_BYTE_LEN);
}

void DynamixelSerial::cmdFastSyncReadHardwareErrorStatus(ServoMask servo_mask)
{
	cmdFastSyncRead(CTRL_HARDWARE_ERROR_STATUS,
	                HARDWARE_ERROR_STATUS_BYTE_LEN, servo_mask);
}

void DynamixelSerial::cmdFastSyncReadMoving(ServoMask servo_mask)
{
	cmdFastSyncRead(CTRL_MOVING, MOVING_BYTE_LEN, servo_mask);
}

void DynamixelSerial::cmdFastSyncReadPresentCurVelPos(ServoMask servo_mask)
{
	cmdFastSyncRead(CTRL_PRESENT_CURRENT, PRESENT_CUR_VEL_POS_BYTE_LEN,
	                servo_mask);
}

void DynamixelSerial::cmdFastSyncReadPresentTemperature(ServoMask servo_mask)
{
	cmdFastSyncRead(CTRL_PRESENT_TEMPERATURE, PRESENT_TEMPERATURE_BYTE_LEN,
	                servo_mask);
}

int16_t DynamixelSerial::calculateMITGoalCurrent(
  const ServoData& servo) const
{
  if (servo.first_get_pos_flag_ || !servo.target_received_ ||
      servo.watchdog_timed_out_) {
    return 0;
  }

  const int64_t kp_scaled =
    static_cast<int64_t>(servo.kp_integer_) * MIT_GAIN_DECIMAL_SCALE +
    servo.kp_decimal_;
  const int64_t kd_scaled =
    static_cast<int64_t>(servo.kd_integer_) * MIT_GAIN_DECIMAL_SCALE +
    servo.kd_decimal_;
  const int64_t position_error =
    static_cast<int64_t>(servo.target_position_) -
    servo.present_position_;
  const int64_t velocity_error =
    static_cast<int64_t>(servo.target_velocity_) -
    servo.present_velocity;
  const int64_t current_scaled =
    static_cast<int64_t>(servo.target_current_) *
      MIT_GAIN_DECIMAL_SCALE +
    kp_scaled * position_error +
    kd_scaled * velocity_error;

  return clampGoalCurrent(
    symmetricDivide(current_scaled, MIT_GAIN_DECIMAL_SCALE),
    servo.current_limit_);
}

void DynamixelSerial::cmdBulkWriteGoalStates(ServoMask servo_mask)
{
  uint8_t parameters[INSTRUCTION_PACKET_SIZE];
  int param_idx = 0;

  auto append_value = [&parameters, &param_idx](
    uint8_t id, uint16_t address, uint16_t data_length,
    uint32_t value) {
    const int entry_length = 5 + data_length;
    if (param_idx + entry_length > INSTRUCTION_PACKET_SIZE) return false;
    parameters[param_idx++] = id;
    parameters[param_idx++] = address & 0xFF;
    parameters[param_idx++] = (address >> 8) & 0xFF;
    parameters[param_idx++] = data_length & 0xFF;
    parameters[param_idx++] = (data_length >> 8) & 0xFF;
    for (uint16_t i = 0; i < data_length; ++i) {
      parameters[param_idx++] = (value >> (8 * i)) & 0xFF;
    }
    return true;
  };

  ServoMask written_mask = 0;
  ServoMask cbp_mask = 0;
  for (uint8_t i = 0; i < servo_num_; ++i) {
    const ServoMask bit = servoBit(i);
    if ((servo_mask & bit) == 0) continue;
    ServoData& servo = servo_[i];
    bool appended = false;

    switch (servo.mode_) {
    case DIRECT_SERVO_MODE_DP:
    case DIRECT_SERVO_MODE_DEP:
      appended = append_value(
        servo.id_, CTRL_GOAL_POSITION, GOAL_POSITION_BYTE_LEN,
        static_cast<uint32_t>(servo.goal_position_));
      break;
    case DIRECT_SERVO_MODE_CBP:
      servo.goal_current_ = clampGoalCurrent(
        servo.target_current_ < 0
          ? -static_cast<int64_t>(servo.target_current_)
          : servo.target_current_,
        servo.current_limit_);
      appended = append_value(
        servo.id_, CTRL_GOAL_CURRENT, GOAL_CURRENT_BYTE_LEN,
        static_cast<uint16_t>(servo.goal_current_));
      cbp_mask |= bit;
      break;
    case DIRECT_SERVO_MODE_DV:
      servo.goal_velocity_ =
        (!servo.target_received_ || servo.watchdog_timed_out_)
        ? 0 : servo.target_velocity_;
      appended = append_value(
        servo.id_, CTRL_GOAL_VELOCITY, GOAL_VELOCITY_BYTE_LEN,
        static_cast<uint32_t>(servo.goal_velocity_));
      break;
    case DIRECT_SERVO_MODE_DC:
      servo.goal_current_ =
        (!servo.target_received_ || servo.watchdog_timed_out_)
        ? 0 : clampGoalCurrent(servo.target_current_,
                               servo.current_limit_);
      appended = append_value(
        servo.id_, CTRL_GOAL_CURRENT, GOAL_CURRENT_BYTE_LEN,
        static_cast<uint16_t>(servo.goal_current_));
      break;
    case DIRECT_SERVO_MODE_MIT:
      servo.goal_current_ = calculateMITGoalCurrent(servo);
      appended = append_value(
        servo.id_, CTRL_GOAL_CURRENT, GOAL_CURRENT_BYTE_LEN,
        static_cast<uint16_t>(servo.goal_current_));
      break;
    default:
      break;
    }

    if (!appended) {
      if (servo.mode_ != DIRECT_SERVO_MODE_EM) {
        last_tx_status_ = HAL_ERROR;
      }
      return;
    }
    written_mask |= bit;
  }

  if (param_idx == 0) {
    last_tx_status_ = HAL_OK;
    return;
  }
  transmitInstructionPacket(
    DX_BROADCAST_ID, param_idx + 3, COMMAND_BULK_WRITE, parameters);
  if (last_tx_status_ != HAL_OK) return;

  // Protocol 2.0 Bulk Write permits each ID only once per packet. Send the
  // CbP positions in a second packet instead of repeating an ID after its
  // Goal Current entry above.
  param_idx = 0;
  for (uint8_t i = 0; i < servo_num_; ++i) {
    const ServoMask bit = servoBit(i);
    if ((cbp_mask & bit) == 0) continue;
    const ServoData& servo = servo_[i];
    if (!append_value(
          servo.id_, CTRL_GOAL_POSITION, GOAL_POSITION_BYTE_LEN,
          static_cast<uint32_t>(servo.goal_position_))) {
      last_tx_status_ = HAL_ERROR;
      return;
    }
  }
  if (param_idx != 0) {
    transmitInstructionPacket(
      DX_BROADCAST_ID, param_idx + 3, COMMAND_BULK_WRITE, parameters);
    if (last_tx_status_ != HAL_OK) return;
  }
  recordGoalWrite(written_mask);
}

void DynamixelSerial::recordGoalWrite(ServoMask servo_mask)
{
	if (last_tx_status_ != HAL_OK) return;

	for (unsigned int i = 0; i < servo_num_; ++i) {
		if ((servo_mask & servoBit(i)) == 0) continue;
	}
}

void DynamixelSerial::cmdSyncWriteGoalPosition(ServoMask servo_mask)
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];

	for (unsigned int i = 0; i < servo_num_; i++) {
		int32_t goal_position = servo_[i].getGoalPosition();
		parameters[i * 4 + 0] = (uint8_t)((int32_t)(goal_position) & 0xFF);
		parameters[i * 4 + 1] = (uint8_t)(((int32_t)(goal_position) >> 8) & 0xFF);
		parameters[i * 4 + 2] = (uint8_t)(((int32_t)(goal_position) >> 16) & 0xFF);
		parameters[i * 4 + 3] = (uint8_t)(((int32_t)(goal_position) >> 24) & 0xFF);
	}

	cmdSyncWrite(CTRL_GOAL_POSITION, parameters, GOAL_POSITION_BYTE_LEN,
	             servo_mask);
	recordGoalWrite(servo_mask);
}

void DynamixelSerial::cmdSyncWriteGoalVelocity(ServoMask servo_mask)
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];

	for (unsigned int i = 0; i < servo_num_; i++) {
		int32_t goal_velocity = servo_[i].getGoalVelocity();
		parameters[i * 4 + 0] = (uint8_t)((int32_t)(goal_velocity) & 0xFF);
		parameters[i * 4 + 1] = (uint8_t)(((int32_t)(goal_velocity) >> 8) & 0xFF);
		parameters[i * 4 + 2] = (uint8_t)(((int32_t)(goal_velocity) >> 16) & 0xFF);
		parameters[i * 4 + 3] = (uint8_t)(((int32_t)(goal_velocity) >> 24) & 0xFF);
	}

	cmdSyncWrite(CTRL_GOAL_VELOCITY, parameters, GOAL_VELOCITY_BYTE_LEN,
	             servo_mask);
	recordGoalWrite(servo_mask);
}

void DynamixelSerial::cmdSyncWriteGoalCurrent(ServoMask servo_mask)
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];

	for (unsigned int i = 0; i < servo_num_; i++) {
		int16_t goal_current = servo_[i].getGoalCurrent();
		parameters[i * 2 + 0] = (uint8_t)((int16_t)(goal_current) & 0xFF);
		parameters[i * 2 + 1] = (uint8_t)(((int16_t)(goal_current) >> 8) & 0xFF);
	}

	cmdSyncWrite(CTRL_GOAL_CURRENT, parameters, GOAL_CURRENT_BYTE_LEN,
	             servo_mask);
	recordGoalWrite(servo_mask);
}

void DynamixelSerial::cmdSyncWriteLed()
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];
	for (unsigned int i = 0; i < servo_num_; i++) {
		parameters[i] = (uint8_t)((servo_[i].led_) ? 1 : 0);
	}

	cmdSyncWrite(CTRL_LED, parameters, LED_BYTE_LEN, allServoMask());
}

void DynamixelSerial::cmdSyncWritePositionGains()
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];
        /*p_gain*/
        for (unsigned int i = 0; i < servo_num_; i++) {
          parameters[i * 2 + 0] = servo_[i].p_gain_ & 0xFF;
          parameters[i * 2 + 1] = (servo_[i].p_gain_ >> 8) & 0xFF;
	}
	cmdSyncWrite(CTRL_POSITION_P_GAIN, parameters, 2, allServoMask());

        /*i_gain*/
        for (unsigned int i = 0; i < servo_num_; i++) {
          parameters[i * 2 + 0] = servo_[i].i_gain_ & 0xFF;
          parameters[i * 2 + 1] = (servo_[i].i_gain_ >> 8) & 0xFF;
	}
	cmdSyncWrite(CTRL_POSITION_I_GAIN, parameters, 2, allServoMask());

        /*d_gain*/
        for (unsigned int i = 0; i < servo_num_; i++) {
          parameters[i * 2 + 0] = servo_[i].d_gain_ & 0xFF;
          parameters[i * 2 + 1] = (servo_[i].d_gain_ >> 8) & 0xFF;
	}
	cmdSyncWrite(CTRL_POSITION_D_GAIN, parameters, 2, allServoMask());
}

void DynamixelSerial::cmdSyncWriteProfileVelocity()
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];

	for (unsigned int i = 0; i < servo_num_; i++) {
		parameters[i * 4 + 0] = (uint8_t)((int32_t)(servo_[i].profile_velocity_) & 0xFF);
		parameters[i * 4 + 1] = (uint8_t)(((int32_t)(servo_[i].profile_velocity_) >> 8) & 0xFF);
		parameters[i * 4 + 2] = (uint8_t)(((int32_t)(servo_[i].profile_velocity_) >> 16) & 0xFF);
		parameters[i * 4 + 3] = (uint8_t)(((int32_t)(servo_[i].profile_velocity_) >> 24) & 0xFF);
	}

	cmdSyncWrite(CTRL_PROFILE_VELOCITY, parameters, PROFILE_VELOCITY_BYTE_LEN, allServoMask());
}

void DynamixelSerial::cmdSyncWriteTorqueEnable()
{
	uint8_t parameters[INSTRUCTION_PACKET_SIZE];

	for (unsigned int i = 0; i < servo_num_; i++) {
		parameters[i] = servo_[i].torque_enable_ ? 1U : 0U;
	}

	cmdSyncWrite(CTRL_TORQUE_ENABLE, parameters, TORQUE_ENABLE_BYTE_LEN, allServoMask());
}

ServoResult DynamixelSerial::readOneServoRegister(uint8_t instruction,
                                                  uint8_t servo_index)
{
  if (servo_index >= servo_num_) {
    return SERVO_RESULT_INVALID_ARGUMENT;
  }
  if (pending_transaction_.active) {
    return SERVO_RESULT_BUSY;
  }

  switch (instruction) {
  case INST_GET_HOMING_OFFSET:
    cmdReadHomingOffset(servo_index);
    break;
  case INST_GET_POSITION_GAINS:
    cmdReadPositionGains(servo_index);
    break;
  case INST_GET_PROFILE_VELOCITY:
    cmdReadProfileVelocity(servo_index);
    break;
  case INST_GET_CURRENT_LIMIT:
    cmdReadCurrentLimit(servo_index);
    break;
  case INST_GET_OPERATING_MODE:
    cmdReadOperatingMode(servo_index);
    break;
  default:
    return SERVO_RESULT_INVALID_ARGUMENT;
  }

  return executeBlockingRead(
    {instruction, servoBit(servo_index), 0}, false);
}

ServoResult DynamixelSerial::readAllServoRegister(uint8_t instruction)
{
  if (servo_num_ == 0 || pending_transaction_.active) {
    return SERVO_RESULT_BUSY;
  }

  for (uint8_t i = 0; i < servo_num_; ++i) {
    const ServoResult result = readOneServoRegister(instruction, i);
    if (result != SERVO_RESULT_OK) {
      return result;
    }
  }
  return SERVO_RESULT_OK;
}

ServoResult DynamixelSerial::applyHomingOffset(uint8_t servo_index)
{
  if (servo_index >= servo_num_) {
    return SERVO_RESULT_INVALID_ARGUMENT;
  }

  ServoData& servo = servo_[servo_index];
  if (servo.external_encoder_flag_) {
    servo.joint_offset_ = servo.calib_value_ + servo.joint_offset_ -
                          servo.present_position_;
#ifndef SPINAL
    servo.joint_offset_ = servo.calib_value_;
    encoder_handler_.setOffset(servo.joint_offset_);
#endif
    servo.first_get_pos_flag_ = true;
    ServoFlashConfig* config = findServoFlashConfig(servo.id_);
    if (config == nullptr) return SERVO_RESULT_INVALID_ARGUMENT;
    copyServoToFlashConfig(servo, *config);
    FlashMemory::erase();
    FlashMemory::write();
    return SERVO_RESULT_OK;
  }

  servo.homing_offset_ = 0;
  servo.internal_offset_ = 0;
  cmdWriteHomingOffset(servo_index);
  ServoResult result = lastTransmitResult();
  if (result != SERVO_RESULT_OK) return result;

  cmdReadHomingOffset(servo_index);
  result = executeBlockingRead(
    {INST_GET_HOMING_OFFSET, servoBit(servo_index), 0}, false);
  if (result != SERVO_RESULT_OK) return result;

  cmdReadPresentPosition(servo_index);
  result = executeBlockingRead(
    {INST_GET_PRESENT_POS, servoBit(servo_index), 0}, false);
  if (result != SERVO_RESULT_OK) return result;

  servo.updateHomingOffset();
  cmdWriteHomingOffset(servo_index);
  result = lastTransmitResult();
  if (result != SERVO_RESULT_OK) return result;

  cmdReadHomingOffset(servo_index);
  result = executeBlockingRead(
    {INST_GET_HOMING_OFFSET, servoBit(servo_index), 0}, false);
  if (result != SERVO_RESULT_OK) return result;

  cmdReadPresentPosition(servo_index);
  return executeBlockingRead(
    {INST_GET_PRESENT_POS, servoBit(servo_index), 0}, false);
}

ServoResult DynamixelSerial::applyPositionGains(uint8_t servo_index)
{
  if (servo_index >= servo_num_) {
    return SERVO_RESULT_INVALID_ARGUMENT;
  }
  cmdWritePositionGains(servo_index);
  return lastTransmitResult();
}

ServoResult DynamixelSerial::applyProfileVelocity(uint8_t servo_index)
{
  if (servo_index >= servo_num_) {
    return SERVO_RESULT_INVALID_ARGUMENT;
  }
  cmdWriteProfileVelocity(servo_index);
  return lastTransmitResult();
}

ServoResult DynamixelSerial::applyCurrentLimit(uint8_t servo_index)
{
  if (servo_index >= servo_num_) {
    return SERVO_RESULT_INVALID_ARGUMENT;
  }
  cmdWriteCurrentLimit(servo_index);
  const ServoResult result = lastTransmitResult();
  return result == SERVO_RESULT_OK
    ? readOneServoRegister(INST_GET_CURRENT_LIMIT, servo_index) : result;
}

ServoResult DynamixelSerial::applyOperatingMode(uint8_t servo_index,
                                                uint8_t mode)
{
  if (servo_index >= servo_num_ || !validLogicalMode(mode)) {
    return SERVO_RESULT_INVALID_ARGUMENT;
  }

  ServoData& servo = servo_[servo_index];
  if (mode == DIRECT_SERVO_MODE_EM) {
    servo.mode_ = mode;
    servo.resetGoalTargets();
    if (servo.torque_enable_) {
      servo.torque_enable_ = false;
      __atomic_fetch_or(&torque_off_pending_mask_, 1UL << servo_index,
                        __ATOMIC_RELEASE);
    }
    enforceConfigurationSafety();
    return SERVO_RESULT_OK;
  }
  if (servo.torque_enable_) return SERVO_RESULT_BUSY;

  const uint8_t operating_mode = expectedOperatingMode(mode);
  ServoResult result =
    readOneServoRegister(INST_GET_OPERATING_MODE, servo_index);
  if (result != SERVO_RESULT_OK) return result;

  if (servo.operating_mode_ != operating_mode) {
    cmdWriteOperatingMode(servo_index, operating_mode);
    result = lastTransmitResult();
    if (result != SERVO_RESULT_OK) return result;
    result = readOneServoRegister(INST_GET_OPERATING_MODE, servo_index);
    if (result != SERVO_RESULT_OK) return result;
  }
  if (servo.operating_mode_ != operating_mode) {
    return SERVO_RESULT_INVALID_RESPONSE;
  }

  servo.mode_ = mode;
  servo.resetGoalTargets();
  enforceConfigurationSafety();
  return SERVO_RESULT_OK;
}

ServoFlashConfig* DynamixelSerial::findServoFlashConfig(
  uint8_t servo_id, bool* duplicate)
{
  if (duplicate != nullptr) *duplicate = false;
  ServoFlashConfig* match = nullptr;
  for (uint8_t slot = 0; slot < MAX_SERVO_NUM; ++slot) {
    if (!validFlashConfig(servo_flash_configs_[slot]) ||
        servo_flash_configs_[slot].id != servo_id) {
      continue;
    }
    if (match != nullptr) {
      if (duplicate != nullptr) *duplicate = true;
      return nullptr;
    }
    match = &servo_flash_configs_[slot];
  }
  return match;
}

void DynamixelSerial::copyFlashConfigToServo(
  const ServoFlashConfig& config,
  ServoData& servo)
{
  servo.p_gain_ = config.p_gain;
  servo.i_gain_ = config.i_gain;
  servo.d_gain_ = config.d_gain;
  servo.profile_velocity_ = config.profile_velocity;
  servo.bus_type_ = config.bus_type;
  servo.mode_ = config.mode;
  servo.send_data_flag_ = (config.flags & SERVO_FLASH_FLAG_SEND_DATA) != 0;
  servo.external_encoder_flag_ =
    (config.flags & SERVO_FLASH_FLAG_EXTERNAL_ENCODER) != 0;
  servo.joint_resolution_ = config.joint_resolution;
  servo.servo_resolution_ = config.servo_resolution;
  servo.joint_offset_ = config.joint_offset;
  servo.configured_ = false;
}

void DynamixelSerial::copyServoToFlashConfig(
  const ServoData& servo,
  ServoFlashConfig& config)
{
  config.p_gain = servo.p_gain_;
  config.i_gain = servo.i_gain_;
  config.d_gain = servo.d_gain_;
  config.profile_velocity = servo.profile_velocity_;
  config.id = servo.id_;
  config.bus_type = servo.bus_type_;
  config.mode = servo.mode_;
  config.flags =
    (servo.send_data_flag_ ? SERVO_FLASH_FLAG_SEND_DATA : 0) |
    (servo.external_encoder_flag_ ? SERVO_FLASH_FLAG_EXTERNAL_ENCODER : 0);
  config.joint_resolution = servo.joint_resolution_;
  config.servo_resolution = servo.servo_resolution_;
  config.joint_offset = servo.joint_offset_;
}

ServoFlashConfig* DynamixelSerial::findOrCreateServoFlashConfig(
  uint8_t servo_index, uint8_t bus_type)
{
  if (servo_index >= servo_num_ ||
      (bus_type != SERVO_BUS_RS485 && bus_type != SERVO_BUS_TTL)) {
    return nullptr;
  }

  ServoFlashConfig* selected = nullptr;
  ServoFlashConfig* empty = nullptr;
  ServoData& servo = servo_[servo_index];
  for (uint8_t slot = 0; slot < MAX_SERVO_NUM; ++slot) {
    ServoFlashConfig& config = servo_flash_configs_[slot];
    if (!validFlashConfig(config)) {
      if (empty == nullptr) empty = &config;
      continue;
    }

    if (config.id == servo.id_) {
      if (selected != nullptr) return nullptr;
      selected = &config;
      continue;
    }

    uint8_t detected_index = 0;
    if (empty == nullptr && !findServoIndex(config.id, detected_index)) {
      empty = &config;
    }
  }

  const bool create = selected == nullptr;
  if (create) selected = empty;
  if (selected == nullptr) return nullptr;

  servo.bus_type_ = bus_type;
  if (create) {
    copyServoToFlashConfig(servo, *selected);
    selected->mode = DIRECT_SERVO_MODE_UNKNOWN;
  } else {
    selected->bus_type = bus_type;
  }
  return selected;
}

ServoResult DynamixelSerial::executeServiceRequest(
  const ServoServiceRequest& request)
{
  if (!validateServiceRequest(request.command, request.data,
                              request.data_length)) {
    return SERVO_RESULT_INVALID_ARGUMENT;
  }

  if (request.command == CONFIG_SET_TTL_RS485_MIXED) {
    const uint16_t requested = request.data[0] ? 1 : 0;
    if (ttl_rs485_mixed_ == requested) return SERVO_RESULT_OK;
    for (uint8_t i = 0; i < servo_num_; ++i) {
      if (servo_[i].torque_enable_) return SERVO_RESULT_BUSY;
    }
    ttl_rs485_mixed_ = requested;
    enforceConfigurationSafety();
    FlashMemory::erase();
    FlashMemory::write();
    return SERVO_RESULT_OK;
  }
  if (request.command == CONFIG_SET_FEEDBACK_FREQUENCY) {
    servo_feedback_frequency_hz_ =
      static_cast<uint16_t>(request.data[0]);
    return SERVO_RESULT_OK;
  }
  if (request.command == CONFIG_SET_TARGET_WATCHDOG) {
    target_watchdog_timeout_ms_ =
      static_cast<uint32_t>(request.data[0]);
    return SERVO_RESULT_OK;
  }

  uint8_t servo_index = 0;
  if (!findServoIndex(static_cast<uint8_t>(request.data[0]),
                      servo_index)) {
    return SERVO_RESULT_INVALID_ARGUMENT;
  }
  ServoData& servo = servo_[servo_index];

  switch (request.command) {
  case CONFIG_SET_BUS_TYPE:
  {
    if (servo.torque_enable_) return SERVO_RESULT_BUSY;
    const uint8_t requested_bus = static_cast<uint8_t>(request.data[1]);
    const ServoFlashConfig* existing = findServoFlashConfig(servo.id_);
    const bool flash_changed =
      existing == nullptr || existing->bus_type != requested_bus;
    ServoFlashConfig* config =
      findOrCreateServoFlashConfig(servo_index, requested_bus);
    if (config == nullptr) {
      return SERVO_RESULT_INVALID_ARGUMENT;
    }
    if (flash_changed) {
      FlashMemory::erase();
      FlashMemory::write();
    }
    enforceConfigurationSafety();
    return SERVO_RESULT_OK;
  }

  case CONFIG_SET_CONTROL_MODE:
  {
    if (servo.torque_enable_) return SERVO_RESULT_BUSY;
    ServoFlashConfig* config = findServoFlashConfig(servo.id_);
    if (config == nullptr) return SERVO_RESULT_INVALID_ARGUMENT;
    const uint8_t requested_mode = static_cast<uint8_t>(request.data[1]);
    const bool flash_changed = config->mode != requested_mode;
    ServoResult result = applyOperatingMode(servo_index, requested_mode);
    if (result == SERVO_RESULT_OK) result = applyPositionGains(servo_index);
    if (result == SERVO_RESULT_OK) result = applyProfileVelocity(servo_index);
    if (result != SERVO_RESULT_OK) {
      servo.configured_ = false;
      enforceConfigurationSafety();
      return result;
    }

    copyServoToFlashConfig(servo, *config);
    if (flash_changed) {
      FlashMemory::erase();
      FlashMemory::write();
    }
    servo.configured_ = true;
    enforceConfigurationSafety();
    return SERVO_RESULT_OK;
  }

  case CONFIG_SET_MIT_PD:
    if (servo.torque_enable_) return SERVO_RESULT_BUSY;
    servo.kp_integer_ = static_cast<int16_t>(request.data[1]);
    servo.kp_decimal_ = static_cast<int16_t>(request.data[2]);
    servo.kd_integer_ = static_cast<int16_t>(request.data[3]);
    servo.kd_decimal_ = static_cast<int16_t>(request.data[4]);
    return SERVO_RESULT_OK;

  case CONFIG_SET_HOMING_OFFSET:
    if (servo.torque_enable_) return SERVO_RESULT_BUSY;
    servo.calib_value_ = request.data[1];
    return applyHomingOffset(servo_index);

  case CONFIG_SET_PID_GAIN:
  {
    if (servo.torque_enable_) return SERVO_RESULT_BUSY;
    ServoFlashConfig* config = findServoFlashConfig(servo.id_);
    if (config == nullptr) return SERVO_RESULT_INVALID_ARGUMENT;
    if (servo.p_gain_ == request.data[1] &&
        servo.i_gain_ == request.data[2] &&
        servo.d_gain_ == request.data[3]) {
      return SERVO_RESULT_OK;
    }
    const uint16_t old_p_gain = servo.p_gain_;
    const uint16_t old_i_gain = servo.i_gain_;
    const uint16_t old_d_gain = servo.d_gain_;
    servo.p_gain_ = request.data[1];
    servo.i_gain_ = request.data[2];
    servo.d_gain_ = request.data[3];
    const ServoResult result = applyPositionGains(servo_index);
    if (result == SERVO_RESULT_OK) {
      copyServoToFlashConfig(servo, *config);
      FlashMemory::erase();
      FlashMemory::write();
    } else {
      servo.p_gain_ = old_p_gain;
      servo.i_gain_ = old_i_gain;
      servo.d_gain_ = old_d_gain;
    }
    return result;
  }

  case CONFIG_SET_PROFILE_VELOCITY:
  {
    ServoFlashConfig* config = findServoFlashConfig(servo.id_);
    if (config == nullptr) return SERVO_RESULT_INVALID_ARGUMENT;
    if (servo.profile_velocity_ == request.data[1]) {
      return SERVO_RESULT_OK;
    }
    const uint16_t old_profile_velocity = servo.profile_velocity_;
    servo.profile_velocity_ = request.data[1];
    const ServoResult result = applyProfileVelocity(servo_index);
    if (result == SERVO_RESULT_OK) {
      copyServoToFlashConfig(servo, *config);
      FlashMemory::erase();
      FlashMemory::write();
    } else {
      servo.profile_velocity_ = old_profile_velocity;
    }
    return result;
  }

  case CONFIG_SET_SEND_DATA_FLAG:
  {
    ServoFlashConfig* config = findServoFlashConfig(servo.id_);
    if (config == nullptr) return SERVO_RESULT_INVALID_ARGUMENT;
    if (servo.send_data_flag_ == request.data[1]) {
      return SERVO_RESULT_OK;
    }
    servo.send_data_flag_ = request.data[1];
    copyServoToFlashConfig(servo, *config);
    FlashMemory::erase();
    FlashMemory::write();
    return SERVO_RESULT_OK;
  }

  case CONFIG_SET_CURRENT_LIMIT:
  {
    if (servo.torque_enable_) return SERVO_RESULT_BUSY;
    if (servo.current_limit_ == request.data[1]) {
      return SERVO_RESULT_OK;
    }
    const uint16_t old_current_limit = servo.current_limit_;
    servo.current_limit_ = request.data[1];
    const ServoResult result = applyCurrentLimit(servo_index);
    if (result != SERVO_RESULT_OK) {
      servo.current_limit_ = old_current_limit;
    } else if (servo.mode_ == DIRECT_SERVO_MODE_CBP) {
      servo.target_current_ = clampGoalCurrent(
        static_cast<uint32_t>(servo.current_limit_) * 4U / 5U,
        servo.current_limit_);
      servo.goal_current_ = servo.target_current_;
    }
    return result;
  }

  case CONFIG_SET_EXTERNAL_ENCODER_FLAG:
  {
    if (servo.torque_enable_) return SERVO_RESULT_BUSY;
    ServoFlashConfig* config = findServoFlashConfig(servo.id_);
    if (config == nullptr) return SERVO_RESULT_INVALID_ARGUMENT;
    if (servo.external_encoder_flag_ == request.data[1]) {
      return SERVO_RESULT_OK;
    }
    servo.external_encoder_flag_ = request.data[1];
    servo.first_get_pos_flag_ = true;
    if (!servo.external_encoder_flag_) {
      servo.servo_resolution_ = 1;
      servo.joint_resolution_ = 1;
      servo.resolution_ratio_ = 1;
    }
    copyServoToFlashConfig(servo, *config);
    FlashMemory::erase();
    FlashMemory::write();
    return SERVO_RESULT_OK;
  }

  case CONFIG_SET_RESOLUTION_RATIO:
  {
    if (servo.torque_enable_) return SERVO_RESULT_BUSY;
    ServoFlashConfig* config = findServoFlashConfig(servo.id_);
    if (config == nullptr) return SERVO_RESULT_INVALID_ARGUMENT;
    if (servo.joint_resolution_ == request.data[1] &&
        servo.servo_resolution_ == request.data[2]) {
      return SERVO_RESULT_OK;
    }
    servo.joint_resolution_ = request.data[1];
    servo.servo_resolution_ = request.data[2];
    servo.hardware_error_status_ &= ((1 << RESOLUTION_RATIO_ERROR) - 1);
    servo.resolution_ratio_ = static_cast<float>(servo.servo_resolution_) /
                              static_cast<float>(servo.joint_resolution_);
    servo.first_get_pos_flag_ = true;
    copyServoToFlashConfig(servo, *config);
    FlashMemory::erase();
    FlashMemory::write();
    return SERVO_RESULT_OK;
  }

  default:
    return SERVO_RESULT_INVALID_ARGUMENT;
  }
}


void DynamixelSerial::setStatusReturnLevel()
{
	cmdWriteStatusReturnLevel(DX_BROADCAST_ID, READ);
}

void DynamixelSerial::getHomingOffset()
{
	(void)readAllServoRegister(INST_GET_HOMING_OFFSET);
}

void DynamixelSerial::getCurrentLimit()
{
	(void)readAllServoRegister(INST_GET_CURRENT_LIMIT);
}

void DynamixelSerial::getPositionGains()
{
	(void)readAllServoRegister(INST_GET_POSITION_GAINS);
}

void DynamixelSerial::getProfileVelocity()
{
	(void)readAllServoRegister(INST_GET_PROFILE_VELOCITY);
}

void DynamixelSerial::getOperatingMode()
{
	(void)readAllServoRegister(INST_GET_OPERATING_MODE);
}

static const uint16_t crc_table[256] = {
          0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
          0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
          0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
          0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
          0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
          0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
          0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
          0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
          0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
          0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
          0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
          0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
          0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
          0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
          0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
          0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
          0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
          0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
          0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
          0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
          0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
          0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
          0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
          0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
          0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
          0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
          0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
          0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
          0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
          0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
          0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
          0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
      	};

uint16_t DynamixelSerial::calcCRC16(uint16_t crc_accum, uint8_t *data_blk_ptr, int data_blk_size)
{
	for(int j = 0; j < data_blk_size; j++) {
		int i = ((int)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}
	return crc_accum;
}

HAL_StatusTypeDef DynamixelSerial::readUntil(uint8_t* data, uint32_t deadline_ms)
{
  if (__HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE)) {
    return HAL_ERROR;
  }

  while (true) {
    const uint32_t dma_write_ptr =
      (RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % RX_BUFFER_SIZE;
    if (rd_ptr_ != dma_write_ptr) {
      *data = rx_buf_[rd_ptr_++];
      rd_ptr_ %= RX_BUFFER_SIZE;
      return HAL_OK;
    }

    if (deadlineExpired(HAL_GetTick(), deadline_ms)) {
      return HAL_TIMEOUT;
    }
  }
}

HAL_StatusTypeDef DynamixelSerial::read(uint8_t* data, uint32_t timeout)
{
  if (timeout == 0U) {
    return HAL_TIMEOUT;
  }
  return readUntil(data, HAL_GetTick() + timeout);
}
