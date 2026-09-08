/**
******************************************************************************
* File Name          : spine.cpp
* Description        : can-based internal comm network, spine side interface
 ------------------------------------------------------------------*/

#include "spine.h"

namespace Spine
{
  /* components */
  /* CAUTIONS: be careful about the order of the var definition and func definition */
  namespace
  {
    std::vector<Neuron> neuron_;
    CANMotorSendDevice can_motor_send_device_;
    std::vector<std::reference_wrapper<Servo>> servo_;
    std::vector<std::reference_wrapper<Servo>> servo_with_send_flag_;
    CANInitializer can_initializer_(neuron_);
    std::vector<float> imu_weight_;
    DirectServo* direct_servo_ = nullptr;
    DirectServoBoardSettings direct_servo_settings_;

    uint8_t slave_num_ = 0;
    uint8_t direct_servo_num_ = 0;
    uint8_t servo_num_ = 0;
    int8_t uav_model_ = -1;
    uint8_t baselink_ = 2;

    /* sensor fusion */
    StateEstimate* estimator_;

    /* flight controller */
    FlightControl* controller_;

    /* ros */
    constexpr uint32_t SERVO_TORQUE_PUB_INTERVAL = 1000; //[ms]
    spinal::ServoStates servo_state_msg_;
    spinal::ServoTorqueStates servo_torque_state_msg_;
    ros::Publisher servo_state_pub_("servo/states", &servo_state_msg_);
    // merge torque_states to states
    ros::Publisher servo_torque_state_pub_("servo/torque_states", &servo_torque_state_msg_);

    // rename following subscriber.
    // taget_states -> target_position
    // torque_enable -> control_enable
    ros::Subscriber<spinal::ServoControlCmd> servo_position_sub_("servo/target_states", servoPositionCallback);
    ros::Subscriber<spinal::ServoControlCmd> servo_current_sub_("servo/target_current", servoCurrentCallback);
    ros::Subscriber<spinal::ServoTorqueCmd> servo_torque_ctrl_sub_("servo/torque_enable", servoTorqueControlCallback);

    ros::ServiceServer<spinal::GetBoardInfo::Request, spinal::GetBoardInfo::Response> board_info_srv_("get_board_info", boardInfoCallback);
	    ros::ServiceServer<spinal::SetBoardConfig::Request, spinal::SetBoardConfig::Response> board_config_srv_("set_board_config", boardConfigCallback);
	    ros::ServiceServer<spinal::SetDirectServoConfig::Request,
	                       spinal::SetDirectServoConfig::Response>
	      servo_config_srv_("direct_servo_config", servoConfigCallback);

    spinal::GetBoardInfo::Response board_info_res_;

    ros::NodeHandle* nh_;
    std::vector<uint32_t> neuron_servo_last_pub_time_;
    uint32_t direct_servo_last_pub_time_ = 0;
    uint32_t servo_torque_last_pub_time_ = 0;
    unsigned int can_idle_count_ = 0;
    bool servo_control_flag_ = true;

    uint32_t can_tx_idle_start_time_ = 0; // for pause CAN TX -> TODO: change to another task for spinal process
    uint32_t CAN_TX_PAUSE_TIME = 2000; // 2000 ms for 1Khz task rate. TODO: change to another task for spinal process

    uint32_t last_connected_time_ =0;

    void fillDirectServoBoardInfo(spinal::BoardInfo& board)
    {
      board.slave_id = CAN::MASTER_ID;
      board.imu_send_data_flag = 1;
      board.dynamixel_ttl_rs485_mixed =
        direct_servo_settings_.dynamixel_ttl_rs485_mixed;
      board.servo_pulley_skip_thresh = 0;
      board.servo_feedback_frequency =
        direct_servo_settings_.servo_feedback_frequency;
      board.servo_target_watchdog_timeout =
        static_cast<float>(
          direct_servo_settings_.servo_target_watchdog_timeout_ms) /
        1000.0f;
      if (direct_servo_ != nullptr) direct_servo_->fillBoardInfo(board);
    }

    bool setDirectServoTargets(
      uint8_t global_index,
      bool has_position, int32_t position,
      bool has_velocity, int32_t velocity,
      bool has_current, int16_t current)
    {
      if (direct_servo_ == nullptr || global_index >= direct_servo_num_)
        return false;
      return direct_servo_->setGoalTargets(
        global_index, has_position, position, has_velocity, velocity,
        has_current, current);
    }

    bool setDirectServoTorque(uint8_t global_index, bool enable)
    {
      if (direct_servo_ == nullptr || global_index >= direct_servo_num_)
        return false;
      return direct_servo_->setTorqueEnable(global_index, enable);
    }
  }

  void boardInfoCallback(const spinal::GetBoardInfo::Request& req, spinal::GetBoardInfo::Response& res)
  {
    fillDirectServoBoardInfo(board_info_res_.boards[0]);
    for (unsigned int i = 0; i < slave_num_; i++) {
      Neuron& neuron = neuron_.at(i);
      spinal::BoardInfo& board = board_info_res_.boards[i + 1];
      board.imu_send_data_flag = neuron.can_imu_.getSendDataFlag() ? 1 : 0;
      board.dynamixel_ttl_rs485_mixed = neuron.can_servo_.getDynamixelTTLRS485Mixed() ? 1 : 0;
	      board.servo_pulley_skip_thresh = neuron.can_servo_.getPulleySkipThresh();
	      board.servo_feedback_frequency =
	        neuron.can_servo_.getServoFeedbackFrequency();
	      board.servo_target_watchdog_timeout =
	        static_cast<float>(
	          neuron.can_servo_.getTargetWatchdogTimeoutMs()) / 1000.0f;
      board.slave_id = neuron.getSlaveId();

      for (unsigned int j = 0; j < board.servos_length; j++) {
        Servo& s = neuron.can_servo_.servo_.at(j);
	        board.servos[j].id = s.getId();
	        board.servos[j].configured = s.getConfigured();
	        board.servos[j].bus_type = s.getBusType();
	        board.servos[j].mode = s.getMode();
	        board.servos[j].servo_mode = s.getOperatingMode();
	        board.servos[j].kp_integer = s.getKpInteger();
	        board.servos[j].kp_decimal = s.getKpDecimal();
	        board.servos[j].kd_integer = s.getKdInteger();
	        board.servos[j].kd_decimal = s.getKdDecimal();
        board.servos[j].p_gain = s.getPGain();
        board.servos[j].i_gain = s.getIGain();
        board.servos[j].d_gain = s.getDGain();
        board.servos[j].profile_velocity = s.getProfileVelocity();
        board.servos[j].current_limit = s.getCurrentLimit();
        board.servos[j].send_data_flag = s.getSendDataFlag() ? 1 : 0;
        board.servos[j].external_encoder_flag = s.getExternalEncoderFlag() ? 1 : 0;
        board.servos[j].joint_resolution = s.getJointResolution();
        board.servos[j].servo_resolution = s.getServoResolution();
      }
    }
    res = board_info_res_;
  }

  void servoPositionCallback(const spinal::ServoControlCmd& control_msg)
  {
    if (!servo_control_flag_) return;
    const bool has_position = control_msg.position_length != 0;
    const bool has_velocity = control_msg.velocity_length != 0;
    const bool has_current = control_msg.current_length != 0;
    if (control_msg.index_length == 0 ||
        (!has_position && !has_velocity && !has_current) ||
        (has_position &&
         control_msg.index_length != control_msg.position_length) ||
        (has_velocity &&
         control_msg.index_length != control_msg.velocity_length) ||
        (has_current &&
         control_msg.index_length != control_msg.current_length)) {
      return;
    }

    for (unsigned int i = 0; i < control_msg.index_length; ++i) {
      if (control_msg.index[i] >= servo_num_) return;
    }
    for (unsigned int i = 0; i < control_msg.index_length; ++i) {
      const uint8_t global_index = control_msg.index[i];
      if (global_index < direct_servo_num_) {
        if (!setDirectServoTargets(
              global_index,
              has_position, has_position ? control_msg.position[i] : 0,
              has_velocity, has_velocity ? control_msg.velocity[i] : 0,
              has_current, has_current ? control_msg.current[i] : 0)) {
          return;
        }
        continue;
      }
      Servo& servo = servo_.at(global_index - direct_servo_num_).get();
      if (has_position) servo.setGoalPosition(control_msg.position[i]);
      if (has_velocity) servo.setGoalVelocity(control_msg.velocity[i]);
      if (has_current) servo.setGoalCurrent(control_msg.current[i]);
    }
  }

  void servoCurrentCallback(const spinal::ServoControlCmd& control_msg)
  {
    if (!servo_control_flag_) return;
    if (control_msg.index_length != control_msg.current_length) return;
    for (unsigned int i = 0; i < control_msg.index_length; ++i) {
      if (control_msg.index[i] >= servo_num_) return;
    }
    for (unsigned int i = 0; i < control_msg.index_length; i++) {
      const uint8_t global_index = control_msg.index[i];
      if (global_index < direct_servo_num_) {
        if (!setDirectServoTargets(
              global_index, false, 0, false, 0,
              true, control_msg.current[i])) return;
        continue;
      }
      Servo& servo = servo_.at(global_index - direct_servo_num_).get();
      servo.setGoalCurrent(control_msg.current[i]);
    }
  }

  void servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg)
  {
    if (control_msg.index_length != control_msg.torque_enable_length) return;
    for (unsigned int i = 0; i < control_msg.index_length; ++i) {
      if (control_msg.index[i] >= servo_num_) return;
    }
    for (unsigned int i = 0; i < control_msg.index_length; i++) {
      const uint8_t global_index = control_msg.index[i];
      const bool enable = control_msg.torque_enable[i] != 0;
      if (global_index < direct_servo_num_) {
        if (!setDirectServoTorque(global_index, enable)) return;
        continue;
      }
      Servo& servo = servo_.at(global_index - direct_servo_num_).get();
      servo.setTorqueEnable(enable);

      /* update the target angle */
      if (servo.getSendDataFlag()) {
        servo.setGoalPosition(servo.getPresentPosition());
      }
    }
  }

	  void boardConfigCallback(const spinal::SetBoardConfig::Request& req, spinal::SetBoardConfig::Response& res)
  {
    res.success = false;
    if (req.data_length == 0) return;

    Neuron* target = nullptr;
    const uint8_t slave_id = static_cast<uint8_t>(req.data[0]);
    for (Neuron& neuron : neuron_) {
      if (neuron.getSlaveId() == slave_id) {
        target = &neuron;
        break;
      }
    }
    if (target == nullptr) return;
    if (req.command == spinal::SetBoardConfig::Request::SET_IMU_SEND_FLAG &&
        req.data_length < 2) return;

    /* Pause the spinal sending command for neuron to have enough time for flashmemory erase&write */
    can_tx_idle_start_time_ = HAL_GetTick();
    // TODO: change the return value to bool
    can_initializer_.configDevice(req);

    // TODO: please add string type message for consoling
	    res.success = true;
	  }

	  void servoConfigCallback(
	    const spinal::SetDirectServoConfig::Request& req,
	    spinal::SetDirectServoConfig::Response& res)
	  {
	    res.success = false;
	    res.data_length = 0;
	    if (req.data_length > SERVO_SERVICE_DATA_CAPACITY) return;
	    const bool board_command =
	      req.command == spinal::SetDirectServoConfig::Request::SET_DYNAMIXEL_TTL_RS485_MIXED ||
	      req.command == spinal::SetDirectServoConfig::Request::SET_SERVO_FEEDBACK_FREQUENCY ||
      req.command == spinal::SetDirectServoConfig::Request::SET_SERVO_TARGET_WATCHDOG_TIMEOUT;

      if (req.board_id == CAN::MASTER_ID) {
        if (direct_servo_ == nullptr) return;
        res.success = board_command
          ? direct_servo_->requestConfig(
              req.command, req.data,
              static_cast<uint8_t>(req.data_length))
          : direct_servo_->requestConfigByIndex(
              req.servo_index, req.command, req.data,
              static_cast<uint8_t>(req.data_length));
        return;
      }

	    CANServo* can_servo = nullptr;
	    for (Neuron& neuron : neuron_) {
	      if (neuron.getSlaveId() == req.board_id) {
	        can_servo = &neuron.can_servo_;
	        break;
	      }
	    }
	    if (can_servo == nullptr) return;

	    const uint8_t result = board_command
	      ? can_servo->requestConfig(
	          req.command, req.data,
	          static_cast<uint8_t>(req.data_length), 250)
	      : can_servo->requestConfigByIndex(
	          req.servo_index, req.command, req.data,
	          static_cast<uint8_t>(req.data_length), 250);
	    res.success = result == CAN::SERVO_CONFIG_RESULT_OK;
	  }

  bool init(CAN_GeranlHandleTypeDef* hcan, ros::NodeHandle* nh,
            StateEstimate* estimator, FlightControl* controller,
            GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,
            DirectServo* direct_servo,
            const DirectServoBoardSettings* direct_servo_settings)
  {
    /* CAN */
    CANDeviceManager::init(hcan, GPIOx, GPIO_Pin);

    /* Estimation */
    estimator_ = estimator;

    /* Control */
    controller_ = controller;
    direct_servo_ = direct_servo;
    direct_servo_settings_ = direct_servo_settings == nullptr
      ? DirectServoBoardSettings() : *direct_servo_settings;
    direct_servo_num_ = direct_servo_ == nullptr
      ? 0 : direct_servo_->getServoNum();

    HAL_Delay(5000); //wait neuron initialization
    CANDeviceManager::addDevice(can_initializer_);
    CANDeviceManager::CAN_START();
    can_initializer_.initDevices();

    slave_num_ = neuron_.size();
    if (slave_num_ > CAN::CANFD_MAX_NEURON_NUM) return false;
    neuron_servo_last_pub_time_.assign(slave_num_, 0);

    //add CAN devices to CANDeviceManager
    for (unsigned int i = 0; i < neuron_.size(); i++) {
      CANDeviceManager::addDevice(neuron_.at(i).can_motor_);
      can_motor_send_device_.addMotor(neuron_.at(i).can_motor_);
      CANDeviceManager::addDevice(neuron_.at(i).can_imu_);
      CANDeviceManager::addDevice(neuron_.at(i).can_servo_);
      for (unsigned int j = 0; j < neuron_.at(i).can_servo_.servo_.size(); j++) {
        neuron_.at(i).can_servo_.servo_.at(j).setIndex(
          direct_servo_num_ + servo_.size());
        servo_.push_back(neuron_.at(i).can_servo_.servo_.at(j));
        if (neuron_.at(i).can_servo_.servo_.at(j).getSendDataFlag()) {
          servo_with_send_flag_.push_back(neuron_.at(i).can_servo_.servo_.at(j));
        }
      }
    }
    servo_num_ = direct_servo_num_ + servo_.size();

    /* ros */
    nh_ = nh;

    if (servo_num_ > 0)
      {
        nh_->advertise(servo_state_pub_);
        nh_->advertise(servo_torque_state_pub_);
        nh_->subscribe(servo_position_sub_);
        nh_->subscribe(servo_current_sub_);
        nh_->subscribe(servo_torque_ctrl_sub_);
      }

	    nh_->advertiseService(board_info_srv_);
	    nh_->advertiseService(board_config_srv_);
	    nh_->advertiseService(servo_config_srv_);

    /* uav model: special rule based on the number of gimbals (no send data flag servos) */
    uint8_t gimbal_servo_num = servo_.size() - servo_with_send_flag_.size();

    /* TODO: not good case processing */
    if(gimbal_servo_num == 0)
      {
        uav_model_ = spinal::UavInfo::HYDRUS;
      }
    if(gimbal_servo_num  == slave_num_)
      {
        uav_model_ = spinal::UavInfo::HYDRUS_XI;
      }
    if(gimbal_servo_num  == 2 * slave_num_)
      {
        uav_model_ = spinal::UavInfo::DRAGON;
      }

    /* update controller */
    controller_->setUavModel(uav_model_);
    controller_->setMotorNumber(slave_num_);

    servo_state_msg_.servos_length = servo_num_;
    servo_state_msg_.servos = new spinal::ServoState[servo_num_];
    servo_torque_state_msg_.torque_enable_length = servo_num_;
    servo_torque_state_msg_.torque_enable = new uint8_t[servo_num_];

    /* other component */
    imu_weight_.resize(slave_num_ + 1);

    /* set IMU weights */
    // no fusion
    imu_weight_[0] = 1.0;
    for (uint i = 1; i < imu_weight_.size(); i++) imu_weight_[i] = 0.0;

    for (int i = 0; i < slave_num_; i++) {
      HAL_Delay(100);
      neuron_.at(i).can_imu_.init();

      IMU_ROS_CMD::addImu(&(neuron_.at(i).can_imu_));
    }

    //set response for get_board_info
    board_info_res_.boards_length = slave_num_ + 1;
    board_info_res_.boards = new spinal::BoardInfo[slave_num_ + 1];
    spinal::BoardInfo& direct_board = board_info_res_.boards[0];
    direct_board.servos_length = direct_servo_num_;
    direct_board.servos = direct_servo_num_ == 0
      ? nullptr : new spinal::ServoInfo[direct_servo_num_];
    for (unsigned int i = 0; i < slave_num_; i++) {
      Neuron& neuron = neuron_.at(i);
      spinal::BoardInfo& board = board_info_res_.boards[i + 1];
      board.servos_length = neuron.can_servo_.servo_.size();
      board.servos = board.servos_length == 0
        ? nullptr : new spinal::ServoInfo[board.servos_length];
    }

    return slave_num_ > 0;
  }

  void send()
  {
    if (slave_num_ == 0) return;

    const uint32_t now = HAL_GetTick();
    if (now < can_tx_idle_start_time_ + CAN_TX_PAUSE_TIME) return;

    if (now % 2 == 0) {
      // Aggregate all five Neuron rotor targets at 500 Hz.
      can_motor_send_device_.sendData();
    }

    // A 1 kHz base schedule with five slots gives each Neuron 200 Hz.
    const uint8_t servo_slot = now % CAN::CANFD_MAX_NEURON_NUM;
    if (servo_slot < slave_num_) {
      neuron_.at(servo_slot).can_servo_.sendData();
    }

    can_initializer_.sendData(); // if necessary
  }

  void update(void)
  {
    /* Publish the unified DirectServo + Neuron state stream. */
    servoPublish();
    if (slave_num_ == 0) return;

    /* update the motor PWM command */
    for(int i = 0; i < slave_num_; i++) {
      float pwm_rate = controller_->getTargetPwm(i);
      uint16_t pwm_bit = pwm_rate * 2000 - 1000;
      neuron_.at(i).can_motor_.setPwm(pwm_bit);
    }

    /* uodate IMU */
    for (int i = 0; i < slave_num_; i++)
      neuron_.at(i).can_imu_.update();

    CANDeviceManager::tick(1);

    uint32_t now_time = HAL_GetTick();
    if(CANDeviceManager::connected()) last_connected_time_ = now_time;

    if(now_time - last_connected_time_ > 1000 /* ms */)
      {
        if(nh_->connected()) nh_->logerror("CAN disconnected!!");
        last_connected_time_ = now_time;
      }
  }

  void useRTOS(osMailQId* handle)
  {
    CANDeviceManager::useRTOS(handle);
  }

  void setMotorPwm(uint16_t pwm, uint8_t motor)
  {
    if(slave_num_ == 0) {
      return;
    }
    neuron_.at(motor).can_motor_.setPwm(pwm);
  }

  bool connected()
  {
    if (slave_num_ > 0) return true;

    return false;
  }

  uint8_t getSlaveNum()
  {
    return slave_num_;
  }

  int8_t getUavModel()
  {
    return uav_model_;
  }

  void setServoControlFlag(bool flag)
  {
    servo_control_flag_ = flag;
  }

  void servoPublish()
  {
    if (servo_num_ == 0) return;

    uint32_t now_time = HAL_GetTick();
    bool publish_neuron = false;
    for (unsigned int i = 0; i < neuron_.size(); ++i) {
      const CANServo& can_servo = neuron_.at(i).can_servo_;
      const uint16_t frequency = can_servo.getServoFeedbackFrequency();
      const uint32_t interval = frequency == 0 ? UINT32_MAX :
        (1000U + frequency - 1U) / frequency;
      if (!can_servo.servo_.empty() && frequency != 0 &&
          now_time - neuron_servo_last_pub_time_.at(i) >= interval) {
        publish_neuron = true;
      }
    }
    bool publish_direct = false;
    if (direct_servo_ != nullptr && direct_servo_->hasFreshState()) {
      const uint16_t frequency = direct_servo_->getServoFeedbackFrequency();
      const uint32_t interval = frequency == 0 ? UINT32_MAX :
        (1000U + frequency - 1U) / frequency;
      publish_direct = frequency != 0 &&
        now_time - direct_servo_last_pub_time_ >= interval;
    }
    if (publish_direct || publish_neuron)
      {
        /* send servo */
        servo_state_msg_.stamp = nh_->now();
        unsigned int output_index = 0;
        if (publish_direct) {
          for (unsigned int i = 0; i < direct_servo_num_; ++i) {
            const ServoData& direct = direct_servo_->getServoData(i);
            if (direct.send_data_flag_ == 0) continue;
            spinal::ServoState state;
            state.index = i;
            state.angle = direct.present_position_;
            state.velocity = direct.present_velocity;
            state.temp = direct.present_temp_;
            state.load = direct.present_current_;
            state.error = direct.hardware_error_status_;
            servo_state_msg_.servos[output_index++] = state;
          }
          direct_servo_->markStatePublished();
          direct_servo_last_pub_time_ = now_time;
        }
        if (publish_neuron) {
          for (unsigned int i = 0; i < neuron_.size(); ++i) {
            CANServo& can_servo = neuron_.at(i).can_servo_;
            const uint16_t frequency = can_servo.getServoFeedbackFrequency();
            const uint32_t interval = frequency == 0 ? UINT32_MAX :
              (1000U + frequency - 1U) / frequency;
            if (frequency == 0 ||
                now_time - neuron_servo_last_pub_time_.at(i) < interval) {
              continue;
            }
            for (Servo& neuron_servo : can_servo.servo_) {
              if (!neuron_servo.getSendDataFlag()) continue;
              spinal::ServoState state;
              state.index = neuron_servo.getIndex();
              state.angle = neuron_servo.getPresentPosition();
              state.velocity = neuron_servo.getPresentVelocity();
              state.temp = neuron_servo.getPresentTemperature();
              state.load = neuron_servo.getPresentCurrent();
              state.error = neuron_servo.getError();
              servo_state_msg_.servos[output_index++] = state;
            }
            neuron_servo_last_pub_time_.at(i) = now_time;
          }
        }
        servo_state_msg_.servos_length = output_index;

        if (output_index > 0) servo_state_pub_.publish(&servo_state_msg_);
      }

    if( now_time - servo_torque_last_pub_time_ >= SERVO_TORQUE_PUB_INTERVAL)
      {
        for (unsigned int i = 0; i < direct_servo_num_; ++i) {
          servo_torque_state_msg_.torque_enable[i] =
            direct_servo_->getServoData(i).torque_enable_ ? 1 : 0;
        }
        for (unsigned int i = 0; i < servo_.size(); ++i) {
          servo_torque_state_msg_.torque_enable[direct_servo_num_ + i] =
            servo_.at(i).get().getTorqueFeedback() ? 1 : 0;
        }
        servo_torque_state_pub_.publish(&servo_torque_state_msg_);
        servo_torque_last_pub_time_ = now_time;
      }
  }
};
