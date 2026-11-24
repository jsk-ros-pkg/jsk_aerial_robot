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

    uint8_t slave_num_ = 0;
    uint8_t servo_num_ = 0;
    int8_t uav_model_ = -1;
    uint8_t baselink_ = 2;

    /* sensor fusion */
    StateEstimate* estimator_;

    /* flight controller */
    FlightControl* controller_;

    /* ros */
    ros::NodeHandle* nh_;
    unsigned int can_idle_count_ = 0;
    bool servo_control_flag_ = true;

    uint32_t can_tx_idle_start_time_ = 0; // for pause CAN TX -> TODO: change to another task for spinal process
    uint32_t CAN_TX_PAUSE_TIME = 2000; // 2000 ms for 1Khz task rate. TODO: change to another task for spinal process
    unsigned int send_board_index = 0; // incremental board id assignment for CAN TX

    uint32_t last_connected_time_ =0;
  }

  bool init(CAN_GeranlHandleTypeDef* hcan, ros::NodeHandle* nh, StateEstimate* estimator, FlightControl* controller, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
  {
    /* CAN */
    CANDeviceManager::init(hcan, GPIOx, GPIO_Pin);

    /* Estimation */
    estimator_ = estimator;

    /* Control */
    controller_ = controller;

    HAL_Delay(5000); //wait neuron initialization
    CANDeviceManager::addDevice(can_initializer_);
    CANDeviceManager::CAN_START();
    can_initializer_.initDevices();

    slave_num_ = neuron_.size();
    if(slave_num_ == 0) return false;

    //add CAN devices to CANDeviceManager
    for (unsigned int i = 0; i < neuron_.size(); i++) {
      CANDeviceManager::addDevice(neuron_.at(i).can_motor_);
      can_motor_send_device_.addMotor(neuron_.at(i).can_motor_);
      CANDeviceManager::addDevice(neuron_.at(i).can_imu_);
      CANDeviceManager::addDevice(neuron_.at(i).can_servo_);
      for (unsigned int j = 0; j < neuron_.at(i).can_servo_.servo_.size(); j++) {
        neuron_.at(i).can_servo_.servo_.at(j).setIndex(servo_.size());
        servo_.push_back(neuron_.at(i).can_servo_.servo_.at(j));
        if (neuron_.at(i).can_servo_.servo_.at(j).getSendDataFlag()) {
          servo_with_send_flag_.push_back(neuron_.at(i).can_servo_.servo_.at(j));
        }
      }
    }
    servo_num_ = servo_.size();

    /* ros */
    nh_ = nh;

    /* uav model: special rule based on the number of gimbals (no send data flag servos) */
    uint8_t gimbal_servo_num = servo_num_ - servo_with_send_flag_.size();

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

    return true;
  }

  void send()
  {
    if (slave_num_ == 0) return;

    if(HAL_GetTick() < can_tx_idle_start_time_ + CAN_TX_PAUSE_TIME) return;

    if(HAL_GetTick() % 2 == 0) {
      // 500Hz
      can_motor_send_device_.sendData();
    }
    else {
      if (slave_num_ != 0) {
        // 500Hz
        neuron_.at(send_board_index).can_servo_.sendData();
        send_board_index++;
        if (send_board_index == slave_num_) send_board_index = 0;
      }
    }

    can_initializer_.sendData(); // if necessary
  }

  void update(void)
  {
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

  std::vector<Neuron>& getNeuron()
  {
    return neuron_;
  }

  std::vector<std::reference_wrapper<Servo>>& getServo()
  {
    return servo_;
  }

  std::vector<std::reference_wrapper<Servo>>& getServoWithSendFlag()
  {
    return servo_with_send_flag_;
  }

  CANInitializer& getCANInitializer()
  {
    return can_initializer_;
  }

  int8_t getServoNum()
  {
    return servo_num_;
  }

  uint8_t getSlaveNum()
  {
    return slave_num_;
  }

  int8_t getUavModel()
  {
    return uav_model_;
  }

  bool getServoControlFlag()
  {
    return servo_control_flag_;
  }

  void setCanTxIdleStartTime(uint32_t time)
  {
    can_tx_idle_start_time_ = time;
  }

  void setServoControlFlag(bool flag)
  {
    servo_control_flag_ = flag;
  }
};
