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
    std::vector<std::reference_wrapper<CANIMU>> imu_with_send_flag_;
    CANInitializer can_initializer_(neuron_);

    uint8_t slave_num_;
    int8_t uav_model_ = -1;
    uint8_t baselink_ = 2;

    /* sensor fusion */
    StateEstimate* estimator_;

    /* ros */
    constexpr uint8_t SERVO_PUB_INTERVAL = 20; //[ms]
    constexpr uint8_t SERVO_TORQUE_PUB_INTERVAL = 1000; //[ms]
    uint8_t NEURON_IMU_PUB_INTERVAL; // dynamic [ms]
    spinal::ServoStates servo_state_msg_;
    spinal::ServoTorqueStates servo_torque_state_msg_;
    ros::Publisher servo_state_pub_("servo/states", &servo_state_msg_);
    ros::Publisher servo_torque_state_pub_("servo/torque_states", &servo_torque_state_msg_);

    spinal::NeuronImus neuron_imus_msg_;
    ros::Publisher neuron_imus_pub_("neuron_imus", &neuron_imus_msg_);

    ros::Subscriber<spinal::ServoControlCmd> servo_ctrl_sub_("servo/target_states", servoControlCallback);
    ros::Subscriber<spinal::ServoTorqueCmd> servo_torque_ctrl_sub_("servo/torque_enable", servoTorqueControlCallback);

    ros::ServiceServer<spinal::GetBoardInfo::Request, spinal::GetBoardInfo::Response> board_info_srv_("get_board_info", boardInfoCallback);
    ros::ServiceServer<spinal::SetBoardConfig::Request, spinal::SetBoardConfig::Response> board_config_srv_("set_board_config", boardConfigCallback);

    spinal::GetBoardInfo::Response board_info_res_;

    ros::NodeHandle* nh_;
    uint32_t servo_last_pub_time_ = 0;
    uint32_t servo_torque_last_pub_time_ = 0;
    uint32_t neuron_imu_last_pub_time_ = 0;
    unsigned int can_idle_count_ = 0;
    bool servo_control_flag_ = true;

    uint32_t last_connected_time_ =0;
  }

  void boardInfoCallback(const spinal::GetBoardInfo::Request& req, spinal::GetBoardInfo::Response& res)
  {
	  for (unsigned int i = 0; i < slave_num_; i++) {
		  Neuron& neuron = neuron_.at(i);
		  spinal::BoardInfo& board = board_info_res_.boards[i];
		  board.imu_send_data_flag = neuron.can_imu_.getSendDataFlag() ? 1 : 0;
		  board.dynamixel_ttl_rs485_mixed = neuron.can_servo_.getDynamixelTTLRS485Mixed() ? 1 : 0;
		  board.slave_id = neuron.getSlaveId();

		  for (unsigned int j = 0; j < board.servos_length; j++) {
			  Servo& s = neuron.can_servo_.servo_.at(j);
			  board.servos[j].id = s.getId();
			  board.servos[j].p_gain = s.getPGain();
			  board.servos[j].i_gain = s.getIGain();
			  board.servos[j].d_gain = s.getDGain();
			  board.servos[j].profile_velocity = s.getProfileVelocity();
			  board.servos[j].current_limit = s.getCurrentLimit();
			  board.servos[j].send_data_flag = s.getSendDataFlag() ? 1 : 0;
		  }
	  }
	  res = board_info_res_;
  }

  void servoControlCallback(const spinal::ServoControlCmd& control_msg)
  {
      if (!servo_control_flag_) return;
	  if (control_msg.index_length != control_msg.angles_length) return;
	  for (unsigned int i = 0; i < control_msg.index_length; i++) {
		  servo_.at(control_msg.index[i]).get().setGoalPosition(control_msg.angles[i]);
	  }
  }

  void servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg)
  {
	  if (control_msg.index_length != control_msg.torque_enable_length) return;
	  for (unsigned int i = 0; i < control_msg.index_length; i++) {
		  servo_.at(control_msg.index[i]).get().setTorqueEnable((control_msg.torque_enable[i] != 0) ? true : false);

		  /* update the target angle */
		  if (servo_.at(control_msg.index[i]).get().getSendDataFlag()) {
			  servo_.at(control_msg.index[i]).get().setGoalPosition(servo_.at(control_msg.index[i]).get().getPresentPosition());
		  }
	  }
  }

  void boardConfigCallback(const spinal::SetBoardConfig::Request& req, spinal::SetBoardConfig::Response& res)
  {
	  can_idle_count_ = 3000;
	  //need this cheap delay
	  for (int i = 0; i < 1000000; ++i) {
			  asm("nop");
	  }
	  can_initializer_.configDevice(req);
	  res.success = true;
  }

  void init(CAN_HandleTypeDef* hcan, ros::NodeHandle* nh, StateEstimate* estimator, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
  {
    /* CAN */
    CANDeviceManager::init(hcan, GPIOx, GPIO_Pin);

    /* Estimation */
    estimator_ = estimator;

    /* ros */
    nh_ = nh;
    nh_->advertise(servo_state_pub_);
    nh_->advertise(servo_torque_state_pub_);
    nh_->advertise(neuron_imus_pub_);

    nh_->subscribe< ros::Subscriber<spinal::ServoControlCmd> >(servo_ctrl_sub_);
    nh_->subscribe< ros::Subscriber<spinal::ServoTorqueCmd> >(servo_torque_ctrl_sub_);

    nh_->advertiseService(board_info_srv_);
    nh_->advertiseService(board_config_srv_);

    HAL_Delay(5000); //wait neuron initialization
    CANDeviceManager::addDevice(can_initializer_);
    CANDeviceManager::Receive_IT();
    can_initializer_.initDevices();

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
    slave_num_ = neuron_.size();

    if(slave_num_  == 0) return;

    /* uav model: special rule based on the number of gimbals (no send data flag servos) */
    uint8_t gimbal_servo_num = servo_.size() - servo_with_send_flag_.size();

    /* Tongtybj: TODO not good case processing */
    if(gimbal_servo_num == 0)
        {
          if(slave_num_ < 6) uav_model_ = spinal::UavInfo::HYDRUS; // less than hex
          else uav_model_ = spinal::UavInfo::HYDRUS_XI;
        }
   if(gimbal_servo_num  == slave_num_)
        {
          uav_model_ = spinal::UavInfo::HYDRUS_XI;
        }
    if(gimbal_servo_num  == 2 * slave_num_)
        {
          uav_model_ = spinal::UavInfo::DRAGON;
          /* special smoothing flag for dragon */
          estimator_->getAttEstimator()->setGyroSmoothFlag(true);
        }

    servo_state_msg_.servos_length = servo_with_send_flag_.size();
    servo_state_msg_.servos = new spinal::ServoState[servo_with_send_flag_.size()];
    servo_torque_state_msg_.torque_enable_length = servo_.size();
    servo_torque_state_msg_.torque_enable = new uint8_t[servo_.size()];

    // set imu 
    for (int i = 0; i < slave_num_; i++) {
      HAL_Delay(100);
      neuron_.at(i).can_imu_.init();

      if(neuron_.at(i).can_imu_.getSendDataFlag())
        {
          IMU_ROS_CMD::addImu(&(neuron_.at(i).can_imu_)); // TODO: should for all neuron 
          imu_with_send_flag_.push_back(neuron_.at(i).can_imu_);
        }
    }
    neuron_imus_msg_.neurons_length = imu_with_send_flag_.size();
    neuron_imus_msg_.neurons = new spinal::NeuronImu[imu_with_send_flag_.size()]; // TODO: this is not adaptive for a change of imu_with_send_flag_num. So, if you change the status of send_flag in a certain neuron imu, please reboot spinal.
    NEURON_IMU_PUB_INTERVAL = 2 * slave_num_; // the min interval (ms per message) that neuron publish the data
    if(NEURON_IMU_PUB_INTERVAL < 10) NEURON_IMU_PUB_INTERVAL = 10; // the min interval that spinal rosserial can handle, that is 10 ms/message == 100 Hz.

    //set response for get_board_info
    board_info_res_.boards_length = slave_num_;
    board_info_res_.boards = new spinal::BoardInfo[slave_num_];
    for (unsigned int i = 0; i < slave_num_; i++) {
    	Neuron& neuron = neuron_.at(i);
    	spinal::BoardInfo& board = board_info_res_.boards[i];
    	board.servos_length = neuron.can_servo_.servo_.size();
    	board.servos = new spinal::ServoInfo[board.servos_length];
    }
  }

  void send()
  {
	static int send_board_index = 0;
	if (can_idle_count_ > 0) {
		can_idle_count_--;
		return;
	}
	if(HAL_GetTick() % 2 == 0) {
	  can_motor_send_device_.sendData();
	  if (slave_num_ != 0) {
		  neuron_.at(send_board_index).can_servo_.sendData();
		  send_board_index++;
		  if (send_board_index == slave_num_) send_board_index = 0;
	  }
    }
  }

  void update(void)
  {
    for (int i = 0; i < slave_num_; i++)
      neuron_.at(i).can_imu_.update();

    /* ros publish */
    uint32_t now_time = HAL_GetTick();
    if( now_time - servo_last_pub_time_ >= SERVO_PUB_INTERVAL)
      {
    	/* send servo */
    	servo_state_msg_.stamp = nh_->now();
        for (unsigned int i = 0; i < servo_with_send_flag_.size(); i++)
          {
        	spinal::ServoState servo;

        	servo.index = servo_with_send_flag_.at(i).get().getIndex();
            servo.angle = servo_with_send_flag_.at(i).get().getPresentPosition();
            servo.temp = servo_with_send_flag_.at(i).get().getPresentTemperature();
            servo.load = servo_with_send_flag_.at(i).get().getPresentCurrent();
            servo.error = servo_with_send_flag_.at(i).get().getError();

            servo_state_msg_.servos[i] = servo;
          }

        servo_state_pub_.publish(&servo_state_msg_);
        servo_last_pub_time_ = now_time;
      }

    if(now_time - neuron_imu_last_pub_time_ >= NEURON_IMU_PUB_INTERVAL)
      {
        /* send gyro data */
        neuron_imus_msg_.stamp = nh_->now();
        for (int i = 0; i < imu_with_send_flag_.size(); i++)
          {
            spinal::NeuronImu imu;

            ap::Vector3f acc = imu_with_send_flag_.at(i).get().getAcc();
            ap::Vector3f gyro = imu_with_send_flag_.at(i).get().getGyro();
            ap::Vector3f rpy = imu_with_send_flag_.at(i).get().getRPY();
            imu.neuron_id = imu_with_send_flag_.at(i).get().getSlaveId();
            imu.gyro[0] = gyro[0]; // / MPU9250_GYRO_SCALE;
            imu.gyro[1] = gyro[1]; // / MPU9250_GYRO_SCALE;
            imu.gyro[2] = gyro[2]; // / MPU9250_GYRO_SCALE;
            imu.acc[0] = acc[0]; // / MPU9250_ACC_SCALE;
            imu.acc[1] = acc[1]; // / MPU9250_ACC_SCALE;
            imu.acc[2] = acc[2]; // / MPU9250_ACC_SCALE;
            imu.rpy[0] = rpy[0]; // * 1000
            imu.rpy[1] = rpy[1]; // * 1000
            imu.rpy[2] = rpy[2]; // * 1000
            neuron_imus_msg_.neurons[i] = imu;
          }
        neuron_imus_pub_.publish(&neuron_imus_msg_);
        neuron_imu_last_pub_time_ = now_time;
      }

    if( now_time - servo_torque_last_pub_time_ >= SERVO_TORQUE_PUB_INTERVAL)
      {
    	for (unsigned int i = 0; i < servo_.size(); i++)
    	  {
    		servo_torque_state_msg_.torque_enable[i] = servo_.at(i).get().getTorqueEnable() ? 1 : 0;
    	  }
    	servo_torque_state_pub_.publish(&servo_torque_state_msg_);
    	servo_torque_last_pub_time_ = now_time;

      }

    CANDeviceManager::tick(1);

   if(CANDeviceManager::connected()) last_connected_time_ = now_time;

    if(now_time - last_connected_time_ > 1000 /* ms */)
    {
    	if(nh_->connected()) nh_->logerror("CAN is not connected");
    	last_connected_time_ = now_time;
    }
  }

  void setMotorPwm(uint16_t pwm, uint8_t motor)
  {
	  neuron_.at(motor).can_motor_.setPwm(pwm);
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
};
