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

    uint8_t slave_num_;
    int8_t uav_model_ = -1;
    uint8_t baselink_ = 2;

    /* sensor fusion */
    StateEstimate* estimator_;

  /* ros */
    constexpr uint8_t SERVO_PUB_INTERVAL = 20; //[ms]
    hydrus::ServoStates servo_state_msg_;
    ros::Publisher servo_state_pub_("/servo/states", &servo_state_msg_);
    hydrus::BoardInfo board_info_msg_;
    ros::Publisher board_info_pub_("/board_info", &board_info_msg_);
#if SEND_GYRO
    hydrus::Gyro gyro_msg_;
    ros::Publisher gyro_pub_("/hydrus_gyro", &gyro_msg_);
#endif
    ros::Subscriber<hydrus::ServoControlCmd> servo_ctrl_sub_("/servo/target_states", servoControlCallback);
    ros::Subscriber<hydrus::ServoTorqueCmd> servo_torque_ctrl_sub_("/servo/torque_enable", servoTorqueControlCallback);
    ros::Subscriber<std_msgs::Empty> board_info_request_sub_("/get_board_info", boardInfoRequestCallback);
    ros::Subscriber<hydrus::BoardConfigCmd> board_config_sub_("/board_config", boardConfigCallback);

    /* test rosserive server without class */
    ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> test_rosservice("/test_rosservice_server2", testRosseriveCallback);

    ros::NodeHandle* nh_;
    uint32_t last_pub_time_;
    unsigned int can_idle_count_ = 0;
  }


  /* rosserive test */
  void testRosseriveCallback(const std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res)
  {
	  res.success = true;
	  res.message = "nenetti~";
  }

  void boardInfoRequestCallback(const std_msgs::Empty& msg)
  {
	  for (unsigned int i = 0; i < slave_num_; i++) {
		  Neuron& neuron = neuron_.at(i);
		  board_info_msg_.imu_send_data_flag = neuron.can_imu_.getSendDataFlag() ? 1 : 0;
		  board_info_msg_.slave_id = neuron.getSlaveId();
		  board_info_msg_.servos_length = neuron.can_servo_.servo_.size();
		  std::vector<hydrus::ServoInfo> servo_infos(board_info_msg_.servos_length);
		  for (unsigned int j = 0; j < servo_infos.size(); j++) {
			  Servo& s = neuron.can_servo_.servo_.at(j);
			  servo_infos.at(j).id = s.getId();
			  servo_infos.at(j).p_gain = s.getPGain();
			  servo_infos.at(j).i_gain = s.getIGain();
			  servo_infos.at(j).d_gain = s.getDGain();
			  servo_infos.at(j).profile_velocity = s.getProfileVelocity();
			  servo_infos.at(j).current_limit = s.getCurrentLimit();
			  servo_infos.at(j).send_data_flag = s.getSendDataFlag() ? 1 : 0;
		  }
		  board_info_msg_.servos = servo_infos.data();
		  board_info_pub_.publish(&board_info_msg_);
	  }
  }

  void servoControlCallback(const hydrus::ServoControlCmd& control_msg)
  {
	  if (control_msg.index_length != control_msg.angles_length) return;
	  for (unsigned int i = 0; i < control_msg.index_length; i++) {
		  servo_.at(control_msg.index[i]).get().setGoalPosition(control_msg.angles[i]);
	  }
  }

  void servoTorqueControlCallback(const hydrus::ServoTorqueCmd& control_msg)
  {
	  if (control_msg.index_length != control_msg.torque_enable_length) return;
	  for (unsigned int i = 0; i < control_msg.index_length; i++) {
		  servo_.at(control_msg.index[i]).get().setTorqueEnable((control_msg.torque_enable[i] != 0) ? true : false);

		  /* update the target angle */
		  servo_.at(control_msg.index[i]).get().setGoalPosition(servo_.at(control_msg.index[i]).get().getPresentPosition());
	  }
  }

  void boardConfigCallback(const hydrus::BoardConfigCmd& config_msg)
  {
	  can_idle_count_ = 2000;
	  can_initializer_.configDevice(config_msg);
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
    nh_->advertise(board_info_pub_);
#if SEND_GYRO
    nh_->advertise(gyro_pub_);
#endif

    nh_->subscribe< ros::Subscriber<hydrus::ServoControlCmd> >(servo_ctrl_sub_);
    nh_->subscribe< ros::Subscriber<hydrus::ServoTorqueCmd> >(servo_torque_ctrl_sub_);
    nh_->subscribe< ros::Subscriber<std_msgs::Empty> >(board_info_request_sub_);
    nh_->subscribe< ros::Subscriber<hydrus::BoardConfigCmd> >(board_config_sub_);

    /* speical template: https://qiita.com/narumi_/items/f656678c78d50c40bc1c */
    nh_->advertiseService< std_srvs::Trigger::Request, std_srvs::Trigger::Response > (test_rosservice);

    HAL_Delay(3000); //wait neuron initialization
    CANDeviceManager::addDevice(can_initializer_);
    CANDeviceManager::Receive_IT();
    //init CAN devicesvoid callback(const Test::Request & req, Test::Response & res){
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

    /* uav model: special rule based on the number of gimbals (no send data flag servos) */
    uint8_t gimbal_servo_num = servo_.size() - servo_with_send_flag_.size();
      if(gimbal_servo_num == 0)
        {
          if(slave_num_ < 6) uav_model_ = aerial_robot_base::UavInfo::HYDRUS; // less than hex
          else uav_model_ = aerial_robot_base::UavInfo::HYDRUS_XI;
        }
      if(gimbal_servo_num  == slave_num_)
        {
          uav_model_ = aerial_robot_base::UavInfo::HYDRUS_XI;
        }
      if(gimbal_servo_num  == 2 * slave_num_)
        {
          uav_model_ = aerial_robot_base::UavInfo::DRAGON;
        }

    servo_state_msg_.servos_length = servo_with_send_flag_.size();
    servo_state_msg_.servos = new hydrus::ServoState[servo_with_send_flag_.size()];

    /* other component */
    imu_weight_.resize(slave_num_ + 1);

    /* set IMU weights */
    // no fusion
    imu_weight_[0] = 1.0;
    for (uint i = 1; i < imu_weight_.size(); i++) imu_weight_[i] = 0.0;

    estimator_->getAttEstimator()->setImuWeight(0, imu_weight_[0]);
    for (int i = 0; i < slave_num_; i++) {
      HAL_Delay(100);
      neuron_.at(i).can_imu_.init();
      if(i != baselink_) neuron_.at(i).can_imu_.setVirtualFrame(true);
      estimator_->getAttEstimator()->addImu(&(neuron_.at(i).can_imu_), imu_weight_[i + 1]);

      IMU_ROS_CMD::addImu(&(neuron_.at(i).can_imu_));
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

    //convertGyroFromJointvalues();

    /* ros publish */
    uint32_t now_time = HAL_GetTick();
    if( now_time - last_pub_time_ >= SERVO_PUB_INTERVAL)
      {
    	/* send servo */
    	servo_state_msg_.stamp = nh_->now();
        for (unsigned int i = 0; i < servo_with_send_flag_.size(); i++)
          {
        	hydrus::ServoState servo;

        	servo.index = servo_with_send_flag_.at(i).get().getIndex();
            servo.angle = servo_with_send_flag_.at(i).get().getPresentPosition();
            servo.temp = servo_with_send_flag_.at(i).get().getPresentTemperature();
            servo.load = servo_with_send_flag_.at(i).get().getPresentCurrent();
            servo.error = servo_with_send_flag_.at(i).get().getError();

            servo_state_msg_.servos[i] = servo;
          }
        servo_state_pub_.publish(&servo_state_msg_);


#if SEND_GYRO
        /* send gyro data */
        gyro_msg_.stamp = nh_->now();
        for (int i = 0; i < CAN::SLAVE_NUM; i++)
          {
        	gyro_msg_.x[i] = can_imu_[i].getGyro().x / CANIMU::GYRO_SCALE;
        	gyro_msg_.y[i] = can_imu_[i].getGyro().y / CANIMU::GYRO_SCALE;
        	gyro_msg_.z[i] = can_imu_[i].getGyro().z / CANIMU::GYRO_SCALE;
          }
        gyro_pub_.publish(&gyro_msg_);
#endif

        last_pub_time_ = now_time;
      }

    CANDeviceManager::tick(1);
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

  void convertGyroFromJointvalues()
  {
	  //It's mendokusai to implement for dragon, so comment out now.
	  /*
	  Vector3f gyro_v, acc_v, mag_v;
	  double current_joint_angle[slave_num_ - 1];

	  for (int i = 0; i < slave_num_ - 1; i++)
		  current_joint_angle[i] = (can_servo_[i].getServoData().angle - 2048) * 2 * 3.1415926f / 4096;

	  double theta = 0.0;

	  for (int i = baselink_ - 1; i >= 0; i--)
	  {
		  theta -= current_joint_angle[i];
		  Vector3f gyro = can_imu_[i].getGyro(true); // get the value in board frame
		  gyro_v[0] = gyro[0] * cos(theta) - gyro[1] * sin(theta);
		  gyro_v[1] = gyro[0] * sin(theta) + gyro[1] * cos(theta);
		  gyro_v[2] = gyro[2];
		  can_imu_[i].setGyroV(gyro_v);
	  }
	  theta = 0.0f;
	  for (int i = baselink_; i < slave_num_ - 1; i++)
	  {
		  theta += current_joint_angle[i];
		  Vector3f gyro = can_imu_[i + 1].getGyro(true); // get the value in board frame
		  gyro_v[0] = gyro[0] * cos(theta) - gyro[1] * sin(theta);
		  gyro_v[1] = gyro[0] * sin(theta) + gyro[1] * cos(theta);
		  gyro_v[2] = gyro[2];
		  can_imu_[i+1].setGyroV(gyro_v);
	  }
	  */
  }
};
