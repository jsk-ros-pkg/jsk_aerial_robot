#include <di_control/di_tilt_control.h>

TiltControl::TiltControl(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp_)
{
  tiltModulesInit();

  attitude_sub_ = nh_.subscribe<jsk_stm::JskImu>("jsk_imu", 1, &TiltControl::attitudeCallback, this, ros::TransportHints().tcpNoDelay());

  desire_attitude_sub_ = nh_.subscribe<geometry_msgs::Vector3>("desire_attitude", 1, &TiltControl::desireAttitudeCallback, this, ros::TransportHints().tcpNoDelay());

  tilt_command_flag_ = false;

  nhp_.param("control_rate", control_rate_, 20.0);
  control_timer_ = nhp_.createTimer(ros::Duration(1.0 / control_rate_), &TiltControl::controlFunc, this);


}

TiltControl::~TiltControl(){};

void TiltControl::tiltModulesInit()
{
  nhp_.param("tilt_module_num", tilt_module_num_, 4);
  nhp_.param("tilt_mode", tilt_mode_, 0);
  nhp_.param("tilt_thre", tilt_thre_, 0.0);

  nhp_.param("gimable_debug", gimbal_debug_, false);

  tilt_modules_.resize(tilt_module_num_);

  for(int i = 0; i < tilt_module_num_; i++)
    {
      std::stringstream module_no;
      module_no << i + 1;

      nhp_.param(std::string("tilt_module") + module_no.str() + std::string("_rotate_angle"), tilt_modules_[i].rotate_angle, 0.0);

      for(int j = 0; j < 2; j ++)
        {
          std::stringstream servo_no;
          servo_no << tilt_module_num_* i + j + 1;

          tilt_modules_[i].servos_ctrl_pub_name[j] = std::string("/j") + servo_no.str()  + std::string("_controller/command");
          tilt_modules_[i].servos_ctrl_pub[j] = nh_.advertise<std_msgs::Float64>(tilt_modules_[i].servos_ctrl_pub_name[j], 1); 

          tilt_modules_[i].servos_state_sub_name[j] = std::string("/j") + servo_no.str()  + std::string("_controller/state");
          tilt_modules_[i].servos_state_sub[j] = nh_.subscribe<dynamixel_msgs::JointState>(tilt_modules_[i].servos_state_sub_name[j], 1, boost::bind(&TiltControl::servoCallback, this, _1, i, j)); //reverse

          tilt_modules_[i].servos_torque_enable_service_name[j] = std::string("/j") + servo_no.str()  + std::string("_torque_enable");
          tilt_modules_[i].servos_torque_enable_client[j] = nh_.serviceClient<dynamixel_controllers::TorqueEnable>(tilt_modules_[i].servos_torque_enable_service_name[j]);

          //torque enable
          dynamixel_controllers::TorqueEnable srv;
          srv.request.torque_enable = true;
          if (tilt_modules_[i].servos_torque_enable_client[j].call(srv))
            {
              ROS_INFO("no.%d torque_enable", tilt_module_num_ * i + j + 1);
            }
          else
            {
              ROS_ERROR("Failed to call service torque enable");
            }

          nhp_.param(std::string("servo") + servo_no.str() + std::string("_angle_max"), tilt_modules_[i].angle_max[j], 1.5 * M_PI ); 
          nhp_.param(std::string("servo") + servo_no.str() + std::string("_angle_min"), tilt_modules_[i].angle_min[j], 0.5 * M_PI ); 
          nhp_.param(std::string("servo") + servo_no.str() + std::string("_angle_sgn"), tilt_modules_[i].angle_sgn[j], 1);

          nhp_.param(std::string("servo") + servo_no.str() + std::string("_angle_offset"), tilt_modules_[i].angle_offset[j], M_PI); 
          std_msgs::Float64 command;
          command.data = tilt_modules_[i].angle_offset[j];
          tilt_modules_[i].servos_ctrl_pub[j].publish(command);

        }

    }
}

void TiltControl::desireAttitudeCallback(const geometry_msgs::Vector3ConstPtr& msg)
{
  desire_attitude_ = *msg;
  tilt_command_flag_ = true;

}

void TiltControl::attitudeCallback(const jsk_stm::JskImuConstPtr& msg)
{
  current_attitude_ = msg->angles;
}


void TiltControl::servoCallback(const dynamixel_msgs::JointStateConstPtr& msg, int i, int j)
{
  tilt_modules_[i].current_angle[j] = tilt_modules_[i].angle_sgn[j] * (msg->current_pos - tilt_modules_[i].angle_offset[j]);
}

void TiltControl::gimbalControl()
{
  Eigen::Quaternion<double> q_att ;
  if (gimbal_debug_)
    {
      q_att = Eigen::AngleAxisd(current_attitude_.y, Eigen::Vector3d::UnitY()) 
        * Eigen::AngleAxisd(current_attitude_.x, Eigen::Vector3d::UnitX());
    }
  else
    {
      q_att = Eigen::AngleAxisd(desire_attitude_.y, Eigen::Vector3d::UnitY()) 
        * Eigen::AngleAxisd(desire_attitude_.x, Eigen::Vector3d::UnitX());
    }

  for(int i = 0 ; i < tilt_module_num_; i++)
    {
      Eigen::Quaternion<double> q =  q_att * Eigen::AngleAxisd(tilt_modules_[i].rotate_angle, Eigen::Vector3d::UnitZ());
      Eigen::Matrix3d rotation = q.matrix();
      float roll = atan2(rotation(2,1),rotation(2,2));
      float pitch = atan2(-rotation(2,0),  sqrt(rotation(2,1)*rotation(2,1) + rotation(2,2)* rotation(2,2)));

      std_msgs::Float64 command;
      command.data = tilt_modules_[i].angle_offset[0] - tilt_modules_[i].angle_sgn[0] * pitch;
      tilt_modules_[i].servos_ctrl_pub[0].publish(command);

      command.data = tilt_modules_[i].angle_offset[1] - tilt_modules_[i].angle_sgn[1] * roll;
      tilt_modules_[i].servos_ctrl_pub[1].publish(command);
    }
}

void TiltControl::controlFunc(const ros::TimerEvent & e)
{
  if(tilt_mode_ == ACTIVE_TILT_MODE)
    {
      if(!tilt_command_flag_) return; //need?

      gimbalControl();
      tilt_command_flag_ = false;
    }
  else if(tilt_mode_ == PASSIVE_TILT_MODE)
    {
      tilt_command_flag_ = false;

      if(fabs(current_attitude_.x - desire_attitude_.x ) > tilt_thre_)
        {
          current_attitude_.x = desire_attitude_.x;
          tilt_command_flag_ = true;
        }
      if(fabs(current_attitude_.y - desire_attitude_.y ) > tilt_thre_)
        {
          current_attitude_.y = desire_attitude_.y;
          tilt_command_flag_ = true;
        }

      if(tilt_command_flag_) gimbalControl();

    }
  
}
