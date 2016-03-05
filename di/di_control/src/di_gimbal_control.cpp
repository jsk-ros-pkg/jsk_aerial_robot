#include <di_control/di_gimbal_control.h>

GimbalControl::GimbalControl(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp)
{
  gimbalModulesInit();

  attitude_sub_ = nh_.subscribe<jsk_stm::JskImu>("jsk_imu", 1, &GimbalControl::attitudeCallback, this, ros::TransportHints().tcpNoDelay());

  desire_attitude_sub_ = nh_.subscribe<geometry_msgs::Vector3>("desire_attitude", 1, &GimbalControl::desireAttitudeCallback, this, ros::TransportHints().tcpNoDelay());

  gimbal_command_flag_ = false;

  nhp_.param("control_rate", control_rate_, 20.0);
  control_timer_ = nhp_.createTimer(ros::Duration(1.0 / control_rate_), &GimbalControl::controlFunc, this);

}

GimbalControl::~GimbalControl(){};

void GimbalControl::gimbalModulesInit()
{
  nhp_.param("gimbal_module_num", gimbal_module_num_, 4);
  nhp_.param("gimbal_mode", gimbal_mode_, 0);
  nhp_.param("gimbal_thre", gimbal_thre_, 0.0);

  nhp_.param("gimable_debug", gimbal_debug_, false);

  nhp_.param("body_diameter", body_diameter_, 1.2);

  gimbal_modules_.resize(gimbal_module_num_);

  for(int i = 0; i < gimbal_module_num_; i++)
    {
      std::stringstream module_no;
      module_no << i + 1;

      nhp_.param(std::string("gimbal_module") + module_no.str() + std::string("_rotate_angle"), gimbal_modules_[i].rotate_angle, 0.0);

      for(int j = 0; j < 2; j ++)
        {
          std::stringstream servo_no;
          servo_no << 2 * i + j + 1;

          gimbal_modules_[i].servos_ctrl_pub_name[j] = std::string("/j") + servo_no.str()  + std::string("_controller/command");
          gimbal_modules_[i].servos_ctrl_pub[j] = nh_.advertise<std_msgs::Float64>(gimbal_modules_[i].servos_ctrl_pub_name[j], 1); 

          gimbal_modules_[i].servos_state_sub_name[j] = std::string("/j") + servo_no.str()  + std::string("_controller/state");
          gimbal_modules_[i].servos_state_sub[j] = nh_.subscribe<dynamixel_msgs::JointState>(gimbal_modules_[i].servos_state_sub_name[j], 1, boost::bind(&GimbalControl::servoCallback, this, _1, i, j)); //reverse

          gimbal_modules_[i].servos_torque_enable_service_name[j] = std::string("/j") + servo_no.str()  + std::string("_controller/torque_enable");
          gimbal_modules_[i].servos_torque_enable_client[j] = nh_.serviceClient<dynamixel_controllers::TorqueEnable>(gimbal_modules_[i].servos_torque_enable_service_name[j]);

          //torque enable
           dynamixel_controllers::TorqueEnable srv;
           srv.request.torque_enable = true;
           if (gimbal_modules_[i].servos_torque_enable_client[j].call(srv))
             {
               ROS_INFO("no.%d torque_enable", 2 * i + j + 1);
             }
           else
             {
               ROS_ERROR("Failed to call service torque enable");
             }

          nhp_.param(std::string("servo") + servo_no.str() + std::string("_angle_max"), gimbal_modules_[i].angle_max[j], 1.5 * M_PI ); 
          nhp_.param(std::string("servo") + servo_no.str() + std::string("_angle_min"), gimbal_modules_[i].angle_min[j], 0.5 * M_PI ); 
          nhp_.param(std::string("servo") + servo_no.str() + std::string("_angle_sgn"), gimbal_modules_[i].angle_sgn[j], 1);

          nhp_.param(std::string("servo") + servo_no.str() + std::string("_angle_offset"), gimbal_modules_[i].angle_offset[j], M_PI); 
        }

    }

  //init servo angle command
  for(int i = 0; i < gimbal_module_num_; i++)
    {
      for(int j = 0; j < 2; j ++)
        {
          std_msgs::Float64 command;
          command.data = gimbal_modules_[i].angle_offset[j];
          gimbal_modules_[i].servos_ctrl_pub[j].publish(command);
        }
    }
}

void GimbalControl::desireAttitudeCallback(const geometry_msgs::Vector3ConstPtr& msg)
{
  desire_attitude_ = *msg;
  gimbal_command_flag_ = true;

}

void GimbalControl::attitudeCallback(const jsk_stm::JskImuConstPtr& msg)
{
  current_attitude_ = msg->angles;
}


void GimbalControl::servoCallback(const dynamixel_msgs::JointStateConstPtr& msg, int i, int j)
{
  gimbal_modules_[i].current_angle[j] = gimbal_modules_[i].angle_sgn[j] * (msg->current_pos - gimbal_modules_[i].angle_offset[j]);
}

void GimbalControl::gimbalControl()
{
  Eigen::Quaternion<double> q_att;
  static Eigen::Quaternion<double> prev_q_att = Eigen::Quaternion<double>(1,0,0,0);
  static float tilt_alt_sum = 0;

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

  //gimbal
  for(int i = 0 ; i < gimbal_module_num_; i++)
    {
      Eigen::Quaternion<double> q =  q_att * Eigen::AngleAxisd(gimbal_modules_[i].rotate_angle, Eigen::Vector3d::UnitZ());
      Eigen::Matrix3d rotation = q.matrix();
      float roll = atan2(rotation(2,1),rotation(2,2));
      float pitch = atan2(-rotation(2,0),  sqrt(rotation(2,1)*rotation(2,1) + rotation(2,2)* rotation(2,2)));

      std_msgs::Float64 command;
      command.data = gimbal_modules_[i].angle_offset[0] - gimbal_modules_[i].angle_sgn[0] * pitch;
      gimbal_modules_[i].servos_ctrl_pub[0].publish(command);

      command.data = gimbal_modules_[i].angle_offset[1] - gimbal_modules_[i].angle_sgn[1] * roll;
      gimbal_modules_[i].servos_ctrl_pub[1].publish(command);
    }

  //alt
  Eigen::Matrix3d r = q_att.matrix();
  Eigen::Matrix3d prev_r = prev_q_att.matrix();
  //float alt_tilt = sin(atan2(sqrt(r(0,2) * r(0,2)  + r(1,2) * r(1,2)), fabs(r(2,2)))) * body_diameter;
  float alt_tilt = sqrt(r(0,2) * r(0,2)  + r(1,2) * r(1,2));
  float prev_alt_tilt = sqrt(prev_r(0,2) * prev_r(0,2)  + prev_r(1,2) * prev_r(1,2));

  aerial_robot_base::FlightNav flight_nav_msg;
  flight_nav_msg.header.stamp = ros::Time::now();
  flight_nav_msg.command_mode = NO_NAVIGATION;
  flight_nav_msg.pos_z_navi_mode = VEL_FLIGHT_MODE_COMMAND;
  flight_nav_msg.target_pos_diff_z = body_diameter_ / 2 * (alt_tilt - prev_alt_tilt);
 
  tilt_alt_sum += flight_nav_msg.target_pos_diff_;
  prev_q_att = q_att;

  ROS_INFO("tilt_alt: %f, sum: %f", flight_nav_msg.target_pos_diff_, tilt_alt_sum);
}

void GimbalControl::controlFunc(const ros::TimerEvent & e)
{
  if(gimbal_mode_ == ACTIVE_GIMBAL_MODE)
    {

      if(!gimbal_command_flag_) return; //need?

      gimbalControl();
      gimbal_command_flag_ = false;
    }
  else if(gimbal_mode_ == PASSIVE_GIMBAL_MODE)
    {
      gimbal_command_flag_ = false;

      if(fabs(current_attitude_.x - desire_attitude_.x ) > gimbal_thre_)
        {
          current_attitude_.x = desire_attitude_.x;
          gimbal_command_flag_ = true;
        }
      if(fabs(current_attitude_.y - desire_attitude_.y ) > gimbal_thre_)
        {
          current_attitude_.y = desire_attitude_.y;
          gimbal_command_flag_ = true;
        }

      if(gimbal_command_flag_) gimbalControl();
    }

}
