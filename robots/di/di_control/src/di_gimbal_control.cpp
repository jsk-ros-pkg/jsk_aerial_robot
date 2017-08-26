#include <di_control/di_gimbal_control.h>

GimbalControl::GimbalControl(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp)
{
  gimbalModulesInit();

  alt_control_pub_ = nh_.advertise<aerial_robot_base::FlightNav>("flight_nav", 1);
  desire_tilt_pub_ = nh_.advertise<geometry_msgs::Vector3>("desire_tilt", 1);

  attitude_sub_ = nh_.subscribe<aerial_robot_msgs::Imu>("imu", 1, &GimbalControl::attitudeCallback, this, ros::TransportHints().tcpNoDelay());

  desire_attitude_sub_ = nh_.subscribe<geometry_msgs::Vector3>("desire_attitude", 1, &GimbalControl::desireAttitudeCallback, this, ros::TransportHints().tcpNoDelay());

  attitude_command_sub_ = nh_.subscribe("attitude_command", 1, &GimbalControl::attCommandCallback, this, ros::TransportHints().tcpNoDelay());

  joy_stick_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy_stick_command", 1, &GimbalControl::joyStickControl, this, ros::TransportHints().udp());

  stop_teleop_pub_ = nh_.advertise<std_msgs::UInt8>("teleop_sub", 1);

  //debug for passive att compare method
  att_diff_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("att_command_current_compare", 1);

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


  // active gimbal mode param
  nhp_.param("active_gimbal_tilt_duration", active_gimbal_tilt_duration_, 4.0);
  nhp_.param("active_gimbal_tilt_interval", active_gimbal_tilt_interval_, 0.04);

  //dynamic reconfigure server
  gimbal_server_ = new dynamic_reconfigure::Server<di_control::GimbalDynReconfConfig>(nhp_);
  dyn_reconf_func_ = boost::bind(&GimbalControl::GimbalDynReconfCallback, this, _1, _2);
  gimbal_server_->setCallback(dyn_reconf_func_);


  control_timer_ = nhp_.createTimer(ros::Duration(1.0 / control_rate_), &GimbalControl::controlFunc, this);

}

GimbalControl::~GimbalControl(){};

void GimbalControl::gimbalModulesInit()
{
  nhp_.param("control_rate", control_rate_, 20.0);

  nhp_.param("gimbal_module_num", gimbal_module_num_, 4);
  nhp_.param("gimbal_mode", gimbal_mode_, 0);

  ROS_ERROR("gimbal mode: %s", (gimbal_mode_ == ACTIVE_GIMBAL_MODE)?("ACTIVE_GIMBAL_MODE"):("PASSIVE_GIMBAL_MODE"));
  nhp_.param("gimbal_thre", gimbal_thre_, 0.0);

  nhp_.param("gimabal_debug", gimbal_debug_, false);

  nhp_.param("body_diameter", body_diameter_, 1.2);

  nhp_.param("att_control_rate", att_control_rate_, 40.0);
  nhp_.param("att_comp_duration", att_comp_duration_, 0.5);
  nhp_.param("passive_level_back_duration", passive_level_back_duration_, 10.0);
  nhp_.param("attitude_outlier_thre", attitude_outlier_thre_, 0.5);
  nhp_.param("passive_loop_rate", passive_loop_rate_, 20.0);


  passive_loop_cnt_ = control_rate_ / passive_loop_rate_;
  ROS_INFO("passive loop cnt: %d", passive_loop_cnt_);


  att_comp_duration_size_ = att_control_rate_ * att_comp_duration_;

  gimbal_modules_.resize(gimbal_module_num_);

  final_attitude_.x = 0;
  final_attitude_.y = 0;
  desire_attitude_.x = 0;
  desire_attitude_.y = 0;

  passive_tilt_mode_ = false;
  active_tilt_mode_ = false;

  roll_diff_ = 0; 
  pitch_diff_ = 0; 
  roll_delay_ = 0; 
  pitch_delay_ = 0; 


  //attack mode
  nhp_.param("attack_vel_y", attack_vel_y_, -0.3);
  nhp_.param("attack_vel_x", attack_vel_x_, 0.0);
  nhp_.param("rebound_vel_y", rebound_vel_y_, 0.1);
  nhp_.param("attack_acc_thre", attack_acc_thre_, 2.5);
  nhp_.param("attack_tilt_angle", attack_tilt_angle_, 0.17);
  nhp_.param("attack_back_level_interval", attack_back_level_interval_, 10.0);


  wall_attack_flag_ = false;
  attack_back_level_flag_ = false;
  attack_back_level_start_time_ = ros::Time::now();



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

}

void GimbalControl::attCommandCallback(const aerial_robot_base::FlatnessPidConstPtr& cmd_msg)
{
  boost::lock_guard<boost::mutex> lock(queue_mutex_);
  while (att_command_qu_.size() >= att_comp_duration_size_)
    {
      att_command_qu_.pop_front();
    }

  geometry_msgs::Vector3 new_command;
  new_command.x = cmd_msg->roll.total[0];
  new_command.y = cmd_msg->pitch.total[0];

  att_command_qu_.push_back(new_command);
}

void GimbalControl::desireAttitudeCallback(const geometry_msgs::Vector3ConstPtr& msg)
{

  final_attitude_ = *msg;
  active_tilt_mode_ = true;
}

void GimbalControl::attitudeCallback(const aerial_robot_msgs::ImuConstPtr& msg)
{
  current_attitude_.x = msg->angles[0];
  current_attitude_.y = msg->angles[1];
  current_attitude_.z = msg->angles[2];
  geometry_msgs::Vector3 current_acc_;
  current_acc_.x = msg->acc_data[0];
  current_acc_.y = msg->acc_data[1];
  current_acc_.z = msg->acc_data[2];

  if(wall_attack_flag_)
    {

      //demo, temporary
      if(current_acc_.y > attack_acc_thre_) 
        {
          ROS_WARN("Right Side Attack!!");
          active_tilt_mode_ = true;
          final_attitude_.x = attack_tilt_angle_;
          final_attitude_.y = 0;
          final_attitude_.z = 0;
        }
      if(current_acc_.y < -attack_acc_thre_) 
        {
          ROS_WARN("Left Side Attack!!");
          active_tilt_mode_ = true;
          final_attitude_.x = -attack_tilt_angle_;
          final_attitude_.y = 0;
          final_attitude_.z = 0;
        }

    }

  att_comp_time_ = msg->stamp;
}


void GimbalControl::servoCallback(const dynamixel_msgs::JointStateConstPtr& msg, int i, int j)
{
  gimbal_modules_[i].current_angle[j] = gimbal_modules_[i].angle_sgn[j] * (msg->current_pos - gimbal_modules_[i].angle_offset[j]);
}

void GimbalControl::gimbalControl(Eigen::Quaternion<double> q_att)
{

  static Eigen::Quaternion<double> prev_q_att = Eigen::Quaternion<double>(1,0,0,0);
  static float tilt_alt_sum = 0;

  if (gimbal_debug_)
    {
      q_att = Eigen::AngleAxisd(current_attitude_.y, Eigen::Vector3d::UnitY()) 
        * Eigen::AngleAxisd(current_attitude_.x, Eigen::Vector3d::UnitX());
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
  flight_nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
  flight_nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
  flight_nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::VEL_MODE;
  flight_nav_msg.target_pos_diff_z = body_diameter_ / 2 * (alt_tilt - prev_alt_tilt);
  alt_control_pub_.publish(flight_nav_msg);

  tilt_alt_sum += flight_nav_msg.target_pos_diff_z;

  //ROS_INFO("tilt_alt: %f, sum: %f", flight_nav_msg.target_pos_diff_z, tilt_alt_sum);

  prev_q_att = q_att;
}

void GimbalControl::controlFunc(const ros::TimerEvent & e)
{
  static ros::Time prev_update_time = ros::Time::now();

  static ros::Time passive_tilt_start_time = ros::Time::now();
  static int passive_loop_cnt = -1;

  if (gimbal_debug_)
    {
      Eigen::Quaternion<double> q_att(1,0,0,0);
      gimbalControl(q_att);
      return;
    }

  //attack mode
  if(wall_attack_flag_)
    {
      if(attack_back_level_flag_)
        {
          if(ros::Time::now().toSec() - attack_back_level_start_time_.toSec() 
             > attack_back_level_interval_)
            {
              final_attitude_.x = 0;
              final_attitude_.y = 0;
              final_attitude_.z = 0;
              active_tilt_mode_ = true;
              attack_back_level_flag_ = false;
              wall_attack_flag_ = false;
              active_gimbal_tilt_duration_ *= 3;
            }
        }

      //send vel command
      aerial_robot_base::FlightNav flight_nav_msg;
      flight_nav_msg.header.stamp = ros::Time::now();
      flight_nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::VEL_MODE;
      flight_nav_msg.target_vel_x = attack_vel_x_;
      flight_nav_msg.target_vel_y = attack_vel_y_;
      if(active_tilt_mode_ || attack_back_level_flag_)
        {
           flight_nav_msg.target_vel_x = 0;
          // flight_nav_msg.target_vel_y = 0;
           // demo temporary
          flight_nav_msg.target_vel_y = rebound_vel_y_;
        }
      flight_nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
      alt_control_pub_.publish(flight_nav_msg);

    }

  if(gimbal_mode_ == ACTIVE_GIMBAL_MODE || gimbal_mode_ == ACTIVE_GIMBAL_MODE + PASSIVE_GIMBAL_MODE)
    {

      if(!active_tilt_mode_) return;

      //duration processing => should be erro estimation processing
      if(ros::Time::now().toSec() - prev_update_time.toSec() < active_gimbal_tilt_duration_) return;

      if(final_attitude_.x - desire_attitude_.x > active_gimbal_tilt_interval_)
        desire_attitude_.x += active_gimbal_tilt_interval_;
      else if (final_attitude_.x - desire_attitude_.x < -active_gimbal_tilt_interval_)
        desire_attitude_.x -= active_gimbal_tilt_interval_;
      else desire_attitude_.x = final_attitude_.x;

      if(final_attitude_.y - desire_attitude_.y > active_gimbal_tilt_interval_)
        desire_attitude_.y += active_gimbal_tilt_interval_;
      else if (final_attitude_.y - desire_attitude_.y < -active_gimbal_tilt_interval_)
        desire_attitude_.y -= active_gimbal_tilt_interval_;
      else desire_attitude_.y = final_attitude_.y;

      ROS_INFO("desire roll: %f, desire pitch: %f", desire_attitude_.x, desire_attitude_.y);


      Eigen::Quaternion<double> q_att = Eigen::AngleAxisd(desire_attitude_.y, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(desire_attitude_.x, Eigen::Vector3d::UnitX());
      gimbalControl(q_att);

      prev_update_time = ros::Time::now();

      //publish to control board for desire tilt angle
      desire_attitude_.z = ABSOLUTE_ATTITUDE; // absolute mode
      desire_tilt_pub_.publish(desire_attitude_);

      if(desire_attitude_.y == final_attitude_.y && desire_attitude_.x == final_attitude_.x)
        {
          active_tilt_mode_ = false;

          //attack mode
          if(wall_attack_flag_ && !attack_back_level_flag_)
            {
              attack_back_level_flag_ = true;
              attack_back_level_start_time_ = ros::Time::now();
            }

          if(gimbal_mode_ == ACTIVE_GIMBAL_MODE + PASSIVE_GIMBAL_MODE)
            {

              gimbal_mode_ = PASSIVE_GIMBAL_MODE;
              ROS_WARN("back to PASSIVE_GIMBAL_MODE");
            }
        }

    }
  else if(gimbal_mode_ == PASSIVE_GIMBAL_MODE)
    {
      if(passive_loop_cnt >= 0)  passive_loop_cnt++;
      if(passive_loop_cnt >= passive_loop_cnt_) passive_loop_cnt = -1;
      if(passive_loop_cnt > 0) return; // for the delay of current attitude in this thread, when compare with attitude_command(controller debug);

      geometry_msgs::Vector3Stamped comp_msg;
      comp_msg.header.stamp = att_comp_time_;

      if(!attCommandCompare()) return;

      comp_msg.vector.x = roll_diff_;
      comp_msg.vector.y = pitch_diff_;
      comp_msg.vector.z = 0;

      if(fabs(roll_diff_) > gimbal_thre_ ||
         fabs(pitch_diff_) > gimbal_thre_)
        {
          //final_attitude_ = current_attitude_ ;
          //desire_attitude_ = final_attitude_;
          Eigen::Quaternion<double> q_curr = Eigen::AngleAxisd(current_attitude_.y, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(current_attitude_.x, Eigen::Vector3d::UnitX()) ;
          Eigen::Quaternion<double> q_des = Eigen::AngleAxisd(desire_attitude_.y, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(desire_attitude_.x, Eigen::Vector3d::UnitX()) ;
          Eigen::Quaternion<double> q_att = q_curr * q_des;
          gimbalControl(q_att);

          Eigen::Matrix3d rotation = q_att.matrix();
          desire_attitude_.x = atan2(rotation(2,1),rotation(2,2));
          desire_attitude_.y = atan2(-rotation(2,0),  sqrt(rotation(2,1)*rotation(2,1) + rotation(2,2)* rotation(2,2)));


          //publish to control board for desire tilt angle
          //desire_attitude_.z = RELATIVE_ATTITUDE; //relative mode
          desire_attitude_.z = ABSOLUTE_ATTITUDE; // absolute mode
          desire_tilt_pub_.publish(desire_attitude_);

          ROS_WARN("big att change in passive mode: roll_diff: %f, roll_delay:%f, pitch_diff: %f, pitch_delay: %f", roll_diff_, roll_delay_, pitch_diff_, pitch_delay_);

          comp_msg.vector.z = 1;
          passive_tilt_mode_ = true;
          passive_tilt_start_time = ros::Time::now();

          passive_loop_cnt = 0;
        }
      att_diff_pub_.publish(comp_msg);

      if( passive_tilt_mode_ && ros::Time::now().toSec() - passive_tilt_start_time.toSec() > passive_level_back_duration_)
        {
          passive_tilt_mode_ = false;
          active_tilt_mode_ = true;
          final_attitude_.x = 0;
          final_attitude_.y = 0;

          gimbal_mode_ = ACTIVE_GIMBAL_MODE + PASSIVE_GIMBAL_MODE;
          ROS_WARN("in order to back to level, turn to acitve tilt mode");
        }
      
    }
}

bool GimbalControl::attCommandCompare()
{
  boost::lock_guard<boost::mutex> lock(queue_mutex_);

  static geometry_msgs::Vector3 previous_attitude_;

  //outlier attitude data
  if(fabs(previous_attitude_.x - current_attitude_.x) > attitude_outlier_thre_ ||
     fabs(previous_attitude_.y - current_attitude_.y) > attitude_outlier_thre_)
    return false;

  //no enough buffer to compare
  if(att_command_qu_.size() < att_comp_duration_size_) return false;

  int i = 0;
  float min_roll_time = 0, min_pitch_time = 0;
  float min_roll_diff = 1e6, min_pitch_diff = 1e6;
  for(std::deque<geometry_msgs::Vector3>::iterator itr = att_command_qu_.begin(); itr != att_command_qu_.end(); ++itr) 
    {
      //roll
      if(fabs((*itr).x - current_attitude_.x) <  min_roll_diff)
        {
          min_roll_diff = fabs((*itr).x - current_attitude_.x);
          min_roll_time = (att_comp_duration_size_- i ) * (1 / att_control_rate_);
        }
      //pitch
      if(fabs((*itr).y - current_attitude_.y) <  min_pitch_diff)
        {
          min_pitch_diff = fabs((*itr).y - current_attitude_.y);
          min_pitch_time = (att_comp_duration_size_-i ) * (1 / att_control_rate_);
        }

      i++;
    }

  roll_diff_ = min_roll_diff; 
  pitch_diff_ = min_pitch_diff; 
  roll_delay_ = min_roll_time; 
  pitch_delay_ = min_pitch_time; 

  return true;
}

void GimbalControl::GimbalDynReconfCallback(di_control::GimbalDynReconfConfig &config, uint32_t level)
{
  if(config.gimbal_control_flag)
    {
      active_gimbal_tilt_interval_ = config.active_gimbal_tilt_interval;
      active_gimbal_tilt_duration_ = config.active_gimbal_tilt_duration;
      ROS_WARN("new gimbal interval :%f, new gimbal duration :%f", active_gimbal_tilt_interval_, active_gimbal_tilt_duration_);
    }
}

void GimbalControl::joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
{
    static bool attack_flag   = false;
    static bool stop_attack_flag   = false;
      if(joy_msg->buttons[4] == 1 && !attack_flag)
        {
          ROS_INFO("Start attack mode");
          attack_flag = true;
	  wall_attack_flag_ = true;

	  //tilt mode change
	  gimbal_mode_ = ACTIVE_GIMBAL_MODE;

          //stop teleop
          std_msgs::UInt8 stop_teleop_msg;
          stop_teleop_msg.data = 1;
          stop_teleop_pub_.publish(stop_teleop_msg);

        }
      if(joy_msg->buttons[4] == 0 && attack_flag)
        attack_flag = false;


      if(joy_msg->buttons[6] == 1 && !stop_attack_flag)
        {
          ROS_INFO("Stop attack mode");
          stop_attack_flag = true;
	  wall_attack_flag_ = false;

	  //tilt mode change
	  gimbal_mode_ = ACTIVE_GIMBAL_MODE;

          //stop teleop
          std_msgs::UInt8 start_teleop_msg;
          start_teleop_msg.data = 0;
          stop_teleop_pub_.publish(start_teleop_msg);

        }
      if(joy_msg->buttons[6] == 0 && stop_attack_flag)
        stop_attack_flag = false;
}
