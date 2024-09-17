#include <delta/util/pwm_republisher.h>

MotorPWMRepublisher::MotorPWMRepublisher(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh),
  nhp_(nhp)
{
  delta_robot_model_ = boost::make_shared<RollingRobotModel>();

  motor_num_ = delta_robot_model_->getRotorNum();
  m_f_rate_ = delta_robot_model_->getMFRate();
  nhp_.param("tf_prefix", tf_prefix_, std::string(""));

  /* motor pwms */
  motor_pwm_sub_ = nh_.subscribe("motor_pwms", 1, &MotorPWMRepublisher::motorPwmCallback, this);
  motor_pwm_pubs_.resize(0);
  for(int i = 0; i < motor_num_; i++)
    {
      motor_pwm_pubs_.push_back(nh_.advertise<std_msgs::Float32>("debug/motor_pwm/motor" + std::to_string(i + 1), 1));
    }

  /* desire coordinate */
  desire_coordinate_sub_ = nh_.subscribe("desire_coordinate", 1, &MotorPWMRepublisher::desireCoordinateCallback, this);
  desire_coordinate_roll_pub_ = nh_.advertise<std_msgs::Float32>("debug/desire_coordinate/roll", 1);
  desire_coordinate_pitch_pub_ = nh_.advertise<std_msgs::Float32>("debug/desire_coordinate/pitch", 1);
  desire_coordinate_roll_ = 0.0;
  desire_coordinate_pitch_ = 0.0;

  /* gimbal angles */
  gimbals_control_sub_ = nh_.subscribe("gimbals_ctrl", 1, &MotorPWMRepublisher::gimbalsControlCallback, this);
  for(int i = 0; i < motor_num_; i++)
    {
      gimbal_control_pubs_.push_back(nh_.advertise<std_msgs::Float32>("debug/gimbals_ctrl/gimbal" + std::to_string(i + 1), 1));
    }

  /* four axis command */
  four_axis_command_sub_ = nh_.subscribe("four_axes/command", 1, &MotorPWMRepublisher::fourAxisCommandCallback, this);
  for(int i = 0; i < motor_num_; i++)
    {
      thruster_wrench_pubs_.push_back(nh_.advertise<geometry_msgs::WrenchStamped>("debug/thruster_wrench/thruster" + std::to_string(i + 1), 1));
    }

  /* timer */
  timer_ = nh_.createTimer(ros::Duration(0.1), &MotorPWMRepublisher::timerCallback, this);
}

void MotorPWMRepublisher::motorPwmCallback(const spinal::PwmsPtr & pwm_msg)
{
  if(pwm_msg->motor_value.size() < motor_num_) return;
  for(int i = 0; i < motor_num_; i++)
    {
      std_msgs::Float32 msg;
      msg.data = pwm_msg->motor_value.at(i);
      motor_pwm_pubs_.at(i).publish(msg);
    }
}

void MotorPWMRepublisher::desireCoordinateCallback(const spinal::DesireCoord & msg)
{
  desire_coordinate_roll_ = msg.roll;
  desire_coordinate_pitch_ = msg.pitch;
}

void MotorPWMRepublisher::gimbalsControlCallback(const sensor_msgs::JointState & joint_state_msg)
{
  if(joint_state_msg.name.size() != joint_state_msg.position.size())
    {
      if(joint_state_msg.name.size() == 0 && joint_state_msg.position.size() == motor_num_)
        {
          for(int i = 0; i < motor_num_; i++)
            {
              std_msgs::Float32 msg;
              msg.data = joint_state_msg.position.at(i);
              gimbal_control_pubs_.at(i).publish(msg);
            }
          return;
        }
      else
        {
          ROS_ERROR("size of gimbal names and positions is not same");
          return;
        }
    }
  for(int i = 0; i < joint_state_msg.name.size(); i++)
    {
      std_msgs::Float32 msg;
      msg.data = joint_state_msg.position.at(i);
      gimbal_control_pubs_.at(i).publish(msg);
    }
}

void MotorPWMRepublisher::fourAxisCommandCallback(const spinal::FourAxisCommand & msg)
{
  geometry_msgs::WrenchStamped thruster_wrench_msg;
  std::map<int, int> rotor_direction = delta_robot_model_->getRotorDirection();

  for(int i = 0; i < motor_num_; i++)
    {
      thruster_wrench_msg.header.frame_id = tf::resolve(tf_prefix_, std::string("thrust") + std::to_string(i + 1));
      thruster_wrench_msg.wrench.force.z = msg.base_thrust.at(i);
      thruster_wrench_msg.wrench.torque.z = rotor_direction.at(i + 1) *  m_f_rate_ * msg.base_thrust.at(i);
      thruster_wrench_pubs_.at(i).publish(thruster_wrench_msg);
    }
}

void MotorPWMRepublisher::timerCallback(const ros::TimerEvent& e)
{
  std_msgs::Float32 msg;
  msg.data = desire_coordinate_roll_;
  desire_coordinate_roll_pub_.publish(msg);
  msg.data = desire_coordinate_pitch_;
  desire_coordinate_pitch_pub_.publish(msg);
}

int main (int argc, char **argv)
{
  ros::init (argc, argv, "motor_pwm_republisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  MotorPWMRepublisher *motor_pwm_republisher = new MotorPWMRepublisher(nh, nh_private);
  ros::spin();
  delete motor_pwm_republisher;
  return 0;
}
