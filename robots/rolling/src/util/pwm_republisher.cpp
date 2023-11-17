#include <rolling/util/pwm_republisher.h>

MotorPWMRepublisher::MotorPWMRepublisher(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh),
  nhp_(nhp)
{
  nhp_.param("motor_num", motor_num_, 0);

  motor_pwm_sub_ = nh_.subscribe("motor_pwms", 1, &MotorPWMRepublisher::motorPwmCallback, this);
  motor_pwm_pubs_.resize(0);
  for(int i = 0; i < motor_num_; i++)
    {
      motor_pwm_pubs_.push_back(nh_.advertise<std_msgs::Float32>("debug/motor_pwm/motor" + std::to_string(i + 1), 1));
    }
}

void MotorPWMRepublisher::motorPwmCallback(const spinal::PwmsPtr & pwm_msg)
{
  if(pwm_msg->motor_value.size() != motor_num_) return;
  for(int i = 0; i < motor_num_; i++)
    {
      std_msgs::Float32 msg;
      msg.data = pwm_msg->motor_value.at(i);
      motor_pwm_pubs_.at(i).publish(msg);
    }
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
