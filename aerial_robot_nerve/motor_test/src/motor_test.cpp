#include <ros/ros.h>
#include <fstream>
#include <geometry_msgs/WrenchStamped.h>
#include <string>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <takasako_sps/PowerInfo.h>
#include <spinal/PwmTest.h>

namespace Mode
{
  enum {STEP = 0, ONESHOT = 1,};
};

class MotorTest
{
public:
  MotorTest(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), nhp_(nh_private), start_flag_(false), pwm_value_(0), once_flag_(true)
  {
    nhp_.param("test_mode", test_mode_, (int)Mode::STEP);
    ROS_WARN("test mode: %d", test_mode_);
    nhp_.param("run_duration", run_duration_, 4.0);
    nhp_.param("pwm_incremental_value", pwm_incremental_value_, 50);
    nhp_.param("min_pwm_value", min_pwm_value_, 1100);
    nhp_.param("max_pwm_value", max_pwm_value_, 1950);

    nhp_.param("stop_pwm_value", stop_pwm_value_, 1000);
    nhp_.param("pwm_range", pwm_range_, 2000.0);

    /* one-shot mode */
    nhp_.param("raise_duration", raise_duration_, 1.0);
    nhp_.param("brake_duration", brake_duration_, 1.0);

    std::string topic_name;
    nhp_.param("force_sensor_sub_name", topic_name, std::string("forces"));
    force_snesor_sub_ = nh_.subscribe(topic_name, 1, &MotorTest::forceSensorCallback, this, ros::TransportHints().tcpNoDelay());
    nhp_.param("power_info_sub_name", topic_name, std::string("power_info"));
    power_info_sub_ = nh_.subscribe(topic_name, 1, &MotorTest::powerInfoCallback, this, ros::TransportHints().tcpNoDelay());
    nhp_.param("motor_pwm_sub_name", topic_name, std::string("power_pwm"));
    motor_pwm_pub_ = nh_.advertise<spinal::PwmTest>(topic_name,1);

    start_cmd_sub_ =  nh_.subscribe("start_log_cmd", 1,  &MotorTest::startCallback, this, ros::TransportHints().tcpNoDelay());
    sps_on_pub_ = nh.advertise<std_msgs::Empty>("/power_on_cmd", 1);

    ROS_WARN("run: %f, raise: %f, brake: %f", run_duration_, raise_duration_, brake_duration_);

    ros::ServiceClient calib_client = nh_.serviceClient<std_srvs::Empty>("/cfs_sensor_calib");
    std_srvs::Empty srv;
    if (calib_client.call(srv))
      {
        ROS_INFO("done force sensor init calib");
      }
    else
      {
        ROS_ERROR("Failed to call service /cfs_sensor_calib");
      }

    sps_on_pub_.publish(std_msgs::Empty());

    pwm_timer_ = nhp_.createTimer(ros::Duration(1.0 / 50), &MotorTest::pwmFunc,this);
  }

  ~MotorTest()
  {
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Subscriber force_snesor_sub_;
  ros::Subscriber power_info_sub_;
  ros::Subscriber start_cmd_sub_;
  ros::Publisher motor_pwm_pub_;
  ros::Publisher sps_on_pub_;

  ros::Timer  pwm_timer_;

  uint16_t pwm_value_;

  int test_mode_;
  bool start_flag_;
  bool once_flag_;
  double run_duration_;
  double raise_duration_, brake_duration_; // for one-shot mode
  int pwm_incremental_value_;
  int stop_pwm_value_, min_pwm_value_, max_pwm_value_;
  double pwm_range_;

  float currency_;
  ros::Time init_time_;

  std::ofstream ofs_;

  void startCallback(const std_msgs::EmptyConstPtr & msg)
  {
    std::string file_name  = std::string("motor_test_") + std::to_string((int)ros::Time::now().toSec()) + std::string(".txt");
    ofs_.open(file_name, std::ios::out);

    pwm_value_ = min_pwm_value_;

    spinal::PwmTest cmd_msg;
    cmd_msg.pwms.push_back(pwm_value_  / pwm_range_);
    motor_pwm_pub_.publish(cmd_msg);
    init_time_ = ros::Time::now();
    ROS_INFO("start pwm test");
    start_flag_ = true;
  }

  void powerInfoCallback(const takasako_sps::PowerInfoConstPtr& msg)
  {
    currency_ = msg->currency;
  }

  void forceSensorCallback(const geometry_msgs::WrenchStampedConstPtr & msg)
  {
    if(!start_flag_) return;

    double force_norm = sqrt(msg->wrench.force.x * msg->wrench.force.x +
                             msg->wrench.force.y * msg->wrench.force.y +
                             msg->wrench.force.z * msg->wrench.force.z);

    ofs_ << pwm_value_ << " "
        << msg->wrench.force.x << " "
        << msg->wrench.force.y << " "
        << msg->wrench.force.z << " "
        << force_norm << " "
        << msg->wrench.torque.x << " "
        << msg->wrench.torque.y << " "
        << msg->wrench.torque.z << " "
        << currency_;

    if(test_mode_ == Mode::ONESHOT)
      {
        if(ros::Time::now().toSec() - init_time_.toSec() < raise_duration_)
          {
            ofs_ << " " << std::string("raise");
          }
        else
          {
            if(ros::Time::now().toSec() - init_time_.toSec() < raise_duration_ + run_duration_)
              {
                ofs_ << " " << std::string("valid");
              }
            else
              {
                ofs_ << " " << std::string("brake");
              }
          }
      }

    ofs_ << std::endl;
  }

  void pwmFunc(const ros::TimerEvent & e)
  {
    if(!start_flag_) return;

    if(ros::Time::now().toSec() - init_time_.toSec() > run_duration_)
      {
        if(test_mode_ == Mode::STEP)
          {
            pwm_value_ += pwm_incremental_value_;
            init_time_ = ros::Time::now();
          }
        else if(test_mode_ == Mode::ONESHOT)
          {
            if(ros::Time::now().toSec() - init_time_.toSec() > run_duration_ + raise_duration_)
              {
                if(once_flag_)
                  {
                    spinal::PwmTest cmd_msg;
                    cmd_msg.pwms.push_back(stop_pwm_value_  / pwm_range_);
                    motor_pwm_pub_.publish(cmd_msg);
                    once_flag_ = false;
                    ROS_WARN("STOP");
                    return;
                  }
              }
            else
              {
                return;
              }

            if(ros::Time::now().toSec() - init_time_.toSec() > run_duration_ + raise_duration_ + brake_duration_)
              {
                /* need to wait 1s for calibration */
                ros::ServiceClient calib_client = nh_.serviceClient<std_srvs::Empty>("/cfs_sensor_calib");
                std_srvs::Empty srv;
                if (calib_client.call(srv))
                  {
                    ROS_INFO("done force sensor calib");
                    once_flag_ = true;
                    pwm_value_ += pwm_incremental_value_;
                    init_time_ = ros::Time::now();
                  }
                else
                  {
                    ROS_ERROR("Failed to call service add_two_ints");
                    pwm_value_ = max_pwm_value_ + pwm_incremental_value_;
                  }
              }
          }

        if(pwm_value_ > max_pwm_value_)
          {
            start_flag_ = false;
            spinal::PwmTest cmd_msg;
            cmd_msg.pwms.push_back(stop_pwm_value_  / pwm_range_);
            motor_pwm_pub_.publish(cmd_msg);

            ROS_WARN("finish pwm test");
            ofs_ << "done" << std::endl;
            ofs_.close();

            return;
          }

        if(once_flag_)
          {
            ROS_INFO("target_pwm: %d", pwm_value_);
            spinal::PwmTest cmd_msg;
            cmd_msg.pwms.push_back(pwm_value_  / pwm_range_);
            motor_pwm_pub_.publish(cmd_msg);
          }
      }
  }

};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "force_sensor_log");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  MotorTest*  logNode = new MotorTest(nh, nh_private);
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();

  delete logNode;
  return 0;
}
