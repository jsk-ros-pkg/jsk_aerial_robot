#include <ros/ros.h>
#include <fstream>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <unistd.h>

class Opt
{
public:
  Opt(ros::NodeHandle nh, ros::NodeHandle nh_private): nh_(nh), nhp_(nh_private), start_flag_(false)
  {
    gimbals_control_pub_ = nhp_.advertise<sensor_msgs::JointState>("/rolling/gimbals_ctrl", 1);
    start_sub_ = nh_.subscribe<std_msgs::Empty>("start_log", 1, &Opt::startCallback, this);
    radius_sub_ = nhp_.subscribe<std_msgs::Float32>("/rolling/feasible_control_torque_radius", 1, &Opt::radiusCallback, this);

    nh_.param("step_num", step_num_, 100);
    nh_.param("hz", hz_, 10);
    nh_.param("max_tilt", max_tilt_, M_PI/4.0);

    ROS_INFO("waiting start log");
    timer_ = nhp_.createTimer(ros::Duration((float)1.0/(float)hz_), &Opt::optFunc, this);
  }

  ~Opt()
  {
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher gimbals_control_pub_;
  ros::Subscriber start_sub_;
  ros::Subscriber radius_sub_;
  ros::Timer timer_;
  std::ofstream ofs_;

  sensor_msgs::JointState joint_state_msg_;

  int step_num_;
  int hz_;
  int rotor1_tilt_cnt_ = 0;
  int rotor2_tilt_cnt_ = 0;
  int rotor3_tilt_cnt_ = 0;

  bool start_flag_;
  bool reset_flag_ = false;

  double radius_;
  double max_tilt_;

  void startCallback(const std_msgs::EmptyConstPtr & msg)
  {
    std::string file_name  = std::string("opt_") + std::to_string((int)ros::Time::now().toSec()) + std::string(".txt");
    ofs_.open(file_name, std::ios::out);
    std::vector<std::string> rotors(3);
    rotors.at(0) = "gimbal_coord1";
    rotors.at(1) = "gimbal_coord2";
    rotors.at(2) = "gimbal_coord3";
    joint_state_msg_.name = rotors;
    ROS_INFO("start searching");
    start_flag_ = true;
    reset_flag_ = true;
  }

  double cntToRadianConvertion(int num)
  {
    return (double)num/step_num_ * 2.0 * max_tilt_ - max_tilt_;
  }

  void radiusCallback(const std_msgs::Float32ConstPtr & msg)
  {
    radius_ = msg->data;
  }

  void optFunc(const ros::TimerEvent & e)
  {
    if(!start_flag_) return;
    std::vector<double> radians(3);
    radians.at(0) = cntToRadianConvertion(rotor1_tilt_cnt_);
    radians.at(1) = cntToRadianConvertion(rotor2_tilt_cnt_);
    radians.at(2) = cntToRadianConvertion(rotor3_tilt_cnt_);

    joint_state_msg_.position = radians;
    gimbals_control_pub_.publish(joint_state_msg_);

    if(reset_flag_)
      {
        usleep(1000 * 1000);
        reset_flag_ = false;
      }

    std::cout << radians.at(0) << " " << radians.at(1) << " " << radians.at(2) << " " << radius_ <<std::endl;
    ofs_ << radians.at(0) << " " << radians.at(1) << " " << radians.at(2) << " " << radius_ <<std::endl;

    rotor3_tilt_cnt_++;
    if(rotor3_tilt_cnt_ > step_num_)
      {
        rotor2_tilt_cnt_++;
        rotor3_tilt_cnt_ = 0;
        reset_flag_ = true;

        if(rotor2_tilt_cnt_ > step_num_)
          {
            rotor1_tilt_cnt_++;
            rotor2_tilt_cnt_ = 0;
            reset_flag_ = true;

            if(rotor1_tilt_cnt_ > step_num_)
              {
                ROS_INFO("finish searching");
                ofs_ << "done" << std::endl;
                ofs_.close();
                start_flag_ = false;
              }
          }
      }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "opt");
  ros::NodeHandle nh("~");
  ros::NodeHandle nhp("rolling");
  Opt* node = new Opt(nh, nhp);
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  delete node;
  return 0;
}
