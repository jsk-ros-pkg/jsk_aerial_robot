#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseArray.h>
#include <unistd.h>

class Opt
{
public:
  Opt(ros::NodeHandle nh, ros::NodeHandle nh_private): nh_(nh), nhp_(nh_private), start_flag_(false)
  {
    gimbals_control_pub_ = nhp_.advertise<sensor_msgs::JointState>("/rolling/gimbals_ctrl", 1);
    start_sub_ = nh_.subscribe<std_msgs::Empty>("start_log", 1, &Opt::startCallback, this);
    radius_sub_ = nhp_.subscribe<std_msgs::Float32>("/rolling/feasible_control_torque_radius", 1, &Opt::radiusCallback, this);
    feasible_force_convex_sub_ = nhp_.subscribe<geometry_msgs::PoseArray>("/rolling/feasible_control_force_convex", 1, &Opt::forceConvexCallback, this);
    feasible_torque_convex_sub_ = nhp_.subscribe<geometry_msgs::PoseArray>("/rolling/feasible_control_torque_convex", 1, &Opt::torqueConvexCallback, this);
    nh_.param("step_num", step_num_, 50);
    nh_.param("hz", hz_, 30);
    nh_.param("max_tilt", max_tilt_, M_PI/3.0);
    nh_.param("mass", mass_, 3.0);
    nh_.param("max_thrust", max_thrust_, 20.0);
    nh_.param("max_gimbal", max_gimbal_, M_PI/2.0);
    ROS_INFO("waiting start log");

    force_convex_.resize(6);
    torque_convex_.resize(6);
    radians_.resize(3);
    max_radians_.resize(3);
    max_radius_ = 0.0;

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
  ros::Subscriber feasible_force_convex_sub_;
  ros::Subscriber feasible_torque_convex_sub_;
  ros::Timer timer_;
  std::ofstream ofs_;

  sensor_msgs::JointState joint_state_msg_;

  int step_num_;
  int hz_;
  int rotor1_tilt_cnt_ = 0;
  int rotor2_tilt_cnt_ = 0;
  int rotor3_tilt_cnt_ = 0;
  int thrust_step_num_ = 100;

  bool start_flag_;
  bool reset_flag_ = false;
  bool finished_ = false;

  double radius_;
  double max_radius_;
  double max_tilt_;
  double mass_;
  double max_thrust_;
  double max_gimbal_;

  std::vector<Eigen::Vector3d> force_convex_;
  std::vector<Eigen::Vector3d> torque_convex_;
  std::vector<double> radians_;
  std::vector<double> max_radians_;

  void startCallback(const std_msgs::EmptyConstPtr & msg)
  {
    std::string file_name  = std::string("opt_") + std::to_string((int)ros::Time::now().toSec()) + std::string(".txt");
    ofs_.open(file_name, std::ios::out);
    std::vector<std::string> gimbals(3);
    // gimbals.at(0) = "gimbal1";
    // gimbals.at(1) = "gimbal2";
    // gimbals.at(2) = "gimbal3";
    gimbals.at(0) = "gimbal_pitch1";
    gimbals.at(1) = "gimbal_pitch2";
    gimbals.at(2) = "gimbal_pitch3";
    joint_state_msg_.name = gimbals;
    ROS_INFO("start searching");
    start_flag_ = true;
    reset_flag_ = true;
  }

  double cntToTiltRadianConvertion(int num)
  {
    return (double)num/step_num_ * 2.0 * max_tilt_ - max_tilt_;
  }

  double cntToGimbalRadianConversion(int num)
  {
    return (double)num/step_num_ * 2.0 * max_gimbal_ - max_gimbal_;
  }

  void forceConvexCallback(const geometry_msgs::PoseArrayConstPtr & msg)
  {
    for(int i = 0; i < msg->poses.size(); i++)
      {
        force_convex_.at(i) = Eigen::Vector3d(msg->poses.at(i).position.x, msg->poses.at(i).position.y, msg->poses.at(i).position.z);
      }
  }

  void torqueConvexCallback(const geometry_msgs::PoseArrayConstPtr & msg)
  {
    for(int i = 0; i < msg->poses.size(); i++)
      {
        torque_convex_.at(i) = Eigen::Vector3d(msg->poses.at(i).position.x, msg->poses.at(i).position.y, msg->poses.at(i).position.z);
      }
  }

  void radiusCallback(const std_msgs::Float32ConstPtr & msg)
  {
    radius_ = msg->data;
  }

  void thrustSearch()
  {
    for(int i = 0; i < thrust_step_num_; i++)
      {
        double thrust_i = (double)i / (double)thrust_step_num_ * max_thrust_;
        Eigen::Vector3d force_i = force_convex_.at(0) * thrust_i;
        Eigen::Vector3d torque_i = torque_convex_.at(0) * thrust_i;
        for(int j = 0; j < thrust_step_num_; j++)
          {
            double thrust_j = (double)j / (double)thrust_step_num_ * max_thrust_;
            Eigen::Vector3d force_j = force_convex_.at(1) * thrust_j;
            Eigen::Vector3d torque_j = torque_convex_.at(1) * thrust_j;
            for(int k = 0; k < thrust_step_num_; k++)
              {
                double thrust_k = (double)k / (double)thrust_step_num_ * max_thrust_;
                Eigen::Vector3d force_k = force_convex_.at(2) * thrust_k;
                Eigen::Vector3d torque_k = torque_convex_.at(2) * thrust_k;

                Eigen::Vector3d force = force_i + force_j + force_k;
                Eigen::Vector3d torque = torque_i + torque_j + torque_k;

                // if(force(2) > mass_ * 9.8)
                //   {
                //     std::cout << radians_.at(0) << " " << radians_.at(1) << " " << radians_.at(2) << " " << radius_ <<std::endl;
                //     ofs_ << radians_.at(0) << " " << radians_.at(1) << " " << radians_.at(2) << " " << radians_.at(3) << " " << radians_.at(4) << " " << radians_.at(5) << " " << std::min(fabs(torque(0)), std::min(fabs(torque(1)), fabs(torque(2)))) <<std::endl;
                //   }
              }
          }
      }
  }

  void optFunc(const ros::TimerEvent & e)
  {
    if(!start_flag_) return;
    radians_.at(0) = cntToTiltRadianConvertion(rotor1_tilt_cnt_);
    radians_.at(1) = cntToTiltRadianConvertion(rotor2_tilt_cnt_);
    radians_.at(2) = cntToTiltRadianConvertion(rotor3_tilt_cnt_);
    // radians_.at(0) = cntToGimbalRadianConversion(rotor1_tilt_cnt_);
    // radians_.at(1) = cntToGimbalRadianConversion(rotor2_tilt_cnt_);
    // radians_.at(2) = cntToGimbalRadianConversion(rotor3_tilt_cnt_);

    joint_state_msg_.position = radians_;
    gimbals_control_pub_.publish(joint_state_msg_);

    if(reset_flag_)
      {
        usleep(500 * 1000);
        reset_flag_ = false;
      }

    std::cout << radians_.at(0) << " " << radians_.at(1) << " " << radians_.at(2) << " " << radius_ <<std::endl;
    ofs_ << radians_.at(0) << " " << radians_.at(1) << " " << radians_.at(2) << " " << radius_ << std::endl;

    if(radius_ > max_radius_)
      {
        max_radius_ = radius_;
        max_radians_.at(0) = radians_.at(0);
        max_radians_.at(1) = radians_.at(1);
        max_radians_.at(2) = radians_.at(2);
      }

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
                std::cout << "max_radius: " << max_radius_ << ". pitch pairs [ " << max_radians_.at(0) << " " << max_radians_.at(1) << " " << max_radians_.at(2) << "]."<< std::endl;
                ofs_ << "max_radius: " << max_radius_ << ". pitch pairs [ " << max_radians_.at(0) << " " << max_radians_.at(1) << " " << max_radians_.at(2) << "]."<< std::endl;
                ofs_.close();
                start_flag_ = false;
                finished_ = true;
              }
          }
      }

    if(finished_)
      {
        // publish optimized angles
        radians_.at(0) = max_radians_.at(0);
        radians_.at(1) = max_radians_.at(1);
        radians_.at(2) = max_radians_.at(2);
        joint_state_msg_.position = radians_;
        gimbals_control_pub_.publish(joint_state_msg_);
      }

    // for(int i = 0; i < step_num_; i++)
    //   {
    //     for(int j = 0; j < step_num_; j++)
    //       {
    //         for(int k = 0; k < step_num_; k++)
    //           {
    //             for(int l = 0; l < step_num_; l++)
    //               {
    //                 for
    //               }
    //           }
    //       }
    //   }

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
