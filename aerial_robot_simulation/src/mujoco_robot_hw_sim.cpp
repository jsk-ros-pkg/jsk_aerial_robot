#include <aerial_robot_simulation/mujoco_robot_hw_sim.h>

MujocoRobotHWSim::MujocoRobotHWSim(ros::NodeHandle nh):
  nh_(nh),
  clock_spinner_(1, &clock_loop_queue_),
  read_spinner_(1, &read_loop_queue_)
{

  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 10);

  zero_time_ = ros::Time::now();
  ros::TimerOptions clock_ops(ros::Duration(1.0 / 1000.0),
                              boost::bind(&MujocoRobotHWSim::clockCallback, this, _1),
                              &clock_loop_queue_);
  clock_timer_ = nh_.createTimer(clock_ops);
  clock_spinner_.start();

  ros::TimerOptions read_ops(ros::Duration(1.0 / 200.0),
                                boost::bind(&MujocoRobotHWSim::readSim, this, _1),
                                &read_loop_queue_);
  read_timer_ = nh_.createTimer(read_ops);
  read_spinner_.start();

  while(ros::ok())
    {
    }
}

MujocoRobotHWSim::~MujocoRobotHWSim()
{
  clock_timer_.stop();
  clock_spinner_.stop();
  read_timer_.stop();
  read_spinner_.stop();

}

void MujocoRobotHWSim::clockCallback(const ros::TimerEvent & e)
{
  rosgraph_msgs::Clock ros_time;
  ros_time.clock.fromSec((ros::Time::now() - zero_time_).toSec());
  clock_pub_.publish(ros_time);

}

void MujocoRobotHWSim::readSim(const ros::TimerEvent & e)
{
}


void MujocoRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mujoco_robot_hw_sim");
  ros::NodeHandle nh;

  MujocoRobotHWSim mujoco_robot_hw_sim(nh);
  // mujoco_robot_hw_sim.initSim(nh);
}
