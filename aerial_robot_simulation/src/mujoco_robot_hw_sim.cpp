#include <aerial_robot_simulation/mujoco_robot_hw_sim.h>

bool MujocoRobotHWSim::initSim(ros::NodeHandle nh,
                               )
{
  nh_ = nh;
  spinal_interface_.init(nh_);

  
}

void MujocoRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  

}


void MujocoRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mujoco_robot_hw_sim");
  ros::NodeHandle nh;

  
}
