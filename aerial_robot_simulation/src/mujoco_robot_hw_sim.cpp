#include <aerial_robot_simulation/mujoco_robot_hw_sim.h>

namespace mujoco_ros_control
{
  bool MujocoRobotHWSim::initSim(const std::string& robot_namespace,
                                 ros::NodeHandle model_nh,
                                 mjModelPtr m,
                                 mjDataPtr d,
                                 std::vector<transmission_interface::TransmissionInfo> transmissions
                                 )
  {

  }

  void readSim(ros::Time time, ros::Duration period)
  {

  }


  void writeSim(ros::Time time, ros::Duration period)
  {

  }

}
