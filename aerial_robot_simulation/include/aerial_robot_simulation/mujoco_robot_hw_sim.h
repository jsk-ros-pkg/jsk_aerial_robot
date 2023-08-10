#include <ros/ros.h>
#include <aerial_robot_model/model/aerial_robot_model.h>
#include <aerial_robot_simulation/mujoco_spinal_interface.h>
#include <mujoco/mujoco.h>

class MujocoRobotHWSim
{
public:
  bool initSim(const std::string& robot_namespace,
               ros::NodeHandle model_nh,
               mjModelPtr m,
               mjDataPtr d,
               std::vector<transmission_interface::TransmissionInfo> transmissions
               ) override;
  void readSim(ros::Time time, ros::Duration period) override;
  void writeSim(ros::Time time, ros::Duration period) override;

private:
  hardware_interface::SpinalInterface spinal_interface_;

};

