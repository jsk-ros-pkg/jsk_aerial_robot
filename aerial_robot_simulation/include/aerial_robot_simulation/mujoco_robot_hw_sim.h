#include <ros/ros.h>
#include <aerial_robot_model/model/aerial_robot_model.h>
#include <aerial_robot_simulation/noise_model.h>
#include <aerial_robot_simulation/spinal_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/transmission_info.h>
#include <urdf/model.h>
#include <mujoco/mujoco.h>

namespace mujoco_ros_control
{
  class MujocoRobotHWSim: public hardware_interface::RobotHW
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

  protected:
    hardware_interface::SpinalInterface spinal_interface_;

  }

}
