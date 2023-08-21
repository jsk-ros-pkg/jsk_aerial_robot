#include <aerial_robot_simulation/robot_hw_sim.h>

namespace mujoco_ros_control
{

  bool MujocoRobotHWSim::init(const std::string& robot_namespace,
                              ros::NodeHandle model_nh,
                              mjModel* mujoco_model,
                              mjData* mujoco_data
                              )
  {
    mujoco_model_ = mujoco_model;
    mujoco_data_ = mujoco_data;

    /* Initialize spinal interface */
    spinal_interface_.init(model_nh);
    registerInterface(&spinal_interface_);

    std::cout << "mujoco robot hw sim init" << std::endl;
    return true;
  }

  void MujocoRobotHWSim::read(const ros::Time& time, const ros::Duration& period)
  {
    int fc_id = mj_name2id(mujoco_model_, mjtObj_::mjOBJ_SITE, "fc");
    mjtNum* site_xpos = mujoco_data_->site_xpos;
    mjtNum* site_xmat = mujoco_data_->site_xmat;

    // std::cout << "read func" << std::endl;
  }

  void MujocoRobotHWSim::write(const ros::Time& time, const ros::Duration& period)
  {
    // std::cout << "write func" << std::endl;
  }
}

PLUGINLIB_EXPORT_CLASS(mujoco_ros_control::MujocoRobotHWSim, mujoco_ros_control::MujocoRobotHWSimPlugin);
