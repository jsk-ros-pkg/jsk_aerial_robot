#include <aerial_robot_simulation/mujoco_attitude_controller.h>

namespace flight_controllers
{
  MujocoAttitudeController::MujocoAttitudeController():
    controller_core_(new FlightControl())
  {
    std::cout << "contoller constructor" << std::endl;
  }

  bool MujocoAttitudeController::init(hardware_interface::MujocoSpinalInterface *robot, ros::NodeHandle &n)
  {
    spinal_interface_ = robot;

    std::cout << "mujoco attitude controller init function" << std::endl;

    int index = n.getNamespace().rfind('/');
    std::string robot_ns = n.getNamespace().substr(0, index);
    ros::NodeHandle n_robot = ros::NodeHandle(robot_ns);
    controller_core_->init(&n_robot, robot->getEstimatorPtr());

    return true;

  }

  void MujocoAttitudeController::starting(const ros::Time& time)
  {
  }

  void MujocoAttitudeController::update(const ros::Time& time, const ros::Duration& period)
  {
    /* set ground truth value for control */
    auto true_cog_rpy = spinal_interface_->getTrueCogRPY();
    controller_core_->getAttController().setTrueRPY(true_cog_rpy.x(), true_cog_rpy.y(), true_cog_rpy.z());
    auto true_cog_angular = spinal_interface_->getTrueCogAngular();
    controller_core_->getAttController().setTrueAngular(true_cog_angular.x(), true_cog_angular.y(), true_cog_angular.z());

    /* freeze the attitude estimator while touching the ground, since the bad contact simulation performance in gazebo */
    spinal_interface_->onGround(!controller_core_->getAttController().getIntegrateFlag());

    /* update the controller */
    controller_core_->update();

    // std::cout << "controller update" << std::endl;

  }
}


PLUGINLIB_EXPORT_CLASS(flight_controllers::MujocoAttitudeController, controller_interface::ControllerBase)
