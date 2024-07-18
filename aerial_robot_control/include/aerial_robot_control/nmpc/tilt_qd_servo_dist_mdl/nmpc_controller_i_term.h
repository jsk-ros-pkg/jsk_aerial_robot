//
// Created by lijinjie on 23/11/29.
//

#ifndef TILT_QD_SERVO_NMPC_W_ITERM_CONTROLLER_H
#define TILT_QD_SERVO_NMPC_W_ITERM_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_qd_servo_mdl/nmpc_controller.h"
#include "nmpc_solver.h"
#include "aerial_robot_control/nmpc/i_term.h"

#include "geometry_msgs/WrenchStamped.h"

using NMPCControlDynamicConfig = dynamic_reconfigure::Server<aerial_robot_control::NMPCConfig>;

namespace aerial_robot_control
{

namespace nmpc
{

class TiltQdServoNMPCwITerm : public nmpc::TiltQdServoNMPC
{
public:
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;

protected:
  ros::Publisher pub_disturb_wrench_;  // for disturbance wrench
  ITerm pos_i_term_[6];                // for x, y, z, roll, pitch, yaw
  geometry_msgs::Vector3 dist_force_w_ = geometry_msgs::Vector3();
  geometry_msgs::Vector3 dist_torque_cog_ = geometry_msgs::Vector3();

  void initParams() override;

  inline void initMPCSolverPtr() override
  {
    mpc_solver_ptr_ = std::make_unique<mpc_solver::TiltQdServoDistMdlMPCSolver>();
  }

  std::vector<double> meas2VecX() override;

  virtual void calcDisturbWrench();

  void callbackViz(const ros::TimerEvent& event) override;

  void cfgNMPCCallback(NMPCConfig& config, uint32_t level) override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_QD_SERVO_NMPC_W_ITERM_CONTROLLER_H