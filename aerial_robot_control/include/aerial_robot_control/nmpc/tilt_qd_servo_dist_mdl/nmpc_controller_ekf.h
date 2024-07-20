//
// Created by lijinjie on 24/07/18.
//

#ifndef TILT_QD_SERVO_NMPC_W_EKF_CONTROLLER_H
#define TILT_QD_SERVO_NMPC_W_EKF_CONTROLLER_H

#include "nmpc_controller.h"
#include "nsim_solver.h"
#include "aerial_robot_control/nmpc/kalman_filter.h"

#include "geometry_msgs/WrenchStamped.h"

using NMPCControlDynamicConfig = dynamic_reconfigure::Server<aerial_robot_control::NMPCConfig>;

namespace aerial_robot_control
{

namespace nmpc
{

class TiltQdServoNMPCwEKF : public nmpc::TiltQdServoDistNMPC
{
protected:
  std::unique_ptr<mpc_sim_solver::TiltQdServoDistMdlSimSolver> sim_solver_ptr_;
  KalmanFilter ekf_;

  inline void initMPCSolverPtr() override
  {
    mpc_solver_ptr_ = std::make_unique<mpc_solver::TiltQdServoDistMdlMPCSolver>();
    sim_solver_ptr_ = std::make_unique<mpc_sim_solver::TiltQdServoDistMdlSimSolver>();
  }

  void initParams() override;

  void calcDisturbWrench() override;

  std::vector<double> meas2VecX() override;

  void cfgNMPCCallback(NMPCConfig& config, uint32_t level) override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_QD_SERVO_NMPC_W_EKF_CONTROLLER_H