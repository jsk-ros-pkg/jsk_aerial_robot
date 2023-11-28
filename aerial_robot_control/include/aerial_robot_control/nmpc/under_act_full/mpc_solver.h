// -*- mode: c++ -*-
//
// Created by lijinjie on 23/11/25.
//

#ifndef UNDER_ACT_FULL_MPC_SOLVER_H
#define UNDER_ACT_FULL_MPC_SOLVER_H

#endif  // UNDER_ACT_FULL_MPC_SOLVER_H

// acados
#include "aerial_robot_control/nmpc/base_mpc_solver.h"
#include "aerial_robot_control/nmpc/under_act_full/c_generated_code/acados_solver_qd_full_model.h"

namespace aerial_robot_control
{

namespace nmpc_under_act_full
{

class MPCSolver : public base_nmpc::BaseMPCSolver
{
public:
  MPCSolver() = default;
  ~MPCSolver() = default;
  void initialize() override;
  void updateConstraints(double v_max, double v_min, double w_max, double w_min, double thrust_max, double thrust_min);

private:
  void initAcadosVariables() override;
  void initParameters() override;
  void setFeedbackConstraints(const nav_msgs::Odometry& odom_now) override;
};

}  // namespace nmpc_under_act_full

}  // namespace aerial_robot_control
