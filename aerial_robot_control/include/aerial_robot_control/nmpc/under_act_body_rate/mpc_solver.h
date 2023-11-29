// -*- mode: c++ -*-
//
// Created by lijinjie on 23/10/22.
//

#ifndef UNDER_ACT_BODY_RATE_MPC_SOLVER_H
#define UNDER_ACT_BODY_RATE_MPC_SOLVER_H

#endif  // UNDER_ACT_BODY_RATE_MPC_SOLVER_H

// acados
#include "aerial_robot_control/nmpc/base_mpc_solver.h"
#include "aerial_robot_control/nmpc/under_act_body_rate/c_generated_code/acados_solver_qd_body_rate_model.h"

namespace aerial_robot_control
{

namespace nmpc_under_act_body_rate
{

class MPCSolver : public base_nmpc::BaseMPCSolver
{
public:
  MPCSolver();
  ~MPCSolver() = default;
  void updateConstraints(double v_max, double v_min, double w_max, double w_min, double c_thrust_max,
                         double c_thrust_min, double mass, double gravity);

private:
  double gravity_;
  void initParameters() override;
  void setFeedbackConstraints(const nav_msgs::Odometry& odom_now) override;
};

}  // namespace nmpc_under_act_body_rate

}  // namespace aerial_robot_control
