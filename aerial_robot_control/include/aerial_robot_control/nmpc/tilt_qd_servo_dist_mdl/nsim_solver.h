//
// Created by jinjie on 24/07/19.
//

#ifndef AERIAL_ROBOT_CONTROL_NSIM_SOLVER_H
#define AERIAL_ROBOT_CONTROL_NSIM_SOLVER_H

#include "aerial_robot_control/nmpc/base_sim_solver.h"
#include "aerial_robot_control/nmpc/tilt_qd_servo_dist_mdl/c_generated_code/acados_sim_solver_tilt_qd_servo_dist_mdl.h"

namespace aerial_robot_control
{

namespace mpc_sim_solver
{

class TiltQdServoDistMdlSimSolver : public BaseSimSolver
{
public:
  TiltQdServoDistMdlSimSolver()
  {
    // acados macro
    NX_ = TILT_QD_SERVO_DIST_MDL_NX;
    NZ_ = TILT_QD_SERVO_DIST_MDL_NZ;
    NU_ = TILT_QD_SERVO_DIST_MDL_NU;
    NP_ = TILT_QD_SERVO_DIST_MDL_NP;

    // acados functions that only using once
    acados_sim_capsule_ = tilt_qd_servo_dist_mdl_acados_sim_solver_create_capsule();

    int status = tilt_qd_servo_dist_mdl_acados_sim_create(acados_sim_capsule_);
    if (status)
      throw std::runtime_error("tilt_qd_servo_dist_mdl_acados_sim_create() returned status " + std::to_string(status) +
                               ". Exiting.");

    sim_config_ = tilt_qd_servo_dist_mdl_acados_get_sim_config(acados_sim_capsule_);
    sim_dims_ = tilt_qd_servo_dist_mdl_acados_get_sim_dims(acados_sim_capsule_);
    sim_in_ = tilt_qd_servo_dist_mdl_acados_get_sim_in(acados_sim_capsule_);
    sim_out_ = tilt_qd_servo_dist_mdl_acados_get_sim_out(acados_sim_capsule_);
    sim_solver_ = tilt_qd_servo_dist_mdl_acados_get_sim_solver(acados_sim_capsule_);
    sim_opts_ = tilt_qd_servo_dist_mdl_acados_get_sim_opts(acados_sim_capsule_);
  };

  ~TiltQdServoDistMdlSimSolver() override
  {
    int status = tilt_qd_servo_dist_mdl_acados_sim_free(acados_sim_capsule_);
    if (status)
      std::cout << "tilt_qd_servo_dist_mdl_acados_sim_free() returned status " << status << ". \n" << std::endl;

    status = tilt_qd_servo_dist_mdl_acados_sim_solver_free_capsule(acados_sim_capsule_);
    if (status)
      std::cout << "tilt_qd_servo_dist_mdl_acados_sim_solver_free_capsule() returned status " << status << ". \n"
                << std::endl;
  }

protected:
  tilt_qd_servo_dist_mdl_sim_solver_capsule* acados_sim_capsule_ = nullptr;

  inline int acadosSolve() override
  {
    return tilt_qd_servo_dist_mdl_acados_sim_solve(acados_sim_capsule_);
  }
};

}  // namespace mpc_sim_solver

}  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_NSIM_SOLVER_H
