//
// Created by lijinjie on 23/11/29.
//

#ifndef TILT_BI_SERVO_NMPC_SOLVER_H
#define TILT_BI_SERVO_NMPC_SOLVER_H

#include "aerial_robot_control/nmpc/base_mpc_solver.h"
#include "aerial_robot_control/nmpc/tilt_bi_servo_mdl/c_generated_code/acados_solver_tilt_bi_servo_mdl.h"

namespace aerial_robot_control
{

namespace mpc_solver
{

class TiltBiServoMdlMPCSolver : public BaseMPCSolver
{
public:
  TiltBiServoMdlMPCSolver()
  {
    // acados macro
    NN_ = TILT_BI_SERVO_MDL_N;
    NX_ = TILT_BI_SERVO_MDL_NX;
    NZ_ = TILT_BI_SERVO_MDL_NZ;
    NU_ = TILT_BI_SERVO_MDL_NU;
    NP_ = TILT_BI_SERVO_MDL_NP;
    NBX_ = TILT_BI_SERVO_MDL_NBX;
    NBX0_ = TILT_BI_SERVO_MDL_NBX0;
    NBU_ = TILT_BI_SERVO_MDL_NBU;
    NSBX_ = TILT_BI_SERVO_MDL_NSBX;
    NSBU_ = TILT_BI_SERVO_MDL_NSBU;
    NSH_ = TILT_BI_SERVO_MDL_NSH;
    NSH0_ = TILT_BI_SERVO_MDL_NSH0;
    NSG_ = TILT_BI_SERVO_MDL_NSG;
    NSPHI_ = TILT_BI_SERVO_MDL_NSPHI;
    NSHN_ = TILT_BI_SERVO_MDL_NSHN;
    NSGN_ = TILT_BI_SERVO_MDL_NSGN;
    NSPHIN_ = TILT_BI_SERVO_MDL_NSPHIN;
    NSPHI0_ = TILT_BI_SERVO_MDL_NSPHI0;
    NSBXN_ = TILT_BI_SERVO_MDL_NSBXN;
    NS_ = TILT_BI_SERVO_MDL_NS;
    NS0_ = TILT_BI_SERVO_MDL_NS0;
    NSN_ = TILT_BI_SERVO_MDL_NSN;
    NG_ = TILT_BI_SERVO_MDL_NG;
    NBXN_ = TILT_BI_SERVO_MDL_NBXN;
    NGN_ = TILT_BI_SERVO_MDL_NGN;
    NY0_ = TILT_BI_SERVO_MDL_NY0;
    NY_ = TILT_BI_SERVO_MDL_NY;
    NYN_ = TILT_BI_SERVO_MDL_NYN;
    NH_ = TILT_BI_SERVO_MDL_NH;
    NHN_ = TILT_BI_SERVO_MDL_NHN;
    NH0_ = TILT_BI_SERVO_MDL_NH0;
    NPHI0_ = TILT_BI_SERVO_MDL_NPHI0;
    NPHI_ = TILT_BI_SERVO_MDL_NPHI;
    NPHIN_ = TILT_BI_SERVO_MDL_NPHIN;
    NR_ = TILT_BI_SERVO_MDL_NR;

    // acados functions that only using once
    acados_ocp_capsule_ = tilt_bi_servo_mdl_acados_create_capsule();

    int status = tilt_bi_servo_mdl_acados_create(acados_ocp_capsule_);
    if (status)
      throw std::runtime_error("tilt_bi_servo_mdl_acados_create() returned status " + std::to_string(status) +
                               ". Exiting.");

    nlp_config_ = tilt_bi_servo_mdl_acados_get_nlp_config(acados_ocp_capsule_);
    nlp_dims_ = tilt_bi_servo_mdl_acados_get_nlp_dims(acados_ocp_capsule_);
    nlp_in_ = tilt_bi_servo_mdl_acados_get_nlp_in(acados_ocp_capsule_);
    nlp_out_ = tilt_bi_servo_mdl_acados_get_nlp_out(acados_ocp_capsule_);
    nlp_solver_ = tilt_bi_servo_mdl_acados_get_nlp_solver(acados_ocp_capsule_);
    nlp_opts_ = tilt_bi_servo_mdl_acados_get_nlp_opts(acados_ocp_capsule_);
  };

  ~TiltBiServoMdlMPCSolver() override
  {
    int status = tilt_bi_servo_mdl_acados_free(acados_ocp_capsule_);
    if (status)
      std::cout << "tilt_bi_servo_mdl_acados_free() returned status " << status << ". \n" << std::endl;

    status = tilt_bi_servo_mdl_acados_free_capsule(acados_ocp_capsule_);
    if (status)
      std::cout << "tilt_bi_servo_mdl_acados_free_capsule() returned status " << status << ". \n" << std::endl;
  };

protected:
  tilt_bi_servo_mdl_solver_capsule* acados_ocp_capsule_ = nullptr;

  // acados functions that using multiple times
  inline int acadosUpdateParams(int stage, std::vector<double>& value) override
  {
    return tilt_bi_servo_mdl_acados_update_params(acados_ocp_capsule_, stage, value.data(), NP_);
  }

  inline int acadosUpdateParamsSparse(int stage, std::vector<int>& idx, std::vector<double>& p, int n_update) override
  {
    return tilt_bi_servo_mdl_acados_update_params_sparse(acados_ocp_capsule_, stage, idx.data(), p.data(), n_update);
  }

  inline int acadosSolve() override
  {
    return tilt_bi_servo_mdl_acados_solve(acados_ocp_capsule_);
  }

  inline void acadosPrintStats() override
  {
    tilt_bi_servo_mdl_acados_print_stats(acados_ocp_capsule_);
  }
};

}  // namespace mpc_solver

}  // namespace aerial_robot_control

#endif  // TILT_BI_SERVO_NMPC_SOLVER_H
