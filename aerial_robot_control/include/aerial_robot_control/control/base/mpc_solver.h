//
// Created by lijinjie on 23/10/22.
//

#ifndef AERIAL_ROBOT_CONTROL_MPC_SOLVER_H
#define AERIAL_ROBOT_CONTROL_MPC_SOLVER_H

#endif  // AERIAL_ROBOT_CONTROL_MPC_SOLVER_H

#include <ros/console.h>

#include <iostream>

// acados
#include "acados/utils/math.h"
#include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
#include "aerial_robot_control/control/base/nmpc_body_rate/c_generated_code/acados_solver_qd_body_rate_model.h"

#define NX QD_BODY_RATE_MODEL_NX
#define NZ QD_BODY_RATE_MODEL_NZ
#define NU QD_BODY_RATE_MODEL_NU
#define NP QD_BODY_RATE_MODEL_NP
#define NBX QD_BODY_RATE_MODEL_NBX
#define NBX0 QD_BODY_RATE_MODEL_NBX0
#define NBU QD_BODY_RATE_MODEL_NBU
#define NSBX QD_BODY_RATE_MODEL_NSBX
#define NSBU QD_BODY_RATE_MODEL_NSBU
#define NSH QD_BODY_RATE_MODEL_NSH
#define NSG QD_BODY_RATE_MODEL_NSG
#define NSPHI QD_BODY_RATE_MODEL_NSPHI
#define NSHN QD_BODY_RATE_MODEL_NSHN
#define NSGN QD_BODY_RATE_MODEL_NSGN
#define NSPHIN QD_BODY_RATE_MODEL_NSPHIN
#define NSBXN QD_BODY_RATE_MODEL_NSBXN
#define NS QD_BODY_RATE_MODEL_NS
#define NSN QD_BODY_RATE_MODEL_NSN
#define NG QD_BODY_RATE_MODEL_NG
#define NBXN QD_BODY_RATE_MODEL_NBXN
#define NGN QD_BODY_RATE_MODEL_NGN
#define NY0 QD_BODY_RATE_MODEL_NY0
#define NY QD_BODY_RATE_MODEL_NY
#define NYN QD_BODY_RATE_MODEL_NYN
#define NH QD_BODY_RATE_MODEL_NH
#define NPHI QD_BODY_RATE_MODEL_NPHI
#define NHN QD_BODY_RATE_MODEL_NHN
#define NPHIN QD_BODY_RATE_MODEL_NPHIN
#define NR QD_BODY_RATE_MODEL_NR

namespace MPC {

struct OutXU {
    double x_traj[NX * (QD_BODY_RATE_MODEL_N + 1)];
    double u_traj[NU * QD_BODY_RATE_MODEL_N];
};

class MPCSolver {
  public:
    int status;
    int N;
    OutXU out_xu;

    // initial constraints
    int idxbx0[NBX0];
    double lbx0[NBX0];
    double ubx0[NBX0];

    // initial values
    double x_init[NX];
    double u0[NU];
    double p[NP];

    MPCSolver();
    ~MPCSolver();
    void reset();
    int solve(double *x0, double *xr, double *ur, double *pr);

  private:
    qd_body_rate_model_solver_capsule *acados_ocp_capsule;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
};

}  // namespace MPC
