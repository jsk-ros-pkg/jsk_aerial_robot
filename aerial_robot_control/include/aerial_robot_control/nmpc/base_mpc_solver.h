//
// Created by li-jinjie on 23-11-25.
//

#ifndef AERIAL_ROBOT_CONTROL_BASE_MPC_SOLVER_H
#define AERIAL_ROBOT_CONTROL_BASE_MPC_SOLVER_H

#endif  // AERIAL_ROBOT_CONTROL_BASE_MPC_SOLVER_H

#include <ros/console.h>
#include <aerial_robot_msgs/PredXU.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

// acados
#include "acados/utils/math.h"
#include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"

namespace aerial_robot_control
{

namespace base_nmpc
{

class BaseMPCSolver
{
public:
  /* parameters for acados */
  int NN_, NX_, NZ_, NU_, NP_, NBX_, NBX0_, NBU_, NSBX_, NSBU_, NSH_, NSG_, NSPHI_, NSHN_, NSGN_, NSPHIN_, NSBXN_, NS_,
      NSN_, NG_, NBXN_, NGN_, NY0_, NY_, NYN_, NH_, NPHI_, NHN_, NPHIN_, NR_;

  aerial_robot_msgs::PredXU x_u_out_;

  BaseMPCSolver();
  ~BaseMPCSolver();
  void initialize();
  void reset(const aerial_robot_msgs::PredXU& x_u);
  int solve(const nav_msgs::Odometry& odom_now, const aerial_robot_msgs::PredXU& x_u_ref);

  void initPredXU(aerial_robot_msgs::PredXU& x_u) const;

protected:
  // funtion ptrs for acados
  void* (*acados_create_capsule)();
  ocp_nlp_config* (*acados_get_nlp_config)(void*);
  ocp_nlp_dims* (*acados_get_nlp_dims)(void*);
  ocp_nlp_in* (*acados_get_nlp_in)(void*);
  ocp_nlp_out* (*acados_get_nlp_out)(void*);
  ocp_nlp_solver* (*acados_get_nlp_solver)(void*);
  void* (*acados_get_nlp_opts)(void*);
  int (*acados_create_with_discretization)(void*, int, double*);
  int (*acados_solve)(void*);
  int (*acados_update_params)(void*, int, double*, int);
  int (*acados_update_params_sparse)(void*, int, int*, double*, int);
  int (*acados_free)(void*);
  int (*acados_free_capsule)(void*);
  void (*acados_print_stats)(void*);

  // variables for acados
  void* acados_ocp_capsule_;
  ocp_nlp_config* nlp_config_;
  ocp_nlp_dims* nlp_dims_;
  ocp_nlp_in* nlp_in_;
  ocp_nlp_out* nlp_out_;
  ocp_nlp_solver* nlp_solver_;
  void* nlp_opts_;

  double* new_time_steps;

private:
  virtual void initParameters() = 0;

  void setReference(const aerial_robot_msgs::PredXU& x_u_ref, unsigned int x_stride, unsigned int u_stride);
  virtual void setFeedbackConstraints(const nav_msgs::Odometry& odom_now);
  double solveOCPInLoop(int N_timings);
  void getSolution(unsigned int x_stride, unsigned int u_stride);

  void printStatus(int N_timings, double min_time);
  void printSolution();
};
}  // namespace base_nmpc
}  // namespace aerial_robot_control
