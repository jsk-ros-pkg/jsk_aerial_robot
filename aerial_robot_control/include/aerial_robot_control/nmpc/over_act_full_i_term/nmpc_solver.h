//
// Created by lijinjie on 23/11/29.
//

#ifndef BEETLE_NMPC_SOLVER_H
#define BEETLE_NMPC_SOLVER_H

#endif  // BEETLE_NMPC_SOLVER_H

#include <ros/console.h>
#include <aerial_robot_msgs/PredXU.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

// acados
#include "acados/utils/math.h"
#include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
#include "aerial_robot_control/nmpc/over_act_full_i_term/c_generated_code/acados_solver_beetle_full_w_disturb_model.h"

#define NN BEETLE_FULL_W_DISTURB_MODEL_N
#define NX BEETLE_FULL_W_DISTURB_MODEL_NX
#define NZ BEETLE_FULL_W_DISTURB_MODEL_NZ
#define NU BEETLE_FULL_W_DISTURB_MODEL_NU
#define NP BEETLE_FULL_W_DISTURB_MODEL_NP
#define NBX BEETLE_FULL_W_DISTURB_MODEL_NBX
#define NBX0 BEETLE_FULL_W_DISTURB_MODEL_NBX0
#define NBU BEETLE_FULL_W_DISTURB_MODEL_NBU
#define NSBX BEETLE_FULL_W_DISTURB_MODEL_NSBX
#define NSBU BEETLE_FULL_W_DISTURB_MODEL_NSBU
#define NSH BEETLE_FULL_W_DISTURB_MODEL_NSH
#define NSG BEETLE_FULL_W_DISTURB_MODEL_NSG
#define NSPHI BEETLE_FULL_W_DISTURB_MODEL_NSPHI
#define NSHN BEETLE_FULL_W_DISTURB_MODEL_NSHN
#define NSGN BEETLE_FULL_W_DISTURB_MODEL_NSGN
#define NSPHIN BEETLE_FULL_W_DISTURB_MODEL_NSPHIN
#define NSBXN BEETLE_FULL_W_DISTURB_MODEL_NSBXN
#define NS BEETLE_FULL_W_DISTURB_MODEL_NS
#define NSN BEETLE_FULL_W_DISTURB_MODEL_NSN
#define NG BEETLE_FULL_W_DISTURB_MODEL_NG
#define NBXN BEETLE_FULL_W_DISTURB_MODEL_NBXN
#define NGN BEETLE_FULL_W_DISTURB_MODEL_NGN
#define NY0 BEETLE_FULL_W_DISTURB_MODEL_NY0
#define NY BEETLE_FULL_W_DISTURB_MODEL_NY
#define NYN BEETLE_FULL_W_DISTURB_MODEL_NYN
#define NH BEETLE_FULL_W_DISTURB_MODEL_NH
#define NPHI BEETLE_FULL_W_DISTURB_MODEL_NPHI
#define NHN BEETLE_FULL_W_DISTURB_MODEL_NHN
#define NPHIN BEETLE_FULL_W_DISTURB_MODEL_NPHIN
#define NR BEETLE_FULL_W_DISTURB_MODEL_NR

namespace aerial_robot_control
{

namespace nmpc_over_act_full_i_term
{

struct Constraints
{
  double v_max;
  double v_min;
  double w_max;
  double w_min;
  double thrust_max;
  double thrust_min;
  double a_max;
  double a_min;
};

class MPCSolver
{
public:
  aerial_robot_msgs::PredXU x_u_out_;
  double* W_;
  double* WN_;

  MPCSolver();
  ~MPCSolver();
  void initialize();
  void reset(const aerial_robot_msgs::PredXU& x_u);
  int solve(const aerial_robot_msgs::PredXU& x_u_ref, const nav_msgs::Odometry& odom_now, double joint_angles[4],
            double f_disturb_w[3], double tau_disturb_cog[3], bool is_debug);

  /* Setters */
  void setCostWDiagElement(int index, double value, bool is_set_WN = true) const;
  void setCostWeight(bool is_update_W, bool is_update_WN);

  /* for debugging */
  void printStatus(double min_time);
  void printSolution();

private:
  double* new_time_steps;
  beetle_full_w_disturb_model_solver_capsule* acados_ocp_capsule_;
  ocp_nlp_config* nlp_config_;
  ocp_nlp_dims* nlp_dims_;
  ocp_nlp_in* nlp_in_;
  ocp_nlp_out* nlp_out_;
  ocp_nlp_solver* nlp_solver_;
  void* nlp_opts_;

  void setReference(const aerial_robot_msgs::PredXU& x_u_ref, unsigned int x_stride, unsigned int u_stride, double* params);
  void setFeedbackConstraints(const nav_msgs::Odometry& odom_now, const double joint_angles[4]);
  double solveOCPOnce();
  void getSolution(unsigned int x_stride, unsigned int u_stride);
};

void initPredXU(aerial_robot_msgs::PredXU& x_u);

}  // namespace nmpc_over_act_full_i_term

}  // namespace aerial_robot_control
