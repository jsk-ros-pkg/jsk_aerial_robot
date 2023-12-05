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
#include "aerial_robot_control/nmpc/over_act_no_servo_delay/c_generated_code/acados_solver_beetle_no_servo_delay_model.h"

#define NN BEETLE_NO_SERVO_DELAY_MODEL_N
#define NX BEETLE_NO_SERVO_DELAY_MODEL_NX
#define NZ BEETLE_NO_SERVO_DELAY_MODEL_NZ
#define NU BEETLE_NO_SERVO_DELAY_MODEL_NU
#define NP BEETLE_NO_SERVO_DELAY_MODEL_NP
#define NBX BEETLE_NO_SERVO_DELAY_MODEL_NBX
#define NBX0 BEETLE_NO_SERVO_DELAY_MODEL_NBX0
#define NBU BEETLE_NO_SERVO_DELAY_MODEL_NBU
#define NSBX BEETLE_NO_SERVO_DELAY_MODEL_NSBX
#define NSBU BEETLE_NO_SERVO_DELAY_MODEL_NSBU
#define NSH BEETLE_NO_SERVO_DELAY_MODEL_NSH
#define NSG BEETLE_NO_SERVO_DELAY_MODEL_NSG
#define NSPHI BEETLE_NO_SERVO_DELAY_MODEL_NSPHI
#define NSHN BEETLE_NO_SERVO_DELAY_MODEL_NSHN
#define NSGN BEETLE_NO_SERVO_DELAY_MODEL_NSGN
#define NSPHIN BEETLE_NO_SERVO_DELAY_MODEL_NSPHIN
#define NSBXN BEETLE_NO_SERVO_DELAY_MODEL_NSBXN
#define NS BEETLE_NO_SERVO_DELAY_MODEL_NS
#define NSN BEETLE_NO_SERVO_DELAY_MODEL_NSN
#define NG BEETLE_NO_SERVO_DELAY_MODEL_NG
#define NBXN BEETLE_NO_SERVO_DELAY_MODEL_NBXN
#define NGN BEETLE_NO_SERVO_DELAY_MODEL_NGN
#define NY0 BEETLE_NO_SERVO_DELAY_MODEL_NY0
#define NY BEETLE_NO_SERVO_DELAY_MODEL_NY
#define NYN BEETLE_NO_SERVO_DELAY_MODEL_NYN
#define NH BEETLE_NO_SERVO_DELAY_MODEL_NH
#define NPHI BEETLE_NO_SERVO_DELAY_MODEL_NPHI
#define NHN BEETLE_NO_SERVO_DELAY_MODEL_NHN
#define NPHIN BEETLE_NO_SERVO_DELAY_MODEL_NPHIN
#define NR BEETLE_NO_SERVO_DELAY_MODEL_NR

namespace aerial_robot_control
{

namespace nmpc_over_act_no_servo_delay
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

  MPCSolver();
  ~MPCSolver();
  void initialize(Constraints& constraints);
  void reset(const aerial_robot_msgs::PredXU& x_u);
  int solve(const nav_msgs::Odometry& odom_now, const aerial_robot_msgs::PredXU& x_u_ref);

private:
  double* new_time_steps;
  beetle_no_servo_delay_model_solver_capsule* acados_ocp_capsule_;
  ocp_nlp_config* nlp_config_;
  ocp_nlp_dims* nlp_dims_;
  ocp_nlp_in* nlp_in_;
  ocp_nlp_out* nlp_out_;
  ocp_nlp_solver* nlp_solver_;
  void* nlp_opts_;

  void setReference(const aerial_robot_msgs::PredXU& x_u_ref, unsigned int x_stride, unsigned int u_stride);
  void setFeedbackConstraints(const nav_msgs::Odometry& odom_now);
  double solveOCPInLoop(int N_timings);
  void getSolution(unsigned int x_stride, unsigned int u_stride);

  void printStatus(int N_timings, double min_time);
  void printSolution();
};

void initPredXU(aerial_robot_msgs::PredXU& x_u);

}  // namespace nmpc_over_act_no_servo_delay

}  // namespace aerial_robot_control
