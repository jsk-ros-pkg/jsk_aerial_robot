#include <rolling/control/rolling_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

double slsqpMinObjective(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = 2.0 * x.at(i);
    }

  double ret = 0.0;
  for(int i = 0; i < x.size(); i++) ret += (x.at(i) * x.at(i));

  return ret;
}

double rollLowerBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  double target_roll_acc = controller->getTargetWrenchAccTargetFrame()(ROLL);
  Eigen::MatrixXd full_q_mat_target_frame_roll = controller->getFullQTargetFrame().row(ROLL);

  double roll_acc = 0.0;
  for(int i = 0; i < x.size(); i++) roll_acc += (full_q_mat_target_frame_roll(i) * x.at(i));

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = -full_q_mat_target_frame_roll(i);
    }

  return target_roll_acc - roll_acc;
}

double rollUpperBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  double target_roll_acc = controller->getTargetWrenchAccTargetFrame()(ROLL);
  Eigen::MatrixXd full_q_mat_target_frame_roll = controller->getFullQTargetFrame().row(ROLL);

  double roll_acc = 0.0;
  for(int i = 0; i < x.size(); i++) roll_acc += (full_q_mat_target_frame_roll(i) * x.at(i));

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = full_q_mat_target_frame_roll(i);
    }

  return roll_acc - target_roll_acc;
}

double pitchLowerBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  double target_pitch_acc = controller->getTargetWrenchAccTargetFrame()(PITCH);
  Eigen::MatrixXd full_q_mat_target_frame_pitch = controller->getFullQTargetFrame().row(PITCH);

  double pitch_acc = 0.0;
  for(int i = 0; i < x.size(); i++) pitch_acc += (full_q_mat_target_frame_pitch(i) * x.at(i));

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = -full_q_mat_target_frame_pitch(i);
    }

  return target_pitch_acc - pitch_acc;
}

double pitchUpperBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  double target_pitch_acc = controller->getTargetWrenchAccTargetFrame()(PITCH);
  Eigen::MatrixXd full_q_mat_target_frame_pitch = controller->getFullQTargetFrame().row(PITCH);

  double pitch_acc = 0.0;
  for(int i = 0; i < x.size(); i++) pitch_acc += (full_q_mat_target_frame_pitch(i) * x.at(i));

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = full_q_mat_target_frame_pitch(i);
    }

  return pitch_acc - target_pitch_acc;
}

double yawLowerBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  double target_yaw_acc = controller->getTargetWrenchAccTargetFrame()(YAW);
  Eigen::MatrixXd full_q_mat_target_frame_yaw = controller->getFullQTargetFrame().row(YAW);

  double yaw_acc = 0.0;
  for(int i = 0; i < x.size(); i++) yaw_acc += (full_q_mat_target_frame_yaw(i) * x.at(i));

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = -full_q_mat_target_frame_yaw(i);
    }

  return target_yaw_acc - yaw_acc;
}

double yawUpperBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  double target_yaw_acc = controller->getTargetWrenchAccTargetFrame()(YAW);
  Eigen::MatrixXd full_q_mat_target_frame_yaw = controller->getFullQTargetFrame().row(YAW);

  double yaw_acc = 0.0;
  for(int i = 0; i < x.size(); i++) yaw_acc += (full_q_mat_target_frame_yaw(i) * x.at(i));

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = full_q_mat_target_frame_yaw(i);
    }


  return yaw_acc - target_yaw_acc;
}

double zUpperBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  Eigen::MatrixXd full_q_mat_target_frame_z = controller->getFullQTargetFrame().row(Z) * controller->getRobotMassForOpt();

  double z_force = 0.0;
  for(int i = 0; i < x.size(); i++) z_force += (full_q_mat_target_frame_z(i) * x.at(i));

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = full_q_mat_target_frame_z(i);
    }

  return z_force - controller->getRobotMassForOpt() * controller->getGravityForOpt();
}

double xLowerBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  Eigen::MatrixXd full_q_mat_target_frame_x = controller->getFullQTargetFrame().row(X) * controller->getRobotMassForOpt();
  Eigen::MatrixXd full_q_mat_target_frame_z = controller->getFullQTargetFrame().row(Z) * controller->getRobotMassForOpt();

  Eigen::MatrixXd lambda_coefficient = full_q_mat_target_frame_x + controller->getSteeringMu() * full_q_mat_target_frame_z;

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = lambda_coefficient(i);
    }

  double ret = -controller->getSteeringMu() * controller->getRobotMassForOpt() * controller->getGravityForOpt();
  for(int i = 0; i < x.size(); i++) ret += (lambda_coefficient(i) * x.at(i));

  return ret;
}

double xUpperBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  Eigen::MatrixXd full_q_mat_target_frame_x = controller->getFullQTargetFrame().row(X) * controller->getRobotMassForOpt();
  Eigen::MatrixXd full_q_mat_target_frame_z = controller->getFullQTargetFrame().row(Z) * controller->getRobotMassForOpt();

  Eigen::MatrixXd lambda_coefficient = -full_q_mat_target_frame_x + controller->getSteeringMu() * full_q_mat_target_frame_z;

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = lambda_coefficient(i);
    }

  double ret = -controller->getSteeringMu() * controller->getRobotMassForOpt() * controller->getGravityForOpt();
  for(int i = 0; i < x.size(); i++) ret += (lambda_coefficient(i) * x.at(i));

  return ret;
}


double yLowerBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  Eigen::MatrixXd full_q_mat_target_frame_y = controller->getFullQTargetFrame().row(Y) * controller->getRobotMassForOpt();
  Eigen::MatrixXd full_q_mat_target_frame_z = controller->getFullQTargetFrame().row(Z) * controller->getRobotMassForOpt();

  Eigen::MatrixXd lambda_coefficient = full_q_mat_target_frame_y + controller->getSteeringMu() * full_q_mat_target_frame_z;

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = lambda_coefficient(i);
    }

  double ret = -controller->getSteeringMu() * controller->getRobotMassForOpt() * controller->getGravityForOpt();
  for(int i = 0; i < x.size(); i++) ret += (lambda_coefficient(i) * x.at(i));

  return ret;
}

double yUpperBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  Eigen::MatrixXd full_q_mat_target_frame_y = controller->getFullQTargetFrame().row(Y) * controller->getRobotMassForOpt();
  Eigen::MatrixXd full_q_mat_target_frame_z = controller->getFullQTargetFrame().row(Z) * controller->getRobotMassForOpt();

  Eigen::MatrixXd lambda_coefficient = -full_q_mat_target_frame_y + controller->getSteeringMu() * full_q_mat_target_frame_z;

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = lambda_coefficient(i);
    }

  double ret = -controller->getSteeringMu() * controller->getRobotMassForOpt() * controller->getGravityForOpt();
  for(int i = 0; i < x.size(); i++) ret += (lambda_coefficient(i) * x.at(i));

  return ret;
}



void RollingController::slsqpSolve()
{
  int n_variables = 2 * motor_num_;
  double epsilon = 0.0001;

  target_wrench_acc_target_frame_.resize(6);
  target_wrench_acc_target_frame_.tail(3) = Eigen::Vector3d(pid_controllers_.at(ROLL).result(),
                                                            pid_controllers_.at(PITCH).result(),
                                                            pid_controllers_.at(YAW).result());

  KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
  KDL::Frame contact_point_alined_to_cog = contact_point_alined_.Inverse() * cog;
  Eigen::Vector3d contact_point_alined_to_cog_p = aerial_robot_model::kdlToEigen(contact_point_alined_to_cog.p);
  Eigen::Matrix3d contact_point_alined_to_cog_p_skew = aerial_robot_model::skew(contact_point_alined_to_cog_p);
  Eigen::VectorXd gravity = robot_model_->getGravity3d();
  Eigen::Vector3d gravity_ang_acc_from_contact_point_alined = robot_model_->getMass() * contact_point_alined_to_cog_p_skew * gravity;

  target_wrench_acc_target_frame_.tail(3) = target_wrench_acc_target_frame_.tail(3) + gravity_ang_acc_from_contact_point_alined;
  robot_mass_for_opt_ = robot_model_->getMass();
  gravity_for_opt_ = robot_model_->getGravity()(Z);

  nlopt::opt nl_solver(nlopt::LD_SLSQP, n_variables);

  nl_solver.set_min_objective(slsqpMinObjective, this);
  nl_solver.add_inequality_constraint(rollLowerBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(rollUpperBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(pitchLowerBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(pitchUpperBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(yawLowerBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(yawUpperBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(zUpperBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(xLowerBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(xUpperBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(yLowerBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(yUpperBoundConstraint, this, epsilon);

  nl_solver.set_xtol_rel(1e-4);
  nl_solver.set_maxeval(1000);

  std::vector<double> x(n_variables);
  for(int i = 0; i < x.size(); i++)
    {
      x.at(i) = full_lambda_all_(i);
    }

  double minf;
  double start_t = ros::Time::now().toSec();
  try
    {
      nlopt::result result = nl_solver.optimize(x, minf);
      for(int i = 0; i < motor_num_; i++)
        {
          std::cout << "thrust" << i + 1 << " " << sqrt(x.at(2 * i + 0) * x.at(2 * i + 0) + x.at(2 * i + 1) * x.at(2 * i + 1)) << std::endl;;
          std::cout << "gimbal" << i + 1 << " " << atan2(-x.at(2 * i + 0), x.at(2 * i + 1)) << std::endl;
          // std::cout << x.at(i) << " ";
        }
      std::cout << std::endl;
      std::cout << std::endl;
      // for(int i = 0; i < x.size(); i++)
      //   {
      //     full_lambda_all_(i) = x.at(i);
      //     full_lambda_trans_(i) = x.at(i);
      //   }
    }
  catch(std::exception &e)
    {
      std::cout << "nlopt failed: " << e.what() << std::endl;
      std::cout << "nlopt process time: " << ros::Time::now().toSec() - start_t << std::endl;
    }
}



/* nonlinearQP

variable: [lambda, gimbal] in R^6

minimize lambda^2
s.t.
Q^rot(q, gimbal) lambda = tau^des
Q^trans(q, gimbal) lambda = f
f_z < mg
-mu (mg - f_z) < f_x < mu(mg - f_z)
-mu (mg - f_z) < f_y < mu(mg - f_z)

=>

minimize lambda^2
s.t.
Q^rot(q, gimbal) lambda = tau^des
Q^trans(q, gimbal) lambda = f
f_z - mg < 0
-mu (mg - f_z) < f_x < mu(mg - f_z)
-mu (mg - f_z) < f_y < mu(mg - f_z)

(mu * Q^trans(q, gimbal)_z - Q^trans(q, gimbal)_x) lambda - mu * mg < 0 // lower
(mu * Q^trans(q, gimbal)_z + Q^trans(q, gimbal)_x) lambda - mu * mg < 0 // upper

*/

double nonlinearQpObjective(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  double ret = 0.0;
  for(int i = 0; i < x.size() / 2; i++)
    {
      ret += x.at(i) * x.at(i);
    }

  return ret;
}

void eqConstraint(unsigned m, double* result, unsigned n, const double* x, double* grad, void* f_data)
{
  RollingController *controller = reinterpret_cast<RollingController*>(f_data);
  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model = controller->getRobotModel();
  boost::shared_ptr<RollingRobotModel> rolling_robot_model_for_opt = controller->getRollingRobotModelForOpt();

  int rotor_num = robot_model->getRotorNum();
  Eigen::VectorXd target_wrench_acc_target_frame = controller->getTargetWrenchAccTargetFrame();
  Eigen::VectorXd target_ang_acc_target_frame = target_wrench_acc_target_frame.tail(3);

  auto joint_index_map = robot_model->getJointIndexMap();
  KDL::JntArray joint_positions = robot_model->getJointPositions();

  for(int i = 0; i < rotor_num; i++)
    {
      joint_positions(joint_index_map.find(std::string("gimbal") + std::to_string(i + 1))->second) = x[i + rotor_num];
    }
  rolling_robot_model_for_opt->updateRobotModel(joint_positions);

  rolling_robot_model_for_opt->calcRobotModelFromFrame("cp");

  Eigen::MatrixXd wrench_allocation_matrix_from_target_frame = rolling_robot_model_for_opt->getWrenchAllocationMatrixFromTargetFrame();
  Eigen::VectorXd exerted_thrust(3);
  exerted_thrust << x[0], x[1], x[2];

  Eigen::Matrix3d inertia_from_target_frame_inv = rolling_robot_model_for_opt->getInertiaFromTargetFrame<Eigen::Matrix3d>().inverse();
  Eigen::VectorXd exerted_ang_acc = inertia_from_target_frame_inv *  wrench_allocation_matrix_from_target_frame.bottomRows(3) * exerted_thrust;
  for(int i = 0; i < rotor_num; i++)
    {
      result[i] = exerted_ang_acc(i) - target_ang_acc_target_frame(i);
    }
}

void inEqConstraint(unsigned m, double* result, unsigned n, const double* x, double* grad, void* f_data)
{
  RollingController *controller = reinterpret_cast<RollingController*>(f_data);
  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model = controller->getRobotModel();
  boost::shared_ptr<RollingRobotModel> rolling_robot_model_for_opt = controller->getRollingRobotModelForOpt();

  int rotor_num = robot_model->getRotorNum();
  auto joint_index_map = robot_model->getJointIndexMap();
  KDL::JntArray joint_positions = robot_model->getJointPositions();

  for(int i = 0; i < rotor_num; i++)
    {
      joint_positions(joint_index_map.find(std::string("gimbal") + std::to_string(i + 1))->second) = x[i + rotor_num];
    }
  rolling_robot_model_for_opt->updateRobotModel(joint_positions);

  rolling_robot_model_for_opt->calcRobotModelFromFrame("cp");

  Eigen::MatrixXd wrench_allocation_matrix_from_target_frame = rolling_robot_model_for_opt->getWrenchAllocationMatrixFromTargetFrame();
  Eigen::VectorXd exerted_thrust(3);
  exerted_thrust << x[0], x[1], x[2];

  Eigen::VectorXd exerted_force = wrench_allocation_matrix_from_target_frame.topRows(3) * exerted_thrust;

  result[0] = exerted_force(2) - controller->getRobotMassForOpt() * controller->getGravityForOpt();;
  result[1] = controller->getSteeringMu() * exerted_force(2) - exerted_force(0) - controller->getSteeringMu() * controller->getRobotMassForOpt() * controller->getGravityForOpt();
  result[2] = controller->getSteeringMu() * exerted_force(2) + exerted_force(0) - controller->getSteeringMu() * controller->getRobotMassForOpt() * controller->getGravityForOpt();
  result[3] = controller->getSteeringMu() * exerted_force(2) - exerted_force(1) - controller->getSteeringMu() * controller->getRobotMassForOpt() * controller->getGravityForOpt();
  result[4] = controller->getSteeringMu() * exerted_force(2) + exerted_force(1) - controller->getSteeringMu() * controller->getRobotMassForOpt() * controller->getGravityForOpt();

}

void RollingController::nonlinearQP()
{
  int n_variables = 2 * motor_num_;
  double epsilon = 0.0001;

  target_wrench_acc_target_frame_.resize(6);
  target_wrench_acc_target_frame_.tail(3) = Eigen::Vector3d(pid_controllers_.at(ROLL).result(),
                                                            pid_controllers_.at(PITCH).result(),
                                                            pid_controllers_.at(YAW).result());

  KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
  KDL::Frame contact_point_alined_to_cog = contact_point_alined_.Inverse() * cog;
  Eigen::Vector3d contact_point_alined_to_cog_p = aerial_robot_model::kdlToEigen(contact_point_alined_to_cog.p);
  Eigen::Matrix3d contact_point_alined_to_cog_p_skew = aerial_robot_model::skew(contact_point_alined_to_cog_p);
  Eigen::VectorXd gravity = robot_model_->getGravity3d();
  Eigen::Vector3d gravity_ang_acc_from_contact_point_alined = robot_model_->getMass() * contact_point_alined_to_cog_p_skew * gravity;

  target_wrench_acc_target_frame_.tail(3) = target_wrench_acc_target_frame_.tail(3) + gravity_ang_acc_from_contact_point_alined;
  robot_mass_for_opt_ = robot_model_->getMass();
  gravity_for_opt_ = robot_model_->getGravity()(Z);

  /* update robot model for optimization with current state */
  KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  rolling_robot_model_for_opt_->setCogDesireOrientation(cog_desire_orientation);
  KDL::JntArray joint_positions = robot_model_->getJointPositions();
  rolling_robot_model_for_opt_->updateRobotModel(joint_positions);

  nlopt::opt nl_solver(nlopt::LN_COBYLA, n_variables);

  nl_solver.set_min_objective(nonlinearQpObjective, this);
  nl_solver.add_equality_mconstraint(eqConstraint, this, {epsilon, epsilon, epsilon});
  nl_solver.add_inequality_mconstraint(inEqConstraint, this, {epsilon, epsilon, epsilon, epsilon, epsilon});

  vector<double> lb(n_variables), ub(n_variables);
  for(int i = 0; i < n_variables / 2; i++)
    {
      lb.at(i) = 0;
      ub.at(i) = INFINITY;
      lb.at(i + motor_num_) = -M_PI;
      ub.at(i + motor_num_) =  M_PI;
    }

  nl_solver.set_lower_bounds(lb);
  nl_solver.set_upper_bounds(ub);

  nl_solver.set_xtol_rel(1e-4);
  nl_solver.set_maxeval(1000);

  std::vector<double> x(n_variables);
  for(int i = 0; i < motor_num_; i++)
    {
      x.at(i) = target_base_thrust_.at(i);
      x.at(i + motor_num_) = target_gimbal_angles_.at(i);
    }
  double minf;
  double start_t = ros::Time::now().toSec();
  try
    {
      nlopt::result result = nl_solver.optimize(x, minf);
      for(int i = 0; i < x.size(); i++)
        {
          std::cout << x.at(i) << " ";
        }
      std::cout << "objective: " << minf << std::endl;

      auto joint_index_map = robot_model_->getJointIndexMap();
      KDL::JntArray joint_positions = robot_model_->getJointPositions();
      for(int i = 0; i < motor_num_; i++)
        {
          joint_positions(joint_index_map.find(std::string("gimbal") + std::to_string(i + 1))->second) = x[i + motor_num_];
        }
      rolling_robot_model_for_opt_->updateRobotModel(joint_positions);

      std::cout << "wrench is " << (rolling_robot_model_for_opt_->calcWrenchMatrixOnCoG() * Eigen::Vector3d(x.at(0), x.at(1), x.at(2))).transpose() << std::endl;
      std::cout << std::endl;
      std::cout << std::endl;
    }
  catch(std::exception &e)
    {
      std::cout << "nlopt failed: " << e.what() << std::endl;
      std::cout << "nlopt process time: " << ros::Time::now().toSec() - start_t << std::endl;
    }
}
