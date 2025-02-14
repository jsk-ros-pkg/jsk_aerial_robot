#include <delta/control/delta_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

void RollingController::calcFlightControlInput()
{
  KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_control_->setCogDesireOrientation(cog_desire_orientation);
  KDL::JntArray joint_positions = robot_model_->getJointPositions();
  robot_model_for_control_->updateRobotModel(joint_positions);
  jointTorquePreComputation();

  calcFlightFullLambda();
  nonlinearWrenchAllocation();
}

void RollingController::calcAccFromCog()
{
  tf::Quaternion cog2baselink_rot;
  tf::quaternionKDLToTF(robot_model_->getCogDesireOrientation<KDL::Rotation>(), cog2baselink_rot);
  tf::Matrix3x3 cog_rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_) * tf::Matrix3x3(cog2baselink_rot).inverse();

  tf::Vector3 target_linear_acc_w(pid_controllers_.at(X).result(),
                                  pid_controllers_.at(Y).result(),
                                  pid_controllers_.at(Z).result());
  tf::Vector3 target_linear_acc_cog = cog_rot.inverse() * target_linear_acc_w;

  Eigen::VectorXd target_acc_cog = Eigen::VectorXd::Zero(6);

  target_acc_cog.head(3) = Eigen::Vector3d(target_linear_acc_cog.x(), target_linear_acc_cog.y(), target_linear_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();

  Eigen::Vector3d omega;
  tf::vectorTFToEigen(omega_, omega);
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  Eigen::Vector3d gyro = omega.cross(inertia * omega);

  target_acc_cog.tail(3)
    = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z)
    + inertia.inverse() * gyro;

  setTargetAccCog(target_acc_cog);

  if(navigator_->getForceLandingFlag() && target_linear_acc_w.z() < 5.0) // heuristic measures to avoid to large gimbal angles after force land
    start_rp_integration_ = false;
}

void RollingController::calcFlightFullLambda()
{
  /* calculate integarated allocation */
  Eigen::MatrixXd full_q_mat = rolling_robot_model_->getFullWrenchAllocationMatrixFromControlFrame("cog");

  /* change dimension of convertion matrix from wrench to acc */
  full_q_mat.topRows(3) = 1.0 / robot_model_->getMass() * full_q_mat.topRows(3);
  full_q_mat.bottomRows(3) = rolling_robot_model_->getInertiaFromControlFrame<Eigen::Matrix3d>().inverse() * full_q_mat.bottomRows(3);

  Eigen::MatrixXd full_q_mat_inv = aerial_robot_model::pseudoinverse(full_q_mat);

  full_lambda_trans_ = full_q_mat_inv.leftCols(3) * getTargetAccCog().head(3);
  full_lambda_rot_ = full_q_mat_inv.rightCols(3) * getTargetAccCog().tail(3);
  full_lambda_all_ = full_lambda_trans_ + full_lambda_rot_;

  if(control_verbose_)
    {
      ROS_INFO_STREAM("[control][flight] target acc: " << getTargetAccCog().transpose());
      ROS_INFO_STREAM("[control][flight] full_lambda: " << full_lambda_all_.transpose());
    }
}

double nonlinearWrenchAllocationMinObjective(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  // 0 ~ motor_num : lambda
  // motor_num ~ 2 * motor_num : gimbal roll (phi)

  RollingRobotModel *rolling_robot_model = reinterpret_cast<RollingRobotModel*>(ptr);
  double ret = 0;
  int motor_num = rolling_robot_model->getRotorNum();

  // objective
  // f = sum(lambda_i * lambda_i)
  for(int i = 0; i < motor_num; i++)
    {
      ret += x.at(i) * x.at(i);
    }

  // gradient
  // df/dlambda_i = 2 * lambda_i
  // df/dphi_i = 0
  if(grad.empty()) return ret;

  for(int i = 0; i < motor_num; i++)
    {
      grad.at(i) = 2 * x.at(i);
      grad.at(i + motor_num) = 0;
    }
  return ret;
}

void nonlinearWrenchAllocationEqConstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* ptr)
{
  // 0 ~ 6 : target_wrench_cog - Q * lambda = 0

  RollingController *controller = reinterpret_cast<RollingController*>(ptr);
  auto robot_model_for_control = controller->getRobotModelForControl();

  int motor_num = robot_model_for_control->getRotorNum();

  std::vector<Eigen::Matrix3d> links_rotation_from_cog = robot_model_for_control->getLinksRotationFromCog<Eigen::Matrix3d>();
  std::vector<Eigen::Vector3d> rotors_origin_from_cog = robot_model_for_control->getRotorsOriginFromCog<Eigen::Vector3d>();
  std::vector<double> rotor_tilt = controller->getRotorTilt();

  Eigen::VectorXd target_acc_cog = controller->getTargetAccCog();
  double mass = robot_model_for_control->getMass();
  Eigen::Matrix3d inertia = robot_model_for_control->getInertia<Eigen::Matrix3d>();
  double m_f_rate = robot_model_for_control->getMFRate();
  std::map<int, int> rotor_direction = robot_model_for_control->getRotorDirection();

  Eigen::VectorXd target_wrench_cog = target_acc_cog;
  target_wrench_cog.head(3) = mass * target_wrench_cog.head(3);
  target_wrench_cog.tail(3) = inertia * target_wrench_cog.tail(3);

  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(motor_num);
  for(int i = 0; i < motor_num; i++) lambda(i) = x[i];

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, motor_num);
  for(int i = 0; i < motor_num; i++)
    {
      Eigen::Vector3d u_Li;
      u_Li <<
         sin(rotor_tilt.at(i)),
        -sin(x[i + motor_num]) * cos(rotor_tilt.at(i)),
         cos(x[i + motor_num]) * cos(rotor_tilt.at(i));

      Q.block(0, i, 3, 1) = links_rotation_from_cog.at(i) * u_Li;
      Q.block(3, i, 3, 1) = (aerial_robot_model::skew(rotors_origin_from_cog.at(i)) + m_f_rate * rotor_direction.at(i + 1) * Eigen::MatrixXd::Identity(3, 3)) * links_rotation_from_cog.at(i) * u_Li;
    }

  Eigen::VectorXd wrench_diff = Q * lambda - target_wrench_cog;
  for(int i = 0; i < m; i++) result[i] = wrench_diff(i);

  if(grad == NULL) return;

  /* dwrench_dlambda = Q */
  for(int j = 0; j < motor_num; j++)
    {
      for(int i = 0; i < 6; i++)
        {
          grad[i * n + j] = Q(i, j);
        }
    }

  /* gimbal part */
  Eigen::MatrixXd df_dphi = Eigen::MatrixXd::Zero(3, motor_num);
  Eigen::MatrixXd dtau_dphi = Eigen::MatrixXd::Zero(3, motor_num);
  for(int i = 0; i < motor_num; i++)
    {
      Eigen::Vector3d du_Li_dphi;
      du_Li_dphi <<
        0,
       -cos(x[i + motor_num]) * cos(rotor_tilt.at(i)),
       -sin(x[i + motor_num]) * cos(rotor_tilt.at(i));

      df_dphi.block(0, i, 3, 1) = links_rotation_from_cog.at(i) * du_Li_dphi * x[i];
      dtau_dphi.block(0, i, 3, 1) =  (aerial_robot_model::skew(rotors_origin_from_cog.at(i)) + m_f_rate * rotor_direction.at(i + 1) * Eigen::MatrixXd::Identity(3, 3)) * links_rotation_from_cog.at(i) * du_Li_dphi * x[i];
    }

  for(int i = 0; i < 3; i++)
    {
      for(int j = 0; j < 3; j++)
        {
          grad[i       * n + (j + 3)] = df_dphi(i, j);
          grad[(i + 3) * n + (j + 3)] = dtau_dphi(i, j);
        }
    }
}

void nonlinearWrenchAllocationTorqueConstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* ptr)
{
  // tau + J_{lambda}^{t} w_{lambda} + J_{m_i}^{t} m_{i}g + J_{ext}^t w_{ext} = 0
  // w_{ext} = [-f_{lambda, x}, -f_{lambda, y}, mg - f_{lambda, z}]^t or use estimated external wrench

  RollingController *controller = reinterpret_cast<RollingController*>(ptr);
  auto robot_model_for_control = controller->getRobotModelForControl();

  int motor_num = robot_model_for_control->getRotorNum();

  const std::vector<Eigen::Matrix3d>& links_rotation_from_control_frame = robot_model_for_control->getLinksRotationFromControlFrame<Eigen::Matrix3d>();
  const std::vector<double>& rotor_tilt = controller->getRotorTilt();
  const double m_f_rate = robot_model_for_control->getMFRate();
  const auto& sigma = robot_model_for_control->getRotorDirection();

  const int joint_num = robot_model_for_control->getJointNum();

  /* get joint torque by mass */
  Eigen::VectorXd tau = controller->getJointTorque(); // gimbal1, joint1, gimbal2, joint2, gimbal3

  /* consider joint torque by thrust */
  Eigen::Vector3d exerted_force_cp = Eigen::Vector3d::Zero();
  const std::vector<Eigen::MatrixXd>& gimbal_link_jacobians = controller->getGimbalLinkJacobians();
  Eigen::MatrixXd contact_point_jacobian = controller->getContactPointJacobian();
  for(int i = 0; i < motor_num; i++)
    {
      Eigen::Vector3d u_Li;
      u_Li <<
         sin(rotor_tilt.at(i)),
        -sin(x[i + motor_num]) * cos(rotor_tilt.at(i)),
         cos(x[i + motor_num]) * cos(rotor_tilt.at(i));

      Eigen::VectorXd thrust_wrench_unit = Eigen::VectorXd::Zero(6);
      thrust_wrench_unit.head(3) = links_rotation_from_control_frame.at(i) * u_Li;
      thrust_wrench_unit.tail(3) = m_f_rate * sigma.at(i + 1) * links_rotation_from_control_frame.at(i) * u_Li;

      Eigen::VectorXd wrench = thrust_wrench_unit * x[i];
      tau -= gimbal_link_jacobians.at(i).rightCols(joint_num).transpose() * wrench;

      exerted_force_cp += thrust_wrench_unit.head(3) * x[i];
    }

  /* consider joint torque by external force */
  int ground_navigation_mode = controller->getGroundNavigationMode();
  if(ground_navigation_mode == aerial_robot_navigation::STANDING_STATE || ground_navigation_mode == aerial_robot_navigation::ROLLING_STATE || ground_navigation_mode == aerial_robot_navigation::DOWN_STATE)
    {
      if(!controller->getUseEstimatedExternalForce())
        {
          Eigen::VectorXd contact_wrench = Eigen::VectorXd::Zero(6);
          contact_wrench.head(3) = Eigen::Vector3d(-exerted_force_cp(0),
                                                   -exerted_force_cp(1),
                                                   robot_model_for_control->getMass() * robot_model_for_control->getGravity()(2) - exerted_force_cp(2));

          tau -= contact_point_jacobian.rightCols(joint_num).transpose() * contact_wrench;
        }
    }

  /* set result using joint index */
  std::vector<int> joint_index = std::vector<int>(0);
  const std::vector<std::string>& joint_names = robot_model_for_control->getJointNames();
  for(int i = 0; i < m; i++)
    {
      for(int j = 0; j < joint_names.size(); j++)
        {
          if(joint_names.at(j) == std::string("joint") + std::to_string(i + 1))
            {
              result[i] = x[2 * motor_num + i] - tau(j);
              joint_index.push_back(j);
            }
        }
    }

  if(grad == NULL) return;

  /* set gradient */
  Eigen::MatrixXd dtau_dlambda = Eigen::MatrixXd::Zero(tau.size(), motor_num);
  Eigen::MatrixXd dtau_dphi = Eigen::MatrixXd::Zero(tau.size(), motor_num);
  for(int i = 0; i < motor_num; i++)
    {
      Eigen::Vector3d u_Li, du_Li_dphi;
      u_Li <<
         sin(rotor_tilt.at(i)),
        -sin(x[i + motor_num]) * cos(rotor_tilt.at(i)),
         cos(x[i + motor_num]) * cos(rotor_tilt.at(i));

      du_Li_dphi <<
        0,
       -cos(x[i + motor_num]) * cos(rotor_tilt.at(i)),
       -sin(x[i + motor_num]) * cos(rotor_tilt.at(i));

      Eigen::VectorXd thrust_wrench_uniti = Eigen::VectorXd::Zero(6);
      thrust_wrench_uniti.head(3) = links_rotation_from_control_frame.at(i) * u_Li;
      thrust_wrench_uniti.tail(3) = m_f_rate * sigma.at(i + 1) * links_rotation_from_control_frame.at(i) * u_Li;

      Eigen::VectorXd dthrust_wrench_uniti_dphi = Eigen::VectorXd::Zero(6);
      dthrust_wrench_uniti_dphi.head(3) = links_rotation_from_control_frame.at(i) * du_Li_dphi;
      dthrust_wrench_uniti_dphi.tail(3) = m_f_rate * sigma.at(i + 1) * links_rotation_from_control_frame.at(i) * du_Li_dphi;

      dtau_dlambda.block(0, i, joint_num, 1) = -gimbal_link_jacobians.at(i).rightCols(joint_num).transpose() * thrust_wrench_uniti;
      dtau_dphi.block(0, i, joint_num, 1) = -gimbal_link_jacobians.at(i).rightCols(joint_num).transpose() * dthrust_wrench_uniti_dphi * x[i];

      if(ground_navigation_mode == aerial_robot_navigation::STANDING_STATE || ground_navigation_mode == aerial_robot_navigation::ROLLING_STATE || ground_navigation_mode == aerial_robot_navigation::DOWN_STATE)
        {
          if(!controller->getUseEstimatedExternalForce())
            {
              Eigen::VectorXd thrust_force_uniti = Eigen::VectorXd::Zero(6);
              Eigen::VectorXd dthrust_force_uniti_dphi = Eigen::VectorXd::Zero(6);
              thrust_force_uniti.head(3) = thrust_wrench_uniti.head(3);
              dthrust_force_uniti_dphi.head(3) = dthrust_wrench_uniti_dphi.head(3);

              dtau_dlambda.block(0, i, joint_num, 1) -= contact_point_jacobian.rightCols(joint_num).transpose() * links_rotation_from_control_frame.at(i) * (-thrust_force_uniti);
              dtau_dphi.block(0, i, joint_num, 1) -= contact_point_jacobian.rightCols(joint_num).transpose() * links_rotation_from_control_frame.at(i) * (-dthrust_force_uniti_dphi) * x[i];
            }
        }
    }

  /* thrust and gimbal part */
  for(int i = 0; i < m; i++)
    {
      for(int j = 0; j < motor_num; j++)
        {
          grad[i * n + j] = -dtau_dlambda(joint_index.at(i), j);
          grad[i * n + motor_num + j] = -dtau_dphi(joint_index.at(i), j);
        }
    }

  /* joint torque part */
  for(int i = 0; i < m; i++)
    {
      for(int j = 0; j < m; j++)
        {
          grad[i * n + 2 * motor_num + j] = (i == j) ? 1.0 : 0.0;
        }
    }
}

void RollingController::nonlinearWrenchAllocation()
{
  int n_variables;
  if(aerial_mode_add_joint_torque_constraints_)
    n_variables = 2 * motor_num_ + robot_model_->getJointNum() - motor_num_;
  else
    n_variables = 2 * motor_num_;

  if(first_run_)
    {
      opt_x_prev_.resize(n_variables, 0.0);
      nlopt_log_.resize(n_variables, 0.0);
    }

  nlopt::opt slsqp_solver(nlopt::LD_SLSQP, n_variables);
  slsqp_solver.set_min_objective(nonlinearWrenchAllocationMinObjective, robot_model_for_control_.get());
  slsqp_solver.add_equality_mconstraint(nonlinearWrenchAllocationEqConstraints, this, {1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6});
  if(n_variables > 2 * motor_num_)
    {
      if(first_run_) ROS_INFO_STREAM("[control] add constraint about joint torque for aerial mode");
      slsqp_solver.add_equality_mconstraint(nonlinearWrenchAllocationTorqueConstraints, this, std::vector<double>(n_variables - 2 * motor_num_, 1e-4));
    }

  std::vector<double> lb(n_variables, -INFINITY);
  std::vector<double> ub(n_variables, INFINITY);
  for(int i = 0; i < motor_num_; i++)
    {
      lb.at(i) = 0;
      ub.at(i) = robot_model_->getThrustUpperLimit();
      double gimbal_angle_margin = 0.2;
      lb.at(i + motor_num_) = -M_PI - gimbal_angle_margin;
      ub.at(i + motor_num_) =  M_PI + gimbal_angle_margin;
    }
  for(int i = 2 * motor_num_; i < n_variables; i++)
    {
      lb.at(i) = -joint_torque_limit_;
      ub.at(i) =  joint_torque_limit_;
    }

  slsqp_solver.set_lower_bounds(lb);
  slsqp_solver.set_upper_bounds(ub);
  slsqp_solver.set_xtol_rel(1e-6);
  slsqp_solver.set_ftol_rel(1e-6);
  slsqp_solver.set_maxeval(1000);

  /* set initial variable */
  std::vector<double> opt_x(n_variables, 0);
  // thrust and gimbal part (use MP-invese matrix based solution)
  for(int i = 0; i < motor_num_; i++)
    {
      opt_x.at(i) = std::clamp((double)full_lambda_all_.segment(2 * i, 2).norm() / fabs(cos(rotor_tilt_.at(i))),
                               lb.at(i),
                               ub.at(i));
      opt_x.at(i + motor_num_) = std::clamp((double)angles::normalize_angle(atan2(-full_lambda_all_(2 * i + 0), full_lambda_all_(2 * i + 1))),
                                            lb.at(i + motor_num_),
                                            ub.at(i + motor_num_));
    }

  // joint torque part (calculate from jacobian and full lambda)
  Eigen::VectorXd joint_torque = getJointTorque();
  std::vector<Eigen::MatrixXd> gimbal_link_jacobians = getGimbalLinkJacobians();
  std::vector<Eigen::Matrix3d> links_rotation_from_cog = robot_model_for_control_->getLinksRotationFromCog<Eigen::Matrix3d>();
  double m_f_rate = robot_model_->getMFRate();
  std::map<int, int> rotor_direction = robot_model_->getRotorDirection();
  for(int i = 0; i < motor_num_; i++)
    {
      Eigen::VectorXd gimbal_link_wrench = Eigen::VectorXd::Zero(6);
      gimbal_link_wrench.head(3) = links_rotation_from_cog.at(i) * Eigen::Vector3d(0, full_lambda_all_(2 * i + 0), full_lambda_all_(2 * i + 1));
      gimbal_link_wrench.tail(3) = links_rotation_from_cog.at(i) * m_f_rate * rotor_direction.at(i + 1) * Eigen::Vector3d(0, full_lambda_all_(2 * i + 0), full_lambda_all_(2 * i + 1));

      joint_torque -= gimbal_link_jacobians.at(i).rightCols(robot_model_->getJointNum()).transpose() * gimbal_link_wrench;
    }

  std::vector<std::string> joint_names = robot_model_->getJointNames();
  for(int i = 0; i < n_variables - 2 * motor_num_; i++)
    {
      auto it = std::find(joint_names.begin(), joint_names.end(), "joint" + std::to_string(i + 1));
      int index = std::distance(joint_names.begin(), it);
      opt_x.at(2 * motor_num_ + i) = std::clamp(joint_torque(index),
                                                lb.at(2 * motor_num_ + i),
                                                ub.at(2 * motor_num_ + i));
      // opt_x.at(2 * motor_num_ + i) = opt_x_prev_.at(2 * motor_num_ + i); // use previous optimal solution
    }

  std::string init_x_string = "";
  for(int i = 0; i < opt_x.size(); i++)
    {
      init_x_string = init_x_string + std::to_string(opt_x.at(i)) + " ";
    }
  if(control_verbose_)
    ROS_INFO_STREAM("[control][flight][nlopt] initial variable: " << init_x_string);

  double max_val;
  nlopt::result result;
  try
    {
      result = slsqp_solver.optimize(opt_x, max_val);
      nlopt_result_ = result;
    }
  catch(std::runtime_error error)
    {}

  if (result < 0) ROS_ERROR_STREAM("[nlopt] failed to solve. result is " << result);

  for(int i = 0; i < opt_x.size(); i++)
    {
      nlopt_log_.at(i) = opt_x.at(i);
      opt_x_prev_.at(i) = opt_x.at(i);
    }


  /* set optimal variables to actuator input */
  for(int i = 0; i < motor_num_; i++)
    {
      lambda_all_.at(i) = opt_x.at(i);
      lambda_trans_.at(i) = opt_x.at(i);
      target_gimbal_angles_.at(i) = opt_x.at(i + motor_num_);
    }
}
