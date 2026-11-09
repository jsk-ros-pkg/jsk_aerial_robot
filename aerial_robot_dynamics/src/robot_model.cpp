#include <aerial_robot_dynamics/robot_model.h>
#include <regex>

using namespace aerial_robot_dynamics;

PinocchioRobotModel::PinocchioRobotModel(std::string robot_description, std::string pinocchio_robot_description,
                                         bool is_floating_base)
  : PinocchioRobotModel(robot_description, pinocchio_robot_description, is_floating_base, Config())
{
}

PinocchioRobotModel::PinocchioRobotModel(std::string robot_description, std::string pinocchio_robot_description,
                                         bool is_floating_base, const Config& config)
  : robot_description_(robot_description)
  , pinocchio_robot_description_(pinocchio_robot_description)
  , is_floating_base_(is_floating_base)
  , config_(config)
{
  const bool verbose = config_.verbose;

  // Initialize the model and data
  model_ = std::make_shared<pinocchio::Model>();

  if (!urdf_.initString(robot_description_))
  {
    std::cout << "Failed to extract urdf model from string." << std::endl;
    return;
  }
  std::vector<urdf::LinkSharedPtr> urdf_links;
  urdf_.getLinks(urdf_links);

  if (is_floating_base_)
    pinocchio::urdf::buildModelFromXML(pinocchio_robot_description_, pinocchio::JointModelFreeFlyer(), *model_);
  else
    pinocchio::urdf::buildModelFromXML(pinocchio_robot_description_, *model_);

  if (is_floating_base_)
  {
    model_->lowerPositionLimit.segment<3>(0).setConstant(-100);  // position
    model_->upperPositionLimit.segment<3>(0).setConstant(100);   // position
    model_->lowerPositionLimit.segment<4>(3).setConstant(-1.0);  // quaternion
    model_->upperPositionLimit.segment<4>(3).setConstant(1.0);   // quaternion
  }

  // create zero gravity model
  zero_gravity_model_ = std::make_shared<pinocchio::Model>(*model_);
  zero_gravity_model_->gravity.setZero();

  // Initialize the data structure
  data_ = std::make_shared<pinocchio::Data>(*model_);

  if (verbose)
  {
    std::cout << "model nq: " << model_->nq << std::endl;
    std::cout << "model nv: " << model_->nv << std::endl;
    std::cout << "model njoints: " << model_->njoints << std::endl;
    std::cout << "model nframes: " << model_->nframes << std::endl;
  }

  // initialize robot model with neutral configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nq);
  pinocchio::computeAllTerms(*model_, *data_, q, Eigen::VectorXd::Zero(model_->nv));
  pinocchio::framesForwardKinematics(*model_, *data_, q);

  // Parse the URDF string to xml
  TiXmlDocument robot_model_xml;
  robot_model_xml.Parse(pinocchio_robot_description_.c_str());

  // get baselink name from urdf
  TiXmlElement* baselink_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("baselink");
  std::string baselink;
  if (!baselink_attr)
  {
    if (verbose)
      std::cout << "Can not get baselink attribute from urdf model" << std::endl;
  }
  else
    baselink = std::string(baselink_attr->Attribute("name"));
  if (verbose)
    std::cout << "Baselink name: " << baselink << std::endl;

  // get rotor property
  TiXmlElement* m_f_rate_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("m_f_rate");
  if (!m_f_rate_attr)
  {
    if (verbose)
      std::cout << "Can not get m_f_rate attribute from urdf model" << std::endl;
  }
  else
    m_f_rate_attr->Attribute("value", &m_f_rate_);
  if (verbose)
    std::cout << "m_f_rate: " << m_f_rate_ << std::endl;

  // get joint torque limits
  joint_torque_limits_.resize(model_->nv);
  if (is_floating_base_)
    joint_torque_limits_.head(6).setConstant(0);  // no external force can be added to root link directly
  for (int i = is_floating_base_ ? 2 : 1; i < model_->njoints; i++)
  {
    std::string joint_name = model_->names[i];
    double torque_limit = 0;
    double max_position_limit = std::numeric_limits<double>::infinity();
    double min_position_limit = -std::numeric_limits<double>::infinity();
    pinocchio::JointIndex q_index = model_->joints[model_->getJointId(joint_name)].idx_q();
    pinocchio::JointIndex v_index = model_->joints[model_->getJointId(joint_name)].idx_v();

    for (const auto& link : urdf_links)
    {
      if (link->parent_joint)
      {
        if (link->parent_joint->name == joint_name)
        {
          torque_limit = link->parent_joint->limits->effort;
          max_position_limit = link->parent_joint->limits->upper;
          min_position_limit = link->parent_joint->limits->lower;

          joint_torque_limits_(v_index) = torque_limit;
          model_->upperPositionLimit(q_index) = max_position_limit;
          model_->lowerPositionLimit(q_index) = min_position_limit;
        }
      }
    }
  }
  if (verbose)
    std::cout << "Joint torque limits: " << joint_torque_limits_.transpose() << std::endl;

  // get rotor number (only frames named exactly "rotorN", e.g. exclude "rotor_arm1")
  rotor_num_ = 0;
  rotor_names_.clear();
  const std::regex rotor_name_pattern("rotor[0-9]+");
  for (int i = 0; i < model_->nframes; i++)
  {
    std::string frame_name = model_->frames[i].name;
    if (std::regex_match(frame_name, rotor_name_pattern))
    {
      rotor_names_.push_back(frame_name);
      rotor_num_++;
    }
  }
  if (verbose)
    std::cout << "Rotor number: " << rotor_num_ << std::endl;

  // sort by the numeric suffix (rotor1, rotor2, ..., rotor10)
  std::sort(rotor_names_.begin(), rotor_names_.end(), [](const std::string& a, const std::string& b) {
    auto num = [](const std::string& s) { return std::stoi(s.substr(s.find_first_of("0123456789"))); };
    return num(a) < num(b);
  });

  // build rotor frame indices in the same (numeric) order as rotor_names_
  rotor_frame_indices_.clear();
  for (const auto& rotor_name : rotor_names_)
    rotor_frame_indices_.push_back(model_->getFrameId(rotor_name));

  // rotor offset from parent joint
  joint_M_rotors_.clear();
  for (int i = 0; i < rotor_num_; i++)
  {
    std::string rotor_frame_name = rotor_names_.at(i);
    pinocchio::FrameIndex rotor_frame_index = model_->getFrameId(rotor_frame_name);
    pinocchio::JointIndex rotor_parent_joint_index = model_->frames[rotor_frame_index].parentJoint;

    pinocchio::SE3 w_M_rotor = data_->oMf[rotor_frame_index];
    pinocchio::SE3 w_M_joint = data_->oMi[rotor_parent_joint_index];
    pinocchio::SE3 joint_M_rotor = w_M_joint.inverse() * w_M_rotor;
    joint_M_rotors_.push_back(joint_M_rotor);
    if (verbose)
      std::cout << rotor_frame_name << " offset: \n" << joint_M_rotor << std::endl;
  }

  // Get thrust limits and rotor direction
  thrust_upper_limits_.resize(rotor_num_);
  thrust_lower_limits_.resize(rotor_num_);
  rotor_direction_.resize(rotor_num_);
  for (int i = 0; i < rotor_num_; i++)
  {
    for (const auto& link : urdf_links)
    {
      if (link->parent_joint)
      {
        if (link->parent_joint->name == rotor_names_.at(i))
        {
          double max_thrust = link->parent_joint->limits->upper;
          double min_thrust = link->parent_joint->limits->lower;
          int direction = link->parent_joint->axis.z;
          if (verbose)
            std::cout << rotor_names_.at(i) << " " << min_thrust << " " << max_thrust << " " << direction << std::endl;
          thrust_upper_limits_(i) = max_thrust;
          thrust_lower_limits_(i) = min_thrust;
          rotor_direction_.at(i) = direction;
        }
      }
    }
  }
  if (verbose)
  {
    std::cout << std::endl;

    // Print joint information
    std::cout << "joints:" << std::endl;
    for (int i = 0; i < model_->njoints; i++)
    {
      std::string joint_type = model_->joints[i].shortname();
      std::cout << model_->names[i] << " " << joint_type << " "
                << model_->joints[model_->getJointId(model_->names[i])].idx_q() << std::endl;
    }
    std::cout << std::endl;

    // Print frame information
    std::cout << "frames:" << std::endl;
    for (int i = 0; i < model_->nframes; i++)
    {
      std::string frame_name = model_->frames[i].name;
      std::cout << frame_name << std::endl;
    }

    std::cout << "hessian weight: " << config_.thrust_hessian_weight << std::endl;
  }
}

Eigen::VectorXd PinocchioRobotModel::forwardDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                     const Eigen::VectorXd& tau, Eigen::VectorXd& thrust)
{
  std::vector<pinocchio::Force> fext = computeFExtByThrust(thrust);

  // Compute the forward dynamics with external forces
  Eigen::VectorXd a = pinocchio::aba(*model_, *data_, q, v, tau, fext, pinocchio::Convention::LOCAL);

  return a;
}

Eigen::MatrixXd PinocchioRobotModel::forwardDynamicsDerivatives(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                                const Eigen::VectorXd& tau, Eigen::VectorXd& thrust)
{
  std::vector<pinocchio::Force> fext = computeFExtByThrust(thrust);

  // Compute the forward dynamics with external forces
  pinocchio::computeABADerivatives(*model_, *data_, q, v, tau, fext);

  Eigen::MatrixXd tauext_partial_thrust = computeTauExtByThrustDerivative(q);

  return data_->Minv * tauext_partial_thrust;
}

void PinocchioRobotModel::jointTorqueBounds(const Eigen::VectorXd& base_residual_lower,
                                            const Eigen::VectorXd& base_residual_upper, Eigen::VectorXd& lower,
                                            Eigen::VectorXd& upper) const
{
  lower = -joint_torque_limits_;
  upper = joint_torque_limits_;
  if (!is_floating_base_)
    return;

  if (base_residual_lower.size() != 6 || base_residual_upper.size() != 6)
    throw std::invalid_argument("base residual bounds must have six elements");
  if ((base_residual_upper.array() < base_residual_lower.array()).any())
    throw std::invalid_argument("base residual lower bound must not exceed the upper bound");

  lower.head(6) = base_residual_lower;
  upper.head(6) = base_residual_upper;
}

bool PinocchioRobotModel::inverseDynamicsOsqp(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                              const Eigen::VectorXd& a, Eigen::VectorXd& tau,
                                              const Eigen::VectorXd& base_residual_lower,
                                              const Eigen::VectorXd& base_residual_upper)
{
  auto start = std::chrono::high_resolution_clock::now();

  // Compute normal inverse dynamics
  Eigen::VectorXd rnea_solution = pinocchio::rnea(*model_, *data_, q, v, a);

  int n_variables = model_->nv + rotor_num_;
  int n_constraints = (model_->nv + rotor_num_) + model_->nv;  // box constraint + rnea constraint

  // make hessian matrix
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_variables, n_variables);
  H.setIdentity();
  H.bottomRightCorner(rotor_num_, rotor_num_) *= config_.thrust_hessian_weight;

  // make gradient vector
  gradient_ = Eigen::VectorXd::Zero(n_variables);

  // make constraint matrix
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_constraints, n_variables);
  A.setIdentity();                         // box constraint
  A.bottomRows(model_->nv).setIdentity();  // rnea constraint
  A.block(n_variables, model_->nv, model_->nv, rotor_num_) =
      this->computeTauExtByThrustDerivative(q);  // thrust constraint

  // make bounds
  lower_bound_ = Eigen::VectorXd::Zero(n_constraints);
  upper_bound_ = Eigen::VectorXd::Zero(n_constraints);

  Eigen::VectorXd torque_lower, torque_upper;
  jointTorqueBounds(base_residual_lower, base_residual_upper, torque_lower, torque_upper);

  lower_bound_.head(model_->nv) = torque_lower;                         // joint torque inequality constraint
  lower_bound_.segment(model_->nv, rotor_num_) = thrust_lower_limits_;  // thrust inequality constraint
  lower_bound_.tail(model_->nv) = rnea_solution;                        // rnea equality constraint

  upper_bound_.head(model_->nv) = torque_upper;                         // joint torque inequality constraint
  upper_bound_.segment(model_->nv, rotor_num_) = thrust_upper_limits_;  // thrust inequality constraint
  upper_bound_.tail(model_->nv) = rnea_solution;                        // rnea equality constraint

  // qp solver
  bool ok = true;
  Eigen::SparseMatrix<double> H_s = H.sparseView();
  Eigen::SparseMatrix<double> A_s = A.sparseView();
  if (!id_solver_.isInitialized())
  {
    id_solver_.settings()->setVerbosity(false);
    id_solver_.settings()->setWarmStart(true);
    id_solver_.settings()->setPolish(false);
    id_solver_.settings()->setMaxIteraction(1000);
    id_solver_.settings()->setAbsoluteTolerance(1e-8);
    id_solver_.settings()->setRelativeTolerance(1e-8);

    id_solver_.data()->setNumberOfVariables(n_variables);
    id_solver_.data()->setNumberOfConstraints(n_constraints);
    ok &= id_solver_.data()->setHessianMatrix(H_s);
    ok &= id_solver_.data()->setGradient(gradient_);
    ok &= id_solver_.data()->setLinearConstraintsMatrix(A_s);
    ok &= id_solver_.data()->setLowerBound(lower_bound_);
    ok &= id_solver_.data()->setUpperBound(upper_bound_);
    ok &= id_solver_.initSolver();
  }
  else
  {
    ok &= id_solver_.updateHessianMatrix(H_s);
    ok &= id_solver_.updateGradient(gradient_);
    ok &= id_solver_.updateBounds(lower_bound_, upper_bound_);
    ok &= id_solver_.updateLinearConstraintsMatrix(A_s);
  }
  ok &= id_solver_.solve();

  tau = id_solver_.getSolution();

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  latest_id_solve_time_ = duration.count();  // microseconds

  return ok;
}

bool PinocchioRobotModel::inverseDynamicsProxqp(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                const Eigen::VectorXd& a, Eigen::VectorXd& tau,
                                                const Eigen::VectorXd& base_residual_lower,
                                                const Eigen::VectorXd& base_residual_upper)
{
  auto start = std::chrono::high_resolution_clock::now();

  // Compute normal inverse dynamics
  Eigen::VectorXd rnea_solution = pinocchio::rnea(*model_, *data_, q, v, a);

  // Decision variables x = [joint torque (nv); thrust (rotor_num)].  ProxQP
  // solves 0.5 x'Hx + g'x s.t. A x = b (equality), l <= C x <= u (inequality)
  int n_variables = model_->nv + rotor_num_;
  int n_eq = model_->nv;               // rnea equality constraint
  int n_in = model_->nv + rotor_num_;  // joint torque + thrust box constraint

  // cost
  Eigen::MatrixXd H = Eigen::MatrixXd::Identity(n_variables, n_variables);
  H.bottomRightCorner(rotor_num_, rotor_num_) *= config_.thrust_hessian_weight;
  Eigen::VectorXd g = Eigen::VectorXd::Zero(n_variables);

  // equality: joint_torque + dtauext/dthrust * thrust = rnea_solution
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_eq, n_variables);
  A.leftCols(model_->nv).setIdentity();
  A.rightCols(rotor_num_) = this->computeTauExtByThrustDerivative(q);
  Eigen::VectorXd b = rnea_solution;

  // inequality (box): torque/thrust limits
  Eigen::MatrixXd C = Eigen::MatrixXd::Identity(n_in, n_variables);
  Eigen::VectorXd l = Eigen::VectorXd::Zero(n_in);
  Eigen::VectorXd u = Eigen::VectorXd::Zero(n_in);
  Eigen::VectorXd torque_lower, torque_upper;
  jointTorqueBounds(base_residual_lower, base_residual_upper, torque_lower, torque_upper);
  l.head(model_->nv) = torque_lower;
  l.tail(rotor_num_) = thrust_lower_limits_;
  u.head(model_->nv) = torque_upper;
  u.tail(rotor_num_) = thrust_upper_limits_;

  if (!id_solver_proxqp_)
  {
    id_solver_proxqp_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(n_variables, n_eq, n_in);
    id_solver_proxqp_->settings.eps_abs = 1e-8;
    id_solver_proxqp_->settings.eps_rel = 0.0;
    id_solver_proxqp_->settings.max_iter = 1000;
    id_solver_proxqp_->settings.verbose = false;
    id_solver_proxqp_->init(H, g, A, b, C, l, u);
  }
  else
  {
    // A and b change with the configuration, and l and u change with residual bounds
    id_solver_proxqp_->settings.initial_guess =
        config_.warm_start_inverse_dynamics ? proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT :
                                              proxsuite::proxqp::InitialGuessStatus::EQUALITY_CONSTRAINED_INITIAL_GUESS;
    id_solver_proxqp_->update(proxsuite::nullopt, proxsuite::nullopt, A, b, proxsuite::nullopt, l, u);
  }

  id_solver_proxqp_->solve();

  bool ok = id_solver_proxqp_->results.info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED;
  tau = id_solver_proxqp_->results.x;

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  latest_id_solve_time_ = duration.count();  // microseconds

  return ok;
}

std::vector<Eigen::MatrixXd> PinocchioRobotModel::computeTauExtByThrustDerivativeQDerivatives(const Eigen::VectorXd& q)
{
  std::vector<Eigen::MatrixXd> tauext_partial_thrust_partial_q(model_->nv,
                                                               Eigen::MatrixXd::Zero(model_->nv, rotor_num_));

  pinocchio::computeJointKinematicHessians(*model_, *data_, q);

  Eigen::Tensor<double, 3> rotor_i_frame_hessian(6, model_->nv, model_->nv);
  for (int i = 0; i < rotor_num_; i++)
  {
    // get rotor frame index
    std::string rotor_frame_name = rotor_names_.at(i);
    pinocchio::FrameIndex rotor_frame_index = model_->getFrameId(rotor_frame_name);

    // get rotor frame kinematic hessian
    rotor_i_frame_hessian.setZero();
    pinocchio::getFrameKinematicHessian(*model_, *data_, rotor_frame_index, pinocchio::LOCAL,
                                        rotor_i_frame_hessian);  // 6 * nv * nv

    // make thrust wrench unit
    pinocchio::Force thrust_wrench_unit;
    thrust_wrench_unit.linear() = Eigen::Vector3d(0, 0, 1);
    thrust_wrench_unit.angular() = Eigen::Vector3d(0, 0, rotor_direction_.at(i) * m_f_rate_);

    // get jacobian of rotor_i jacobian w.r.t q_j
    for (int j = 0; j < model_->nv; j++)
    {
      const double* ptr = rotor_i_frame_hessian.data() + 6 * model_->nv * j;
      Eigen::Map<const Eigen::Matrix<double, 6, Eigen::Dynamic>> rotor_i_frame_jacobian_partial_q_j(ptr, 6, model_->nv);

      tauext_partial_thrust_partial_q.at(j).col(i) =
          rotor_i_frame_jacobian_partial_q_j.transpose() * thrust_wrench_unit.toVector();
    }
  }

  return tauext_partial_thrust_partial_q;
}

std::vector<Eigen::MatrixXd>
PinocchioRobotModel::computeTauExtByThrustDerivativeQDerivativesNum(const Eigen::VectorXd& q)
{
  std::vector<Eigen::MatrixXd> tauext_partial_thrust_partial_q(model_->nv,
                                                               Eigen::MatrixXd::Zero(model_->nv, rotor_num_));

  double epsilon = 1e-6;
  Eigen::VectorXd original_q = q;
  Eigen::MatrixXd original_tauext_partial_thrust = this->computeTauExtByThrustDerivative(original_q);
  Eigen::VectorXd tmp_q = original_q;

  Eigen::VectorXd v = Eigen::VectorXd::Zero(model_->nv);
  for (int i = 0; i < model_->nv; i++)
  {
    v = Eigen::VectorXd::Zero(model_->nv);
    v(i) = 1.0;

    tmp_q = pinocchio::integrate(*model_, original_q, v * epsilon);

    Eigen::MatrixXd tauext_partial_thrust_plus = this->computeTauExtByThrustDerivative(tmp_q);
    tauext_partial_thrust_partial_q.at(i) = (tauext_partial_thrust_plus - original_tauext_partial_thrust) / epsilon;
  }

  return tauext_partial_thrust_partial_q;
}

Eigen::VectorXd PinocchioRobotModel::computeTauExtByThrust(const Eigen::VectorXd& q, const Eigen::VectorXd& thrust)
{
  return computeTauExtByThrustDerivative(q) * thrust;
}

Eigen::MatrixXd PinocchioRobotModel::computeTauExtByThrustDerivative(const Eigen::VectorXd& q)
{
  Eigen::MatrixXd tauext_partial_thrust = Eigen::MatrixXd::Zero(model_->nv, rotor_num_);

  for (int i = 0; i < rotor_num_; i++)
  {
    Eigen::MatrixXd rotor_i_jacobian =
        Eigen::MatrixXd::Zero(6, model_->nv);  // must be initialized by zeros. see frames.hpp

    std::string rotor_frame_name = rotor_names_.at(i);
    pinocchio::FrameIndex rotor_frame_index = model_->getFrameId(rotor_frame_name);

    pinocchio::computeFrameJacobian(*model_, *data_, q, rotor_frame_index, pinocchio::LOCAL,
                                    rotor_i_jacobian);  // LOCAL

    // thrust wrench unit
    Eigen::VectorXd thrust_wrench_unit = Eigen::VectorXd::Zero(6);
    thrust_wrench_unit.head<3>() = Eigen::Vector3d(0, 0, 1);
    thrust_wrench_unit.tail<3>() = Eigen::Vector3d(0, 0, rotor_direction_.at(i) * m_f_rate_);
    tauext_partial_thrust.col(i) = rotor_i_jacobian.transpose() * thrust_wrench_unit;
  }

  return tauext_partial_thrust;
}

Eigen::MatrixXd PinocchioRobotModel::computeTauExtByThrustQDerivativeRnea(const Eigen::VectorXd& q,
                                                                          const Eigen::VectorXd& thrust)
{
  // Compute RNEA derivatives with external forces
  std::vector<pinocchio::Force> fext = computeFExtByThrust(thrust);
  pinocchio::computeRNEADerivatives(*zero_gravity_model_, *data_, q, Eigen::VectorXd::Zero(model_->nv),
                                    Eigen::VectorXd::Zero(model_->nv), fext);

  return -data_->dtau_dq;
}

Eigen::MatrixXd PinocchioRobotModel::computeTauExtByThrustQDerivativeStaticTorque(const Eigen::VectorXd& q,
                                                                                  const Eigen::VectorXd& thrust)
{
  // Compute static torque derivatives with external forces
  std::vector<pinocchio::Force> fext = computeFExtByThrust(thrust);
  Eigen::MatrixXd dtau_dq = Eigen::MatrixXd::Zero(model_->nv, model_->nv);
  pinocchio::computeStaticTorqueDerivatives(*zero_gravity_model_, *data_, q, fext, dtau_dq);

  return -dtau_dq;
}

Eigen::MatrixXd PinocchioRobotModel::computeTauExtByThrustQDerivativeHessian(const Eigen::VectorXd& q,
                                                                             const Eigen::VectorXd& thrust)
{
  // hessian based method. thrust_genforce_units_dq * thrust
  std::vector<Eigen::MatrixXd> tauext_partial_thrust_partial_q = this->computeTauExtByThrustDerivativeQDerivatives(q);

  Eigen::MatrixXd tauext_by_thrust_q_derivative = Eigen::MatrixXd::Zero(model_->nv, model_->nv);
  for (int i = 0; i < model_->nv; i++)
  {
    tauext_by_thrust_q_derivative.col(i) = tauext_partial_thrust_partial_q.at(i) * thrust;
  }

  return tauext_by_thrust_q_derivative;
}

Eigen::MatrixXd PinocchioRobotModel::computeTauExtByThrustQDerivativeNum(const Eigen::VectorXd& q,
                                                                         const Eigen::VectorXd& thrust)
{
  Eigen::MatrixXd tauext_by_thrust_q_derivative = Eigen::MatrixXd::Zero(model_->nv, model_->nv);

  double epsilon = 1e-6;
  Eigen::VectorXd original_q = q;
  Eigen::VectorXd original_tauext_by_thrust = this->computeTauExtByThrust(original_q, thrust);

  for (int i = 0; i < model_->nv; i++)
  {
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_->nv);
    v(i) = 1.0;

    Eigen::VectorXd tmp_q = pinocchio::integrate(*model_, original_q, v * epsilon);

    Eigen::VectorXd tauext_by_thrust_plus = this->computeTauExtByThrust(tmp_q, thrust);
    tauext_by_thrust_q_derivative.col(i) = (tauext_by_thrust_plus - original_tauext_by_thrust) / epsilon;
  }

  return tauext_by_thrust_q_derivative;
}

std::vector<pinocchio::Force> PinocchioRobotModel::computeFExtByThrust(const Eigen::VectorXd& thrust)
{
  // Compute external wrench by thrust
  std::vector<pinocchio::Force> fext(model_->njoints, pinocchio::Force::Zero());
  for (int i = 0; i < rotor_num_; i++)
  {
    std::string rotor_frame_name = rotor_names_.at(i);
    pinocchio::FrameIndex rotor_frame_index = model_->getFrameId(rotor_frame_name);
    pinocchio::JointIndex rotor_parent_joint_index = model_->frames[rotor_frame_index].parentJoint;

    // LOCAL
    pinocchio::Force rotor_frame_wrench;
    rotor_frame_wrench.linear() = Eigen::Vector3d(0, 0, thrust(i));
    rotor_frame_wrench.angular() = Eigen::Vector3d(0, 0, rotor_direction_.at(i) * m_f_rate_ * thrust(i));

    // Convert to parent joint frame
    pinocchio::Force rotor_parent_joint_wrench =
        joint_M_rotors_.at(i).act(rotor_frame_wrench);  // rotor frame to parent joint frame

    fext.at(rotor_parent_joint_index) = rotor_parent_joint_wrench;  // add to parent joint
  }

  return fext;
}

Eigen::VectorXd PinocchioRobotModel::getResetConfiguration()
{
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nq);
  if (is_floating_base_)
    q(6) = 1;

  for (int i = 1; i < getRotorNum(); i++)
    q(model_->joints[model_->getJointId("joint" + std::to_string(i) + "_yaw")].idx_q()) = 2.0 * M_PI / getRotorNum();

  return q;
}
