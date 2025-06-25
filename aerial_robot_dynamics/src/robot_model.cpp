#include <aerial_robot_dynamics/robot_model.h>

using namespace aerial_robot_dynamics;

PinocchioRobotModel::PinocchioRobotModel(bool is_floating_base)
{
  is_floating_base_ = is_floating_base;

  // Initialize the model and data
  model_ = std::make_shared<pinocchio::Model>();

  if (!urdf_.initParam("robot_description"))
  {
    ROS_ERROR("Failed to extract urdf model from rosparam");
    return;
  }
  std::vector<urdf::LinkSharedPtr> urdf_links;
  urdf_.getLinks(urdf_links);

  // Initialize model with URDF file
  std::string pinocchio_robot_description = "";
  while (!getRobotModelXml("pinocchio_robot_description", pinocchio_robot_description))
  {
  }

  if (is_floating_base_)
    pinocchio::urdf::buildModelFromXML(pinocchio_robot_description, pinocchio::JointModelFreeFlyer(), *model_);
  else
    pinocchio::urdf::buildModelFromXML(pinocchio_robot_description, *model_);

  if (is_floating_base_)
  {
    model_->lowerPositionLimit.segment<3>(0).setConstant(-100);  // position
    model_->upperPositionLimit.segment<3>(0).setConstant(100);   // position
    model_->lowerPositionLimit.segment<4>(3).setConstant(-1.0);  // quaternion
    model_->upperPositionLimit.segment<4>(3).setConstant(1.0);   // quaternion
  }

  // Initialize the data structure
  data_ = std::make_shared<pinocchio::Data>(*model_);

  std::cout << "model nq: " << model_->nq << std::endl;
  std::cout << "model nv: " << model_->nv << std::endl;
  std::cout << "model njoints: " << model_->njoints << std::endl;
  std::cout << "model nframes: " << model_->nframes << std::endl;

  // initialize robot model with neutral configuration
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nq);
  pinocchio::framesForwardKinematics(*model_, *data_, q);

  // Parse the URDF string to xml
  TiXmlDocument robot_model_xml;
  robot_model_xml.Parse(pinocchio_robot_description.c_str());

  // get baselink name from urdf
  TiXmlElement* baselink_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("baselink");
  std::string baselink;
  if (!baselink_attr)
    ROS_DEBUG("Can not get baselink attribute from urdf model");
  else
    baselink = std::string(baselink_attr->Attribute("name"));
  std::cout << "Baselink name: " << baselink << std::endl;

  // get rotor property
  TiXmlElement* m_f_rate_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("m_f_rate");
  if (!m_f_rate_attr)
    ROS_ERROR("Can not get m_f_rate attribute from urdf model");
  else
    m_f_rate_attr->Attribute("value", &m_f_rate_);
  std::cout << "m_f_rate: " << m_f_rate_ << std::endl;

  // get joint torque limits
  joint_torque_limits_.resize(model_->nv);
  for (int i = is_floating_base_ ? 2 : 1; i < model_->njoints; i++)
  {
    std::string joint_name = model_->names[i];
    double torque_limit = 0;
    pinocchio::JointIndex v_index = model_->joints[model_->getJointId(joint_name)].idx_v();

    for (const auto& link : urdf_links)
    {
      if (link->parent_joint)
      {
        if (link->parent_joint->name == joint_name)
        {
          torque_limit = link->parent_joint->limits->effort;
          joint_torque_limits_(v_index) = torque_limit;
        }
      }
    }
  }
  std::cout << "Joint torque limits: " << joint_torque_limits_.transpose() << std::endl;

  // get rotor number
  rotor_num_ = 0;
  rotor_names_.clear();
  for (int i = 0; i < model_->nframes; i++)
  {
    std::string frame_name = model_->frames[i].name;
    if (frame_name.find("rotor") != std::string::npos)
    {
      rotor_names_.push_back(frame_name);
      rotor_num_++;
    }
  }
  std::cout << "Rotor number: " << rotor_num_ << std::endl;
  std::sort(rotor_names_.begin(), rotor_names_.end());  // alphabetical order

  // rotor offset from parent joint
  joint_M_rotors_.clear();
  for (int i = 0; i < rotor_num_; i++)
  {
    std::string rotor_frame_name = rotor_names_.at(i);
    pinocchio::FrameIndex rotor_frame_index = model_->getFrameId(rotor_frame_name);
    pinocchio::JointIndex rotor_parent_joint_index = model_->frames[rotor_frame_index].parent;

    pinocchio::SE3 w_M_rotor = data_->oMf[rotor_frame_index];
    pinocchio::SE3 w_M_joint = data_->oMi[rotor_parent_joint_index];
    pinocchio::SE3 joint_M_rotor = w_M_joint.inverse() * w_M_rotor;
    joint_M_rotors_.push_back(joint_M_rotor);
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
          std::cout << rotor_names_.at(i) << " " << min_thrust << " " << max_thrust << " " << direction << std::endl;
          thrust_upper_limits_(i) = max_thrust;
          thrust_lower_limits_(i) = min_thrust;
          rotor_direction_.at(i) = direction;
        }
      }
    }
  }
  std::cout << std::endl;

  // Print joint information
  std::vector<int> q_dims(model_->njoints);
  int joint_index = 0;
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

  // Get parameters from ROS parameter server
  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle dynamics_nh(nh, "dynamics");
  getParam<double>(dynamics_nh, "thrust_hessian_weight", thrust_hessian_weight_, 1.0);
  std::cout << "hessian weight: " << thrust_hessian_weight_ << std::endl;
}

Eigen::VectorXd PinocchioRobotModel::forwardDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                     const Eigen::VectorXd& tau, Eigen::VectorXd& thrust)
{
  pinocchio::container::aligned_vector<pinocchio::Force> fext = computeFExtByThrust(thrust);

  // Compute the forward dynamics with external forces
  Eigen::VectorXd a = pinocchio::aba(*model_, *data_, q, v, tau, fext, pinocchio::Convention::LOCAL);

  return a;
}

Eigen::MatrixXd PinocchioRobotModel::forwardDynamicsDerivatives(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                                const Eigen::VectorXd& tau, Eigen::VectorXd& thrust)
{
  pinocchio::container::aligned_vector<pinocchio::Force> fext = computeFExtByThrust(thrust);

  // Compute the forward dynamics with external forces
  pinocchio::computeABADerivatives(*model_, *data_, q, v, tau, fext);

  Eigen::MatrixXd tauext_partial_thrust = computeTauExtByThrustDerivative(q);

  return data_->Minv * tauext_partial_thrust;
}

bool PinocchioRobotModel::inverseDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& a,
                                          Eigen::VectorXd& tau)
{
  auto start = std::chrono::high_resolution_clock::now();

  // Compute normal inverse dynamics
  Eigen::VectorXd rnea_solution = pinocchio::rnea(*model_, *data_, q, v, a);

  int n_variables = model_->nv + rotor_num_;
  int n_constraints = (model_->nv + rotor_num_) + model_->nv;  // box constraint + rnea constraint

  // make hessian matrix
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_variables, n_variables);
  H.setIdentity();
  H.bottomRightCorner(rotor_num_, rotor_num_) *= thrust_hessian_weight_;

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

  lower_bound_.head(model_->nv) = -joint_torque_limits_;                // joint torque inequality constraint
  lower_bound_.segment(model_->nv, rotor_num_) = thrust_lower_limits_;  // thrust inequality constraint
  lower_bound_.tail(model_->nv) = rnea_solution;                        // rnea equality constraint

  upper_bound_.head(model_->nv) = joint_torque_limits_;                 // joint torque inequality constraint
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

bool PinocchioRobotModel::inverseDynamicsDerivatives(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                     const Eigen::VectorXd& a, Eigen::MatrixXd& id_partial_dq,
                                                     Eigen::MatrixXd& id_partial_dv, Eigen::MatrixXd& id_partial_da)
{
  // make solver parameters
  int n_variables = model_->nv + rotor_num_;
  int n_ineq_constraints = model_->nv + rotor_num_;  // box constraint
  int n_eq_constraints = model_->nv;                 // rnea constraint
  int n_constraints = n_ineq_constraints + n_eq_constraints;

  // Compute the inverse dynamics with external forces
  Eigen::VectorXd id_solution;
  bool ok = this->inverseDynamics(q, v, a, id_solution);
  Eigen::VectorXd id_solution_thrust = id_solution.tail(rotor_num_);
  Eigen::VectorXd id_dual_solution = id_solver_.getDualSolution();
  Eigen::VectorXd id_ineq_dual_solution = id_dual_solution.head(n_ineq_constraints);
  Eigen::VectorXd id_eq_dual_solution = id_dual_solution.tail(n_eq_constraints);

  int n_active_ineq_constraints = 0;
  for (int i = 0; i < n_ineq_constraints; i++)
  {
    if (fabs(id_dual_solution(i)) > 1e-6)
    {
      n_active_ineq_constraints++;
    }
  }

  // hessian
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_variables, n_variables);
  H.setIdentity();
  H.bottomRightCorner(rotor_num_, rotor_num_) *= thrust_hessian_weight_;

  // equality constraint matrix
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_eq_constraints, n_variables);
  A.setIdentity();                                                                                  // box constraint
  A.block(0, model_->nv, n_eq_constraints, rotor_num_) = this->computeTauExtByThrustDerivative(q);  // thrust constraint

  // active inequality constraint matrix and bound
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(n_active_ineq_constraints, n_variables);
  int last_row = 0;
  for (int i = 0; i < n_ineq_constraints; i++)
  {
    if (fabs(id_dual_solution(i)) > 1e-6)
    {
      C.row(last_row) = A.row(i);
      last_row++;
    }
  }

  // make KKT condition matrix
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(n_variables + n_eq_constraints + n_active_ineq_constraints,
                                            n_variables + n_eq_constraints + n_active_ineq_constraints);
  K.block(0, 0, n_variables, n_variables) = H;
  K.block(n_variables, 0, n_eq_constraints, n_variables) = A;
  K.block(n_variables + n_eq_constraints, 0, n_active_ineq_constraints, n_variables) = C;
  K.block(0, n_variables, n_variables, n_eq_constraints) = A.transpose();
  K.block(0, n_variables + n_eq_constraints, n_variables, n_active_ineq_constraints) = C.transpose();

  Eigen::SparseMatrix<double> K_s = K.sparseView();
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> K_ldlt;
  K_ldlt.compute(K_s);

  pinocchio::container::aligned_vector<pinocchio::Force> fext = computeFExtByThrust(id_solution_thrust);
  pinocchio::computeRNEADerivatives(*model_, *data_, q, v, a);  // not sure we need set fext

  Eigen::MatrixXd rnea_partial_dq = data_->dtau_dq;
  Eigen::MatrixXd rnea_partial_dv = data_->dtau_dv;
  Eigen::MatrixXd rnea_partial_da = data_->M;
  rnea_partial_da.triangularView<Eigen::StrictlyLower>() = rnea_partial_da.transpose();

  Eigen::MatrixXd kkt_sensitivity =
      Eigen::MatrixXd::Zero(n_variables + n_eq_constraints + n_active_ineq_constraints,
                            model_->nv);  // the number of cols is equal to parameters = model_->nv

  std::vector<Eigen::MatrixXd> tauext_partial_thrust_partial_q = this->computeTauExtByThrustDerivativeQDerivatives(q);
  std::vector<Eigen::MatrixXd> A_partial_dq(model_->nv, Eigen::MatrixXd::Zero(n_eq_constraints, n_variables));
  std::vector<Eigen::MatrixXd> A_partial_dq_transpose(model_->nv, Eigen::MatrixXd::Zero(n_variables, n_eq_constraints));
  for (int i = 0; i < tauext_partial_thrust_partial_q.size(); i++)
  {
    A_partial_dq.at(i).block(0, model_->nv, n_eq_constraints, rotor_num_) = tauext_partial_thrust_partial_q.at(i);
    A_partial_dq_transpose.at(i) = A_partial_dq.at(i).transpose();
  }
  Eigen::MatrixXd A_partial_dq_transpose_lambda_contraction =
      tensorContraction(A_partial_dq_transpose, id_eq_dual_solution);
  Eigen::MatrixXd A_partial_dq_u_contraction = tensorContraction(A_partial_dq, id_solution);

  kkt_sensitivity.setZero();
  kkt_sensitivity.topRows(n_variables) = -A_partial_dq_transpose_lambda_contraction;
  ;
  kkt_sensitivity.block(n_variables, 0, n_eq_constraints, model_->nv) = -A_partial_dq_u_contraction + rnea_partial_dq;
  id_partial_dq = K_ldlt.solve(kkt_sensitivity).topRows(n_variables);

  kkt_sensitivity.setZero();
  kkt_sensitivity.block(n_variables, 0, n_eq_constraints, model_->nv) = rnea_partial_dv;
  id_partial_dv = K_ldlt.solve(kkt_sensitivity).topRows(n_variables);

  kkt_sensitivity.setZero();
  kkt_sensitivity.block(n_variables, 0, n_eq_constraints, model_->nv) = rnea_partial_da;
  id_partial_da = K_ldlt.solve(kkt_sensitivity).topRows(n_variables);

  return ok;
}

std::vector<Eigen::MatrixXd> PinocchioRobotModel::computeTauExtByThrustDerivativeQDerivatives(const Eigen::VectorXd& q)
{
  std::vector<Eigen::MatrixXd> tauext_partial_thrust_partial_q(model_->nv,
                                                               Eigen::MatrixXd::Zero(model_->nv, rotor_num_));

  pinocchio::computeJointKinematicHessians(*model_, *data_, q);
  for (int i = 0; i < rotor_num_; i++)
  {
    // get rotor joint index
    std::string rotor_frame_name = rotor_names_.at(i);
    pinocchio::FrameIndex rotor_frame_index = model_->getFrameId(rotor_frame_name);
    pinocchio::JointIndex rotor_parent_joint_index = model_->frames[rotor_frame_index].parent;

    // get rotor joint kinematic hessian
    Eigen::Tensor<double, 3> rotor_i_parent_joint_hessian(6, model_->nv, model_->nv);
    rotor_i_parent_joint_hessian.setZero();
    pinocchio::getJointKinematicHessian(*model_, *data_, rotor_parent_joint_index, pinocchio::LOCAL,
                                        rotor_i_parent_joint_hessian);  // 6 * nv * nv

    // make thrust wrench unit in parent joint frame
    pinocchio::Force thrust_wrench_unit;
    thrust_wrench_unit.linear() = Eigen::Vector3d(0, 0, 1);
    thrust_wrench_unit.angular() = Eigen::Vector3d(0, 0, rotor_direction_.at(i) * m_f_rate_);
    pinocchio::Force thrust_wrench_unit_parent_joint = joint_M_rotors_.at(i).act(thrust_wrench_unit);

    // get jacobian of rotor_i jacobian w.r.t q_j
    for (int j = 0; j < model_->nv; j++)
    {
      const double* ptr = rotor_i_parent_joint_hessian.data() + 6 * model_->nv * j;
      Eigen::Map<const Eigen::Matrix<double, 6, Eigen::Dynamic>> rotor_i_parent_joint_jacobian_partial_q_j(ptr, 6,
                                                                                                           model_->nv);

      tauext_partial_thrust_partial_q.at(j).col(i) =
          rotor_i_parent_joint_jacobian_partial_q_j.transpose() * thrust_wrench_unit_parent_joint.toVector();
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

pinocchio::container::aligned_vector<pinocchio::Force>
PinocchioRobotModel::computeFExtByThrust(const Eigen::VectorXd& thrust)
{
  // Compute external wrench by thrust
  pinocchio::container::aligned_vector<pinocchio::Force> fext(model_->njoints, pinocchio::Force::Zero());
  for (int i = 0; i < rotor_num_; i++)
  {
    std::string rotor_frame_name = rotor_names_.at(i);
    pinocchio::FrameIndex rotor_frame_index = model_->getFrameId(rotor_frame_name);
    pinocchio::JointIndex rotor_parent_joint_index = model_->frames[rotor_frame_index].parent;

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

bool PinocchioRobotModel::getRobotModelXml(const std::string& param_name, std::string& pinocchio_robot_description,
                                           ros::NodeHandle nh)
{
  // This function should retrieve the robot model XML string from the parameter server
  if (!nh.getParam(param_name, pinocchio_robot_description))
  {
    ROS_ERROR_STREAM_THROTTLE(1.0, "Failed to get " << param_name << " from ros parameter server");
    return false;
  }
  else
    return true;
}

Eigen::VectorXd PinocchioRobotModel::getResetConfiguration()
{
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_->nq);
  if (is_floating_base_)
    q(6) = 1;
  q(model_->joints[model_->getJointId("joint1_yaw")].idx_q()) = M_PI / 2.0;
  q(model_->joints[model_->getJointId("joint2_yaw")].idx_q()) = M_PI / 2.0;
  q(model_->joints[model_->getJointId("joint3_yaw")].idx_q()) = M_PI / 2.0;

  return q;
}
