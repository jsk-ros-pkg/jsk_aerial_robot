#include <aerial_robot_dynamics/robot_model.h>

using namespace aerial_robot_dynamics;

PinocchioRobotModel::PinocchioRobotModel()
{
  // Initialize the model and data
  model_ = std::make_shared<pinocchio::Model>();

  // Initialize model with URDF file
  std::string robot_model_string = getRobotModelXml("robot_description");
  pinocchio::urdf::buildModelFromXML(robot_model_string, pinocchio::JointModelFreeFlyer(), *model_);
  model_->lowerPositionLimit.segment<3>(0).setConstant(-100); // position
  model_->upperPositionLimit.segment<3>(0).setConstant(100);  // position
  model_->lowerPositionLimit.segment<4>(3).setConstant(-1.0); // quaternion
  model_->upperPositionLimit.segment<4>(3).setConstant(1.0);  // quaternion

  // Initialize the data structure
  data_ = std::make_shared<pinocchio::Data>(*model_);

  std::cout << "model nq: " << model_->nq << std::endl;
  std::cout << "model nv: " << model_->nv << std::endl;
  std::cout << "model njoints: " << model_->njoints << std::endl;
  std::cout << "model nframes: " << model_->nframes << std::endl;

  // Parse the URDF string to xml
  TiXmlDocument robot_model_xml;
  robot_model_xml.Parse(robot_model_string.c_str());

  // get baselink name from urdf
  TiXmlElement* baselink_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("baselink");
  std::string baselink;
  if(!baselink_attr)
    ROS_DEBUG("Can not get baselink attribute from urdf model");
  else
    baselink = std::string(baselink_attr->Attribute("name"));
  std::cout << "Baselink name: " << baselink << std::endl;

  // get rotor property
  TiXmlElement* m_f_rate_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("m_f_rate");
  if(!m_f_rate_attr)
    ROS_ERROR("Can not get m_f_rate attribute from urdf model");
  else
    m_f_rate_attr->Attribute("value", &m_f_rate_);
  std::cout << "m_f_rate: " << m_f_rate_ << std::endl;
  TiXmlElement* max_thrust_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("max_force");
  if(!max_thrust_attr)
    ROS_ERROR("Can not get max_force attribute from urdf model");
  else
    max_thrust_attr->Attribute("value", &max_thrust_);
  std::cout << "max thrust: " << max_thrust_ << std::endl;
  TiXmlElement* min_thrust_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("min_force");
  if(!min_thrust_attr)
    ROS_ERROR("Can not get min_force attribute from urdf model");
  else
    min_thrust_attr->Attribute("value", &min_thrust_);
  std::cout << "min thrust: " << min_thrust_ << std::endl;

  // get joint torque limit
  TiXmlElement* joint_torque_limit_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("joint_torque_limit");
  if(!joint_torque_limit_attr)
    ROS_ERROR("Can not get joint_torque_limit attribute from urdf model");
  else
    joint_torque_limit_attr->Attribute("value", &joint_torque_limit_);
  std::cout << "joint torque limit: " << joint_torque_limit_ << std::endl;

  // get rotor number
  rotor_num_ = 0;
  for(int i = 0; i < model_->nframes; i++)
    {
      std::string frame_name = model_->frames[i].name;
      if(frame_name.find("rotor") != std::string::npos)
        {
          rotor_num_++;
        }
    }
  std::cout << "Rotor number: " << rotor_num_ << std::endl;
  std::cout << std::endl;

  // Print joint information
  std::vector<int> q_dims(model_->njoints);
  int joint_index = 0;
  for(int i = 0; i < model_->njoints; i++)
    {
      std::string joint_type = model_->joints[i].shortname();
      std::cout << model_->names[i] << " " << joint_type <<  " " << model_->joints[model_->getJointId(model_->names[i])].idx_q() << std::endl;
    }
  std::cout << std::endl;

  // Print frame information
  for(int i = 0; i < model_->nframes; i++)
    {
      std::string frame_name = model_->frames[i].name;
      std::cout << frame_name << std::endl;
    }
}

Eigen::VectorXd PinocchioRobotModel::forwardDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& tau, Eigen::VectorXd& thrust)
{
  pinocchio::container::aligned_vector<pinocchio::Force> fext = computeFExtByThrust(thrust);

  // Compute the forward dynamics with external forces
  Eigen::VectorXd a = pinocchio::aba(*model_, *data_, q, v, tau, fext, pinocchio::Convention::LOCAL);

  return a;
}

Eigen::MatrixXd PinocchioRobotModel::forwardDynamicsDerivatives(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& tau, Eigen::VectorXd& thrust)
{
  pinocchio::container::aligned_vector<pinocchio::Force> fext = computeFExtByThrust(thrust);

  // Compute the forward dynamics with external forces
  pinocchio::computeABADerivatives(*model_, *data_, q, v, tau, fext);

  Eigen::MatrixXd tauext_partial_thrust = computeTauExtByThrustDerivative(q);

  return data_->Minv * tauext_partial_thrust;
}

Eigen::VectorXd PinocchioRobotModel::inverseDynamics(const Eigen::VectorXd & q, const Eigen::VectorXd& v, const Eigen::VectorXd& a)
{
  // Compute normal inverse dynamics
  Eigen::VectorXd rnea_solution = pinocchio::rnea(*model_, *data_, q, v, a);

  int n_variables = model_->nv + rotor_num_;
  int n_constraints = (model_->nv + rotor_num_) + model_->nv; // box constraint + rnea constraint

  // make hessian matrix
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_variables, n_variables);
  H.setIdentity();
  H.bottomRightCorner(rotor_num_, rotor_num_) *= thrust_hessian_weight_;

  // make gradient vector
  gradient_ = Eigen::VectorXd::Zero(n_variables);

  // make constraint matrix
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_constraints, n_variables);
  A.setIdentity(); // box constraint
  A.bottomRows(model_->nv).setIdentity(); // rnea constraint
  A.block(n_variables, model_->nv, model_->nv, rotor_num_) = this->computeTauExtByThrustDerivative(q); // thrust constraint

  // make bounds
  lower_bound_ = Eigen::VectorXd::Zero(n_constraints);
  upper_bound_ = Eigen::VectorXd::Zero(n_constraints);

  lower_bound_.head(model_->nv) = Eigen::VectorXd::Constant(model_->nv, -joint_torque_limit_); // joint torque inequality constraint
  lower_bound_.segment(model_->nv, rotor_num_) = Eigen::VectorXd::Constant(rotor_num_, min_thrust_); // thrust inequality constraint
  lower_bound_.tail(model_->nv) = rnea_solution; // rnea equality constraint

  upper_bound_.head(model_->nv) = Eigen::VectorXd::Constant(model_->nv, joint_torque_limit_); // joint torque inequality constraint
  upper_bound_.segment(model_->nv, rotor_num_) = Eigen::VectorXd::Constant(rotor_num_, max_thrust_); // thrust inequality constraint
  upper_bound_.tail(model_->nv) = rnea_solution; // rnea equality constraint

  // qp solver
  Eigen::SparseMatrix<double> H_s = H.sparseView();
  Eigen::SparseMatrix<double> A_s = A.sparseView();
  if(!id_solver_.isInitialized())
    {
      id_solver_.settings()->setVerbosity(false);
      id_solver_.settings()->setWarmStart(true);
      id_solver_.settings()->setPolish(false);
      id_solver_.settings()->setMaxIteraction(1000);
      id_solver_.settings()->setAbsoluteTolerance(1e-8);
      id_solver_.settings()->setRelativeTolerance(1e-8);

      id_solver_.data()->setNumberOfVariables(n_variables);
      id_solver_.data()->setNumberOfConstraints(n_constraints);
      id_solver_.data()->setHessianMatrix(H_s);
      id_solver_.data()->setGradient(gradient_);
      id_solver_.data()->setLinearConstraintsMatrix(A_s);
      id_solver_.data()->setLowerBound(lower_bound_);
      id_solver_.data()->setUpperBound(upper_bound_);
      id_solver_.initSolver();
    }
  else
    {
      id_solver_.updateHessianMatrix(H_s);
      id_solver_.updateGradient(gradient_);
      id_solver_.updateBounds(lower_bound_, upper_bound_);
      id_solver_.updateLinearConstraintsMatrix(A_s);
    }

  id_solver_.solve();
  Eigen::VectorXd solution = id_solver_.getSolution();

  return solution;
}


std::vector<Eigen::MatrixXd> PinocchioRobotModel::computeTauExtByThrustDerivativeQDerivativesNum(const Eigen::VectorXd& q)
{
    std::vector<Eigen::MatrixXd> tauext_partial_thrust_partial_q(model_->nv, Eigen::MatrixXd::Zero(model_->nv, rotor_num_));

    double epsilon = 1e-6;
    Eigen::VectorXd original_q = q;
    Eigen::MatrixXd original_tauext_partial_thrust = this->computeTauExtByThrustDerivative(original_q);
    Eigen::VectorXd tmp_q = original_q;

    // root link position
    for(int i = 0; i < 3; i++)
    {
        tmp_q = original_q;
        tmp_q(i) += epsilon;
        Eigen::MatrixXd tauext_partial_thrust_plus = this->computeTauExtByThrustDerivative(tmp_q);
        tauext_partial_thrust_partial_q.at(i) = (tauext_partial_thrust_plus - original_tauext_partial_thrust) / epsilon;
    }

    // root link quaternion
    for(int i = 0; i < 3; i++)
    {
        tmp_q = original_q;
        double d_roll = i == 0 ? epsilon : 0; Eigen::AngleAxisd roll(Eigen::AngleAxisd(d_roll, Eigen::Vector3d::UnitX()));
        double d_pitch = i == 1 ? epsilon : 0; Eigen::AngleAxisd pitch(Eigen::AngleAxisd(d_pitch, Eigen::Vector3d::UnitY()));
        double d_yaw = i == 2 ? epsilon : 0; Eigen::AngleAxisd yaw(Eigen::AngleAxisd(d_yaw, Eigen::Vector3d::UnitZ()));

        Eigen::Matrix3d dR = (yaw * pitch * roll).toRotationMatrix();
        Eigen::Quaterniond dQuat(dR);
        Eigen::Quaterniond original_quat = Eigen::Quaterniond(original_q(6), original_q(3), original_q(4), original_q(5));
        Eigen::Quaterniond new_quat = dQuat * original_quat;
        new_quat.normalize();

        tmp_q(3) = new_quat.x();
        tmp_q(4) = new_quat.y();
        tmp_q(5) = new_quat.z();
        tmp_q(6) = new_quat.w();

        Eigen::MatrixXd tauext_partial_thrust_plus = this->computeTauExtByThrustDerivative(tmp_q);
        tauext_partial_thrust_partial_q.at(i + 3) = (tauext_partial_thrust_plus - original_tauext_partial_thrust) / epsilon;
    }

    // joint position
    for(int i = 7; i < model_->nq; i++)
    {
        tmp_q = original_q;
        tmp_q(i) += epsilon;
        Eigen::MatrixXd tauext_partial_thrust_plus = this->computeTauExtByThrustDerivative(tmp_q);
        tauext_partial_thrust_partial_q.at(i - 1) = (tauext_partial_thrust_plus - original_tauext_partial_thrust) / epsilon;
    }

    return tauext_partial_thrust_partial_q;
}

Eigen::MatrixXd PinocchioRobotModel::computeTauExtByThrustDerivative(const Eigen::VectorXd& q)
{
  Eigen::MatrixXd tauext_partial_thrust = Eigen::MatrixXd::Zero(model_->nv, rotor_num_);

  Eigen::MatrixXd rotor_i_jacobian = Eigen::MatrixXd::Zero(6, model_->nv); // must be initialized by zeros. see frames.hpp
  for(int i = 0; i < rotor_num_; i++)
    {
      std::string rotor_frame_name = "rotor" + std::to_string(i + 1);
      pinocchio::FrameIndex rotor_frame_index = model_->getFrameId(rotor_frame_name);

      pinocchio::computeFrameJacobian(*model_, *data_, q, rotor_frame_index, pinocchio::LOCAL, rotor_i_jacobian); // LOCAL

      // thrust wrench unit
      Eigen::VectorXd thrust_wrench_unit = Eigen::VectorXd::Zero(6);
      thrust_wrench_unit.head<3>() = Eigen::Vector3d(0, 0, 1);
      thrust_wrench_unit.tail<3>() = Eigen::Vector3d(0, 0, m_f_rate_);
      tauext_partial_thrust.col(i) = rotor_i_jacobian.transpose() * thrust_wrench_unit;
    }

  return tauext_partial_thrust;
}

pinocchio::container::aligned_vector<pinocchio::Force> PinocchioRobotModel::computeFExtByThrust(const Eigen::VectorXd& thrust)
{
  // Compute external wrench by thrust
  pinocchio::container::aligned_vector<pinocchio::Force> fext(model_->njoints, pinocchio::Force::Zero());
  for(int i = 0; i < rotor_num_; i++)
    {
      std::string rotor_frame_name = "rotor" + std::to_string(i + 1);
      pinocchio::FrameIndex rotor_frame_index = model_->getFrameId(rotor_frame_name);
      pinocchio::JointIndex rotor_parent_joint_index = model_->frames[rotor_frame_index].parent;

      // LOCAL
      pinocchio::Force rotor_wrench;
      rotor_wrench.linear() = Eigen::Vector3d(0, 0, thrust(i));
      rotor_wrench.angular() = Eigen::Vector3d(0, 0, m_f_rate_ * thrust(i));
      fext.at(rotor_parent_joint_index) = rotor_wrench;
    }

  return fext;
}

std::string PinocchioRobotModel::getRobotModelXml(const std::string& param_name, ros::NodeHandle nh)
{
  // This function should retrieve the robot model XML string from the parameter server
  std::string robot_model_string = "";
  if (!nh.getParam(param_name, robot_model_string))
    ROS_ERROR("Failed to get robot model XML from parameter server");
  return robot_model_string;
}
