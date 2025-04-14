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
  // make thrust vector as external wrench
  pinocchio::container::aligned_vector<pinocchio::Force> fext(model_->njoints, pinocchio::Force::Zero());
  for(int i = 0; i < rotor_num_; i++)
    {
      std::string rotor_frame_name = "rotor" + std::to_string(i + 1);
      pinocchio::FrameIndex rotor_frame_index = model_->getFrameId(rotor_frame_name);
      pinocchio::JointIndex rotor_parent_joint_index = model_->frames[rotor_frame_index].parent;
      pinocchio::Force rotor_wrench;

      // LOCAL
      rotor_wrench.linear() = Eigen::Vector3d(0, 0, thrust(i));
      rotor_wrench.angular() = Eigen::Vector3d(0, 0, m_f_rate_ * thrust(i));
      fext.at(rotor_parent_joint_index) = rotor_wrench;
    }

  // Compute the forward dynamics with external forces
  Eigen::VectorXd a = pinocchio::aba(*model_, *data_, q, v, tau, fext, pinocchio::Convention::LOCAL);

  return a;
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
  H.bottomRightCorner(rotor_num_, rotor_num_) *= 0.1;

  // make gradient vector
  Eigen::VectorXd g = Eigen::VectorXd::Zero(n_variables);

  // make constraint matrix
  Eigen::MatrixXd A(n_constraints, n_variables);
  A.setIdentity(); // box constraint
  A.bottomRows(model_->nv).setIdentity(); // rnea constraint

  // get thrust coordinate jacobians
  Eigen::MatrixXd rotor_i_jacobian = Eigen::MatrixXd::Zero(6, model_->nv); // must be initialized by zeros. see frames.hpp
  for(int i = 0; i < rotor_num_; i++)
    {
      std::string rotor_frame_name = "rotor" + std::to_string(i + 1);
      pinocchio::FrameIndex rotor_frame_index = model_->getFrameId(rotor_frame_name);
      pinocchio::JointIndex rotor_parent_joint_index = model_->frames[rotor_frame_index].parent;

      pinocchio::computeFrameJacobian(*model_, *data_, q, rotor_frame_index, pinocchio::LOCAL, rotor_i_jacobian); // LOCAL

      Eigen::VectorXd thrust_wrench_unit = Eigen::VectorXd::Zero(6);
      thrust_wrench_unit.head<3>() = Eigen::Vector3d(0, 0, 1);
      thrust_wrench_unit.tail<3>() = Eigen::Vector3d(0, 0, m_f_rate_);

      A.block(n_variables, model_->nv + i, model_->nv, 1) = rotor_i_jacobian.transpose() * thrust_wrench_unit;
    }

  // make bounds
  Eigen::VectorXd lb(n_constraints);
  Eigen::VectorXd ub(n_constraints);

  lb.head(model_->nv) = -1.0 * Eigen::VectorXd::Constant(model_->nv, joint_torque_limit_); // joint torque inequality constraint
  lb.segment(model_->nv, rotor_num_) = Eigen::VectorXd::Constant(rotor_num_, min_thrust_); // thrust inequality constraint
  lb.tail(model_->nv) = rnea_solution; // rnea equality constraint

  ub.head(model_->nv) =  1.0 * Eigen::VectorXd::Constant(model_->nv, joint_torque_limit_); // joint torque inequality constraint
  ub.segment(model_->nv, rotor_num_) = Eigen::VectorXd::Constant(rotor_num_, max_thrust_); // thrust inequality constraint
  ub.tail(model_->nv) = rnea_solution; // rnea equality constraint

  // qp solver
  Eigen::SparseMatrix<double> H_s = H.sparseView();
  Eigen::SparseMatrix<double> A_s = A.sparseView();
  if(!id_solver_.isInitialized())
    {
      id_solver_.settings()->setVerbosity(false);
      id_solver_.settings()->setWarmStart(true);
      id_solver_.settings()->setPolish(false);
      id_solver_.settings()->setMaxIteraction(1000);
      id_solver_.settings()->setAbsoluteTolerance(1e-4);
      id_solver_.settings()->setRelativeTolerance(1e-4);

      id_solver_.data()->setNumberOfVariables(n_variables);
      id_solver_.data()->setNumberOfConstraints(n_constraints);
      id_solver_.data()->setHessianMatrix(H_s);
      id_solver_.data()->setGradient(g);
      id_solver_.data()->setLinearConstraintsMatrix(A_s);
      id_solver_.data()->setLowerBound(lb);
      id_solver_.data()->setUpperBound(ub);
      id_solver_.initSolver();
    }
  else
    {
      id_solver_.updateHessianMatrix(H_s);
      id_solver_.updateGradient(g);
      id_solver_.updateLinearConstraintsMatrix(A_s);
      id_solver_.updateBounds(lb, ub);
    }

  id_solver_.solve();
  Eigen::VectorXd solution = id_solver_.getSolution();

  return solution;
}

std::string PinocchioRobotModel::getRobotModelXml(const std::string& param_name, ros::NodeHandle nh)
{
  // This function should retrieve the robot model XML string from the parameter server
  std::string robot_model_string = "";
  if (!nh.getParam(param_name, robot_model_string))
    ROS_ERROR("Failed to get robot model XML from parameter server");
  return robot_model_string;
}
