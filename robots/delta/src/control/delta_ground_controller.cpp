#include <delta/control/delta_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

void RollingController::calcContactPoint()
{
  /* get realtime cog state */
  tf::Vector3 w_p_cog_in_w_tf = estimator_->getPos(Frame::COG, estimate_mode_);
  Eigen::Vector3d w_p_cog_in_w = Eigen::Vector3d(w_p_cog_in_w_tf.x(), w_p_cog_in_w_tf.y(), w_p_cog_in_w_tf.z());

  tf::Quaternion cog2baselink_rot;
  tf::quaternionKDLToTF(robot_model_->getCogDesireOrientation<KDL::Rotation>(), cog2baselink_rot);
  tf::Matrix3x3 w_R_cog_tf = estimator_->getOrientation(Frame::BASELINK, estimate_mode_) * tf::Matrix3x3(cog2baselink_rot).inverse();
  Eigen::Matrix3d w_R_cog;
  for(int i = 0; i < 3; i++)
    {
      for(int j = 0; j < 3; j++)
        {
          if(j == 0) w_R_cog(i, j) = w_R_cog_tf.getRow(i).x();
          if(j == 1) w_R_cog(i, j) = w_R_cog_tf.getRow(i).y();
          if(j == 2) w_R_cog(i, j) = w_R_cog_tf.getRow(i).z();
        }
    }

  Eigen::Vector3d b1 = Eigen::Vector3d(1.0, 0.0, 0.0);
  Eigen::Vector3d b3 = Eigen::Vector3d(0.0, 0.0, 1.0);
  Eigen::Matrix3d rot_mat;
  std::vector<KDL::Frame> links_center_frame_from_cog = rolling_robot_model_->getLinksCenterFrameFromCog();
  double min_z_in_w = 100000;
  int min_index_i, min_index_j;

  /* serach lowest point in world frame */
  for(int i = 0; i < motor_num_; i++)
    {
      KDL::Frame link_i_center_frame_from_cog = links_center_frame_from_cog.at(i);
      Eigen::Vector3d cog_p_center_in_cog = aerial_robot_model::kdlToEigen(link_i_center_frame_from_cog.p);
      Eigen::Matrix3d cog_R_center = aerial_robot_model::kdlToEigen(link_i_center_frame_from_cog.M);
      Eigen::Vector3d w_p_center_in_w = w_p_cog_in_w + w_R_cog * cog_p_center_in_cog;
      for(int j = 30; j <= 150; j++)
        {
          rot_mat = Eigen::AngleAxisd(j / 180.0 * M_PI, b3);
          Eigen::Vector3d center_p_cp_in_center = circle_radius_ * rot_mat * b1;
          Eigen::Vector3d center_p_cp_in_w = w_R_cog * cog_R_center * center_p_cp_in_center;
          Eigen::Vector3d w_p_cp_in_w = w_p_center_in_w + center_p_cp_in_w;
          if(w_p_cp_in_w(2) < min_z_in_w)
            {
              min_z_in_w = w_p_cp_in_w(2);
              min_index_i = i;
              min_index_j = j;
            }
        }
    }

  /* set real contact point to robot model */
  rot_mat = Eigen::AngleAxisd(min_index_j / 180.0 * M_PI, b3);
  Eigen::Vector3d center_p_cp_in_center = circle_radius_ * rot_mat * b1;
  Eigen::Vector3d cog_p_cp_in_cog = aerial_robot_model::kdlToEigen(links_center_frame_from_cog.at(min_index_i).p) + aerial_robot_model::kdlToEigen(links_center_frame_from_cog.at(min_index_i).M) * center_p_cp_in_center;

  KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
  KDL::Frame contact_point_real_in_cog;
  contact_point_real_in_cog.p.x(cog_p_cp_in_cog(0));
  contact_point_real_in_cog.p.y(cog_p_cp_in_cog(1));
  contact_point_real_in_cog.p.z(cog_p_cp_in_cog(2));
  contact_point_real_in_cog.p = cog * contact_point_real_in_cog.p;
  contact_point_real_in_cog.M = cog.M;
  rolling_robot_model_->setContactPointReal(contact_point_real_in_cog);
}


void RollingController::standingPlanning()
{
  if(!start_rp_integration_)
    {
      start_rp_integration_ = true;
      spinal::FlightConfigCmd flight_config_cmd;
      flight_config_cmd.cmd = spinal::FlightConfigCmd::INTEGRATION_CONTROL_ON_CMD;
      navigator_->getFlightConfigPublisher().publish(flight_config_cmd);
      ROS_WARN_ONCE("start roll/pitch I control");
    }
}

void RollingController::calcStandingFullLambda()
{
  int n_variables = 2 * motor_num_;
  int n_constraints = 3 + 1 + 2 + 2;

  int standing_mode_n_constraints = n_constraints;
  if(ground_navigation_mode_ == aerial_robot_navigation::ROLLING_STATE)
    {
      n_constraints += 3;
    }

  Eigen::MatrixXd H(n_variables, n_variables);
  H <<
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  Eigen::SparseMatrix<double> H_s;
  H_s = H.sparseView();

  /* normal pid result */
  Eigen::VectorXd target_wrench_target_frame;
  target_wrench_target_frame.resize(6);
  target_wrench_target_frame.tail(3) = rolling_robot_model_->getInertiaFromTargetFrame<Eigen::Matrix3d>() * Eigen::Vector3d(pid_controllers_.at(ROLL).result(),
                                                                                                                            pid_controllers_.at(PITCH).result(),
                                                                                                                            pid_controllers_.at(YAW).result());

  /* calculate gravity compensation term based on realtime orientation */
  KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
  KDL::Frame contact_point_alined_to_cog = contact_point_alined_.Inverse() * cog;
  Eigen::Vector3d contact_point_alined_to_cog_p = aerial_robot_model::kdlToEigen(contact_point_alined_to_cog.p);
  Eigen::Matrix3d contact_point_alined_to_cog_p_skew = aerial_robot_model::skew(contact_point_alined_to_cog_p);
  Eigen::VectorXd gravity = robot_model_->getGravity3d();
  Eigen::Vector3d gravity_moment_from_contact_point_alined = contact_point_alined_to_cog_p_skew * robot_model_->getMass() * gravity;

  /* use sum of pid result and gravity compensation torque for attitude control */
  Eigen::Vector3d omega;
  tf::vectorTFToEigen(omega_, omega);

  target_wrench_target_frame.tail(3)
    = target_wrench_target_frame.tail(3)
    + gravity_compensate_ratio_ * gravity_moment_from_contact_point_alined
    + omega.cross(rolling_robot_model_->getInertiaFromTargetFrame<Eigen::Matrix3d>() * omega);

  gravity_compensate_term_ = gravity_compensate_ratio_ * gravity_moment_from_contact_point_alined;

  Eigen::MatrixXd full_q_mat = rolling_robot_model_->getFullWrenchAllocationMatrixFromControlFrame();
  Eigen::MatrixXd full_q_mat_trans = full_q_mat.topRows(3);
  Eigen::MatrixXd full_q_mat_rot = full_q_mat.bottomRows(3);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_constraints, n_variables);
  A.topRows(3) = full_q_mat_rot;                                                           //    eq constraint about rpy torque
  A.block(3, 0, 1, n_variables) = full_q_mat_trans.row(Z);                                           // in eq constraint about z
  A.block(4, 0, 1, n_variables) = full_q_mat_trans.row(X) - steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about x
  A.block(5, 0, 1, n_variables) = full_q_mat_trans.row(X) + steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about x
  A.block(6, 0, 1, n_variables) = full_q_mat_trans.row(Y) - steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about y
  A.block(7, 0, 1, n_variables) = full_q_mat_trans.row(Y) + steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about y

  if(ground_navigation_mode_ == aerial_robot_navigation::ROLLING_STATE)
    {
      A.block(8, 0, 1, n_variables)  << 1, 0, 0, 0, 0, 0;
      A.block(9, 0, 1, n_variables)  << 0, 0, 1, 0, 0, 0;
      A.block(10, 0, 1, n_variables) << 0, 0, 0, 0, 1, 0;
    }

  Eigen::SparseMatrix<double> A_s;
  A_s = A.sparseView();
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(n_variables);
  gradient = gradient_weight_ * full_lambda_all_;

  Eigen::VectorXd lower_bound(n_constraints);
  Eigen::VectorXd upper_bound(n_constraints);

  lower_bound.head(standing_mode_n_constraints)
    <<
    target_wrench_target_frame(ROLL),
    target_wrench_target_frame(PITCH),
    target_wrench_target_frame(YAW),
    -INFINITY,
    -steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z),
    -INFINITY,
    -steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z),
    -INFINITY;

  upper_bound.head(standing_mode_n_constraints)
    <<
    target_wrench_target_frame(ROLL),
    target_wrench_target_frame(PITCH),
    target_wrench_target_frame(YAW),
    robot_model_->getMass() * robot_model_->getGravity()(Z),
    INFINITY,
    steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z),
    INFINITY,
    steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z);

  if(ground_navigation_mode_ == aerial_robot_navigation::ROLLING_STATE)
    {
      for(int i = 0; i < motor_num_; i++)
        {
          lower_bound(standing_mode_n_constraints + i) = -robot_model_->getThrustUpperLimit();
          upper_bound(standing_mode_n_constraints + i) = -fabs(rolling_minimum_lateral_force_);
        }
    }

  OsqpEigen::Solver solver;

  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);
  solver.data()->setNumberOfVariables(n_variables);
  solver.data()->setNumberOfConstraints(n_constraints);
  solver.data()->setHessianMatrix(H_s);
  solver.data()->setLinearConstraintsMatrix(A_s);
  solver.data()->setGradient(gradient);
  solver.data()->setLowerBound(lower_bound);
  solver.data()->setUpperBound(upper_bound);

  if(!solver.initSolver())
    {
      ROS_ERROR("[control][OSQP] init solver error!");
    }

  if(!solver.solve())
    {
      ROS_WARN_STREAM("[control][OSQP] could not reach the solution.");
    }
  else
    {
      auto solution = solver.getSolution();

      full_lambda_all_ = solution;
      full_lambda_trans_ = solution;
    }
}

void RollingController::nonlinearQP()
{
  // apply real joint angle
  casadi::SX q_cs = pinocchio_robot_model_->getQCs();
  const auto joint_state  = robot_model_->kdlJointToMsg(robot_model_->getJointPositions());
  std::map<std::string, int> joint_index_map = pinocchio_robot_model_->getJointIndexMap();

  std::vector<casadi::SX> rotors_normal_from_root = pinocchio_robot_model_->getRotorsNormalFromRoot();
  std::vector<casadi::SX> rotors_origin_from_root = pinocchio_robot_model_->getRotorsOriginFromRoot();

  // calculate rotor info from contact point
  KDL::Frame root_T_cp = rolling_robot_model_->getContactPoint<KDL::Frame>();
  casadi::SX root_R_cp;
  pinocchio::casadi::copy(aerial_robot_model::kdlToEigen(root_T_cp.M), root_R_cp);
  casadi::SX root_p_cp_in_root;
  pinocchio::casadi::copy(aerial_robot_model::kdlToEigen(root_T_cp.p), root_p_cp_in_root);

  std::vector<casadi::SX> rotors_normal_from_cp, rotors_origin_from_cp;
  rotors_normal_from_cp.resize(motor_num_);
  rotors_origin_from_cp.resize(motor_num_);

  for(int i = 0; i < motor_num_; i++)
    {
      rotors_normal_from_cp.at(i) = mtimes(root_R_cp.T(), rotors_normal_from_root.at(i));
      rotors_origin_from_cp.at(i) = - mtimes(root_R_cp.T(), root_p_cp_in_root) + mtimes(root_R_cp.T(), rotors_origin_from_root.at(i));
    }

  // make wrench allocation Matrix
  const auto& sigma = robot_model_->getRotorDirection();
  const double m_f_rate = robot_model_->getMFRate();
  casadi::SX wrench_allocation_mat_from_cp = casadi::SX(6, motor_num_);
  for(int i = 0; i < motor_num_; i++)
    {
      casadi::SX v_i = cross(rotors_origin_from_cp.at(i), rotors_normal_from_cp.at(i)) + sigma.at(i + 1) * m_f_rate * rotors_normal_from_cp.at(i);
      for(int j = 0; j < 3; j++)
        {
          wrench_allocation_mat_from_cp(j, i) = rotors_normal_from_cp.at(i)(j);
          wrench_allocation_mat_from_cp(j + 3, i) = v_i(j);
        }
    }

  // calculate wrench
  casadi::SX lambda = casadi::SX::sym("lambda", motor_num_);
  casadi::SX wrench_cp = mtimes(wrench_allocation_mat_from_cp, lambda);

  // target torque
  Eigen::VectorXd target_wrench_target_frame;
  target_wrench_target_frame.resize(6);
  target_wrench_target_frame.tail(3) = rolling_robot_model_->getInertiaFromTargetFrame<Eigen::Matrix3d>() * Eigen::Vector3d(pid_controllers_.at(ROLL).result(),
                                                                                                                            pid_controllers_.at(PITCH).result(),
                                                                                                                            pid_controllers_.at(YAW).result());

  /* calculate gravity compensation term based on realtime orientation */
  KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
  KDL::Frame contact_point_alined_to_cog = contact_point_alined_.Inverse() * cog;
  Eigen::Vector3d contact_point_alined_to_cog_p = aerial_robot_model::kdlToEigen(contact_point_alined_to_cog.p);
  Eigen::Matrix3d contact_point_alined_to_cog_p_skew = aerial_robot_model::skew(contact_point_alined_to_cog_p);
  Eigen::VectorXd gravity = robot_model_->getGravity3d();
  Eigen::Vector3d gravity_moment_from_contact_point_alined = contact_point_alined_to_cog_p_skew * robot_model_->getMass() * gravity;

  /* use sum of pid result and gravity compensation torque for attitude control */
  target_wrench_target_frame.tail(3) = target_wrench_target_frame.tail(3) + gravity_compensate_ratio_ * gravity_moment_from_contact_point_alined;
  gravity_compensate_term_ = gravity_compensate_ratio_ * gravity_moment_from_contact_point_alined;

  // optimization problem
  int n_variables = 8;
  int n_constraints = 8;

  casadi::SX x_opt = casadi::SX(n_variables, 1);
  x_opt(0) = q_cs(joint_index_map.at("gimbal1"));
  x_opt(1) = q_cs(joint_index_map.at("gimbal2"));
  x_opt(2) = q_cs(joint_index_map.at("gimbal3"));
  x_opt(3) = lambda(0);
  x_opt(4) = lambda(1);
  x_opt(5) = lambda(2);
  x_opt(6) = q_cs(joint_index_map.at("joint1"));
  x_opt(7) = q_cs(joint_index_map.at("joint2"));

  casadi::SX cost
    = lambda_weight_ * dot(lambda, lambda)
    + d_gimbal_center_weight_ * (pow(x_opt(0) - M_PI / 2.0, 2) + pow(x_opt(1) - M_PI / 2.0, 2) + 2.0 * pow(x_opt(2) - M_PI / 2.0, 2))
    + d_gimbal_weight_ * (pow(x_opt(0) - prev_opt_gimbal_.at(0), 2) + pow(x_opt(1) - prev_opt_gimbal_.at(1), 2) + pow(x_opt(2) - prev_opt_gimbal_.at(2), 2));

  casadi::SX constraints = casadi::SX(n_constraints, 1);
  constraints(0) = wrench_cp(ROLL);
  constraints(1) = wrench_cp(PITCH);
  constraints(2) = wrench_cp(YAW);
  constraints(3) = wrench_cp(Z);
  constraints(4) = wrench_cp(X) - steering_mu_ * wrench_cp(Z);  // lower
  constraints(5) = wrench_cp(X) + steering_mu_ * wrench_cp(Z);  // upper
  constraints(6) = wrench_cp(Y) - steering_mu_ * wrench_cp(Z);  // lower
  constraints(7) = wrench_cp(Y) + steering_mu_ * wrench_cp(Z);  // upper

  casadi::DM lbg = casadi::DM(n_constraints, 1);
  casadi::DM ubg = casadi::DM(n_constraints, 1);
  casadi::DM lbx = casadi::DM(n_variables, 1);
  casadi::DM ubx = casadi::DM(n_variables, 1);

  lbg(0) = target_wrench_target_frame(ROLL);
  lbg(1) = target_wrench_target_frame(PITCH);
  lbg(2) = target_wrench_target_frame(YAW);
  lbg(3) = -INFINITY;
  lbg(4) = -steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z);
  lbg(5) = -INFINITY;
  lbg(6) = -steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z);
  lbg(7) = -INFINITY;

  ubg(0) = target_wrench_target_frame(ROLL);
  ubg(1) = target_wrench_target_frame(PITCH);
  ubg(2) = target_wrench_target_frame(YAW);
  ubg(3) = robot_model_->getMass() * robot_model_->getGravity()(Z);
  ubg(4) = INFINITY;
  ubg(5) = steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z);
  ubg(6) = INFINITY;
  ubg(7) = steering_mu_ * robot_model_->getMass() * robot_model_->getGravity()(Z);

  if(ground_navigation_mode_ == aerial_robot_navigation::ROLLING_STATE)
    {
      lbx(0) = min(max(0.0, prev_opt_gimbal_.at(0) - gimbal_d_theta_max_), M_PI);
      lbx(1) = min(max(0.0, prev_opt_gimbal_.at(1) - gimbal_d_theta_max_), M_PI);
      lbx(2) = min(max(0.0, prev_opt_gimbal_.at(2) - gimbal_d_theta_max_), M_PI);
    }
  else
    {
      lbx(0) = max(-M_PI, prev_opt_gimbal_.at(0) - gimbal_d_theta_max_);
      lbx(1) = max(-M_PI, prev_opt_gimbal_.at(1) - gimbal_d_theta_max_);
      lbx(2) = max(-M_PI, prev_opt_gimbal_.at(2) - gimbal_d_theta_max_);
    }
  lbx(3) = max(0.0, prev_opt_lambda_.at(0) - d_lambda_max_);
  lbx(4) = max(0.0, prev_opt_lambda_.at(1) - d_lambda_max_);
  lbx(5) = max(0.0, prev_opt_lambda_.at(2) - d_lambda_max_);

  if(ground_navigation_mode_ == aerial_robot_navigation::ROLLING_STATE)
    {
      ubx(0) = max(0.0, min(prev_opt_gimbal_.at(0) + gimbal_d_theta_max_, M_PI));
      ubx(1) = max(0.0, min(prev_opt_gimbal_.at(1) + gimbal_d_theta_max_, M_PI));
      ubx(2) = max(0.0, min(prev_opt_gimbal_.at(2) + gimbal_d_theta_max_, M_PI));
    }
  else
    {
      ubx(0) = min(prev_opt_gimbal_.at(0) + gimbal_d_theta_max_, M_PI);
      ubx(1) = min(prev_opt_gimbal_.at(1) + gimbal_d_theta_max_, M_PI);
      ubx(2) = min(prev_opt_gimbal_.at(2) + gimbal_d_theta_max_, M_PI);
    }
  ubx(3) = min(prev_opt_lambda_.at(0) + d_lambda_max_, robot_model_->getThrustUpperLimit());
  ubx(4) = min(prev_opt_lambda_.at(1) + d_lambda_max_, robot_model_->getThrustUpperLimit());
  ubx(5) = min(prev_opt_lambda_.at(2) + d_lambda_max_, robot_model_->getThrustUpperLimit());

  lbx(6) = current_joint_angles_.at(0);
  lbx(7) = current_joint_angles_.at(1);
  ubx(6) = lbx(6);
  ubx(7) = lbx(7);

  ROS_INFO_STREAM_ONCE("x_opt: \n" << x_opt << "\n");
  ROS_INFO_STREAM_ONCE("cost: \n" << cost << "\n");
  ROS_INFO_STREAM_ONCE("constraints: \n" << constraints << "\n");
  ROS_INFO_STREAM_ONCE("lbg: \n" << lbg << "\n");
  ROS_INFO_STREAM_ONCE("ubg: \n" << ubg << "\n");
  ROS_INFO_STREAM_ONCE("lbx: \n" << lbx << "\n");
  ROS_INFO_STREAM_ONCE("ubx: \n" << ubx << "\n");

  casadi::SXDict nlp = { {"x", x_opt}, {"f", cost}, {"g", constraints} };

  casadi::Dict opt_dict = casadi::Dict();
  opt_dict["ipopt.max_iter"] = ipopt_max_iter_;
  opt_dict["ipopt.print_level"] = 0;
  opt_dict["ipopt.sb"] = "yes";
  opt_dict["ipopt.linear_solver"] = "ma97";
  opt_dict["print_time"] = 0;

  casadi::Function S = casadi::nlpsol("S", "ipopt", nlp, opt_dict);
  casadi::DM initial_x = casadi::DM(n_variables, 1);
  initial_x(0) = prev_opt_gimbal_.at(0);
  initial_x(1) = prev_opt_gimbal_.at(1);
  initial_x(2) = prev_opt_gimbal_.at(2);
  initial_x(3) = prev_opt_lambda_.at(0);
  initial_x(4) = prev_opt_lambda_.at(1);
  initial_x(5) = prev_opt_lambda_.at(2);
  initial_x(6) = current_joint_angles_.at(0);
  initial_x(7) = current_joint_angles_.at(1);

  auto res = S(casadi::DMDict{ {"x0", initial_x}, {"lbg",lbg}, {"ubg", ubg}, {"lbx", lbx}, {"ubx", ubx} });

  prev_opt_gimbal_.at(0) = (double)res.at("x")(0);
  prev_opt_gimbal_.at(1) = (double)res.at("x")(1);
  prev_opt_gimbal_.at(2) = (double)res.at("x")(2);
  prev_opt_lambda_.at(0) = (double)res.at("x")(3);
  prev_opt_lambda_.at(1) = (double)res.at("x")(4);
  prev_opt_lambda_.at(2) = (double)res.at("x")(5);

}
