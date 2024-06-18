#include <delta/control/delta_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

void RollingController::flyingMpc()
{
  double dt = horizon_ / n_step_;

  int n_x = 12;
  int n_u = 6;

  int index_x_p = 0;
  int index_x_v = 3;
  int index_x_a = 6;
  int index_x_w = 9;

  int n_x_p = 3;
  int n_x_v = 3;
  int n_x_a = 3;
  int n_x_w = 3;

  // x_k+1 = x_k + x_k' * dt = x_k +  f(x_k, u_k) * dt = A x_k + B u_k
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_x, n_x);
  A.block(0, 0, 12, 12) = Eigen::MatrixXd::Identity(n_x, n_x);
  A.block(0, 3, 3, 3) = dt * Eigen::MatrixXd::Identity(n_x_v, n_x_v);
  A.block(6, 9, 3, 3) = dt * Eigen::MatrixXd::Identity(n_x_w, n_x_w);

  Eigen::MatrixXd full_q_mat = rolling_robot_model_->getFullWrenchAllocationMatrixFromControlFrame("cog");
  double mass = robot_model_->getMass();
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();

  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n_x, n_u);
  B.block(3, 0, 3, n_u) = 1.0 / mass * full_q_mat.topRows(3);
  B.block(9, 0, 3, n_u) = inertia.inverse() * full_q_mat.bottomRows(3);
  B = dt * B;

  Eigen::VectorXd c = Eigen::VectorXd::Zero(n_x);
  c(index_x_v + 2) = - dt * robot_model_->getGravity()(2);
  // ROS_INFO_STREAM_ONCE("\n" << A);
  // ROS_INFO_STREAM_ONCE("\n" << B);

  // X = [x_0, ... , x_n] = F * x_0 + G * U + H * C
  // U = [u_0, ... , u_n-1]

  Eigen::MatrixXd F = Eigen::MatrixXd::Zero(n_x * (n_step_ + 1), n_x);
  F.block(0, 0, n_x, n_x) = Eigen::MatrixXd::Identity(n_x, n_x);
  for(int i = 1; i < n_step_ + 1; i++)
    {
      F.block(i * n_x, 0, n_x, n_x) = F.block((i - 1) * n_x, 0, n_x, n_x) * A;
    }
  // ROS_INFO_STREAM_ONCE("\n" << F);

  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(n_x * (n_step_ + 1), n_u * n_step_);
  G.block(n_x, 0, n_x, n_u) = B;
  for(int i = 2; i < n_step_ + 1; i++)
    {
      G.block(i * n_x, 0, n_x, n_u) = A * G.block((i - 1) * n_x, 0, n_x, n_u);
    }
  for(int i = 2; i < n_step_ + 1; i++)
    {
      G.block(i * n_x, (i - 1) * n_u, (n_step_ + 1 - i) * n_x, n_u) = G.block((i - 1) * n_x, (i - 2) * n_u, (n_step_ + 1 - i) * n_x, n_u);
    }
  // ROS_INFO_STREAM_ONCE("\n" << G);

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_x * (n_step_ + 1), n_x * n_step_);
  H.block(n_x, 0, n_x, n_x) = Eigen::MatrixXd::Identity(n_x, n_x);
  for(int i = 2; i < n_step_ + 1; i++)
    {
      H.block(i * n_x, 0, n_x, n_x) = A * H.block((i - 1) * n_x, 0, n_x, n_x);
    }
  for(int i = 2; i < n_step_ + 1; i++)
    {
      H.block(i * n_x, (i - 1) * n_x, (n_step_ + 1 - i) * n_x, n_x) = H.block((i - 1) * n_x, (i - 2) * n_x, (n_step_ + 1 - i) * n_x, n_x);
    }
  // ROS_INFO_STREAM_ONCE("\n" << H);

  Eigen::VectorXd C = Eigen::VectorXd::Zero(n_x * n_step_);
  for(int i = 0; i < n_step_; i++)
    {
      C.segment(i * n_x, n_x) = c;
    }
  // ROS_INFO_STREAM_ONCE("\n" << C);

  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n_x * (n_step_ + 1), n_x * (n_step_ + 1));
  Eigen::MatrixXd Q_step = Eigen::MatrixXd(n_x, n_x);
  Q_step.diagonal() << q_x_p_xy_, q_x_p_xy_, q_x_p_z_, q_x_v_xy_, q_x_v_xy_, q_x_v_z_, q_x_a_xy_, q_x_a_xy_, q_x_a_z_, q_x_w_xy_, q_x_w_xy_, q_x_w_z_;
  for(int i = 0; i < n_step_ + 1; i++)
    {
      Q.block(i * n_x, i * n_x, n_x, n_x) = Q_step;
    }
  Q.block(0, 0, n_x * n_step_, n_x * n_step_) = dt * Q.block(0, 0, n_x * n_step_, n_x * n_step_);

  Eigen::MatrixXd R = r_lambda_ * Eigen::MatrixXd::Identity(n_u * n_step_, n_u * n_step_);

  Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(n_x);
  Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(n_x * (n_step_ + 1));

  tf::Quaternion cog2baselink_rot;
  tf::quaternionKDLToTF(robot_model_->getCogDesireOrientation<KDL::Rotation>(), cog2baselink_rot);
  tf::Matrix3x3 cog_rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_) * tf::Matrix3x3(cog2baselink_rot).inverse();

  tf::Vector3 target_pos = navigator_->getTargetPos();
  tf::Vector3 target_vel = navigator_->getTargetVel();
  tf::Vector3 target_rpy = navigator_->getTargetRPY();
  tf::Matrix3x3 target_rot; target_rot.setRPY(target_rpy.x(), target_rpy.y(), target_rpy.z());
  tf::Vector3 target_omega = navigator_->getTargetOmega(); // w.r.t. target cog frame
  target_omega = cog_rot.inverse() * target_rot * target_omega; // w.r.t. current cog frame

  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Vector3 vel = estimator_->getVel(Frame::COG, estimate_mode_);
  double r, p, y; cog_rot.getRPY(r, p, y);
  tf::Vector3 rpy; rpy.setValue(r, p, y);
  tf::Vector3 omega = estimator_->getAngularVel(Frame::COG, estimate_mode_);

  x_0(0) = pos.x();
  x_0(1) = pos.y();
  x_0(2) = pos.z();
  x_0(3) = vel.x();
  x_0(4) = vel.y();
  x_0(5) = vel.z();
  x_0(6) = rpy.x();
  x_0(7) = rpy.y();
  x_0(8) = rpy.z();
  x_0(9) = omega.x();
  x_0(10) = omega.y();
  x_0(11) = omega.z();

  for(int i = 0; i < n_step_ + 1; i++)
    {
      x_ref(i * n_x + 0) = target_pos.x();
      x_ref(i * n_x + 1) = target_pos.y();
      x_ref(i * n_x + 2) = target_pos.z();
      x_ref(i * n_x + 3) = target_vel.x();
      x_ref(i * n_x + 4) = target_vel.y();
      x_ref(i * n_x + 5) = target_vel.z();
      x_ref(i * n_x + 6) = target_rpy.x();
      x_ref(i * n_x + 7) = target_rpy.y();
      x_ref(i * n_x + 8) = target_rpy.z();
      x_ref(i * n_x + 9) = target_omega.x();
      x_ref(i * n_x + 10) = target_omega.y();
      x_ref(i * n_x + 11) = target_omega.z();
    }


  Eigen::MatrixXd hessian = G.transpose() * Q * G + R;
  Eigen::SparseMatrix<double> hessian_s = hessian.sparseView();

  Eigen::VectorXd gradient = 2.0 * (x_0.transpose() * F.transpose() + C.transpose() * H.transpose() - x_ref.transpose()) * Q * G;

  Eigen::MatrixXd constraint_matrix = Eigen::MatrixXd::Identity(n_u * n_step_, n_u * n_step_);
  Eigen::SparseMatrix<double> constraint_matrix_s = constraint_matrix.sparseView();

  Eigen::VectorXd lower_bound = - robot_model_->getThrustUpperLimit() * Eigen::VectorXd::Ones(n_u * n_step_);
  Eigen::VectorXd upper_bound =   robot_model_->getThrustUpperLimit() * Eigen::VectorXd::Ones(n_u * n_step_);


  if(mpc_first_run_)
    {
      mpc_linear_solver_.settings()->setVerbosity(false);
      mpc_linear_solver_.settings()->setTimeLimit(5e-3);
      mpc_linear_solver_.settings()->setWarmStart(true);
      mpc_linear_solver_.data()->setNumberOfVariables(n_u * n_step_);
      mpc_linear_solver_.data()->setNumberOfConstraints(n_u * n_step_);
      mpc_linear_solver_.data()->setHessianMatrix(hessian_s);
      mpc_linear_solver_.data()->setLinearConstraintsMatrix(constraint_matrix_s);
      mpc_linear_solver_.data()->setGradient(gradient);
      mpc_linear_solver_.data()->setLowerBound(lower_bound);
      mpc_linear_solver_.data()->setUpperBound(upper_bound);

      if(!mpc_linear_solver_.initSolver())
        {
          ROS_ERROR("[control][OSQP] init solver error!");
        }
      else
        ROS_INFO_STREAM("[control][OSQP] init solver success");

      Eigen::VectorXd initial_guess_step = aerial_robot_model::pseudoinverse(full_q_mat) * robot_model_->getMass() * robot_model_->getGravity();
      ROS_INFO_STREAM("[control][OSQP] initial guess: \n" << initial_guess_step.transpose());
      Eigen::VectorXd initial_guess = Eigen::VectorXd::Zero(n_u * n_step_);
      for(int i = 0; i < n_step_; i++)
        {
          initial_guess.segment(i * n_u, n_u) = initial_guess_step;
        }
      mpc_linear_solver_.setPrimalVariable(initial_guess); // should be after initSolver. I don't know why...

      mpc_first_run_ = false;
    }
  else
    {
      mpc_linear_solver_.updateHessianMatrix(hessian_s);
      mpc_linear_solver_.updateLinearConstraintsMatrix(constraint_matrix_s);
      mpc_linear_solver_.updateGradient(gradient);
      mpc_linear_solver_.updateBounds(lower_bound, upper_bound);
    }


  if(!mpc_linear_solver_.solve())
    {
      ROS_WARN_STREAM("[control][OSQP] could not reach the solution.");
    }
  else
    {
      mpc_solution_ = mpc_linear_solver_.getSolution();
      std::cout << mpc_solution_.head(6).transpose() << std::endl;
      std::cout << std::endl;
      full_lambda_all_ = mpc_solution_.head(6);
      full_lambda_trans_ = mpc_solution_.head(6);
    }
}

void RollingController::mpcParamInit()
{
  ros::NodeHandle mpc_nh(nh_, "mpc");
  mpc_reconfigure_server_ = boost::make_shared<dynamic_reconfigure::Server<delta::mpcConfig> >(mpc_nh);
  dynamic_reconf_func_mpc_ = boost::bind(&RollingController::mpcCfgCallback, this, _1, _2);
  mpc_reconfigure_server_->setCallback(dynamic_reconf_func_mpc_);

  getParam<double>(mpc_nh, "horizon", horizon_, 2.0);
  getParam<int>(mpc_nh, "n_step", n_step_, 1.0);
  getParam<double>(mpc_nh, "q_x_p_xy", q_x_p_xy_, 300);
  getParam<double>(mpc_nh, "q_x_p_z", q_x_p_z_, 300);
  getParam<double>(mpc_nh, "q_x_v_xy", q_x_v_xy_, 50);
  getParam<double>(mpc_nh, "q_x_v_z", q_x_v_z_, 50);
  getParam<double>(mpc_nh, "q_x_a_xy", q_x_a_xy_, 100);
  getParam<double>(mpc_nh, "q_x_a_z", q_x_a_z_, 100);
  getParam<double>(mpc_nh, "q_x_w_xy", q_x_w_xy_, 10);
  getParam<double>(mpc_nh, "q_x_w_z", q_x_w_z_, 10);
  getParam<double>(mpc_nh, "r_lambda", r_lambda_, 30);
}

void RollingController::mpcCfgCallback(delta::mpcConfig &config, uint32_t level)
{
  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;
  switch(level)
    {
    case Levels::RECONFIGURE_Q_X_P_XY:
      {
        ROS_INFO_STREAM("[MPC] set weight q_x_p_xy from " << q_x_p_xy_ << " to " << config.q_x_p_xy);
        q_x_p_xy_ = config.q_x_p_xy;
      }
    case Levels::RECONFIGURE_Q_X_P_Z:
      {
        ROS_INFO_STREAM("[MPC] set weight q_x_p_z from " << q_x_p_z_ << " to " << config.q_x_p_z);
        q_x_p_z_ = config.q_x_p_z;
      }
    case Levels::RECONFIGURE_Q_X_V_XY:
      {
        ROS_INFO_STREAM("[MPC] set weight q_x_v_xy from " << q_x_v_xy_ << " to " << config.q_x_v_xy);
        q_x_v_xy_ = config.q_x_v_xy;
      }
    case Levels::RECONFIGURE_Q_X_V_Z:
      {
        ROS_INFO_STREAM("[MPC] set weight q_x_v_z from " << q_x_v_z_ << " to " << config.q_x_v_z);
        q_x_v_z_ = config.q_x_v_z;
      }
    case Levels::RECONFIGURE_Q_X_A_XY:
      {
        ROS_INFO_STREAM("[MPC] set weight q_x_a_xy from " << q_x_a_xy_ << " to " << config.q_x_a_xy);
        q_x_a_xy_ = config.q_x_a_xy;
      }
    case Levels::RECONFIGURE_Q_X_A_Z:
      {
        ROS_INFO_STREAM("[MPC] set weight q_x_a_z from " << q_x_a_z_ << " to " << config.q_x_a_z);
        q_x_a_z_ = config.q_x_a_z;
      }
    case Levels::RECONFIGURE_Q_X_W_XY:
      {
        ROS_INFO_STREAM("[MPC] set weight q_x_w_xy from " << q_x_w_xy_ << " to " << config.q_x_w_xy);
        q_x_w_xy_ = config.q_x_w_xy;
      }
    case Levels::RECONFIGURE_Q_X_W_Z:
      {
        ROS_INFO_STREAM("[MPC] set weight q_x_w_z from " << q_x_w_z_ << " to " << config.q_x_w_z);
        q_x_w_z_ = config.q_x_w_z;
      }
    case Levels::RECONFIGURE_R_LAMBDA:
      {
        ROS_INFO_STREAM("[MPC] set weight r_lambda from " << r_lambda_ << " to " << config.r_lambda);
        r_lambda_ = config.r_lambda;
      }
    }
}

