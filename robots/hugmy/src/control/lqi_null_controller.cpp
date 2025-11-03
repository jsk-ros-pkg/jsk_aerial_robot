#include <hugmy/control/lqi_null_controller.h>

using namespace aerial_robot_control;

LQINullController::LQINullController() : UnderActuatedLQIController()
{
}

void LQINullController::initialize(ros::NodeHandle nh,
                                     ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                     boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                     double ctrl_loop_rate)
{
  UnderActuatedLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  rosParamInitSoft();

  f_ref_   = Eigen::VectorXd::Zero(motor_num_);
  Wf_diag_ = Eigen::VectorXd::Zero(motor_num_);
  f_max_   = Eigen::VectorXd::Constant(motor_num_, 1e6);

  nullspace_dim_pub_ = nh_.advertise<std_msgs::Int32>("arm/debug/nullspace_dim", 1);
  svd_singvals_pub_  = nh_.advertise<std_msgs::Float32MultiArray>("arm/debug/Q_singular_values", 1);
  thrust_pub_        = nh_.advertise<std_msgs::Float32MultiArray>("arm/debug/target_thrust", 1);
}

void LQINullController::rosParamInitSoft()
{
  ros::NodeHandle nhs(nh_, "soft_fz");
  nhs.param("w_fz",        w_fz_,        2.0);
  nhs.param("eps_reg",     eps_reg_,     1e-6);
  nhs.param("az_des_bias", az_des_bias_, 0.0);

}

Eigen::MatrixXd LQINullController::buildQ()
{
  const auto p   = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto u   = robot_model_->getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& dir = robot_model_->getRotorDirection();
  const double mfr = robot_model_->getMFRate();

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, motor_num_);

  for (unsigned int i = 0; i < motor_num_; ++i)
  {
    Q(0, i) = u[i].z();
    Q.block<3,1>(1, i) = p[i].cross(u[i]) + mfr * dir.at(i + 1) * u[i];
  }

  const double mass_inv = 1.0 / robot_model_->getMass();
  const Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
  Q.topRows(1)    = mass_inv    * Q.topRows(1);
  Q.bottomRows(3) = inertia_inv * Q.bottomRows(3);

  last_Q_ = Q;
  return Q;
}

void LQINullController::zeroDragTorque(std::vector<float>& thrust)
{
  const double mfr = robot_model_->getMFRate();
  const auto& dir  = robot_model_->getRotorDirection();

  double sum = 0.0, sum_abs = 0.0;
  for (int i = 0; i < motor_num_; ++i)
  {
    const double w = dir.at(i + 1);
    sum     += w * thrust[i];
    sum_abs += std::abs(w);
  }
  if (std::abs(sum) < 1e-6) return;

  const double alpha = 0.3;
  const double corr  = alpha * (sum / sum_abs);
  for (int i = 0; i < motor_num_; ++i)
  {
    const double w = dir.at(i + 1);
    thrust[i] -= corr * w;
    if (thrust[i] < 0.0) thrust[i] = 0.0;
  }
}

Eigen::VectorXd LQINullController::allocSoftFz(
    const Eigen::MatrixXd& Qeq,
    const Eigen::VectorXd& beq,
    const Eigen::RowVectorXd& qz,
    double az_des,
    const Eigen::VectorXd& f_ref,
    const Eigen::VectorXd& Wf_diag,
    const Eigen::VectorXd& f_max,
    double w_fz,
    double eps_reg)
{

  const Eigen::MatrixXd QeqQeqT = Qeq * Qeq.transpose();
  Eigen::VectorXd f_p = Qeq.transpose() * QeqQeqT.ldlt().solve(beq);

  const Eigen::MatrixXd Nns = nullspace(Qeq);
  if (Nns.cols() == 0) {
    Eigen::VectorXd f = f_p;
    Eigen::VectorXd f_clip = f;
    clip(f_clip, f_max);
    const Eigen::VectorXd df = Qeq.transpose() * QeqQeqT.ldlt().solve(beq - Qeq * f_clip);
    f = f_clip + df;
    clip(f, f_max);
    return f;
  }

  Eigen::VectorXd qp = qz.transpose();
  double r  = qp.dot(f_p) - az_des;
  Eigen::MatrixXd W  = Wf_diag.asDiagonal();

  Eigen::MatrixXd A  = w_fz * (Nns.transpose() * qz.transpose() * qz * Nns)
                     + (Nns.transpose() * W * Nns)
                     + eps_reg * (Nns.transpose() * Nns);

  Eigen::VectorXd g  = w_fz * Nns.transpose() * qz.transpose() * r
                     + Nns.transpose() * W * (f_p - f_ref)
                     + eps_reg * (Nns.transpose() * f_p);

  // A y = -g
  Eigen::VectorXd y = - A.ldlt().solve(g);

  Eigen::VectorXd f = f_p + Nns * y;

  // クリップ
  Eigen::VectorXd f_clip = f;
  clip(f_clip, f_max);

  const Eigen::VectorXd df = Qeq.transpose() * QeqQeqT.ldlt().solve(beq - Qeq * f_clip);
  f = f_clip + df;
  clip(f, f_max);

  return f;
}

void LQINullController::controlCore()
{

  PoseLinearController::controlCore();

  const tf::Vector3 a_w(pid_controllers_.at(X).result(),
                        pid_controllers_.at(Y).result(),
                        pid_controllers_.at(Z).result());

  tf::Vector3 a_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * a_w;
  target_pitch_ =  a_dash.x() / aerial_robot_estimation::G;
  target_roll_  = -a_dash.y() / aerial_robot_estimation::G;

  Eigen::MatrixXd Q = buildQ();
  const Eigen::RowVectorXd qz = Q.row(0);          // Fz
  // const Eigen::MatrixXd Qeq   = Q.bottomRows(3);   // Tx,Ty,Tz = 0
  // const Eigen::Vector3d beq   = Eigen::Vector3d::Zero();
  const Eigen::MatrixXd Qeq = Q.middleRows(1, 2);
  const Eigen::Vector2d beq   = Eigen::Vector2d::Zero();
  

  // const double mass = robot_model_->getMass();
  const double g    = robot_model_->getGravity3d().z();
  // double sum_qz = qz.sum();
  // double sum_u_z = sum_qz * mass;
  // if (std::abs(sum_u_z) < 1e-6) sum_u_z = 1e-6;
  // const double f_hover_per_motor = (mass * g) / (std::max(1e-6, sum_u_z) * motor_num_);
  // const double f_max_each = 10.0 * f_hover_per_motor;

  // if (f_max_.size() != motor_num_) f_max_.resize(motor_num_);
  // for (int i=0;i<motor_num_;++i) f_max_(i) = f_max_each;


  Eigen::MatrixXd q_inv = aerial_robot_model::pseudoinverse(Q);

  Eigen::Vector4d b_hover; 
  b_hover << robot_model_->getGravity3d().z(), 0.0, 0.0, 0.0;
  Eigen::VectorXd f_hover = q_inv * b_hover;
  f_ref_ = f_hover;
  double sat_ratio = 2.5;  // rosparam にしてOK
  double f_max_each = sat_ratio * f_hover.cwiseAbs().maxCoeff();
  f_max_.setConstant(motor_num_, f_max_each);

  ROS_INFO_STREAM_THROTTLE(0.5, "[SoftFzLQI] hover_cmd_avg=" 
      << f_hover.mean() << " max=" << f_hover.cwiseAbs().maxCoeff()
      << " f_max_each=" << f_max_each);
      
  // // f_ref_ = Eigen::VectorXd::Constant(motor_num_, f_hover_per_motor);
  // // Wf_diag_.setConstant(motor_num_, 0.0);

  // // const Eigen::MatrixXd q_inv = aerial_robot_model::pseudoinverse(Q);
  // double ff_acc_z = navigator_->getTargetAcc().z();
  // // ff_acc_z += g;
  // // Eigen::VectorXd f_ff = q_inv.col(0) * ff_acc_z;

  // // double az_des = navigator_->getTargetAcc().z();      // FF
  // // if (compensate_gravity_) az_des += robot_model_->getGravity3d().z();
  // // az_des += pid_controllers_.at(Z).result();
  // const double az_des = - a_w.z();


  // // const double az_limit = 8.0 * g;
  // // az_des = std::max(-az_limit, std::min(az_limit, az_des));


  // // double az_des = navigator_->getTargetAcc().z() + g + pid_controllers_.at(Z).result();
  // // const double az_max = (qz * f_max_)(0);
  // // const double az_min = 0.0;
  // // az_des = std::min(std::max(az_des, az_min), az_max);

  // Eigen::VectorXd f = allocSoftFz(Qeq, beq, qz, az_des,
  //                                 f_ref_, Wf_diag_, f_max_,
  //                                 w_fz_, eps_reg_);


  Eigen::VectorXd thrust_z_pid = Eigen::VectorXd::Zero(motor_num_);
  for (int i = 0; i < motor_num_; ++i) {
    const double p = z_gains_[i][0] * pid_controllers_.at(Z).getErrP();
    const double I= z_gains_[i][1] * pid_controllers_.at(Z).getErrI();
    const double d = z_gains_[i][2] * pid_controllers_.at(Z).getErrD();
    thrust_z_pid(i) = p + I + d;
  }

  // Eigen::MatrixXd q_inv = aerial_robot_model::pseudoinverse(Q);
  double ff_acc_z = navigator_->getTargetAcc().z();
  if (compensate_gravity_) ff_acc_z += robot_model_->getGravity3d().z();
  Eigen::VectorXd thrust_z_ff = q_inv.col(0) * ff_acc_z;


  Eigen::VectorXd thrust_z_total = thrust_z_pid + thrust_z_ff;
  int idx; double max_term = thrust_z_total.cwiseAbs().maxCoeff(&idx);
  double limit = pid_controllers_.at(Z).getLimitSum();
  double residual = max_term - limit;
  if (residual > 0) {
    pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
    thrust_z_total *= (1.0 - residual / max_term);
  }

  double az_des = navigator_->getTargetAcc().z();
  if (compensate_gravity_) az_des += std::abs(robot_model_->getGravity3d().z());
  az_des += pid_controllers_.at(Z).result();
  az_des = std::clamp(az_des, -2.0*g, 2.0*g);

  Eigen::VectorXd Wf_diag = Eigen::VectorXd::Zero(motor_num_);

  ROS_ERROR_STREAM_THROTTLE(0.5, "ff_acc_z" << ff_acc_z);
  ROS_ERROR_STREAM_THROTTLE(0.1, "az_des" << az_des);
  ROS_ERROR_STREAM_THROTTLE(0.5, "eps_reg" << eps_reg_);
  ROS_ERROR_STREAM_THROTTLE(0.5, "w_fz" << w_fz_);
  ROS_ERROR_STREAM_THROTTLE(0.5, "f_ref" << f_ref_);


  Eigen::VectorXd f = allocSoftFz(Qeq, beq, qz, az_des, f_ref_, Wf_diag, f_max_, w_fz_, eps_reg_);

  bool saturated = ( (f.array() >= f_max_.array() - 1e-6).any() || (f.array() <= 1e-6).any() );
  if (saturated) {
    pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
  }

  Eigen::VectorXd f_dbg = f;
  const double f_max_dbg = f_max_(0);
  ROS_INFO_STREAM_THROTTLE(0.5, "[SoftFzLQI] f=" << f_dbg.minCoeff()
                           << " f_max=" << f_dbg.maxCoeff()
                           << " f_max_each=" << f_max_dbg);

  for (int i = 0; i < motor_num_; ++i) target_base_thrust_.at(i) = f(i);
  zeroDragTorque(target_base_thrust_);

  allocateYawTerm();

  const Eigen::VectorXd wr = Q * f; // [Fz, αx, αy, αz]
  ROS_INFO_STREAM_THROTTLE(0.5, "[SoftFzLQI] az_des=" << az_des
                           << "  Fz_pred=" << wr(0));

  {
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(last_Q_, Eigen::ComputeThinV);
      const auto S = svd.singularValues();
      const double eps = std::numeric_limits<double>::epsilon();
      const double tol = std::max(last_Q_.rows(), last_Q_.cols()) * eps * S(0);
      const int rank = (S.array() > tol).count();
      const int nullity = last_Q_.cols() - rank;

      std_msgs::Int32 dim; dim.data = nullity; nullspace_dim_pub_.publish(dim);

      std_msgs::Float32MultiArray m; m.data.reserve(S.size());
      for (int i=0;i<S.size();++i) m.data.push_back(static_cast<float>(S(i)));
      svd_singvals_pub_.publish(m);
  }

}

void LQINullController::sendCmd()
{
  // base_thrust とゲイン送信は親の実装を活用
  UnderActuatedLQIController::sendCmd();

  // 追加：thrustのデバッグ配信
  std_msgs::Float32MultiArray thrust_msg;
  thrust_msg.data.resize(target_base_thrust_.size());
  for (size_t i = 0; i < target_base_thrust_.size(); ++i)
    thrust_msg.data[i] = static_cast<float>(target_base_thrust_[i]);
  thrust_pub_.publish(thrust_msg);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::LQINullController, aerial_robot_control::ControlBase);

