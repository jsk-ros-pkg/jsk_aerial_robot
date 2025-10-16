#include <hugmy/control/morphing_arm_controller.h>

using namespace aerial_robot_control;

MorphingController::MorphingController(): PoseLinearController()
{
}

void MorphingController::initialize(ros::NodeHandle nh,
                               ros::NodeHandle nhp,
                               boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                               boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                               boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                               double ctrl_loop_rate)
{
    PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
    rosParamInit();

    /* publishers */
    rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
    flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
    torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);

    debug_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("arm/debug/jointstate",1);
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/quadrotor/joints_ctrl",1);
    thrust_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("arm/debug/target_thrust", 1);


    nullspace_dim_pub_ = nh_.advertise<std_msgs::Int32>("arm/debug/nullspace_dim", 1);
    svd_singvals_pub_  = nh_.advertise<std_msgs::Float32MultiArray>("arm/debug/Q_singular_values", 1);

    use_theta_allocation_ = true;
    theta_meas_.assign(motor_num_, 0.0);
    f_ref_.setZero(motor_num_);
    f_max_ = Eigen::VectorXd::Constant(motor_num_, 1e6);
    Wf_diag_ = Eigen::VectorXd::Constant(motor_num_, 0.0); // 0=効かせない, 大きいほど f_ref 優先
    target_base_thrust_.resize(motor_num_,0.0);

    // subscriber
    theta_sub_ = nh_.subscribe<std_msgs::Float32>("/quadrotor/arm/theta", 1, &MorphingController::thetaCallback, this);
    thrust_ref_sub_ = nh_.subscribe<std_msgs::Float32>("/quadrotor/arm/thrust_ref", 1, &MorphingController::thrustRefCallback, this);


    torque_allocation_matrix_inv_pub_stamp_ = 0.0;
    ns_timer_ = nh_.createTimer(ros::Duration(0.2), &MorphingController::nullspaceTimerCb, this);
}

void MorphingController::controlCore()
{
  PoseLinearController::controlCore();

  tf::Vector3 a_w(pid_controllers_.at(X).result(),
                  pid_controllers_.at(Y).result(),
                  pid_controllers_.at(Z).result());

  if (navigator_->getForceLandingFlag())
  {
    target_pitch_ = 0.0;
    target_roll_  = 0.0;
  }

  tf::Vector3 a_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * a_w;
  if (hovering_approximate_)
  {
    target_pitch_ =  a_dash.x() / aerial_robot_estimation::G;
    target_roll_  = -a_dash.y() / aerial_robot_estimation::G;
  }
  else
  {
    target_pitch_ = atan2(a_dash.x(), a_dash.z());
    target_roll_  = atan2(-a_dash.y(), sqrt(a_dash.x() * a_dash.x() + a_dash.z() * a_dash.z()));
  }

  Eigen::MatrixXd Q = buildQWithTheta();
  last_Q_ = Q;

  // // ヌル空間を計算
  // Eigen::MatrixXd N;
  // int rank = 0; double tol = 0.0;
  // Eigen::VectorXd S;
  // computeNullspaceSVD(Q, N, rank, tol, &S);
  
  // const int nullity = N.cols(); // ヌル空間次元
  // ROS_INFO_STREAM_THROTTLE(0.5,
  //                          "[Q SVD] rank=" << rank
  //                          << "  nullity=" << nullity
  //                          << "  tol=" << tol
  //                          << "  S(0..)=" << S.transpose());
  
  // // Publish: 次元
  // {
  //   std_msgs::Int32 msg;
  //   msg.data = nullity;
  //   nullspace_dim_pub_.publish(msg);
  // }

  // // Publish: 特異値（デバッグや安定度の指標に便利）
  // {
  //   std_msgs::Float32MultiArray m;
  //   m.data.resize(S.size());
  //   for (int i = 0; i < S.size(); ++i) m.data[i] = static_cast<float>(S(i));
  //   svd_singvals_pub_.publish(m);
  // }

  // --- ここを変更：Fz は目的へ、等式は Tx,Ty,Tz のみ ---
  const Eigen::RowVectorXd qz = Q.row(0);        // Fz 行
  const Eigen::MatrixXd    Qeq = Q.bottomRows(3); // Tx,Ty,Tz
  const Eigen::Vector3d    beq = Eigen::Vector3d::Zero();
  
  // 目標 Fz （重力+z PID）
  const double Fz_des = a_w.z() + az_des_bias_;  // 必要なら +9.81 でもOK
  
  // ヌル空間目的の重み
  const double w_fz = 1.0;        // Fz を合わせる重み（0.5〜5で調整）
  const double eps_reg = 1e-6;    // 正則化（数値安定用）

  // f_ref（設計推力）に寄せるなら Wf_diag_ をそのまま渡す
  Eigen::VectorXd f = allocSoftFz(Qeq, beq, qz, Fz_des,
                                  f_ref_, Wf_diag_, f_max_,
                                  w_fz, eps_reg);

  // /* 等式制約: 鉛直加速度 + 姿勢維持（角加速度0） */
  // Eigen::Vector4d b; b.setZero();
  // b(0) = a_w.z() + az_des_bias_;

  // /* ヌル空間で f_ref に寄せつつ、非負＆上限を守り、等式制約を再投影 */
  // Eigen::VectorXd f = allocConstrained(Q, b, f_ref_, Wf_diag_, f_max_);

  for (int i = 0; i < motor_num_; ++i) target_base_thrust_.at(i) = f(i);
  zeroDragTorque(target_base_thrust_);

  q_mat_     = Q;
  q_mat_inv_ = aerial_robot_model::pseudoinverse(q_mat_);

  double max_yaw_scale = 0.0;
  for (unsigned int i = 0; i < motor_num_; ++i)
  {
    if (q_mat_inv_(i, YAW - 2) > max_yaw_scale) max_yaw_scale = q_mat_inv_(i, YAW - 2);
  }
  candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;

  navigator_->setTargetPitch(target_pitch_);
  navigator_->setTargetRoll(target_roll_);

  /*debug*/
  // (1) Qでの予測
  Eigen::VectorXd lam(motor_num_);
  for (int i=0;i<motor_num_;++i) lam(i)=target_base_thrust_[i];
  Eigen::VectorXd wr = q_mat_ * lam; // [Fz, Tx, Ty, Tz] の並び
  ROS_INFO_STREAM_THROTTLE(0.5, "pred Fz=" << wr(0));

  // (2) ドラッグ総和
  const double mfr = robot_model_->getMFRate();
  const auto& dir  = robot_model_->getRotorDirection();
  double sum_drag=0;
  for(int i=0;i<motor_num_;++i) sum_drag += dir.at(i + 1) * mfr * target_base_thrust_[i];
  ROS_INFO_STREAM_THROTTLE(0.5, "sum_drag=" << sum_drag);

  // (3) Yawコマンドの大きさ
  ROS_INFO_STREAM_THROTTLE(0.5,
  "yaw_pid=" << pid_controllers_.at(YAW).result()
  << " scale=" << max_yaw_scale
  << " cand=" << candidate_yaw_term_);

}

void MorphingController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "hovering_approximate", hovering_approximate_, false);
  getParam<double>(control_nh, "torque_allocation_matrix_inv_pub_interval",
                   torque_allocation_matrix_inv_pub_interval_, 0.05);
}

// アームの屈曲向き正しいか確認する
static inline Eigen::Vector3d rotateArm(const Eigen::Vector3d& v, double theta)
{
  Eigen::AngleAxisd R(theta, Eigen::Vector3d::UnitY());
  return (R * v).normalized();
}

void MorphingController::zeroDragTorque(std::vector<double>& thrust)
{
  const double mfr = robot_model_->getMFRate();   // URDFの<m_f_rate value="-0.0068"/>
  const auto& dir  = robot_model_->getRotorDirection();

  double sum = 0.0, sum_abs = 0.0;
  for (int i = 0; i < motor_num_; ++i)
  {
    // ★ 注意：dirの添字規約。iかi+1かは環境に依存。必要なら両方試してログ確認
    const double w = dir.at(i + 1);
    sum     += w * thrust[i];
    sum_abs += std::abs(w);
  }
  if (std::abs(sum) < 1e-6) return;

  // 補正
  const double alpha = 0.3;       // 0.1〜0.5で調整
  const double corr  = alpha * (sum / sum_abs);
  for (int i = 0; i < motor_num_; ++i)
  {
    const double w = dir.at(i + 1);
    thrust[i] -= corr * w;        // 逆符号で相殺方向に
    if (thrust[i] < 0.0) thrust[i] = 0.0; // ついでに負値保護
  }
}

Eigen::MatrixXd MorphingController::buildQWithTheta()
{
  std::vector<Eigen::Vector3d> r = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  std::vector<Eigen::Vector3d> n = robot_model_->getRotorsNormalFromCog<Eigen::Vector3d>();

  /* update n_i by theta */
  // if (use_theta_allocation_)
  // {
  //   for (unsigned int i = 0; i < motor_num_; ++i)
  //     n[i] = rotateArm(n[i], theta_meas_[i]);
  // }

  auto& rotor_direction = robot_model_->getRotorDirection();
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, motor_num_);

  for (unsigned int i = 0; i < motor_num_; ++i)
  {
    double m_f_rate = robot_model_->getMFRate();
    Q(0, i) = n[i].z(); /* Z加速度寄与 */

    /* 角加速度寄与: r×n + m_f_rate * dir * n */
    Q.block<3,1>(1, i) = r[i].cross(n[i]) + m_f_rate * rotor_direction.at(i + 1) * n[i];
  }

  /* 質量・慣性でスケール */
  double mass_inv = 1.0 / robot_model_->getMass();
  Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
  Q.topRows(1)    = mass_inv    * Q.topRows(1);
  Q.bottomRows(3) = inertia_inv * Q.bottomRows(3);
  return Q;
}

Eigen::VectorXd MorphingController::allocConstrained(const Eigen::MatrixXd& Q, const Eigen::Vector4d& b,
                                                    const Eigen::VectorXd& f_ref,
                                                    const Eigen::VectorXd& Wf_diag,
                                                    const Eigen::VectorXd& f_max)
{
  /* 基本解（最小ノルム）： f0 = Q^T (Q Q^T)^{-1} b */
  Eigen::MatrixXd QQT = Q * Q.transpose();
  Eigen::VectorXd f0  = Q.transpose() * QQT.ldlt().solve(b);

  /* ヌル空間で f_ref に寄せる： min ||W (f0 + Ny - f_ref)||^2 */
  Eigen::VectorXd f = f0;
  Eigen::MatrixXd N = nullspace(Q);
  if (N.cols() > 0 && Wf_diag.maxCoeff() > 0.0)
  {
    Eigen::MatrixXd W2 = Wf_diag.asDiagonal();
    Eigen::MatrixXd A  = N.transpose() * W2 * N;
    Eigen::VectorXd c  = N.transpose() * W2 * (f_ref - f0);
    Eigen::VectorXd y  = A.ldlt().solve(c);
    f = f0 + N * y;
  }

  /* clip */
  Eigen::VectorXd f_clip = f;
  clip(f_clip, f_max);

  /* 等式制約を最小補正で再投影： Δf = Q^T (QQ^T)^{-1} (b - Q f_clip) */
  Eigen::VectorXd df = Q.transpose() * QQT.ldlt().solve(b - Q * f_clip);
  f = f_clip + df;

  /* clip */
  clip(f, f_max);
  return f;
}

void MorphingController::sendCmd()
{
  PoseLinearController::sendCmd();
  sendFourAxisCommand();
  sendTorqueAllocationMatrixInv();

  std_msgs::Float32MultiArray thrust_msg;
  thrust_msg.data.resize(target_base_thrust_.size());
  for (size_t i = 0; i < target_base_thrust_.size(); ++i){
    thrust_msg.data[i] = static_cast<float>(target_base_thrust_[i]);
  }
  thrust_pub_.publish(thrust_msg);
  publishCurrentJointStatus();
}

void MorphingController::reset()
{
  PoseLinearController::reset();
  setAttitudeGains();

  torque_allocation_matrix_inv_pub_stamp_ = -1.0;

  //  いまの機体向きをそのまま目標Yaw
  navigator_->setTargetYaw(rpy_.z());

  // 必要なら X,Y の PID も一度クリア（風見回り抑制に効く）
  pid_controllers_.at(X).reset();
  pid_controllers_.at(Y).reset();
}

void MorphingController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.angles[0] = target_roll_;
  flight_command_data.angles[1] = target_pitch_;
  flight_command_data.angles[2] = candidate_yaw_term_;
  flight_command_data.base_thrust.resize(target_base_thrust_.size());
  for (size_t i = 0; i < target_base_thrust_.size(); ++i)
    flight_command_data.base_thrust[i] = static_cast<float>(target_base_thrust_[i]);
  flight_cmd_pub_.publish(flight_command_data);
}

void MorphingController::sendTorqueAllocationMatrixInv()
{
  if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_ > torque_allocation_matrix_inv_pub_interval_)
  {
    torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now().toSec();

    spinal::TorqueAllocationMatrixInv msg;
    msg.rows.resize(motor_num_);

    /* q_mat_inv_ は最新制御周期で更新済み（右３列＝角加速度部） */
    Eigen::MatrixXd M = q_mat_inv_.rightCols(3);
    if (M.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
      ROS_ERROR("Torque Allocation Matrix overflow");

    for (unsigned int i = 0; i < motor_num_; ++i)
    {
      msg.rows.at(i).x = M(i, 0) * 1000.0;
      msg.rows.at(i).y = M(i, 1) * 1000.0;
      msg.rows.at(i).z = M(i, 2) * 1000.0;
    }
    torque_allocation_matrix_inv_pub_.publish(msg);
  }
}

void MorphingController::setAttitudeGains()
{
  spinal::RollPitchYawTerms rpy_gain_msg; // for rosserial
  /* to flight controller via rosserial scaling by 1000 */
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors.at(0).roll_p  = pid_controllers_.at(ROLL).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_i  = pid_controllers_.at(ROLL).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_d  = pid_controllers_.at(ROLL).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_i = pid_controllers_.at(PITCH).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).yaw_d   = pid_controllers_.at(YAW).getDGain() * 1000;
  rpy_gain_pub_.publish(rpy_gain_msg);
}

void MorphingController::thetaCallback(const std_msgs::Float32ConstPtr& msg)
{
  double theta = msg -> data;
  ROS_INFO_STREAM_THROTTLE(1.0, "[hugmy] theta recv = " << msg->data);
  theta = theta * M_PI /180.0;
  const double MAXTheta = 3.14;
  if (std::abs(theta) > MAXTheta){
    ROS_WARN_STREAM_THROTTLE(1.0, "[hugmy] theta out of range: " << theta << " rad. Ignored.");
    return;
  }else{
    applyThetaToModel(theta);
    for(size_t i=0;i < motor_num_;i++)
      {
        theta_meas_[i] = theta;
      }
  }
}

void MorphingController::thrustRefCallback(const std_msgs::Float32ConstPtr& msg)
{
  for(size_t i=0;i < motor_num_;i++)
  {
    f_ref_[i] = msg->data;
  }
}

void MorphingController::publishCurrentJointStatus()
{
  KDL::JntArray q = robot_model_->getJointPositions();
  const auto& idx = robot_model_->getJointIndexMap();

  sensor_msgs::JointState js;
  js.header.stamp = ros::Time::now();

  js.name.resize(q.rows());
  js.position.resize(q.rows());
  for (const auto& kv : idx){
    const std::string& name = kv.first;
    size_t index = kv.second;
    if (index < (size_t)q.rows()){
      js.name[index] = name;
      js.position[index] = q(index);
    }
  }
  debug_joint_state_pub_.publish(js);
}

void MorphingController::applyThetaToModel(float theta)
{
  KDL::JntArray q = robot_model_->getJointPositions();
  const auto& idx = robot_model_->getJointIndexMap();

  sensor_msgs::JointState js;
  js.header.stamp = ros::Time::now();
  js.name.reserve(12);
  js.position.reserve(12);

  for (int id = 1; id <= 4; ++id)
  {
    std::string j1 = "joint_1_" + std::to_string(id);
    std::string j2 = "joint_2_" + std::to_string(id);
    std::string j3 = "joint_3_" + std::to_string(id);

    auto clamp = [](double v, double lo, double hi){ return std::max(lo, std::min(hi, v)); };
    const double lim1 = 0.8727, limN = 1.047;
    ROS_INFO_STREAM_THROTTLE(0.5, "id =" << id);

    double d = theta / 3.0;

    if (auto it = idx.find(j1); it != idx.end()){
      double v = clamp(d,0.0,lim1);
      q(it->second) = v;
      js.name.push_back(j1);
      js.position.push_back(v);
    }

    if (auto it = idx.find(j2); it != idx.end()){
      double v = clamp(d,0.0,limN);
      q(it->second) = v;
      js.name.push_back(j2);
      js.position.push_back(v);
    }
    if (auto it = idx.find(j3); it != idx.end()){
      double v = clamp(d,0.0,limN);
      q(it->second) = v;
      js.name.push_back(j3);
      js.position.push_back(v);
    }
  }
  robot_model_->updateRobotModel(q);
  // ROS_INFO_STREAM_THROTTLE(0.5, "js" << js);
  joint_state_pub_.publish(js);
}

void MorphingController::nullspaceTimerCb(const ros::TimerEvent&)
{
  if (last_Q_.size() == 0) return;

  Eigen::MatrixXd N; int rank=0; double tol=0.0; Eigen::VectorXd S;
  // 軽量化: ComputeThinV だけでOK（Uは不要）
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(last_Q_, Eigen::ComputeThinV);
  S = svd.singularValues();

  const double eps = std::numeric_limits<double>::epsilon();
  tol = std::max(last_Q_.rows(), last_Q_.cols()) * eps * S(0);
  rank = (S.array() > tol).count();
  N = svd.matrixV().rightCols(last_Q_.cols() - rank);

  // 低レートでだけ publish / log
  std_msgs::Int32 dim; dim.data = N.cols();
  nullspace_dim_pub_.publish(dim);

  std_msgs::Float32MultiArray m;
  m.data.reserve(S.size());
  for (int i=0;i<S.size();++i) m.data.push_back(static_cast<float>(S(i)));
  svd_singvals_pub_.publish(m);

  ROS_DEBUG_STREAM("[Q SVD@5Hz] rank=" << rank << " nullity=" << N.cols() << " tol=" << tol);
}


Eigen::VectorXd MorphingController::allocSoftFz(
    const Eigen::MatrixXd& Qeq,          // 3xN : [Tx;Ty;Tz]
    const Eigen::Vector3d& beq,          // = 0
    const Eigen::RowVectorXd& qz,        // 1xN : Fz 行
    double Fz_des,
    const Eigen::VectorXd& f_ref,
    const Eigen::VectorXd& Wf_diag,
    const Eigen::VectorXd& f_max,
    double w_fz,
    double eps_reg)
{
  const int N = Qeq.cols();

  // 1) 等式の最小ノルム解：f_p = Qeq^T (Qeq Qeq^T)^(-1) beq
  Eigen::MatrixXd QeqQeqT = Qeq * Qeq.transpose();
  Eigen::VectorXd f_p = Qeq.transpose() * QeqQeqT.ldlt().solve(beq); // beq=0 -> 通常は 0

  // 2) ヌル空間 N を取る（Qeq * N = 0）
  Eigen::MatrixXd Nns = nullspace(Qeq);

  // ヌルがない（=完全拘束）なら、クリップ→再投影で帰す
  if (Nns.cols() == 0) {
    Eigen::VectorXd f = f_p;
    Eigen::VectorXd f_clip = f;
    clip(f_clip, f_max);
    // 等式再投影
    Eigen::VectorXd df = Qeq.transpose() * QeqQeqT.ldlt().solve(beq - Qeq * f_clip);
    f = f_clip + df;
    clip(f, f_max);
    return f;
  }

  // 3) y に関する二次目的を解く
  //    J(y) = ½ w_fz ( (qz N) y + (qz f_p - Fz_des) )^2
  //         + ½ (N y + (f_p - f_ref))^T W (N y + (f_p - f_ref))
  //         + ½ eps ||N y + f_p||^2
  Eigen::VectorXd qp = qz.transpose();                // Nx1
  double r  = qp.dot(f_p) - Fz_des;          // スカラー
  Eigen::MatrixXd W  = Wf_diag.asDiagonal();

  Eigen::MatrixXd A  = w_fz * (Nns.transpose() * qp * qp.transpose() * Nns)
                     + (Nns.transpose() * W * Nns)
                     + eps_reg * (Nns.transpose() * Nns);

  Eigen::VectorXd g  = w_fz * Nns.transpose() * qp * r
                     + Nns.transpose() * W * (f_p - f_ref)
                     + eps_reg * (Nns.transpose() * f_p);

  // 解く： A y = -g
  Eigen::VectorXd y = - A.ldlt().solve(g);

  Eigen::VectorXd f = f_p + Nns * y;

  // 4) クリップ
  Eigen::VectorXd f_clip = f;
  clip(f_clip, f_max);

  // 5) 等式再投影で Tx,Ty,Tz を厳密化
  Eigen::VectorXd df = Qeq.transpose() * QeqQeqT.ldlt().solve(beq - Qeq * f_clip);
  f = f_clip + df;

  // 6) 最後にもう一度クリップ（数値誤差対策）
  clip(f, f_max);

  return f;
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::MorphingController, aerial_robot_control::ControlBase);

//ｚ方向の推力制御は推力総量なので，それを無力化する
