#include <hugmy/control/lpv_lqi_qp_controller.h>

using namespace aerial_robot_control;

static inline Eigen::Vector3d rotateArmY(const Eigen::Vector3d& v, double theta) {
  Eigen::AngleAxisd R(theta, Eigen::Vector3d::UnitY());
  return (R * v).normalized();
}

LpvLqiQpController::LpvLqiQpController() : UnderActuatedLQIController() {}

void LpvLqiQpController::initialize(ros::NodeHandle nh,
                                    ros::NodeHandle nhp,
                                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                    double ctrl_loop_rate) {
  UnderActuatedLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  theta_meas_.assign(motor_num_, 0.0);
  theta_sub_ = nh_.subscribe<std_msgs::Float32>("arm/theta", 1, &LpvLqiQpController::thetaCallback, this);
  joint_reflect_theta_pub_ = nh_.advertise<sensor_msgs::JointState>("/quadrotor/joints_ctrl", 1);
  z_perching_altitude_sub_ = nh_.subscribe<std_msgs::Float32>("perching/z_perching_altitude", 1, &LpvLqiQpController::zPerchingAltitudeCallback, this);
  descent_trigger_sub_ = nh_.subscribe<std_msgs::Empty>("perching/descent_trigger", 1, &LpvLqiQpController::descentTriggerCallback, this);
                                      
  started_ = ros::Time::now();

  loadQpParams();
}


bool LpvLqiQpController::optimalGain() {
  bool ok = UnderActuatedLQIController::optimalGain();
  if (!ok) return false;
  return true;
}


void LpvLqiQpController::thetaCallback(const std_msgs::Float32ConstPtr& msg) {
  for (auto& v : theta_meas_) v = msg->data;
  applyThetaToModel(msg->data);
}

void LpvLqiQpController::zPerchingAltitudeCallback(const std_msgs::Float32ConstPtr& msg) {
  z_perching_altitude_ = msg->data;
}

void LpvLqiQpController::descentTriggerCallback(const std_msgs::EmptyConstPtr& msg) {
  arm_theta_enabled_ = true;
}

void LpvLqiQpController::activate() {
  UnderActuatedLQIController::activate();
  // 離陸リセット
  started_ = ros::Time::now();
  gravity_scale_ = 0.0;
  f_prev_.setZero();
  pid_controllers_.at(Z).reset();
  arm_theta_enabled_ = false;
  theta_cmd_hold_ = 0.0f;
  applyThetaToModel(0.0f);
}

void LpvLqiQpController::applyThetaToModel(float theta)
{
  KDL::JntArray q = robot_model_->getJointPositions();
  const auto& idx = robot_model_->getJointIndexMap();

  sensor_msgs::JointState js;
  js.header.stamp = ros::Time::now();
  js.name.reserve(12);
  js.position.reserve(12);

  for (int id = 1; id <= 4; ++id)
  {
    std::string j1 = "joint_" + std::to_string(id) + "_1";
    std::string j2 = "joint_" + std::to_string(id) + "_2";
    std::string j3 = "joint_" + std::to_string(id) + "_3";

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
  joint_reflect_theta_pub_.publish(js);
}

void LpvLqiQpController::controlCore() {
  PoseLinearController::controlCore();

  const double th = getSchedTheta();
  ROS_INFO_THROTTLE(0.2, "[LpvLqiQp] theta_sched = %.3f [rad]", th);

  // applyInterpolatedGains(th);
  ROS_INFO_THROTTLE(0.2,
    "[LpvLqiQp] gains (roll_p=%.2f pitch_p=%.2f yaw_p=%.2f z_p=%.2f)",
    roll_gains_.empty()? -1 : roll_gains_[0].x(),
    pitch_gains_.empty()? -1 : pitch_gains_[0].x(),
    yaw_gains_.empty()? -1 : yaw_gains_[0].x(),
    z_gains_.empty()? -1 : z_gains_[0].x());

  // Q
  const auto& rotors_origin   = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto& rotors_normal   = robot_model_->getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& rotor_direction = robot_model_->getRotorDirection();
  const double m_f_rate       = robot_model_->getMFRate();

  const double uav_mass_inv = 1.0 / robot_model_->getMass();
  const Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, motor_num_);
  for (unsigned int i = 0; i < motor_num_; ++i) {
    Q(0, i) = rotors_normal.at(i).z() * uav_mass_inv;

    Q.block<3,1>(1, i) =
      inertia_inv * ( rotors_origin.at(i).cross(rotors_normal.at(i))
                      + m_f_rate * rotor_direction.at(i + 1) * rotors_normal.at(i) );
  }

  ROS_INFO_THROTTLE(0.2,
    "[LpvLqiQp] Q(0,:)=[%.3f %.3f %.3f %.3f] Fz_row", Q(0,0),Q(0,1),Q(0,2),Q(0,3));

  // b
  tf::Vector3 a_w(pid_controllers_.at(X).result(),
                  pid_controllers_.at(Y).result(),
                  pid_controllers_.at(Z).result());
  tf::Vector3 a_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * a_w;

  target_pitch_ =  a_dash.x() / aerial_robot_estimation::G;
  target_roll_  = -a_dash.y() / aerial_robot_estimation::G;

  Eigen::Vector4d b;
  b.setZero();

  double z_pid = pid_controllers_.at(Z).result();
  const double z_lim = pid_controllers_.at(Z).getLimitSum();
  double ff_acc_z = navigator_->getTargetAcc().z();

  // constraint z
  double z_pid_clamped = std::max(-z_lim, std::min(z_lim, z_pid));
  if (z_pid != z_pid_clamped) {
    pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
  }
  z_pid = z_pid_clamped;
  // gravity compensation ramp
  const double ramp_T = 100.0;
  double t = (ros::Time::now() - started_).toSec();
  gravity_scale_ = std::max(0.0, std::min(1.0, t / ramp_T));
  if (compensate_gravity_) ff_acc_z += gravity_scale_ * robot_model_->getGravity3d().z();

  // b(0) = pid_controllers_.at(Z).result() + ff_acc_z;
  b(0) = z_pid + ff_acc_z;
  b(3) = pid_controllers_.at(YAW).result();

  ROS_INFO_STREAM("[LpvLqiQp] ff_acc_z: " << ff_acc_z  << ", z_clamped: " << z_pid);

  ROS_INFO_THROTTLE(0.2,
    "[LpvLqiQp] b=[Fz=%.3f roll=%.3f pitch=%.3f yaw=%.3f]",
    b(0), b(1), b(2), b(3));

  if (w_.array().isNaN().any()) {
    ROS_ERROR("[LpvLqiQp] w has NaN");
    return;
  }

  // QP
  Eigen::VectorXd f = solveAllocationQP(Q, b);

  if (f.size()!=motor_num_ || !f.allFinite()) {
    ROS_ERROR("[LpvLqiQp] QP failed (size=%ld)", f.size());
    return;
  }

  ROS_INFO_THROTTLE(0.2,
    "[LpvLqiQp] f=[%.3f %.3f %.3f %.3f]",
    f(0), f(1), f(2), f(3));

  for (int i=0;i<motor_num_;++i)
    target_base_thrust_.at(i) = f(i);
  f_prev_ = f;

  // wrench検証
  Eigen::MatrixXd Pfull = robot_model_->calcWrenchMatrixOnCoG(); // 6xN
  Eigen::VectorXd wrench6 = Pfull * f;
  ROS_INFO_THROTTLE(0.2,
    "[LpvLqiQp] wrench=[Fx=%.3f Fy=%.3f Fz=%.3f | Tx=%.3f Ty=%.3f Tz=%.3f]",
    wrench6(0), wrench6(1), wrench6(2),
    wrench6(3), wrench6(4), wrench6(5));

  // yaw command to spinal
  Eigen::MatrixXd q_inv = aerial_robot_model::pseudoinverse(Q);
  double max_yaw_scale = 0.0;
  for (int i=0;i<motor_num_;++i)
    if (q_inv(i, YAW - 2) > max_yaw_scale) max_yaw_scale = q_inv(i, YAW - 2);

  double yaw_cmd = pid_controllers_.at(YAW).result() * max_yaw_scale;
  yaw_cmd = std::max(-0.18, std::min(0.18, yaw_cmd));
  static double yaw_lpf = 0.0;
  const double alpha = 0.15;
  yaw_lpf = (1 - alpha) * yaw_lpf + alpha * yaw_cmd;
  candidate_yaw_term_ = yaw_lpf;

  ROS_INFO_THROTTLE(0.2, "[LpvLqiQp] yaw_cmd=%.3f (max_scale=%.3f)", yaw_lpf, max_yaw_scale);

//   Eigen::VectorXd f_delta = solveAllocationQP(Q, b);
//     if (f_delta.size()!=motor_num_ || !f_delta.allFinite()) {
//       ROS_ERROR("[LpvLqiQp] QP failed (size=%ld)", f_delta.size());
//       return;
//     }

//   ROS_INFO_THROTTLE(0.2, "[LpvLqiQp] f=[%.3f %.3f %.3f %.3f]", f_delta(0), f_delta(1), f_delta(2), f_delta(3));

//   Eigen::VectorXd f = (f_bias + f_delta).cwiseMax(f_min_).cwiseMin(f_max_);
//   for (int i=0;i<motor_num_; ++i) target_base_thrust_[i] = f(i);
//   f_prev_ = f;

//   ROS_INFO_THROTTLE(0.5, "[Z] g=%.3f, Fz_hover_from_bias=%.3f",
//                   robot_model_->getGravity3d().z(),
//                   (Q.row(0) * (q_inv.col(0) * robot_model_->getGravity3d().z()))(0));


// }

// double LpvLqiQpController::zPerchingControl() {
//   ros::NodeHandle znh(nh_, "controller/z_perching");
//   bool use_altitude_sensor = false;
//   znh.param("use_altitude_sensor", use_altitude_sensor, false);

//   double z_curr = estimator_->getPos(Frame::COG, estimate_mode_).z();
//   if (use_altitude_sensor){
//     h_ = std::fabs(z_perching_altitude_);
//   }else{
//      // 最終的にnavigatorからやれ
//   double z_ref = estimator_->getPos(Frame::COG, estimate_mode_).z() - z_perching_altitude_;
//   h_ = std::fabs(z_curr - z_ref);
//   }

//   // double theta_perching_max;
//   // znh.param("theta_perching", theta_perching_max, 1.0);
//   // double h_mid = 0.2;
//   // double k = 4.0;
//   // double theta_ref = theta_perching_max / (1.0 + std::exp( k*(h_ - h_mid) ));
//   // applyThetaToModel(theta_ref);

//   double h0=1.0, a_desc_low=0.6, a_desc_high=2.0;
//   znh.param("h0", h0, 1.0);
//   znh.param("a_desc_low", a_desc_low, 0.6);
//   znh.param("a_desc_high", a_desc_high, 2.0);
//   double a_desc_max = a_desc_low + (a_desc_high - a_desc_low) * std::exp(-h_ / std::max(1e-6, h0));
//   return a_desc_max;
}

Eigen::VectorXd LpvLqiQpController::solveAllocationQP(const Eigen::MatrixXd& Q, const Eigen::Vector4d& b) {
  Eigen::Matrix4d W = Eigen::Matrix4d::Zero();
  W(0,0)=w_(0); W(1,1)=w_(1); W(2,2)=w_(2); W(3,3)=w_(3);

  const double mfr = robot_model_->getMFRate();
  const auto& dir  = robot_model_->getRotorDirection();
  Eigen::VectorXd d(motor_num_);
  for (int i=0;i<motor_num_;++i) d(i) = dir.at(i + 1) * mfr;

  Eigen::MatrixXd H = Q.transpose()*W*Q + rho_*(d*d.transpose()) + lambda_*Eigen::MatrixXd::Identity(motor_num_, motor_num_);
  Eigen::VectorXd g = -Q.transpose()*W*b - lambda_*f_prev_;

  Eigen::VectorXd f = -H.ldlt().solve(g);
  if (f.cwiseAbs().maxCoeff() > f_max_.cwiseAbs().maxCoeff()) {
    ROS_ERROR("[LpvLqiQp] initial f > f_max");
  }
  f = f.cwiseMax(f_min_).cwiseMin(f_max_);

  double step = 1.0 / (H.norm() + 1e-6);
  for (int it=0; it<max_iter_; ++it) {
    Eigen::VectorXd grad = H*f + g;
    Eigen::VectorXd f_new = (f - step*grad).cwiseMax(f_min_).cwiseMin(f_max_);
    if ((f_new - f).norm() < 1e-6) { f = f_new; break; }
    f = f_new;
  }
  return f;
}

double LpvLqiQpController::getSchedTheta() const {
  if (theta_meas_.empty()) return 0.0;
  double s=0; for (double v : theta_meas_) s += v;
  return s / theta_meas_.size();
}

void LpvLqiQpController::loadQpParams() {
  ros::NodeHandle qp_nh(nh_, "controller/qp");
  std::vector<double> wv;
  if (!qp_nh.getParam("w", wv) || wv.size() != 4) {
    ROS_WARN_STREAM("[LpvLqiQp] controller/qp/w missing or size!=4; use default");
    w_ << 10.0, 20.0, 20.0, 6.0;
  } else {
    w_ << (double)wv[0], (double)wv[1], (double)wv[2], (double)wv[3];
  }

  qp_nh.param("rho",    rho_,    0.2);
  qp_nh.param("lambda", lambda_, 0.01);
  qp_nh.param("max_iter", max_iter_, 20);

  std::vector<double> fmin, fmax;
  if (qp_nh.getParam("f_min", fmin) && (int)fmin.size()==motor_num_) {
    f_min_ = Eigen::Map<Eigen::VectorXd>(fmin.data(), motor_num_);
  } else {
    f_min_ = Eigen::VectorXd::Zero(motor_num_);
  }
  if (qp_nh.getParam("f_max", fmax) && (int)fmax.size()==motor_num_) {
    f_max_ = Eigen::Map<Eigen::VectorXd>(fmax.data(), motor_num_);
  } else {
    f_max_ = Eigen::VectorXd::Constant(motor_num_, 8.0);
  }
}

void LpvLqiQpController::loadOneAxisTable(
    ros::NodeHandle& nh, const std::string& key,
    std::vector<std::vector<Eigen::Vector3d>>& out)
{
  XmlRpc::XmlRpcValue root;
  if (!nh.getParam(key, root)) {
    ROS_WARN_STREAM("[LpvLqiQp] '" << nh.getNamespace() << "/" << key << "' not found; skip");
    out.clear();
    return;
  }
  if (root.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR_STREAM("[LpvLqiQp] '" << key << "' must be a list");
    out.clear(); return;
  }

  const int T = root.size();
  const int M = motor_num_;
  out.assign(T, std::vector<Eigen::Vector3d>(M, Eigen::Vector3d::Zero()));

  for (int t = 0; t < T; ++t) {
    if (root[t].getType() != XmlRpc::XmlRpcValue::TypeArray || root[t].size() != M) {
      ROS_ERROR_STREAM("[LpvLqiQp] '" << key << "' row " << t << " must be a list of length " << M);
      out.clear(); return;
    }
    for (int m = 0; m < M; ++m) {
      auto& node = root[t][m];
      if (node.getType() != XmlRpc::XmlRpcValue::TypeArray || node.size() != 3) {
        ROS_ERROR_STREAM("[LpvLqiQp] '" << key << "' row " << t << ", motor " << m << " must be [P,I,D]");
        out.clear(); return;
      }
      double P, I, D;
      if (!xmlToDouble(node[0], P) || !xmlToDouble(node[1], I) || !xmlToDouble(node[2], D)) {
        ROS_ERROR_STREAM("[LpvLqiQp] '" << key << "' row " << t << ", motor " << m << " has non-numeric PID");
        out.clear(); return;
      }
      out[t][m] = Eigen::Vector3d(P, I, D);
    }
  }
  ROS_INFO_STREAM("[LpvLqiQp] loaded '" << key << "' T=" << T << " M=" << M);
}


// void LpvLqiQpController::loadLpvTable() {
//   ros::NodeHandle lqi_nh(nh_, "controller/lqi");

//   if (!lqi_nh.getParam("thetas", theta_grid_) || theta_grid_.empty()) {
//     ROS_WARN("[LpvLqiQp] controller/lqi/thetas missing; LPV disabled");
//     theta_grid_.clear();
//     return;
//   }

//   loadOneAxisTable(lqi_nh, "roll_gains",  table_roll_);
//   loadOneAxisTable(lqi_nh, "pitch_gains", table_pitch_);
//   loadOneAxisTable(lqi_nh, "yaw_gains",   table_yaw_);
//   loadOneAxisTable(lqi_nh, "z_gains",     table_z_);

//   if (table_roll_.size() != theta_grid_.size() ||
//       table_pitch_.size() != theta_grid_.size() ||
//       table_yaw_.size() != theta_grid_.size() ||
//       table_z_.size() != theta_grid_.size()) {
//     ROS_ERROR("[LpvLqiQp] LPV gain tables size mismatch with thetas");
//     theta_grid_.clear();
//     table_roll_.clear();
//     table_pitch_.clear();
//     table_yaw_.clear();
//     table_z_.clear();
//     return;
//   }

//   ROS_INFO_STREAM("[LpvLqiQp] LPV enabled: thetas=" << theta_grid_.size());
// }

// void LpvLqiQpController::applyInterpolatedGains(double th) {
//   if (theta_grid_.empty()) return;

//   // 範囲外は端にクランプしているが，受け付けないようにthetaでする
//   if (th <= theta_grid_.front()) {
//     for (int m = 0; m < motor_num_; ++m) {
//       roll_gains_[m]  = table_roll_.front()[m];
//       pitch_gains_[m] = table_pitch_.front()[m];
//       yaw_gains_[m]   = table_yaw_.front()[m];
//       z_gains_[m]     = table_z_.front()[m];
//     }
//     return;
//   }
//   if (th >= theta_grid_.back()) {
//     for (int m = 0; m < motor_num_; ++m) {
//       roll_gains_[m]  = table_roll_.back()[m];
//       pitch_gains_[m] = table_pitch_.back()[m];
//       yaw_gains_[m]   = table_yaw_.back()[m];
//       z_gains_[m]     = table_z_.back()[m];
//     }
//     return;
//   }

//   int i1 = 1;
//   while (i1 < (int)theta_grid_.size() && theta_grid_[i1] < th) ++i1;
//   int i0 = i1 - 1;
//   const double t0 = theta_grid_[i0], t1 = theta_grid_[i1];
//   const double alpha = (th - t0) / std::max(1e-9, (t1 - t0)); // 0..1

//   auto lerp = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
//     return (1.0 - alpha) * a + alpha * b;
//   };

//   for (int m = 0; m < motor_num_; ++m) {
//     roll_gains_[m]  = lerp(table_roll_[i0][m],  table_roll_[i1][m]);
//     pitch_gains_[m] = lerp(table_pitch_[i0][m], table_pitch_[i1][m]);
//     yaw_gains_[m]   = lerp(table_yaw_[i0][m],   table_yaw_[i1][m]);
//     z_gains_[m]     = lerp(table_z_[i0][m],     table_z_[i1][m]);
//   }
// }

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::LpvLqiQpController, aerial_robot_control::ControlBase);

