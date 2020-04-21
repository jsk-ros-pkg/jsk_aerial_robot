#include <hydrus_xi/hydrus_xi_fully_actuated_robot_model.h>

HydrusXiFullyActuatedRobotModel::HydrusXiFullyActuatedRobotModel(bool init_with_rosparam, bool verbose, double stability_margin_thre, double p_det_thre, double f_max, double f_min, double m_f_rate, bool only_three_axis_mode, double epsilon) :
  HydrusRobotModel(init_with_rosparam, verbose, stability_margin_thre, p_det_thre, f_max, f_min, m_f_rate, only_three_axis_mode), epsilon_(epsilon)
{

  if (init_with_rosparam) {
    getParamFromRos();
  }

  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();
  u_jacobian_.resize(rotor_num);
  v_jacobian_.resize(rotor_num);
  p_jacobian_.resize(rotor_num);
  q_jacobian_.resize(joint_num);
  cog_jacobian_.resize(3, joint_num);
  gravity_.resize(6);
  gravity_ <<  0, 0, 9.80665, 0, 0, 0;
  gravity_3d_.resize(3);
  gravity_3d_ << 0, 0, 9.80665;
  lambda_jacobian_.resize(rotor_num, joint_num);
  f_min_ij_.resize(rotor_num * (rotor_num - 1));
  t_min_ij_.resize(rotor_num * (rotor_num - 1));
  f_min_jacobian_.resize(rotor_num * (rotor_num - 1), joint_num);
  t_min_jacobian_.resize(rotor_num * (rotor_num - 1), joint_num);
  joint_torque_.resize(joint_num);
  joint_torque_jacobian_.resize(joint_num, joint_num);
  static_thrust_.resize(rotor_num);

  u_triple_product_jacobian_.resize(rotor_num);
  for (auto& j : u_triple_product_jacobian_) {
    j.resize(rotor_num);
    for (auto& k : j) {
      k.resize(rotor_num);
      for (auto& vec : k) {
        vec.resize(joint_num);
      }
    }
  }

  v_triple_product_jacobian_.resize(rotor_num);
  for (auto& j : v_triple_product_jacobian_) {
    j.resize(rotor_num);
    for (auto& k : j) {
      k.resize(rotor_num);
      for (auto& vec : k) {
        vec.resize(joint_num);
      }
    }
  }
}


void HydrusXiFullyActuatedRobotModel::getParamFromRos()
{
  ros::NodeHandle nhp("~");
  nhp.param("epslion", epsilon_, 10.0);
}

void HydrusXiFullyActuatedRobotModel::calcCOGJacobian()
{
  double mass_all = getMass();
  const auto& segment_map = getTree().getSegments();
  const auto& seg_frames = getSegmentsTf();
  const auto& inertia_map = getInertiaMap();
  const auto& joint_segment_map = getJointSegmentMap();

  int col_index = 0;
  for (const auto& joint_segment : joint_segment_map) {
    std::string joint_child_segment_name = joint_segment.second.at(0);
    KDL::Segment joint_child_segment = GetTreeElementSegment(segment_map.at(joint_child_segment_name));
    KDL::Vector a = seg_frames.at(joint_child_segment_name).M * joint_child_segment.getJoint().JointAxis();

    KDL::Vector r = seg_frames.at(joint_child_segment_name).p;
    KDL::RigidBodyInertia inertia = KDL::RigidBodyInertia::Zero();
    for (const auto& seg : joint_segment.second) {
      if (seg.find("thrust") == std::string::npos) {
        KDL::Frame f = seg_frames.at(seg);
        inertia = inertia + f * inertia_map.at(seg);
      }
    }
    KDL::Vector c = inertia.getCOG();
    double m = inertia.getMass();

    KDL::Vector cog_jacobian_col = a * (c - r) * m / mass_all;

    cog_jacobian_.col(col_index) = aerial_robot_model::kdlToEigen(cog_jacobian_col);
    col_index++;
  }
}

void HydrusXiFullyActuatedRobotModel::updateJacobians(const KDL::JntArray& joint_positions)
{
  updateRobotModel(joint_positions);
  calcCOGJacobian();

  const int joint_num = getJointNum();
  const int rotor_num = getRotorNum();
  const auto& u = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& p = getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto& sigma = getRotorDirection();
  std::vector<Eigen::Vector3d> v(rotor_num);
  Eigen::MatrixXd q_mat = calcQMatrix(u, p, sigma);
  Eigen::MatrixXd q_inv = aerial_robot_model::pseudoinverse(q_mat);
  Eigen::Vector3d fg = getMass() * gravity_3d_;
  std::vector<Eigen::MatrixXd> thrust_coord_jacobians(rotor_num);

  //calc jacobian of u(thrust direction, force vector), p(thrust position), v(torque vector)
  for (int i = 0; i < rotor_num; ++i) {
    std::string seg_name = std::string("thrust") + std::to_string(i + 1);
    Eigen::MatrixXd thrust_coord_jacobian = getJacobian(joint_positions, seg_name);
    thrust_coord_jacobians.at(i) = thrust_coord_jacobian;
    u_jacobian_.at(i) = -skew(u.at(i)) * thrust_coord_jacobian.bottomRows(3);
    p_jacobian_.at(i) = thrust_coord_jacobian.topRows(3) - cog_jacobian_;
    v.at(i) = p.at(i).cross(u.at(i)) + m_f_rate_ * sigma.at(i + 1) * u.at(i);
    v_jacobian_.at(i) = -skew(u.at(i)) * p_jacobian_.at(i) + skew(p.at(i)) * u_jacobian_.at(i) + m_f_rate_ * sigma.at(i + 1) * u_jacobian_.at(i);
  }

  //calc jacobian of q(matrix), lambda(thrust value)
  for (int i = 0; i < joint_num; ++i) {
    Eigen::MatrixXd q_jacobi_i(6, rotor_num);
    for (int j = 0; j < rotor_num; ++j) {
      q_jacobi_i.block(0, j, 3, 1) = u_jacobian_.at(j).col(i);
      q_jacobi_i.block(3, j, 3, 1) = v_jacobian_.at(j).col(i);
    }
    q_jacobian_.at(i) = q_jacobi_i;
    lambda_jacobian_.col(i) = -q_inv * q_jacobi_i * q_inv * getMass() * gravity_;
  }

  //calc jacobian of f_min_ij, t_min_ij
  int f_min_index = 0;
  for (int i = 0; i < rotor_num; ++i) {
    for (int j = 0; j < rotor_num; ++j) {
      double f_min = 0.0;
      double t_min = 0.0;
      Eigen::VectorXd d_f_min = Eigen::VectorXd::Zero(joint_num);
      Eigen::VectorXd d_t_min = Eigen::VectorXd::Zero(joint_num);
      const Eigen::Vector3d& u_i = u.at(i);
      const Eigen::Vector3d& u_j = u.at(j);
      const Eigen::Vector3d uixuj = u_i.cross(u_j);
      const Eigen::Vector3d& v_i = v.at(i);
      const Eigen::Vector3d& v_j = v.at(j);
      const Eigen::Vector3d vixvj = v_i.cross(v_j);

      for (int k = 0; k < rotor_num; ++k) {
        if (i == j || j == k || k == i) {
          u_triple_product_jacobian_.at(i).at(j).at(k) = Eigen::VectorXd::Zero(joint_num);
          v_triple_product_jacobian_.at(i).at(j).at(k) = Eigen::VectorXd::Zero(joint_num);
        } else {
          const Eigen::Vector3d& u_k = u.at(k);
          const Eigen::Vector3d& v_k = v.at(k);
          const double u_triple_product = calcTripleProduct(u_i, u_j, u_k);
          const double v_triple_product = calcTripleProduct(v_i, v_j, v_k);
          for (int l = 0; l < joint_num; ++l) {
            {
              const Eigen::Vector3d& d_u_i = u_jacobian_.at(i).col(l);
              const Eigen::Vector3d& d_u_j = u_jacobian_.at(j).col(l);
              const Eigen::Vector3d& d_u_k = u_jacobian_.at(k).col(l);
              const Eigen::Vector3d d_uixuj = u_i.cross(d_u_j) + d_u_i.cross(u_j);

              double d_u_triple_product = (uixuj / uixuj.norm()).dot(d_u_k) + u_k.dot(1/uixuj.norm() * d_uixuj - uixuj / (uixuj.norm() * uixuj.squaredNorm()) * uixuj.dot(d_uixuj));
              u_triple_product_jacobian_.at(i).at(j).at(k)(l) = d_u_triple_product;
              d_f_min(l) += sigmoid(u_triple_product * f_max_) * d_u_triple_product * f_max_;
            }
            {
              const Eigen::Vector3d& d_v_i = v_jacobian_.at(i).col(l);
              const Eigen::Vector3d& d_v_j = v_jacobian_.at(j).col(l);
              const Eigen::Vector3d& d_v_k = v_jacobian_.at(k).col(l);
              const Eigen::Vector3d d_vixvj = v_i.cross(d_v_j) + d_v_i.cross(v_j);

              double d_v_triple_product = (vixvj / vixvj.norm()).dot(d_v_k) + v_k.dot(1/vixvj.norm() * d_vixvj - vixvj / (vixvj.norm() * vixvj.squaredNorm()) * vixvj.dot(d_vixvj));
              v_triple_product_jacobian_.at(i).at(j).at(k)(l) = d_v_triple_product;
              d_t_min(l) += sigmoid(v_triple_product * f_max_) * d_v_triple_product * f_max_;
            }
          } //l
          f_min += reluApprox(u_triple_product * f_max_);
          t_min += reluApprox(v_triple_product * f_max_);
        } //if
      } //k

      if (i != j) {
        double uixuj_fg = uixuj.dot(fg)/uixuj.norm();
        Eigen::VectorXd d_uixuj_fg(joint_num);
        for (int l = 0; l < joint_num; ++l) {
          const Eigen::Vector3d& d_u_i = u_jacobian_.at(i).col(l);
          const Eigen::Vector3d& d_u_j = u_jacobian_.at(j).col(l);
          const Eigen::Vector3d d_uixuj = u_i.cross(d_u_j) + d_u_i.cross(u_j);
          d_uixuj_fg(l) = fg.dot(1/uixuj.norm() * d_uixuj - uixuj / (uixuj.norm() * uixuj.squaredNorm()) * uixuj.dot(d_uixuj));
        } //l
        f_min_ij_(f_min_index) = absApprox(f_min - uixuj_fg);
        t_min_ij_(f_min_index) = absApprox(t_min);
        f_min_jacobian_.row(f_min_index) = (tanh(f_min - uixuj_fg) * (d_f_min - d_uixuj_fg)).transpose();
        t_min_jacobian_.row(f_min_index) = (tanh(t_min) * (d_t_min)).transpose();
        f_min_index++;
      }
    } //j
  } //i

  //calc jacobian of joint torque
  static_thrust_ = calcStaticThrust(q_inv);
  joint_torque_ = Eigen::VectorXd::Zero(joint_num);
  joint_torque_jacobian_ = Eigen::MatrixXd::Zero(joint_num, joint_num);

  for (int i = 0; i < rotor_num; ++i) {
    Eigen::VectorXd wrench(6);
    Eigen::Vector3d thrust = u.at(i) * static_thrust_(i);
    wrench.block(0, 0, 3, 1) = thrust;
    wrench.block(3, 0, 3, 1) = m_f_rate_ * sigma.at(i + 1) * thrust;
    std::string thrust_name = std::string("thrust") + std::to_string(i + 1);

    joint_torque_ -= thrust_coord_jacobians.at(i).transpose() * wrench;

    for (int j = 0; j < joint_num; ++j) {
      Eigen::MatrixXd hessian(6, joint_num);
      for (int k = 0; k < joint_num; ++k) {
        hessian.col(k) = getSecondDerivative(thrust_name, j, k);
      }
      joint_torque_jacobian_.col(j) += hessian.transpose() * wrench;
    }

    Eigen::MatrixXd wrench_jacobian(6, joint_num);
    wrench_jacobian.topRows(3) = u_jacobian_.at(i) * static_thrust_(i);
    wrench_jacobian.bottomRows(3) = m_f_rate_ * sigma.at(i) * u_jacobian_.at(i);

    joint_torque_jacobian_ += thrust_coord_jacobians.at(i).transpose() * (wrench_jacobian + wrench / static_thrust_(i) * lambda_jacobian_.row(i));
  }

  joint_torque_jacobian_ *= -1;

}

void HydrusXiFullyActuatedRobotModel::updateJacobians(const sensor_msgs::JointState& joint_state)
{
  updateJacobians(jointMsgToKdl(joint_state));
}

Eigen::MatrixXd HydrusXiFullyActuatedRobotModel::getJacobian(const KDL::JntArray& joint_positions, std::string segment_name)
{
  const auto& tree = getTree();
  KDL::TreeJntToJacSolver solver(tree);
  KDL::Jacobian jac(tree.getNrOfJoints());
  int status = solver.JntToJac(joint_positions, jac, segment_name);
  return convertJacobian(jac.data);
}

Eigen::VectorXd HydrusXiFullyActuatedRobotModel::getSecondDerivative(std::string segment_name, int joint_i, int joint_j)
{
  const auto& segment_map = getTree().getSegments();
  const auto& seg_frames = getSegmentsTf();
  const auto& inertia_map = getInertiaMap();
  const auto& joint_hierachy = getJointHierachy();
  const auto& joint_segment_map = getJointSegmentMap();
  const auto& joint_names = getJointNames();

  std::string joint_i_name = joint_names.at(joint_i);
  std::string joint_j_name = joint_names.at(joint_j);
  if (joint_hierachy.at(joint_i_name) > joint_hierachy.at(joint_j_name)) {
    std::swap(joint_i_name, joint_j_name);
  }

  std::vector<std::string> joint_i_child_segments = joint_segment_map.at(joint_i_name);
  std::vector<std::string> joint_j_child_segments = joint_segment_map.at(joint_j_name);

  Eigen::Vector3d p_r = aerial_robot_model::kdlToEigen(seg_frames.at(segment_name).p);

  if (std::find(joint_i_child_segments.begin(), joint_i_child_segments.end(), segment_name) == joint_i_child_segments.end() ||  std::find(joint_j_child_segments.begin(), joint_j_child_segments.end(), segment_name) == joint_j_child_segments.end()) {
    return Eigen::VectorXd::Zero(6);
  }

  std::string joint_i_child_segment_name = joint_i_child_segments.at(0);
  std::string joint_j_child_segment_name = joint_j_child_segments.at(0);
  const KDL::Segment& joint_i_child_segment = GetTreeElementSegment(segment_map.at(joint_i_child_segment_name));
  const KDL::Segment& joint_j_child_segment = GetTreeElementSegment(segment_map.at(joint_j_child_segment_name));
  Eigen::Vector3d a_i = aerial_robot_model::kdlToEigen(seg_frames.at(joint_i_child_segment_name).M * joint_i_child_segment.getJoint().JointAxis());
  Eigen::Vector3d a_j = aerial_robot_model::kdlToEigen(seg_frames.at(joint_j_child_segment_name).M * joint_j_child_segment.getJoint().JointAxis());
  Eigen::Vector3d p_j = aerial_robot_model::kdlToEigen(seg_frames.at(joint_j_child_segment_name).p); //joint pos

  Eigen::VectorXd derivative(6);
  derivative.topRows(3) = a_i.cross(a_j.cross(p_r - p_j));
  derivative.bottomRows(3) = a_i.cross(a_j);
  return derivative;
}

inline Eigen::MatrixXd HydrusXiFullyActuatedRobotModel::convertJacobian(const Eigen::MatrixXd& in)
{
  Eigen::MatrixXd out(6, getJointNum());
  const auto& joint_index_map = getJointIndexMap();
  const auto& joint_segment_map = getJointSegmentMap();

  int col_index = 0;
  for (const auto& joint_segment : joint_segment_map) {
    out.col(col_index) = in.col(joint_index_map.at(joint_segment.first));
    col_index++;
  }

  return out;
}

Eigen::MatrixXd HydrusXiFullyActuatedRobotModel::calcQMatrix()
{
  const std::vector<Eigen::Vector3d> rotors_origin = getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> rotors_normal = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& rotor_direction = getRotorDirection();
  const int rotor_num = getRotorNum();

  //Q : WrenchAllocationMatrix
  Eigen::MatrixXd Q(6, rotor_num);
  for (unsigned int i = 0; i < rotor_num; ++i) {
    Q.block(0, i, 3, 1) = rotors_normal.at(i);
    Q.block(3, i, 3, 1) = rotors_origin.at(i).cross(rotors_normal.at(i)) + m_f_rate_ * rotor_direction.at(i + 1) * rotors_normal.at(i);
  }

  return Q;
}

Eigen::MatrixXd HydrusXiFullyActuatedRobotModel::calcQMatrix(const std::vector<Eigen::Vector3d>& u, const std::vector<Eigen::Vector3d>& p, const std::map<int, int>& sigma)
{
  const int rotor_num = getRotorNum();

  Eigen::MatrixXd Q(6, rotor_num);
  for (unsigned int i = 0; i < rotor_num; ++i) {
    Q.block(0, i, 3, 1) = u.at(i);
    Q.block(3, i, 3, 1) = p.at(i).cross(u.at(i)) + m_f_rate_ * sigma.at(i + 1) * u.at(i);
  }

  return Q;
}

Eigen::MatrixXd HydrusXiFullyActuatedRobotModel::calcStaticThrust()
{
  auto Q_inv = aerial_robot_model::pseudoinverse(calcQMatrix());
  Eigen::MatrixXd static_thrust = Q_inv * getMass() * gravity_;
  return static_thrust;
}

Eigen::MatrixXd HydrusXiFullyActuatedRobotModel::calcStaticThrust(const Eigen::MatrixXd& q_inv)
{
  Eigen::MatrixXd static_thrust = q_inv * getMass() * gravity_;
  return static_thrust;
}

double HydrusXiFullyActuatedRobotModel::calcUTripleProduct(int i, int j, int k)
{
  const auto& u = getRotorsNormalFromCog<Eigen::Vector3d>();
  Eigen::Vector3d uixuj = u.at(i).cross(u.at(j));
  if (uixuj.norm() < 0.00001) {
    return 0.0;
  }
  return uixuj.dot(u.at(k)) / uixuj.norm();
}

inline double HydrusXiFullyActuatedRobotModel::calcTripleProduct(const Eigen::Vector3d& ui, const Eigen::Vector3d& uj, const Eigen::Vector3d& uk)
{
  Eigen::Vector3d uixuj = ui.cross(uj);
  if (uixuj.norm() < 0.00001) {
    return 0.0;
  }
  return uixuj.dot(uk) / uixuj.norm();
}

double HydrusXiFullyActuatedRobotModel::calcVTripleProduct(int i, int j, int k)
{
  const auto v = calcV();

  Eigen::Vector3d vixvj = v.at(i).cross(v.at(j));
  return vixvj.dot(v.at(k)) / vixvj.norm();
}

std::vector<Eigen::Vector3d> HydrusXiFullyActuatedRobotModel::calcV()
{
  const std::vector<Eigen::Vector3d> p = getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> u = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& sigma = getRotorDirection();
  const int rotor_num = getRotorNum();
  std::vector<Eigen::Vector3d> v(rotor_num);

  for (int i = 0; i < rotor_num; ++i)
    v.at(i) = p.at(i).cross(u.at(i)) + m_f_rate_ * sigma.at(i + 1) * u.at(i);
  return v;
}

Eigen::VectorXd HydrusXiFullyActuatedRobotModel::calcFmin()
{
  const int rotor_num = getRotorNum();
  Eigen::VectorXd f_min(rotor_num * (rotor_num - 1));

  const auto& u = getRotorsNormalFromCog<Eigen::Vector3d>();
  Eigen::Vector3d gravity_force = getMass() * gravity_3d_;

  int f_min_index = 0;
  for (int i = 0; i < rotor_num; ++i) {
    for (int j = 0; j < rotor_num; ++j) {
      if (i == j) continue;
      double f_min_ij = 0.0;
      for (int k = 0; k < rotor_num; ++k) {
        if (i == k || j == k) continue;
        double u_triple_product = calcUTripleProduct(i, j, k);
        f_min_ij += reluApprox(u_triple_product * f_max_);
      }
      Eigen::Vector3d uixuj = u.at(i).cross(u.at(j));
      double d_fij = absApprox(f_min_ij - (uixuj.dot(gravity_force) / uixuj.norm()));
      f_min(f_min_index) = d_fij;
      f_min_index++;
    }
  }

  return f_min;
}

Eigen::VectorXd HydrusXiFullyActuatedRobotModel::calcTmin()
{
  const int rotor_num = getRotorNum();
  Eigen::VectorXd t_min(rotor_num * (rotor_num - 1));

  const auto& v = calcV();
  int t_min_index = 0;

  for (int i = 0; i < rotor_num; ++i) {
    for (int j = 0; j < rotor_num; ++j) {
      if (i == j) continue;
      double t_min_ij = 0.0;
      for (int k = 0; k < rotor_num; ++k) {
        if (i == k || j == k) continue;
        double v_triple_product = calcVTripleProduct(i, j, k);
        t_min_ij += reluApprox(v_triple_product * f_max_);
      }
      Eigen::Vector3d vixvj = v.at(i).cross(v.at(j));
      double d_tij = absApprox(t_min_ij);
      t_min(t_min_index) = d_tij;
      t_min_index++;
    }
  }

  return t_min;
}

Eigen::MatrixXd HydrusXiFullyActuatedRobotModel::calcWrenchAllocationMatrix()
{
  const std::vector<Eigen::Vector3d> rotors_origin = getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> rotors_normal = getRotorsNormalFromCog<Eigen::Vector3d>();
  const int rotor_num = getRotorNum();

  //Q : WrenchAllocationMatrix
  Eigen::MatrixXd Q(6, rotor_num);
  double uav_mass_inv = 1.0 / getMass();
  Eigen::Matrix3d inertia_inv = getInertia<Eigen::Matrix3d>().inverse();
  for (unsigned int i = 0; i < rotor_num; ++i) {
    Q.block(0, i, 3, 1) = rotors_normal.at(i) * uav_mass_inv;
    Q.block(3, i, 3, 1) = inertia_inv * (rotors_origin.at(i).cross(rotors_normal.at(i)));
  }

  return Q;
}

Eigen::VectorXd HydrusXiFullyActuatedRobotModel::calcJointTorque(const sensor_msgs::JointState& joint_state)
{
  auto static_thrust = calcStaticThrust();
  KDL::JntArray joint_positions = jointMsgToKdl(joint_state);
  const auto& u = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& sigma = getRotorDirection();
  const int joint_num = getJointNum();
  const int rotor_num = getRotorNum();
  Eigen::VectorXd joint_torque = Eigen::VectorXd::Zero(joint_num);

  for (int i = 0; i < rotor_num; ++i) {
    std::string seg_name = std::string("thrust") + std::to_string(i + 1);
    Eigen::MatrixXd thrust_coord_jacobian = getJacobian(joint_positions, seg_name);
    Eigen::VectorXd wrench(6);
    Eigen::Vector3d thrust = u.at(i) * static_thrust(i);
    wrench.block(0, 0, 3, 1) = thrust;
    wrench.block(3, 0, 3, 1) = m_f_rate_ * sigma.at(i + 1) * thrust;
    joint_torque -= thrust_coord_jacobian.transpose() * wrench;
  }
  return joint_torque;
}

inline Eigen::Matrix3d HydrusXiFullyActuatedRobotModel::skew(const Eigen::Vector3d& vec)
{
  Eigen::Matrix3d skew_mat;
  skew_mat << 0.0, -vec(2), vec(1),
              vec(2), 0.0, -vec(0),
              -vec(1), vec(0), 0.0;
  return skew_mat;
}

inline double HydrusXiFullyActuatedRobotModel::reluApprox(double x)
{
  return std::log(1 + std::exp(x * epsilon_)) / epsilon_;
}

//differential of reluApprox
inline double HydrusXiFullyActuatedRobotModel::sigmoid(double x)
{
  return 1 / (1 + std::exp(- x * epsilon_));
}

inline double HydrusXiFullyActuatedRobotModel::absApprox(double x)
{
  return std::log(std::exp(- x * epsilon_) + std::exp(x * epsilon_)) / epsilon_;
}

//differential of absApprox
inline double HydrusXiFullyActuatedRobotModel::tanh(double x)
{
  double a = std::exp(-x * epsilon_);
  double b = std::exp(x * epsilon_);
  return (b - a) / (b + a);
}
