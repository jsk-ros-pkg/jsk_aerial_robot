#include <aerial_robot_model/model/transformable_aerial_robot_model.h>

#include <limits>

using namespace aerial_robot_model::transformable;

void RobotModel::calcFeasibleControlJacobian()
{
  // reference:
  // https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8967725

  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();
  const int ndof = 6 + joint_num;
  const auto& p = getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto& u = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& sigma = getRotorDirection();
  const double thrust_min = getThrustLowerLimit();
  const double thrust_range = getThrustUpperLimit() - thrust_min;
  Eigen::Vector3d fg = getMass() * getGravity3d();
  const double m_f_rate = getMFRate();
  const double epsilon = getEpsilon();

  const auto v = calcV();
  std::vector<Eigen::MatrixXd> v_jacobians;
  for (int i = 0; i < rotor_num; ++i) {
    v_jacobians.push_back(-skew(u.at(i)) * p_jacobians_.at(i) + skew(p.at(i)) * u_jacobians_.at(i) + m_f_rate * sigma.at(i + 1) * u_jacobians_.at(i));
  }

  //calc jacobian of f_min_ij, t_min_ij
  int index = 0;
  for (int i = 0; i < rotor_num; ++i) {
    for (int j = 0; j < rotor_num; ++j) {

      if (i == j) continue;

      approx_fc_f_dists_(index) = std::numeric_limits<double>::infinity();
      fc_f_dists_jacobian_.row(index) = Eigen::MatrixXd::Zero(1, ndof);
      approx_fc_t_dists_(index) = std::numeric_limits<double>::infinity();
      fc_t_dists_jacobian_.row(index) = Eigen::MatrixXd::Zero(1, ndof);


      double approx_f_dist = 0.0;
      double approx_t_dist = 0.0;
      Eigen::MatrixXd d_f_min = Eigen::MatrixXd::Zero(1, ndof);
      Eigen::MatrixXd d_t_min = Eigen::MatrixXd::Zero(1, ndof);

      const Eigen::Vector3d& u_i = u.at(i);
      const Eigen::Vector3d& u_j = u.at(j);
      const Eigen::Vector3d uixuj = u_i.cross(u_j);
      const bool valid_f_plane = isValidFeasibleControlPlane(u_i, u_j);
      const Eigen::MatrixXd& d_u_i = u_jacobians_.at(i);
      const Eigen::MatrixXd& d_u_j = u_jacobians_.at(j);
      const Eigen::MatrixXd d_uixuj = -skew(u_j) * d_u_i  + skew(u_i) * d_u_j;

      const Eigen::Vector3d& v_i = v.at(i);
      const Eigen::Vector3d& v_j = v.at(j);
      const Eigen::Vector3d vixvj = v_i.cross(v_j);
      const bool valid_t_plane = isValidFeasibleControlPlane(v_i, v_j);
      const Eigen::MatrixXd& d_v_i = v_jacobians.at(i);
      const Eigen::MatrixXd& d_v_j = v_jacobians.at(j);
      const Eigen::MatrixXd d_vixvj = -skew(v_j) * d_v_i  + skew(v_i) * d_v_j;

      for (int k = 0; k < rotor_num; ++k) {

        if (i == k || j == k) continue;

        if (valid_f_plane) {
          const Eigen::Vector3d& u_k = u.at(k);
          const double u_triple_product = calcTripleProduct(u_i, u_j, u_k);
          const Eigen::MatrixXd& d_u_k = u_jacobians_.at(k);
          Eigen::MatrixXd d_u_triple_product = (uixuj / uixuj.norm()).transpose() * d_u_k + u_k.transpose() * (d_uixuj / uixuj.norm() - uixuj / (uixuj.norm() * uixuj.squaredNorm()) * uixuj.transpose() * d_uixuj);
          // max(lower*a, upper*a) = lower*a + relu((upper-lower)*a).
          d_f_min += (thrust_min + sigmoid(u_triple_product * thrust_range, epsilon) * thrust_range) * d_u_triple_product;
          approx_f_dist += u_triple_product * thrust_min + reluApprox(u_triple_product * thrust_range, epsilon);
        }

        if (valid_t_plane) {
          const Eigen::Vector3d& v_k = v.at(k);
          const double v_triple_product = calcTripleProduct(v_i, v_j, v_k);
          const Eigen::MatrixXd& d_v_k = v_jacobians.at(k);
          Eigen::MatrixXd d_v_triple_product = (vixvj / vixvj.norm()).transpose() * d_v_k + v_k.transpose() * (d_vixvj / vixvj.norm() - vixvj / (vixvj.norm() * vixvj.squaredNorm()) * vixvj.transpose() * d_vixvj);
          d_t_min += (thrust_min + sigmoid(v_triple_product * thrust_range, epsilon) * thrust_range) * d_v_triple_product;
          approx_t_dist += v_triple_product * thrust_min + reluApprox(v_triple_product * thrust_range, epsilon);
        }
      }

      if (valid_f_plane) {
        double uixuj_fg = uixuj.dot(fg)/uixuj.norm();
        Eigen::MatrixXd d_uixuj_fg = Eigen::MatrixXd::Zero(1, ndof);
        d_uixuj_fg = fg.transpose() * (d_uixuj / uixuj.norm() - uixuj / (uixuj.norm() * uixuj.squaredNorm()) * uixuj.transpose() * d_uixuj);

        approx_fc_f_dists_(index) = approx_f_dist - uixuj_fg;
        fc_f_dists_jacobian_.row(index) = d_f_min - d_uixuj_fg;
      }

      if (valid_t_plane) {
        approx_fc_t_dists_(index) = approx_t_dist;
        fc_t_dists_jacobian_.row(index) = d_t_min;
      }

      index++;

    } //j
  } //i
}
