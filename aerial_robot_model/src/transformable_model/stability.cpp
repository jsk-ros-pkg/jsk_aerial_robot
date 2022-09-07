#include <aerial_robot_model/transformable_aerial_robot_model.h>

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
  const double thrust_max = getThrustUpperLimit();
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
      double approx_f_dist = 0.0;
      double approx_t_dist = 0.0;
      Eigen::MatrixXd d_f_min = Eigen::MatrixXd::Zero(1, ndof);
      Eigen::MatrixXd d_t_min = Eigen::MatrixXd::Zero(1, ndof);
      const Eigen::Vector3d& u_i = u.at(i);
      const Eigen::Vector3d& u_j = u.at(j);
      const Eigen::Vector3d uixuj = u_i.cross(u_j);
      const Eigen::MatrixXd& d_u_i = u_jacobians_.at(i);
      const Eigen::MatrixXd& d_u_j = u_jacobians_.at(j);
      const Eigen::MatrixXd d_uixuj = -skew(u_j) * d_u_i  + skew(u_i) * d_u_j;

      const Eigen::Vector3d& v_i = v.at(i);
      const Eigen::Vector3d& v_j = v.at(j);
      const Eigen::Vector3d vixvj = v_i.cross(v_j);
      const Eigen::MatrixXd& d_v_i = v_jacobians.at(i);
      const Eigen::MatrixXd& d_v_j = v_jacobians.at(j);
      const Eigen::MatrixXd d_vixvj = -skew(v_j) * d_v_i  + skew(v_i) * d_v_j;

      for (int k = 0; k < rotor_num; ++k) {
        if (i == j || j == k || k == i) {
        } else {

          // u
          const Eigen::Vector3d& u_k = u.at(k);
          const double u_triple_product = calcTripleProduct(u_i, u_j, u_k);
          const Eigen::MatrixXd& d_u_k = u_jacobians_.at(k);
          Eigen::MatrixXd d_u_triple_product = (uixuj / uixuj.norm()).transpose() * d_u_k + u_k.transpose() * (d_uixuj / uixuj.norm() - uixuj / (uixuj.norm() * uixuj.squaredNorm()) * uixuj.transpose() * d_uixuj);
          d_f_min += sigmoid(u_triple_product * thrust_max, epsilon) * d_u_triple_product * thrust_max;
          approx_f_dist += reluApprox(u_triple_product * thrust_max, epsilon);

          // v
          const Eigen::Vector3d& v_k = v.at(k);
          const double v_triple_product = calcTripleProduct(v_i, v_j, v_k);
          const Eigen::MatrixXd& d_v_k = v_jacobians.at(k);
          Eigen::MatrixXd d_v_triple_product = (vixvj / vixvj.norm()).transpose() * d_v_k + v_k.transpose() * (d_vixvj / vixvj.norm() - vixvj / (vixvj.norm() * vixvj.squaredNorm()) * vixvj.transpose() * d_vixvj);
          d_t_min += sigmoid(v_triple_product * thrust_max, epsilon) * d_v_triple_product * thrust_max;
          approx_t_dist += reluApprox(v_triple_product * thrust_max, epsilon);
        } //if
      } //k

      if (i != j) {
        double uixuj_fg = uixuj.dot(fg)/uixuj.norm();
        Eigen::MatrixXd d_uixuj_fg = Eigen::MatrixXd::Zero(1, ndof);
        d_uixuj_fg = fg.transpose() * (d_uixuj / uixuj.norm() - uixuj / (uixuj.norm() * uixuj.squaredNorm()) * uixuj.transpose() * d_uixuj);

        approx_fc_f_dists_(index) = absApprox(approx_f_dist - uixuj_fg, epsilon);
        fc_f_dists_jacobian_.row(index) = tanh(approx_f_dist - uixuj_fg, epsilon) * (d_f_min - d_uixuj_fg);

        approx_fc_t_dists_(index) = approx_t_dist;
        fc_t_dists_jacobian_.row(index) = d_t_min;

        for(int l = 0; l < ndof; l++)
          {
            if(std::isnan(fc_f_dists_jacobian_.row(index)(0, l)))
              fc_f_dists_jacobian_.row(index)(0, l) = 0;
            if(std::isnan(fc_t_dists_jacobian_.row(index)(0, l)))
              fc_t_dists_jacobian_.row(index)(0, l) = 0;
          }
        index++;
      }
    } //j
  } //i
}
