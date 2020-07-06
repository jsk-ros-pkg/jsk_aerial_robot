#include <aerial_robot_model/transformable_aerial_robot_model.h>

namespace aerial_robot_model {


  bool RobotModel::stabilityCheck(bool verbose)
  {
    if(fc_f_min_ < fc_f_min_thre_)
      {
        if(verbose)
          ROS_ERROR_STREAM("the min distance to the plane of feasible control force convex " << fc_f_min_ << " is lower than the threshold " <<  fc_f_min_thre_);
          return false;
      }

    if(fc_t_min_ < fc_t_min_thre_)
      {
        if(verbose)
          ROS_ERROR_STREAM("the min distance to the plane of feasible control torque convex " << fc_t_min_ << " is lower than the threshold " <<  fc_t_min_thre_);
        return false;
      }

    if(static_thrust_.maxCoeff() > thrust_max_ || static_thrust_.minCoeff() < thrust_min_)
      {
        if(verbose)
          ROS_ERROR("Invalid static thrust, max: %f, min: %f", static_thrust_.maxCoeff(), static_thrust_.minCoeff());
        return false;
      }

    return true;
  }


  inline double RobotModel::calcTripleProduct(const Eigen::Vector3d& ui, const Eigen::Vector3d& uj, const Eigen::Vector3d& uk)
  {
    Eigen::Vector3d uixuj = ui.cross(uj);
    if (uixuj.norm() < 0.00001) {
      return 0.0;
    }
    return uixuj.dot(uk) / uixuj.norm();
  }

  std::vector<Eigen::Vector3d> RobotModel::calcV()
  {
    const std::vector<Eigen::Vector3d> p = getRotorsOriginFromCog<Eigen::Vector3d>();
    const std::vector<Eigen::Vector3d> u = getRotorsNormalFromCog<Eigen::Vector3d>();
    const auto& sigma = getRotorDirection();
    const int rotor_num = getRotorNum();
    const double m_f_rate = getMFRate();
    std::vector<Eigen::Vector3d> v(rotor_num);

    for (int i = 0; i < rotor_num; ++i)
      v.at(i) = p.at(i).cross(u.at(i)) + m_f_rate * sigma.at(i + 1) * u.at(i);
    return v;
  }

  void RobotModel::calcFeasibleControlFDists()
  {
    const int rotor_num = getRotorNum();
    const double thrust_max = getThrustUpperLimit();

    const auto& u = getRotorsNormalFromCog<Eigen::Vector3d>();
    Eigen::Vector3d gravity_force = getMass() * gravity_3d_;

    int index = 0;
    for (int i = 0; i < rotor_num; ++i) {
      const Eigen::Vector3d& u_i = u.at(i);
      for (int j = 0; j < rotor_num; ++j) {
        if (i == j) continue;
        const Eigen::Vector3d& u_j = u.at(j);

        double dist_ij = 0.0;
        for (int k = 0; k < rotor_num; ++k) {
          if (i == k || j == k) continue;
          const Eigen::Vector3d& u_k = u.at(k);
          double u_triple_product = calcTripleProduct(u_i, u_j, u_k);
          dist_ij += std::max(0.0, u_triple_product * thrust_max);
        }

        Eigen::Vector3d uixuj = u_i.cross(u_j);
        fc_f_dists_(index) = fabs(dist_ij - (uixuj.dot(gravity_force) / uixuj.norm()));
        index++;
      }
    }
    fc_f_min_ = fc_f_dists_.minCoeff();

  }

  void RobotModel::calcFeasibleControlTDists()
  {
    const int rotor_num = getRotorNum();
    const double thrust_max = getThrustUpperLimit();

    const auto v = calcV();
    int index = 0;

    for (int i = 0; i < rotor_num; ++i) {
      const Eigen::Vector3d& v_i = v.at(i);
      for (int j = 0; j < rotor_num; ++j) {
        if (i == j) continue;
        const Eigen::Vector3d& v_j = v.at(j);
        double dist_ij = 0.0;
        for (int k = 0; k < rotor_num; ++k) {
          if (i == k || j == k) continue;
          const Eigen::Vector3d& v_k = v.at(k);
          double v_triple_product = calcTripleProduct(v_i, v_j, v_k);
          dist_ij += std::max(0.0, v_triple_product * thrust_max);
        }
        fc_t_dists_(index) = dist_ij;
        index++;
      }
    }

    fc_t_min_ = fc_t_dists_.minCoeff();
  }

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
    Eigen::Vector3d fg = getMass() * gravity_3d_;
    const double m_f_rate = getMFRate();

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
            d_f_min += sigmoid(u_triple_product * thrust_max, epsilon_) * d_u_triple_product * thrust_max;
            approx_f_dist += reluApprox(u_triple_product * thrust_max, epsilon_);

            // v
            const Eigen::Vector3d& v_k = v.at(k);
            const double v_triple_product = calcTripleProduct(v_i, v_j, v_k);
            const Eigen::MatrixXd& d_v_k = v_jacobians.at(k);
            Eigen::MatrixXd d_v_triple_product = (vixvj / vixvj.norm()).transpose() * d_v_k + v_k.transpose() * (d_vixvj / vixvj.norm() - vixvj / (vixvj.norm() * vixvj.squaredNorm()) * vixvj.transpose() * d_vixvj);
            d_t_min += sigmoid(v_triple_product * thrust_max, epsilon_) * d_v_triple_product * thrust_max;
            approx_t_dist += reluApprox(v_triple_product * thrust_max, epsilon_);
          } //if
        } //k

        if (i != j) {
          double uixuj_fg = uixuj.dot(fg)/uixuj.norm();
          Eigen::MatrixXd d_uixuj_fg = Eigen::MatrixXd::Zero(1, ndof);
          d_uixuj_fg = fg.transpose() * (d_uixuj / uixuj.norm() - uixuj / (uixuj.norm() * uixuj.squaredNorm()) * uixuj.transpose() * d_uixuj);

          approx_fc_f_dists_(index) = absApprox(approx_f_dist - uixuj_fg, epsilon_);
          fc_f_dists_jacobian_.row(index) = tanh(approx_f_dist - uixuj_fg, epsilon_) * (d_f_min - d_uixuj_fg);

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


  void RobotModel::stabilityInit()
  {
    const int full_body_dof = 6 + joint_num_;

    approx_fc_f_dists_.resize(rotor_num_ * (rotor_num_ - 1));
    approx_fc_t_dists_.resize(rotor_num_ * (rotor_num_ - 1));
    fc_f_dists_.resize(rotor_num_ * (rotor_num_ - 1));
    fc_t_dists_.resize(rotor_num_ * (rotor_num_ - 1));
    fc_f_dists_jacobian_.resize(rotor_num_ * (rotor_num_ - 1), full_body_dof);
    fc_t_dists_jacobian_.resize(rotor_num_ * (rotor_num_ - 1), full_body_dof);
  }

} //namespace aerial_robot_model
