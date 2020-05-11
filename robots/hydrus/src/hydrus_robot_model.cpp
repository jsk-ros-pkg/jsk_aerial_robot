#include <hydrus/hydrus_robot_model.h>

using namespace aerial_robot_model;

HydrusRobotModel::HydrusRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double fc_rp_min_thre, double epsilon, int wrench_dof):
  RobotModel(init_with_rosparam, verbose, 0, fc_t_min_thre, epsilon),
  fc_rp_min_thre_(fc_rp_min_thre),
  wrench_dof_(wrench_dof)
{
  if (init_with_rosparam)
    {
      getParamFromRos();
    }

  if(wrench_dof_ == 3) setFeasibleControlTMinThre(0);

  fc_rp_dists_.resize(getRotorNum());
  approx_fc_rp_dists_.resize(getRotorNum());
}

void HydrusRobotModel::getParamFromRos()
{
  ros::NodeHandle nhp("~");
  nhp.param("fc_rp_min_thre", fc_rp_min_thre_, 1e-6);
  nhp.param("rp_position_margin_thre", rp_position_margin_thre_, 0.01);
  nhp.param("wrench_mat_det_thre", wrench_mat_det_thre_, 1e-6);
  if(nhp.hasParam("wrench_dof"))  nhp.getParam("wrench_dof", wrench_dof_);
}

void HydrusRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  aerial_robot_model::RobotModel::updateRobotModelImpl(joint_positions);
  calcFeasibleControlRollPitchDists();

  // ROS_INFO_STREAM("fc_rp_min " << fc_rp_min_);
}

void HydrusRobotModel::updateJacobians(const KDL::JntArray& joint_positions, bool update_model)
{
  aerial_robot_model::RobotModel::updateJacobians(joint_positions, update_model);

  calcFeasibleControlRollPitchDistsJacobian();

#if 0
  feasibleControlRollPitchDistsNumericalJacobian(getJointPositions(), fc_rp_jacobian_);
  throw std::runtime_error("test");
#endif
}

bool HydrusRobotModel::stabilityCheck(bool verbose)
{
  if(!aerial_robot_model::RobotModel::stabilityCheck(verbose)) return false;

  if(fc_rp_min_ < fc_rp_min_thre_)
    { // only roll & pitch
      ROS_ERROR_STREAM("fc_rp_min " << fc_rp_min_ << " is lower than the threshold " <<  fc_rp_min_thre_);
      return false;
    }

  // deprecated statbility check method
  rollPitchPositionMarginCheck();
  wrenchMatrixDeterminantCheck();

  ROS_DEBUG_STREAM("rp_position_margin: " << rp_position_margin_ << " vs fc_t_min: " << getFeasibleControlTMin() << " vs fc_rp_min: " << fc_rp_min_);

  return true;
}

// @depreacated
bool HydrusRobotModel::rollPitchPositionMarginCheck()
{
  // TODO: depreacated
  /* calcuate the average */
  double average_x = 0, average_y = 0;
  const std::vector<Eigen::Vector3d> rotors_origin_from_cog = getRotorsOriginFromCog<Eigen::Vector3d>();
  const int rotor_num = getRotorNum();

  for(int i = 0; i < rotor_num; i++)
    {
      average_x += rotors_origin_from_cog.at(i)(0);
      average_y += rotors_origin_from_cog.at(i)(1);
      ROS_DEBUG("rotor%d x: %f, y: %f", i + 1, rotors_origin_from_cog.at(i)(0), rotors_origin_from_cog.at(i)(1));
    }
  average_x /= rotor_num;
  average_y /= rotor_num;

  double s_xy = 0, s_xx = 0, s_yy = 0;
  for(const auto& rotor_pos : rotors_origin_from_cog)
    {
      double x_diff = rotor_pos(0) - average_x;
      double y_diff = rotor_pos(1) - average_y;
      s_xy += (x_diff * y_diff);
      s_xx += (x_diff * x_diff);
      s_yy += (y_diff * y_diff);
    }

  Eigen::Matrix2d S;
  S << s_xx / rotor_num, s_xy / rotor_num, s_xy / rotor_num, s_yy / rotor_num;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(S);

  rp_position_margin_ = sqrt(es.eigenvalues()[0]) / getLinkLength();

  if(rp_position_margin_ < rp_position_margin_thre_)
    {
      ROS_WARN("Invalid old control margin against threshold: %f vs %f", rp_position_margin_, rp_position_margin_thre_);
      return false;
    }
  return true;
}

// @depreacated
bool HydrusRobotModel::wrenchMatrixDeterminantCheck()
{
  /* Wrench matrix determinant, should use wrench on CoG */
  Eigen::MatrixXd wrench_mat = calcWrenchMatrixOnCoG();

  // normalized
  wrench_mat.topRows(3) = wrench_mat.topRows(3) / getMass();
  wrench_mat.bottomRows(3) = getInertia<Eigen::Matrix3d>().inverse() *  wrench_mat.bottomRows(3);
  Eigen::MatrixXd valid_wrench_mat = wrench_mat.middleRows(2, wrench_dof_);
  wrench_mat_det_ = (valid_wrench_mat * valid_wrench_mat.transpose()).determinant();

  if(wrench_mat_det_ < wrench_mat_det_thre_)
    {
      ROS_WARN("Invalid wrench matrix determinant against threshold: %f vs %f", wrench_mat_det_, wrench_mat_det_thre_);
      return false;
    }
  return true;
}

void HydrusRobotModel::calcWrenchMatrixOnRoot()
{
  aerial_robot_model::RobotModel::calcWrenchMatrixOnRoot();
  const auto wrench_mat = getWrenchMatrix();
  setWrenchMatrix(wrench_mat.middleRows(2, wrench_dof_));
}

void HydrusRobotModel::calcStaticThrust()
{
  calcWrenchMatrixOnRoot(); // update Q matrix

  Eigen::VectorXd wrench_g = getGravityWrenchOnRoot();
  const auto& wrench_mat = getWrenchMatrix();

  // under-actuated
  Eigen::VectorXd static_thrust = aerial_robot_model::pseudoinverse(wrench_mat) * (-wrench_g.segment(2, wrench_dof_));
  setStaticThrust(static_thrust);
}

void HydrusRobotModel::thrustForceNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_result, std::vector<int> joint_indices)
{
  joint_indices = getJointIndices();

  double delta_angle = 0.00001; // [rad]
  Eigen::MatrixXd J_lambda = Eigen::MatrixXd::Zero(getRotorNum(), getJointNum());

  Eigen::VectorXd nominal_static_thrust = getStaticThrust();

  auto perturbationStaticThrust = [&](int col, KDL::JntArray joint_angles)
    {
      updateRobotModelImpl(joint_angles);
      J_lambda.col(col) = (getStaticThrust() - nominal_static_thrust) / delta_angle;
    };

  int col_index = 0;
  for (const auto& joint_index : joint_indices) {
    KDL::JntArray perturbation_joint_positions = joint_positions;
    perturbation_joint_positions(joint_index) += delta_angle;
    perturbationStaticThrust(col_index, perturbation_joint_positions);
    col_index++;
  }

  ROS_INFO_STREAM("numerical  lambda_jacobian: \n" << J_lambda);

  if(analytical_result.cols() > 0 && analytical_result.rows() > 0)
    {
      ROS_INFO_STREAM("analytical lambda_jacobian: \n" << analytical_result);
      Eigen::MatrixXd diff_mat = J_lambda - analytical_result.rightCols(getJointNum());
      ROS_INFO_STREAM("diff of lambda jacobian: \n" << diff_mat);

      double min_diff = diff_mat.minCoeff();
      double max_diff = diff_mat.maxCoeff();
      if(max_diff > fabs(min_diff)) ROS_INFO_STREAM("max diff of lambda jacobian: " << max_diff);
      else  ROS_INFO_STREAM("max diff of lambda jacobian: " << fabs(min_diff));
    }
}

void HydrusRobotModel::calcFeasibleControlRollPitchDists()
{
  /* only consider Moment for roll and pitch */
  const int rotor_num = getRotorNum();
  const double thrust_max = getThrustUpperLimit();

  auto v = calcV();

  for (auto& v_i: v) v_i.z() = 0;

  for (int i = 0; i < rotor_num; ++i) {
    double t_min_i = 0.0;
    auto v_i_normalized = v.at(i).normalized();
    for (int j = 0; j < rotor_num; ++j) {
      if (i == j) continue;
      double cross_product = (v.at(j).cross(v_i_normalized)).z();

      t_min_i += std::max(0.0, cross_product * thrust_max);
      //ROS_INFO("i: %d, j: %d, cross_product: %f, t_min_ij: %f" , i, j, cross_product, std::max(0.0, cross_product * thrust_max));
    }
    fc_rp_dists_(i) = t_min_i;
  }

  //  ROS_INFO_STREAM("fc_rp_distsj_: " << fc_rp_distsj_.transpose());
  fc_rp_min_ = fc_rp_dists_.minCoeff();
}

void HydrusRobotModel::calcFeasibleControlRollPitchDistsJacobian()
{
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();
  const int ndof = 6 + joint_num;
  const auto& p = getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto& u = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& sigma = getRotorDirection();
  const double thrust_max = getThrustUpperLimit();
  const double m_f_rate = getMFRate();
  const auto& u_jacobians = getUJacobians();
  const auto& p_jacobians = getPJacobians();
  const double epsilon = getEpsilon();

  fc_rp_dists_jacobian_.resize(rotor_num, 6 + joint_num);

  auto v = calcV();
  for (auto& v_i: v) v_i.z() = 0; // 2D

  std::vector<Eigen::MatrixXd> v_jacobians;
  for (int i = 0; i < rotor_num; ++i) {
    v_jacobians.push_back(-skew(u.at(i)) * p_jacobians.at(i) + skew(p.at(i)) * u_jacobians.at(i) + m_f_rate * sigma.at(i + 1) * u_jacobians.at(i));
    v_jacobians.back().row(2).setZero(); // 2D
  }

  for (int i = 0; i < rotor_num; ++i)
    {
      const Eigen::Vector3d& v_i = v.at(i);
      const Eigen::Vector3d v_i_normalized = v.at(i).normalized();
      const Eigen::MatrixXd& d_v_i = v_jacobians.at(i);

      double approx_dist = 0.0;
      Eigen::MatrixXd d_dist = Eigen::MatrixXd::Zero(1, ndof);
      for (int j = 0; j < rotor_num; ++j)
        {
          if (i == j) continue;

          const Eigen::Vector3d& v_j = v.at(j);
          const Eigen::MatrixXd& d_v_j = v_jacobians.at(j);
          const double v_cross_product = (v_j.cross(v_i_normalized)).z();
          Eigen::MatrixXd d_v_cross_product = -skew(v_i_normalized) * d_v_j + skew(v_j) * (1 / v_i.norm() * d_v_i - v_i_normalized / v_i.squaredNorm() * v_i.transpose() * d_v_i);
          approx_dist += reluApprox(v_cross_product * thrust_max, epsilon);
          d_dist += sigmoid(v_cross_product * thrust_max, epsilon) * d_v_cross_product.row(2) * thrust_max;
        } //j

      approx_fc_rp_dists_(i) = approx_dist;
      fc_rp_dists_jacobian_.row(i) = d_dist;
    } //i
}


void HydrusRobotModel::feasibleControlRollPitchDistsNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_result, std::vector<int> joint_indices)
{
  const auto& seg_frames = getSegmentsTf();
  if(joint_indices.empty()) joint_indices = getJointIndices();
  const int rotor_num = getRotorNum();
  const int full_body_dof = 6 + joint_indices.size();
  KDL::Rotation baselink_rot = getCogDesireOrientation<KDL::Rotation>();
  KDL::Rotation root_rot = getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(getBaselinkName()).M.Inverse();

  double delta_angle = 0.00001; // [rad]
  int col_index = 6;

  Eigen::VectorXd nominal_fc_rp_dists = fc_rp_dists_;
  Eigen::VectorXd nominal_approx_fc_rp_dists = approx_fc_rp_dists_;
  ROS_DEBUG_STREAM("nominal_fc_rp_dists :" << nominal_fc_rp_dists.transpose());
  ROS_DEBUG_STREAM("nominal_approx_fc_rp_dists :" << nominal_approx_fc_rp_dists.transpose());
  ROS_DEBUG_STREAM("diff of approx and nominal_fc_rp_dists :" << (nominal_approx_fc_rp_dists - nominal_fc_rp_dists).transpose());

  Eigen::MatrixXd J_rp_dists = Eigen::MatrixXd::Zero(fc_rp_dists_.size(), full_body_dof);
  Eigen::MatrixXd J_approx_rp_dists = Eigen::MatrixXd::Zero(fc_rp_dists_.size(), full_body_dof);

  auto perturbation = [&](int col, KDL::JntArray joint_angles)
    {
      updateRobotModelImpl(joint_angles);
      calcFeasibleControlRollPitchDistsJacobian();

      J_rp_dists.col(col) = (fc_rp_dists_ - nominal_fc_rp_dists) / delta_angle;
      J_approx_rp_dists.col(col) = (approx_fc_rp_dists_ - nominal_approx_fc_rp_dists) / delta_angle;

      for(int i = 0; i < fc_rp_dists_.size(); i++)
        {
          if(std::isnan(J_rp_dists.col(col)(i))) J_rp_dists.col(col)(i) = 0;
          if(std::isnan(J_approx_rp_dists.col(col)(i))) J_approx_rp_dists.col(col)(i) = 0;
        }
    };

  for (const auto& joint_index : joint_indices) {
    KDL::JntArray perturbation_joint_positions = joint_positions;
    perturbation_joint_positions(joint_index) += delta_angle;
    updateRobotModelImpl(perturbation_joint_positions);
    setCogDesireOrientation(root_rot * getSegmentsTf().at(getBaselinkName() ).M);
    perturbation(col_index, perturbation_joint_positions);
    col_index++;
  }

  // roll
  setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_angle, 0, 0) * seg_frames.at(getBaselinkName()).M);
  perturbation(3, joint_positions);

  // pitch
  setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_angle, 0) * seg_frames.at(getBaselinkName()).M);
  perturbation(4, joint_positions);

  // yaw
  setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_angle) * seg_frames.at(getBaselinkName()).M);
  perturbation(5, joint_positions);

  // reset
  setCogDesireOrientation(baselink_rot); // set the orientation of root
  updateRobotModelImpl(joint_positions);

  ROS_DEBUG_STREAM("numerical result of J_rp_dists: \n" << J_rp_dists);
  ROS_DEBUG_STREAM("numerical result of J_approx_rp_dists: \n" << J_approx_rp_dists);

  if(analytical_result.rows() > 0 && analytical_result.cols() > 0)
    {
      ROS_DEBUG_STREAM("analytical_result of J_approx_rp_dists: \n" << analytical_result);

      Eigen::MatrixXd diff_mat = J_approx_rp_dists - analytical_result;
      ROS_DEBUG_STREAM("diff of J_approx_rp_dists: \n" << diff_mat);
      double min_diff = diff_mat.minCoeff();
      double max_diff = diff_mat.maxCoeff();
      if(max_diff > fabs(min_diff)) ROS_INFO_STREAM("max diff of J_approx_rp_dists: " << max_diff);
      else  ROS_INFO_STREAM("max diff of J_approx_rp_dists: " << fabs(min_diff));

      diff_mat = J_rp_dists - analytical_result;
      ROS_DEBUG_STREAM("diff of J_rp_dists: \n" << diff_mat);
      min_diff = diff_mat.minCoeff();
      max_diff = diff_mat.maxCoeff();
      if(max_diff > fabs(min_diff)) ROS_INFO_STREAM("max diff of J_rp_dists: " << max_diff);
      else  ROS_INFO_STREAM("max diff of J_rp_dists: " << fabs(min_diff));
    }
}
