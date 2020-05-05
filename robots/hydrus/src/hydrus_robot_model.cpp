#include <hydrus/hydrus_robot_model.h>

HydrusRobotModel::HydrusRobotModel(bool init_with_rosparam, bool verbose, double epsilon, int wrench_dof, double control_margin_thre, double wrench_mat_det_thre):
  RobotModel(init_with_rosparam, verbose, epsilon),
  wrench_dof_(wrench_dof),
  control_margin_thre_(control_margin_thre),
  wrench_mat_det_thre_(wrench_mat_det_thre)
{
  if (init_with_rosparam)
    {
      getParamFromRos();
    }
}

void HydrusRobotModel::getParamFromRos()
{
  ros::NodeHandle nhp("~");
  nhp.param("control_margin_thre", control_margin_thre_, 0.01);
  nhp.param("wrench_mat_det_thre",wrench_mat_det_thre_, 1e-6);
  if(nhp.hasParam("wrench_dof"))  nhp.getParam("wrench_dof", wrench_dof_);
}

bool HydrusRobotModel::stabilityCheck(bool verbose)
{
  /* calcuate the average */
  double average_x = 0, average_y = 0;
  const std::vector<Eigen::Vector3d> rotors_origin_from_cog = getRotorsOriginFromCog<Eigen::Vector3d>();
  const int rotor_num = getRotorNum();

  for(int i = 0; i < rotor_num; i++)
    {
      average_x += rotors_origin_from_cog.at(i)(0);
      average_y += rotors_origin_from_cog.at(i)(1);
      if(verbose)
        ROS_INFO("rotor%d x: %f, y: %f", i + 1, rotors_origin_from_cog.at(i)(0), rotors_origin_from_cog.at(i)(1));
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

  control_margin_ = sqrt(es.eigenvalues()[0]) / getLinkLength();
  if(verbose) ROS_INFO("control_margin: %f", control_margin_);
  if(control_margin_ < control_margin_thre_)
    {
      ROS_ERROR("Invalid control margin against threshold: %f vs %f", control_margin_, control_margin_thre_);
      return false;
    }

  /* Wrench matrix determinant, should use wrench on CoG */
  Eigen::MatrixXd wrench_mat = calcWrenchMatrixOnCoG();
  // normalized
  wrench_mat.topRows(3) = wrench_mat.topRows(3) / getMass();
  wrench_mat.bottomRows(3) = getInertia<Eigen::Matrix3d>().inverse() *  wrench_mat.bottomRows(3);
  Eigen::MatrixXd valid_wrench_mat = wrench_mat.middleRows(2, wrench_dof_);
  wrench_mat_det_ = (valid_wrench_mat * valid_wrench_mat.transpose()).determinant();

  if(wrench_mat_det_ < wrench_mat_det_thre_)
    {
      ROS_ERROR("Invalid wrench matrix  determinant against threshold: %f vs %f", wrench_mat_det_, wrench_mat_det_thre_);
      return false;
    }

  if(getStaticThrust().maxCoeff() > getThrustUpperLimit() || getStaticThrust().minCoeff() < getThrustLowerLimit())
    {
      ROS_ERROR("Invalid static thrust, max: %f, min: %f", getStaticThrust().maxCoeff(), getStaticThrust().minCoeff());
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
