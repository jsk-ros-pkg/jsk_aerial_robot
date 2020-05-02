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
  Eigen::MatrixXd wrench_mat = calcWrenchMatrixOnCoG().middleRows(2, wrench_dof_);
  wrench_mat_det_ = (wrench_mat * wrench_mat.transpose()).determinant();

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
