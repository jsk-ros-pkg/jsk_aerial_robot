#include <hydrus/numerical_jacobians.h>

HydrusNumericalJacobian::HydrusNumericalJacobian(ros::NodeHandle nh, ros::NodeHandle nhp, std::unique_ptr<aerial_robot_model::transformable::RobotModel> robot_model):
  aerial_robot_model::NumericalJacobian(nh, nhp, std::move(robot_model))
{
  nhp_.param("check_feasible_control_roll_pitch", check_feasible_control_roll_pitch_, true);
}


bool HydrusNumericalJacobian::checkJacobians()
{
  bool flag = aerial_robot_model::NumericalJacobian::checkJacobians();
  if(check_feasible_control_roll_pitch_) flag &= checkFeasibleControlRollPitchJacobian();
  return flag;
}

const Eigen::MatrixXd HydrusNumericalJacobian::thrustForceNumericalJacobian(std::vector<int> joint_indices)
{

  const KDL::JntArray joint_positions = getRobotModel().getJointPositions();
  Eigen::VectorXd nominal_static_thrust = getRobotModel().getStaticThrust();
  Eigen::MatrixXd J_lambda = Eigen::MatrixXd::Zero(getRobotModel().getRotorNum(), getRobotModel().getJointNum());

  auto perturbationStaticThrust = [&](int col, KDL::JntArray joint_angles)
    {
      getRobotModel().updateRobotModel(joint_angles);
      J_lambda.col(col) = (getRobotModel().getStaticThrust() - nominal_static_thrust) / delta_;
    };

  int col_index = 0;
  for (const auto& joint_index : joint_indices) {
    KDL::JntArray perturbation_joint_positions = joint_positions;
    perturbation_joint_positions(joint_index) += delta_;
    perturbationStaticThrust(col_index, perturbation_joint_positions);
    col_index++;
  }

  ROS_DEBUG_STREAM("numerical lambda_jacobian: \n" << J_lambda);

  getRobotModel().updateRobotModel(joint_positions); // reset

  return J_lambda;
}

const std::vector<Eigen::MatrixXd> HydrusNumericalJacobian::feasibleControlRollPitchDistsNumericalJacobian(std::vector<int> joint_indices)
{
  const auto seg_frames = getRobotModel().getSegmentsTf();
  const KDL::JntArray joint_positions = getRobotModel().getJointPositions();
  const int rotor_num = getRobotModel().getRotorNum();
  const int full_body_dof = 6 + joint_indices.size();
  const std::string baselink = getRobotModel().getBaselinkName();
  KDL::Rotation baselink_rot = getRobotModel().getCogDesireOrientation<KDL::Rotation>();
  KDL::Rotation root_rot = getRobotModel().getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink).M.Inverse();

  // reset
  getRobotModel().updateRobotModel(joint_positions);
  getHydrusRobotModel().calcFeasibleControlRollPitchDistsJacobian();

  int col_index = 6;

  const Eigen::VectorXd nominal_fc_rp_dists = getHydrusRobotModel().getFeasibleControlRollPitchDists();
  const Eigen::VectorXd nominal_approx_fc_rp_dists = getHydrusRobotModel().getApproxFeasibleControlRollPitchDists();
  ROS_DEBUG_STREAM("nominal_fc_rp_dists :" << nominal_fc_rp_dists.transpose());
  ROS_DEBUG_STREAM("nominal_approx_fc_rp_dists :" << nominal_approx_fc_rp_dists.transpose());
  ROS_DEBUG_STREAM("diff of approx and nominal_fc_rp_dists :" << (nominal_approx_fc_rp_dists - nominal_fc_rp_dists).transpose());

  Eigen::MatrixXd J_rp_dists = Eigen::MatrixXd::Zero(nominal_fc_rp_dists.size(), full_body_dof);
  Eigen::MatrixXd J_approx_rp_dists = Eigen::MatrixXd::Zero(nominal_fc_rp_dists.size(), full_body_dof);

  auto perturbation = [&](int col, KDL::JntArray joint_angles)
    {
      getRobotModel().updateRobotModel(joint_angles);
      getHydrusRobotModel().calcFeasibleControlRollPitchDistsJacobian();

      const Eigen::VectorXd fc_rp_dists = getHydrusRobotModel().getFeasibleControlRollPitchDists();
      const Eigen::VectorXd approx_fc_rp_dists = getHydrusRobotModel().getApproxFeasibleControlRollPitchDists();

      J_rp_dists.col(col) = (fc_rp_dists - nominal_fc_rp_dists) / delta_;
      J_approx_rp_dists.col(col) = (approx_fc_rp_dists - nominal_approx_fc_rp_dists) / delta_;

      for(int i = 0; i < nominal_fc_rp_dists.size(); i++)
        {
          if(std::isnan(J_rp_dists.col(col)(i))) J_rp_dists.col(col)(i) = 0;
          if(std::isnan(J_approx_rp_dists.col(col)(i))) J_approx_rp_dists.col(col)(i) = 0;
        }
    };

  for (const auto& joint_index : joint_indices) {
    KDL::JntArray perturbation_joint_positions = joint_positions;
    perturbation_joint_positions(joint_index) += delta_;
    getRobotModel().updateRobotModel(perturbation_joint_positions);
    getRobotModel().setCogDesireOrientation(root_rot * getRobotModel().getSegmentsTf().at(baselink).M);
    perturbation(col_index, perturbation_joint_positions);
    col_index++;
  }

  // roll
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(delta_, 0, 0) * seg_frames.at(baselink).M);
  perturbation(3, joint_positions);

  // pitch
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, delta_, 0) * seg_frames.at(baselink).M);
  perturbation(4, joint_positions);

  // yaw
  getRobotModel().setCogDesireOrientation(root_rot * KDL::Rotation::RPY(0, 0, delta_) * seg_frames.at(baselink).M);
  perturbation(5, joint_positions);

  // reset
  getRobotModel().setCogDesireOrientation(baselink_rot); // set the orientation of root
  getRobotModel().updateRobotModel(joint_positions);

  ROS_DEBUG_STREAM("numerical result of J_approx_rp_dists: \n" << J_approx_rp_dists);
  ROS_DEBUG_STREAM("numerical result of J_rp_dists: \n" << J_rp_dists);

  std::vector<Eigen::MatrixXd> out;
  out.push_back(J_approx_rp_dists);
  out.push_back(J_rp_dists);

  return out;
}

bool HydrusNumericalJacobian::checkThrsutForceJacobian(std::vector<int> joint_indices)
{
  if(joint_indices.empty()) joint_indices = getRobotModel().getJointIndices();

  const Eigen::MatrixXd analytical_jacobi = getRobotModel().getLambdaJacobian().rightCols(getRobotModel().getJointNum());
  const Eigen::MatrixXd numerical_jacobi = thrustForceNumericalJacobian(joint_indices);

  ROS_DEBUG_STREAM("analytical lambda_jacobian: \n" << analytical_jacobi);
  ROS_DEBUG_STREAM("diff of lambda jacobian: \n" << numerical_jacobi - analytical_jacobi);

  double max_diff = (numerical_jacobi - analytical_jacobi).maxCoeff();
  double min_diff = (numerical_jacobi - analytical_jacobi).minCoeff();
  if(fabs(min_diff) > max_diff) max_diff = fabs(min_diff);

  if(max_diff < thrust_force_diff_thre_)
    {
      ROS_INFO_STREAM("max diff of lambda jacobian: " << max_diff);
      return true;
    }
  else
    {
      ROS_WARN_STREAM("max diff of lambda jacobian: " << max_diff << ", exceed!");
      return false;
    }
}

bool HydrusNumericalJacobian::checkFeasibleControlRollPitchJacobian(std::vector<int> joint_indices)
{
  if(joint_indices.empty()) joint_indices = getRobotModel().getJointIndices();

  bool flag = true;

  const Eigen::MatrixXd analytical_fc_rp_jacobi = getHydrusRobotModel().getFeasibleControlRollPitchDistsJacobian();
  const std::vector<Eigen::MatrixXd> numerical_jacobis = feasibleControlRollPitchDistsNumericalJacobian(joint_indices);

  ROS_DEBUG_STREAM("analytical appprox feasible control roll/pitch distances jacobian: \n" << analytical_fc_rp_jacobi);
  ROS_DEBUG_STREAM("diff of appprox fc_rp_dists_jacobian with: \n" << numerical_jacobis.at(0) - analytical_fc_rp_jacobi);

  double max_diff = (numerical_jacobis.at(0) - analytical_fc_rp_jacobi).maxCoeff();
  double min_diff = (numerical_jacobis.at(0) - analytical_fc_rp_jacobi).minCoeff();

  if(fabs(min_diff) > max_diff) max_diff = fabs(min_diff);

  if(max_diff < feasible_control_force_diff_thre_)
    {
      ROS_INFO_STREAM("max diff of appprox feasible control roll/pitch distances jacobian: " << max_diff);
    }
  else
    {
      ROS_WARN_STREAM("max diff of appprox feasible control roll/pitch distances jacobian: " << max_diff << ", exceed!");
      flag = false;
    }

  // not critical, compare with non-approximated feasible control (i.e. relu)
  {
    ROS_DEBUG_STREAM("diff of non-approx fc_rp_dists_jacobian: \n" << numerical_jacobis.at(1) - analytical_fc_rp_jacobi);

    max_diff = (numerical_jacobis.at(1) - analytical_fc_rp_jacobi).maxCoeff();
    min_diff = (numerical_jacobis.at(1) - analytical_fc_rp_jacobi).minCoeff();

    if(fabs(min_diff) > max_diff) max_diff = fabs(min_diff);
    ROS_INFO_STREAM("not critical: max diff of non-approx feasible control roll/pitch distances jacobian: " << max_diff);
  }

  return flag;
}
