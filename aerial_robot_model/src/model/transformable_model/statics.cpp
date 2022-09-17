#include <aerial_robot_model/model/transformable_aerial_robot_model.h>

using namespace aerial_robot_model::transformable;

void RobotModel::calcJointTorque(const bool update_jacobian)
{
  const auto& sigma = getRotorDirection();
  const auto& joint_positions = getJointPositions();
  const auto& inertia_map = getInertiaMap();
  const int joint_num = getJointNum();
  const int rotor_num = getRotorNum();
  const double m_f_rate = getMFRate();
  const auto gravity = getGravity();
  const auto& static_thrust =  getStaticThrust();
  const auto& thrust_wrench_units = getThrustWrenchUnits();

  if(update_jacobian)
    calcBasicKinematicsJacobian(); // update thrust_coord_jacobians_

  joint_torque_ = Eigen::VectorXd::Zero(joint_num);

  // update coord jacobians for cog point and convert to joint torque
  int seg_index = 0;
  for(const auto& inertia : inertia_map)
    {
      cog_coord_jacobians_.at(seg_index) = RobotModel::getJacobian(joint_positions, inertia.first, inertia.second.getCOG());
      joint_torque_ -= cog_coord_jacobians_.at(seg_index).rightCols(joint_num).transpose() * inertia.second.getMass() * (-gravity);
      seg_index ++;
    }

  // thrust
  for (int i = 0; i < rotor_num; ++i) {
    Eigen::VectorXd wrench = thrust_wrench_units.at(i) * static_thrust(i);
    joint_torque_ -= thrust_coord_jacobians_.at(i).rightCols(joint_num).transpose() * wrench;
  }
}

void RobotModel::calcLambdaJacobian()
{
  // w.r.t root
  const auto& inertia_map = getInertiaMap();
  const auto& rotor_direction = getRotorDirection();
  const auto& joint_positions = getJointPositions();
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();
  const int ndof = thrust_coord_jacobians_.at(0).cols();
  const double m_f_rate = getMFRate();
  const auto& q_mat = getThrustWrenchMatrix();
  const int wrench_dof = q_mat.rows(); // default: 6, under-actuated: 4
  Eigen::MatrixXd q_pseudo_inv = aerial_robot_model::pseudoinverse(q_mat);
  const auto gravity_3d = getGravity3d();
  const auto& static_thrust =  getStaticThrust();
  const auto& thrust_wrench_units = getThrustWrenchUnits();
  const auto& thrust_wrench_allocations = getThrustWrenchAllocations();

  /* derivative for gravity jacobian */
  Eigen::MatrixXd wrench_gravity_jacobian = Eigen::MatrixXd::Zero(6, ndof);
  for(const auto& inertia : inertia_map){
    wrench_gravity_jacobian.bottomRows(3) -= aerial_robot_model::skew(-inertia.second.getMass() * gravity_3d) * getSecondDerivativeRoot(inertia.first, inertia.second.getCOG());
  }

  ROS_DEBUG_STREAM("wrench_gravity_jacobian w.r.t. root : \n" << wrench_gravity_jacobian);

  if(wrench_dof == 6) // fully-actuated
    lambda_jacobian_ = -q_pseudo_inv * wrench_gravity_jacobian; // trans, rot
  else // under-actuated
    lambda_jacobian_ = -q_pseudo_inv * (wrench_gravity_jacobian).middleRows(2, wrench_dof); // z, rot

  /* derivative for thrust jacobian */
  std::vector<Eigen::MatrixXd> q_mat_jacobians;
  Eigen::MatrixXd q_inv_jacobian = Eigen::MatrixXd::Zero(6, ndof);
  for (int i = 0; i < rotor_num; ++i) {
    std::string thrust_name = std::string("thrust") + std::to_string(i + 1);
    Eigen::MatrixXd q_mat_jacobian = Eigen::MatrixXd::Zero(6, ndof);

    q_mat_jacobian.bottomRows(3) -= aerial_robot_model::skew(thrust_wrench_units.at(i).head(3)) * getSecondDerivativeRoot(thrust_name);

    Eigen::MatrixXd wrench_unit_jacobian = Eigen::MatrixXd::Zero(6, ndof);
    wrench_unit_jacobian.topRows(3) = -skew(thrust_wrench_units.at(i).head(3)) * thrust_coord_jacobians_.at(i).bottomRows(3);
    wrench_unit_jacobian.bottomRows(3) = -skew(thrust_wrench_units.at(i).tail(3)) * thrust_coord_jacobians_.at(i).bottomRows(3);
    q_mat_jacobian += (thrust_wrench_allocations.at(i) * wrench_unit_jacobian);

    q_inv_jacobian += (q_mat_jacobian * static_thrust(i));
    q_mat_jacobians.push_back(q_mat_jacobian);
  }

  if(wrench_dof == 6) // fully-actuated
    lambda_jacobian_ += -q_pseudo_inv * q_inv_jacobian; // trans, rot
  else // under-actuated
    lambda_jacobian_ += -q_pseudo_inv * (q_inv_jacobian).middleRows(2, wrench_dof); // z, rot

  // https://mathoverflow.net/questions/25778/analytical-formula-for-numerical-derivative-of-the-matrix-pseudo-inverse, the third tmer
  Eigen::MatrixXd q_pseudo_inv_jacobian = Eigen::MatrixXd::Zero(rotor_num, ndof);
  Eigen::VectorXd pseudo_wrench = q_pseudo_inv.transpose() * static_thrust;
  for(int i = 0; i < rotor_num; i++)
    {
      if(wrench_dof == 6) // fully-actuated
        q_pseudo_inv_jacobian.row(i) = pseudo_wrench.transpose() * q_mat_jacobians.at(i);
      else // under-actuated
        q_pseudo_inv_jacobian.row(i) = pseudo_wrench.transpose() * q_mat_jacobians.at(i).middleRows(2, wrench_dof);
    }
  lambda_jacobian_ += (Eigen::MatrixXd::Identity(rotor_num, rotor_num) - q_pseudo_inv * q_mat) * q_pseudo_inv_jacobian;

  ROS_DEBUG_STREAM("lambda_jacobian: \n" << lambda_jacobian_);
}

void RobotModel::calcJointTorqueJacobian()
{
  const auto& sigma = getRotorDirection();
  const auto& inertia_map = getInertiaMap();
  const double m_f_rate = getMFRate();
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();
  const int ndof = lambda_jacobian_.cols();
  const auto gravity = getGravity();
  const auto& static_thrust =  getStaticThrust();
  const auto& thrust_wrench_units = getThrustWrenchUnits();

  joint_torque_jacobian_ = Eigen::MatrixXd::Zero(joint_num, ndof);

  // gravity
  for(const auto& inertia : inertia_map)
    {
      for (int j = 0; j < joint_num; ++j) {
        joint_torque_jacobian_.row(j) += inertia.second.getMass() * (-gravity.transpose()) * getSecondDerivative(inertia.first, j, inertia.second.getCOG());
      }
    }

  // thrust
  for (int i = 0; i < rotor_num; ++i) {
    Eigen::VectorXd wrench = thrust_wrench_units.at(i) * static_thrust(i);
    std::string thrust_name = std::string("thrust") + std::to_string(i + 1);

    for (int j = 0; j < joint_num; ++j) {
      joint_torque_jacobian_.row(j) += wrench.transpose() * getSecondDerivative(thrust_name, j);
    }

    Eigen::MatrixXd wrench_unit_jacobian = Eigen::MatrixXd::Zero(6, ndof);
    wrench_unit_jacobian.topRows(3) = -skew(thrust_wrench_units.at(i).head(3)) * thrust_coord_jacobians_.at(i).bottomRows(3);
    wrench_unit_jacobian.bottomRows(3) = -skew(thrust_wrench_units.at(i).tail(3)) * thrust_coord_jacobians_.at(i).bottomRows(3);

    joint_torque_jacobian_ += thrust_coord_jacobians_.at(i).rightCols(joint_num).transpose() * (wrench_unit_jacobian * static_thrust(i) + thrust_wrench_units.at(i) * lambda_jacobian_.row(i));
  }
  joint_torque_jacobian_ *= -1;
}



