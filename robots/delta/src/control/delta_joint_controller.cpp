#include <delta/control/delta_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

void RollingController::jointTorquePreComputation()
{
  /* assume robot_model_for_control_ is updated with current cog_desire_orientation and joint_positions */
  const int joint_num = robot_model_for_control_->getJointNum();
  joint_torque_ = Eigen::VectorXd::Zero(joint_num); // gimbal1, joint1, gimbal2, joint2, gimbal3
  const KDL::JntArray& joint_positions = robot_model_->getJointPositions();

  /* calculate jacobians of gimbal neutral coordinate */
  for(int i = 0; i < motor_num_; i++)
    {
      gimbal_link_jacobians_.at(i) = robot_model_for_control_->getJacobian(joint_positions, std::string("gimbal_link") + std::to_string(i + 1));
    }

  /* calculate coord jacobians for cog point and convert to joint torque */
  const auto& inertia_map = robot_model_for_control_->getInertiaMap();
  const auto gravity = robot_model_for_control_->getGravity();
  for(const auto& inertia : inertia_map)
    {
      Eigen::MatrixXd cog_coord_jacobian = robot_model_for_control_->getJacobian(joint_positions, inertia.first, inertia.second.getCOG());
      joint_torque_ -= cog_coord_jacobian.rightCols(joint_num).transpose() * inertia.second.getMass() * (-gravity);
    }

  /* contact force if ground mode */
  int contacting_link_index = rolling_robot_model_->getContactingLink();
  std::string contacting_link_name = std::string("link") + std::to_string(contacting_link_index + 1);
  KDL::Frame contacting_link_frame = robot_model_for_control_->getSegmentTf(contacting_link_name);
  KDL::Frame contact_point = rolling_robot_model_->getContactPoint<KDL::Frame>();
  KDL::Frame contacting_link_to_contact_point = contacting_link_frame.Inverse() * contact_point;
  Eigen::MatrixXd contact_point_jacobian = robot_model_for_control_->getJacobian(joint_positions, contacting_link_name, contacting_link_to_contact_point.p);

  contact_point_jacobian_ = contact_point_jacobian;

  if(ground_navigation_mode_ == aerial_robot_navigation::STANDING_STATE || ground_navigation_mode_ == aerial_robot_navigation::ROLLING_STATE || ground_navigation_mode_ == aerial_robot_navigation::DOWN_STATE)
    {
      if(getUseEstimatedExternalForce())
        joint_torque_ -= contact_point_jacobian.rightCols(joint_num).transpose() * est_external_wrench_cog_;
    }
}
