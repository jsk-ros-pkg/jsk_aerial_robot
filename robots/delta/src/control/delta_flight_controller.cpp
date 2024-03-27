#include <delta/control/delta_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

void RollingController::calcAccFromCog()
{
  control_dof_ = std::accumulate(controlled_axis_.begin(), controlled_axis_.end(), 0);

  tf::Quaternion cog2baselink_rot;
  tf::quaternionKDLToTF(robot_model_->getCogDesireOrientation<KDL::Rotation>(), cog2baselink_rot);
  tf::Matrix3x3 cog_rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_) * tf::Matrix3x3(cog2baselink_rot).inverse();

  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_cog = cog_rot.inverse() * target_acc_w;
  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;

  target_acc_cog_.at(0) = target_acc_cog.x();
  target_acc_cog_.at(1) = target_acc_cog.y();
  target_acc_cog_.at(2) = target_acc_cog.z();

  target_acc_dash_.at(0) = target_acc_dash.x();
  target_acc_dash_.at(1) = target_acc_dash.y();
  target_acc_dash_.at(2) = target_acc_dash.z();

  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);
  if(controlled_axis_.at(0) == 0 || controlled_axis_.at(1) == 0)
    {
      target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_dash.x(), target_acc_dash.y(), target_acc_dash.z());
    }
  else
    {
      target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());
    }

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();

  target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);
  target_wrench_acc_cog_ = target_wrench_acc_cog;

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

  if(navigator_->getForceLandingFlag() && target_acc_w.z() < 5.0) // heuristic measures to avoid to large gimbal angles after force land
    start_rp_integration_ = false;

  if(!controlled_axis_.at(X))
    {
      if(hovering_approximate_)
        {
          target_pitch_ = target_acc_dash.x() / aerial_robot_estimation::G;
        }
      else
        {
          target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
        }
    }
  if(!controlled_axis_.at(Y))
    {
      if(hovering_approximate_)
        {
          target_roll_ = -target_acc_dash.y() / aerial_robot_estimation::G;
        }
      else
        {
          target_roll_ = atan2(-target_acc_dash.y(), sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));
        }
    }
}

void RollingController::calcFlightFullLambda()
{
  /* calculate integarated allocation */
  Eigen::MatrixXd full_q_mat = rolling_robot_model_->getFullWrenchAllocationMatrixFromControlFrame("cog");

  /* change dimension from wrench to acc */
  full_q_mat.topRows(3) = 1.0 / robot_model_->getMass() * full_q_mat.topRows(3);
  full_q_mat.bottomRows(3) = rolling_robot_model_->getInertiaFromTargetFrame<Eigen::Matrix3d>().inverse() * full_q_mat.bottomRows(3);

  /* extract controlled axis */
  Eigen::MatrixXd controlled_axis_mask = Eigen::MatrixXd::Zero(control_dof_, 6);
  int last_row = 0;
  for(int i = 0; i < controlled_axis_.size(); i++)
    {
      if(controlled_axis_.at(i))
        {
          controlled_axis_mask(last_row, i) = 1;
          last_row++;
        }
    }

  Eigen::MatrixXd controlled_q_mat = controlled_axis_mask * full_q_mat;
  Eigen::MatrixXd controlled_q_mat_inv = aerial_robot_model::pseudoinverse(controlled_q_mat);
  Eigen::VectorXd controlled_wrench_acc_cog = controlled_axis_mask * target_wrench_acc_cog_;

  if(use_sr_inv_)
    {
      // http://www.thothchildren.com/chapter/5bd8d78751d930518903af34
      Eigen::MatrixXd sr_inv = controlled_q_mat.transpose() * (controlled_q_mat * controlled_q_mat.transpose() + sr_inv_weight_ * Eigen::MatrixXd::Identity(controlled_q_mat.rows(), controlled_q_mat.rows())).inverse();
      controlled_q_mat_inv = sr_inv;
      ROS_WARN_STREAM_ONCE("[control] use SR-Inverse. weight is " << sr_inv_weight_);
    }
  else
    {
      ROS_WARN_ONCE("[control] use MP-Inverse");
    }

  /* actuator mapping */
  int rot_dof = 0;
  for(int i = 3; i < 6; i++)
    {
      if(controlled_axis_.at(i))
        {
          rot_dof++;
        }
    }

  full_lambda_trans_ = controlled_q_mat_inv.leftCols(control_dof_ - rot_dof) * controlled_wrench_acc_cog.head(control_dof_ - rot_dof);
  full_lambda_rot_ = controlled_q_mat_inv.rightCols(rot_dof) * controlled_wrench_acc_cog.tail(rot_dof);
  full_lambda_all_ = full_lambda_trans_ + full_lambda_rot_;
}
