#include <delta/control/delta_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

void RollingController::calcAccFromCog()
{
  tf::Quaternion cog2baselink_rot;
  tf::quaternionKDLToTF(robot_model_->getCogDesireOrientation<KDL::Rotation>(), cog2baselink_rot);
  tf::Matrix3x3 cog_rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_) * tf::Matrix3x3(cog2baselink_rot).inverse();

  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_cog = cog_rot.inverse() * target_acc_w;

  target_acc_cog_.at(0) = target_acc_cog.x();
  target_acc_cog_.at(1) = target_acc_cog.y();
  target_acc_cog_.at(2) = target_acc_cog.z();

  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);

  target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();

  Eigen::Vector3d omega;
  tf::vectorTFToEigen(omega_, omega);
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  Eigen::Vector3d gyro = omega.cross(inertia * omega);

  target_wrench_acc_cog.tail(3)
    = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z)
    + inertia.inverse() * gyro;

  target_wrench_acc_cog_ = target_wrench_acc_cog;

  if(navigator_->getForceLandingFlag() && target_acc_w.z() < 5.0) // heuristic measures to avoid to large gimbal angles after force land
    start_rp_integration_ = false;
}

void RollingController::calcFlightFullLambda()
{
  /* calculate integarated allocation */
  Eigen::MatrixXd full_q_mat = rolling_robot_model_->getFullWrenchAllocationMatrixFromControlFrame("cog");

  /* change dimension from wrench to acc */
  full_q_mat.topRows(3) = 1.0 / robot_model_->getMass() * full_q_mat.topRows(3);
  full_q_mat.bottomRows(3) = rolling_robot_model_->getInertiaFromTargetFrame<Eigen::Matrix3d>().inverse() * full_q_mat.bottomRows(3);

  Eigen::MatrixXd full_q_mat_inv = aerial_robot_model::pseudoinverse(full_q_mat);

  if(use_sr_inv_)
    {
      // http://www.thothchildren.com/chapter/5bd8d78751d930518903af34
      Eigen::MatrixXd sr_inv = full_q_mat.transpose() * (full_q_mat * full_q_mat.transpose() + sr_inv_weight_ * Eigen::MatrixXd::Identity(full_q_mat.rows(), full_q_mat.rows())).inverse();
      full_q_mat_inv = sr_inv;
      ROS_WARN_STREAM_ONCE("[control] use SR-Inverse. weight is " << sr_inv_weight_);
    }
  else
    {
      ROS_WARN_ONCE("[control] use MP-Inverse");
    }

  full_lambda_trans_ = full_q_mat_inv.leftCols(3) * target_wrench_acc_cog_.head(3);
  full_lambda_rot_ = full_q_mat_inv.rightCols(3) * target_wrench_acc_cog_.tail(3);
  full_lambda_all_ = full_lambda_trans_ + full_lambda_rot_;
}
