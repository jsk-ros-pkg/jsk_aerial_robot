#include <delta/control/delta_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

void RollingController::calcAccFromCog()
{
  control_dof_ = std::accumulate(controlled_axis_.begin(), controlled_axis_.end(), 0);

  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
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
          navigator_->setTargetPitch(target_pitch_);
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
      navigator_->setTargetRoll(target_roll_);
    }
}

void RollingController::calcWrenchAllocationMatrix()
{
  /* calculate integarated allocation */
  full_q_mat_ = rolling_robot_model_->getFullWrenchAllocationMatrixFromControlFrame();
  full_q_trans_ = full_q_mat_.topRows(3);
  full_q_rot_ = full_q_mat_.bottomRows(3);

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

  controlled_q_mat_ = controlled_axis_mask * full_q_mat_;
  controlled_q_mat_inv_ = aerial_robot_model::pseudoinverse(controlled_q_mat_);
  controlled_wrench_acc_cog_ = controlled_axis_mask * target_wrench_acc_cog_;

  if(use_sr_inv_)
    {
      // http://www.thothchildren.com/chapter/5bd8d78751d930518903af34
      Eigen::MatrixXd sr_inv = controlled_q_mat_.transpose() * (controlled_q_mat_ * controlled_q_mat_.transpose() + sr_inv_weight_ * Eigen::MatrixXd::Identity(controlled_q_mat_.rows(), controlled_q_mat_.rows())).inverse();
      controlled_q_mat_inv_ = sr_inv;
      ROS_WARN_STREAM_ONCE("[control] use SR-Inverse. weight is " << sr_inv_weight_);
    }
  else
    {
      ROS_WARN_ONCE("[control] use MP-Inverse");
    }
}

void RollingController::calcFullLambda()
{
  /* actuator mapping */
  int rot_dof = 0;
  for(int i = 3; i < 6; i++)
    {
      if(controlled_axis_.at(i))
        {
          rot_dof++;
        }
    }
  full_lambda_trans_ = controlled_q_mat_inv_.leftCols(control_dof_ - rot_dof) * controlled_wrench_acc_cog_.head(control_dof_ - rot_dof);
  full_lambda_rot_ = controlled_q_mat_inv_.rightCols(rot_dof) * controlled_wrench_acc_cog_.tail(rot_dof);
  full_lambda_all_ = full_lambda_trans_ + full_lambda_rot_;
}

void RollingController::wrenchAllocation()
{
  int last_col = 0;
  for(int i = 0; i < motor_num_; i++)
    {
      Eigen::VectorXd full_lambda_trans_i = full_lambda_trans_.segment(last_col, 2);
      Eigen::VectorXd full_lambda_all_i = full_lambda_all_.segment(last_col, 2);

      /* calculate base thrusts */
      target_base_thrust_.at(i) = full_lambda_trans_i.norm() / fabs(cos(rotor_tilt_.at(i)));

      /* calculate gimbal angles */
      double gimbal_angle_i = atan2(-full_lambda_all_i(0), full_lambda_all_i(1));
      target_gimbal_angles_.at(i) = (gimbal_lpf_factor_ - 1.0) / gimbal_lpf_factor_ * prev_target_gimbal_angles_.at(i) + 1.0 / gimbal_lpf_factor_ * gimbal_angle_i;

      /* solve round offset */
      if(fabs(target_gimbal_angles_.at(i) - current_gimbal_angles_.at(i)) > M_PI)
        {
          bool converge_flag = false;
          double gimbal_candidate_plus = target_gimbal_angles_.at(i);
          double gimbal_candidate_minus = target_gimbal_angles_.at(i);
          while(!converge_flag)
            {
              gimbal_candidate_plus += 2 * M_PI;
              gimbal_candidate_minus -= 2 * M_PI;
              if(fabs(current_gimbal_angles_.at(i) - gimbal_candidate_plus) < M_PI)
                {
                  ROS_WARN_STREAM("[control] send angle " << gimbal_candidate_plus << " for gimbal" << i << " instead of " << target_gimbal_angles_.at(i));
                  target_gimbal_angles_.at(i) = gimbal_candidate_plus;
                  converge_flag = true;
                }
              else if(fabs(current_gimbal_angles_.at(i) - gimbal_candidate_minus) < M_PI)
                {
                  ROS_WARN_STREAM("[control] send angle " << gimbal_candidate_minus << " for gimbal" << i << " instead of " << target_gimbal_angles_.at(i));
                  target_gimbal_angles_.at(i) = gimbal_candidate_minus;
                  converge_flag = true;
                }
            }
        }

      ROS_WARN_STREAM_ONCE("[control] gimbal lpf factor: " << gimbal_lpf_factor_);

      last_col += 2;
    }

  /* update robot model by calculated gimbal angle */
  const auto& joint_index_map = robot_model_->getJointIndexMap();
  KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_control_->setCogDesireOrientation(cog_desire_orientation);
  KDL::JntArray gimbal_processed_joint = robot_model_->getJointPositions();
  for(int i = 0; i < motor_num_; i++)
    {
      std::string s = std::to_string(i + 1);
      if(realtime_gimbal_allocation_)
        {
          ROS_WARN_STREAM_ONCE("[control] use actual gimbal angle for realtime allocation");
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s)->second) = current_gimbal_angles_.at(i);
        }
      else
        {
          ROS_WARN_STREAM_ONCE("[control] use target gimbal angle for realtime allocation");
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s)->second) = target_gimbal_angles_.at(i);
        }
    }
  robot_model_for_control_->updateRobotModel(gimbal_processed_joint);

  /* calculate allocation matrix for realtime control */
  q_mat_ = robot_model_for_control_->calcWrenchMatrixOnCoG();
  q_mat_inv_ = aerial_robot_model::pseudoinverse(q_mat_);
}

