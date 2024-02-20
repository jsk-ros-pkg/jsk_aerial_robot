#include <delta/control/delta_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

void RollingController::calcContactPoint()
{
  /* get realtime cog state */
  tf::Vector3 w_p_cog_in_w_tf = estimator_->getPos(Frame::COG, estimate_mode_);
  Eigen::Vector3d w_p_cog_in_w = Eigen::Vector3d(w_p_cog_in_w_tf.x(), w_p_cog_in_w_tf.y(), w_p_cog_in_w_tf.z());

  tf::Matrix3x3 w_R_cog_tf = estimator_->getOrientation(Frame::COG, estimate_mode_);
  Eigen::Matrix3d w_R_cog;
  for(int i = 0; i < 3; i++)
    {
      for(int j = 0; j < 3; j++)
        {
          if(j == 0) w_R_cog(i, j) = w_R_cog_tf.getRow(i).x();
          if(j == 1) w_R_cog(i, j) = w_R_cog_tf.getRow(i).y();
          if(j == 2) w_R_cog(i, j) = w_R_cog_tf.getRow(i).z();
        }
    }

  Eigen::Vector3d b1 = Eigen::Vector3d(1.0, 0.0, 0.0);
  Eigen::Vector3d b3 = Eigen::Vector3d(0.0, 0.0, 1.0);
  Eigen::Matrix3d rot_mat;
  std::vector<KDL::Frame> links_center_frame_from_cog = rolling_robot_model_->getLinksCenterFrameFromCog();
  double min_z_in_w = 100000;
  int min_index_i, min_index_j;

  /* serach lowest point in world frame */
  for(int i = 0; i < motor_num_; i++)
    {
      KDL::Frame link_i_center_frame_from_cog = links_center_frame_from_cog.at(i);
      Eigen::Vector3d cog_p_center_in_cog = aerial_robot_model::kdlToEigen(link_i_center_frame_from_cog.p);
      Eigen::Matrix3d cog_R_center = aerial_robot_model::kdlToEigen(link_i_center_frame_from_cog.M);
      Eigen::Vector3d w_p_center_in_w = w_p_cog_in_w + w_R_cog * cog_p_center_in_cog;
      for(int j = 30; j <= 150; j++)
        {
          rot_mat = Eigen::AngleAxisd(j / 180.0 * M_PI, b3);
          Eigen::Vector3d center_p_cp_in_center = circle_radius_ * rot_mat * b1;
          Eigen::Vector3d center_p_cp_in_w = w_R_cog * cog_R_center * center_p_cp_in_center;
          Eigen::Vector3d w_p_cp_in_w = w_p_center_in_w + center_p_cp_in_w;
          if(w_p_cp_in_w(2) < min_z_in_w)
            {
              min_z_in_w = w_p_cp_in_w(2);
              min_index_i = i;
              min_index_j = j;
            }
        }
    }

  /* set real contact point to robot model */
  rot_mat = Eigen::AngleAxisd(min_index_j / 180.0 * M_PI, b3);
  Eigen::Vector3d center_p_cp_in_center = circle_radius_ * rot_mat * b1;
  Eigen::Vector3d cog_p_cp_in_cog = aerial_robot_model::kdlToEigen(links_center_frame_from_cog.at(min_index_i).p) + aerial_robot_model::kdlToEigen(links_center_frame_from_cog.at(min_index_i).M) * center_p_cp_in_center;

  KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
  KDL::Frame contact_point_real_in_cog;
  contact_point_real_in_cog.p.x(cog_p_cp_in_cog(0));
  contact_point_real_in_cog.p.y(cog_p_cp_in_cog(1));
  contact_point_real_in_cog.p.z(cog_p_cp_in_cog(2));
  contact_point_real_in_cog.p = cog * contact_point_real_in_cog.p;
  contact_point_real_in_cog.M = cog.M;
  rolling_robot_model_->setContactPointReal(contact_point_real_in_cog);
}


void RollingController::standingPlanning()
{
  if(!start_rp_integration_)
    {
      start_rp_integration_ = true;
      spinal::FlightConfigCmd flight_config_cmd;
      flight_config_cmd.cmd = spinal::FlightConfigCmd::INTEGRATION_CONTROL_ON_CMD;
      navigator_->getFlightConfigPublisher().publish(flight_config_cmd);
      ROS_WARN_ONCE("start roll/pitch I control");
    }

  // if(standing_baselink_pitch_update_ && ground_navigation_mode_ == aerial_robot_navigation::STANDING_STATE)
  //   {
  //     if(ros::Time::now().toSec() - standing_baselink_ref_pitch_last_update_time_ > standing_baselink_ref_pitch_update_thresh_)
  //       {
  //         standing_baselink_ref_pitch_last_update_time_ = ros::Time::now().toSec();
  //         rolling_navigator_->setFinalTargetBaselinkRotPitch(baselink_pitch);
  //       }
  //   }

  if(ground_navigation_mode_ == aerial_robot_navigation::STANDING_STATE && fabs(pid_msg_.roll.err_p) < standing_baselink_roll_converged_thresh_)
    {
      rolling_navigator_->setGroundNavigationMode(aerial_robot_navigation::ROLLING_STATE);
      rosoutControlParams("rolling_controller");
    }

  double du = ros::Time::now().toSec() - rolling_control_timestamp_;
  if(!rolling_navigator_->getPitchAngVelUpdating())
    {
      double target_pitch = rolling_navigator_->getCurrentTargetBaselinkRpyPitch();
      rolling_navigator_->setCurrentTargetBaselinkRpyPitch(target_pitch);

      Eigen::Matrix3d rot_mat;
      Eigen::Vector3d b1 = Eigen::Vector3d(1.0, 0.0, 0.0), b2 = Eigen::Vector3d(0.0, 1.0, 0.0);
      rot_mat = Eigen::AngleAxisd(target_pitch, b2) * Eigen::AngleAxisd(M_PI / 2.0, b1);
      KDL::Rotation rot_mat_kdl = eigenToKdl(rot_mat);
      double qx, qy, qz, qw;
      rot_mat_kdl.GetQuaternion(qx, qy, qz, qw);

      if(ground_navigation_mode_ != aerial_robot_navigation::DOWN_STATE)
        {
          rolling_navigator_->setCurrentTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
          rolling_navigator_->setFinalTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
        }
      else
        {
          rolling_navigator_->setFinalTargetBaselinkQuat(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        }

      navigator_->setTargetOmegaY(0);
    }
  else
    {
      double target_pitch_ang_vel = rolling_navigator_->getTargetPitchAngVel();
      // rolling_navigator_->setCurrentTargetBaselinkRotPitch(target_baselink_pitch + du * target_pitch_ang_vel);

      double target_pitch = rolling_navigator_->getCurrentTargetBaselinkRpyPitch();
      if(fabs(pid_msg_.pitch.err_p) < 0.15)
        {
          target_pitch += du * target_pitch_ang_vel;
        }
      else
        {
          ROS_WARN_STREAM_THROTTLE(0.5, "[control] do not update target pitch until convergence");
        }
      rolling_navigator_->setCurrentTargetBaselinkRpyPitch(target_pitch);

      Eigen::Matrix3d rot_mat;
      Eigen::Vector3d b1 = Eigen::Vector3d(1.0, 0.0, 0.0), b2 = Eigen::Vector3d(0.0, 1.0, 0.0);
      rot_mat = Eigen::AngleAxisd(target_pitch, b2) * Eigen::AngleAxisd(M_PI / 2.0, b1);
      KDL::Rotation rot_mat_kdl = eigenToKdl(rot_mat);
      double qx, qy, qz, qw;
      rot_mat_kdl.GetQuaternion(qx, qy, qz, qw);

      if(ground_navigation_mode_ != aerial_robot_navigation::DOWN_STATE)
        {
          rolling_navigator_->setCurrentTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
          rolling_navigator_->setFinalTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
        }
      else
        {
          rolling_navigator_->setFinalTargetBaselinkQuat(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        }

      navigator_->setTargetOmegaY(target_pitch_ang_vel);

      // rpy_ = estimator_->getEuler(Frame::COG, estimate_mode_);
      // omega_ = estimator_->getAngularVel(Frame::COG, estimate_mode_);
      // target_rpy_ = navigator_->getTargetRPY();
      // target_omega_ = navigator_->getTargetOmega();

      tf::Quaternion cog2baselink_rot;
      tf::quaternionKDLToTF(robot_model_->getCogDesireOrientation<KDL::Rotation>(), cog2baselink_rot);
      tf::Matrix3x3 cog_rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_) * tf::Matrix3x3(cog2baselink_rot).inverse();
      double r, p, y; cog_rot.getRPY(r, p, y);
      rpy_.setValue(r, p, y);

      omega_ = estimator_->getAngularVel(Frame::COG, estimate_mode_);
      target_rpy_ = navigator_->getTargetRPY();
      tf::Matrix3x3 target_rot; target_rot.setRPY(target_rpy_.x(), target_rpy_.y(), target_rpy_.z());
      tf::Vector3 target_omega = navigator_->getTargetOmega(); // w.r.t. target cog frame
      target_omega_ = cog_rot.inverse() * target_rot * target_omega; // w.r.t. current cog frame

      pid_controllers_.at(PITCH).update(target_rpy_.y() - rpy_.y(), du, target_omega_.y() - omega_.y());
    }

  rolling_control_timestamp_ = ros::Time::now().toSec();

  pid_msg_.roll.total.at(0) = pid_controllers_.at(ROLL).result();
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = pid_controllers_.at(PITCH).result();
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

}

void RollingController::calcStandingFullLambda()
{
  int n_variables = 2 * motor_num_;
  int n_constraints = 3 + 1 + 2 + 2;

  int standing_mode_n_constraints = n_constraints;
  if(ground_navigation_mode_ == aerial_robot_navigation::ROLLING_STATE)
    {
      n_constraints += 3;
    }

  double epsilon = 0.0001;

  Eigen::MatrixXd H(n_variables, n_variables);
  H <<
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  Eigen::SparseMatrix<double> H_s;
  H_s = H.sparseView();

  /* normal pid result */
  Eigen::VectorXd target_wrench_acc_target_frame;
  target_wrench_acc_target_frame.resize(6);
  target_wrench_acc_target_frame.tail(3) = Eigen::Vector3d(pid_controllers_.at(ROLL).result(),
                                                           pid_controllers_.at(PITCH).result(),
                                                           pid_controllers_.at(YAW).result());

  /* calculate gravity compensation term based on realtime orientation */
  KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
  KDL::Frame contact_point_alined_to_cog = contact_point_alined_.Inverse() * cog;
  Eigen::Vector3d contact_point_alined_to_cog_p = aerial_robot_model::kdlToEigen(contact_point_alined_to_cog.p);
  Eigen::Matrix3d contact_point_alined_to_cog_p_skew = aerial_robot_model::skew(contact_point_alined_to_cog_p);
  Eigen::VectorXd gravity = robot_model_->getGravity3d();
  Eigen::Vector3d gravity_ang_acc_from_contact_point_alined = rolling_robot_model_->getInertiaFromTargetFrame<Eigen::Matrix3d>().inverse() * contact_point_alined_to_cog_p_skew * robot_model_->getMass() * gravity;

  /* use sum of pid result and gravity compensation torque for attitude control */
  target_wrench_acc_target_frame.tail(3) = target_wrench_acc_target_frame.tail(3) + gravity_compensate_ratio_ * gravity_ang_acc_from_contact_point_alined;
  gravity_compensate_term_ = gravity_compensate_ratio_ * gravity_ang_acc_from_contact_point_alined;

  Eigen::MatrixXd full_q_mat = rolling_robot_model_->getFullWrenchAllocationMatrixFromControlFrame();
  Eigen::MatrixXd full_q_mat_trans = full_q_mat.topRows(3);
  Eigen::MatrixXd full_q_mat_rot = full_q_mat.bottomRows(3);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_constraints, n_variables);
  A.topRows(3) = full_q_mat_rot;                                                           //    eq constraint about rpy torque
  A.block(3, 0, 1, n_variables) = full_q_mat_trans.row(Z);                                           // in eq constraint about z
  A.block(4, 0, 1, n_variables) = full_q_mat_trans.row(X) - steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about x
  A.block(5, 0, 1, n_variables) = full_q_mat_trans.row(X) + steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about x
  A.block(6, 0, 1, n_variables) = full_q_mat_trans.row(Y) - steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about y
  A.block(7, 0, 1, n_variables) = full_q_mat_trans.row(Y) + steering_mu_ * full_q_mat_trans.row(Z);  // in eq constraint about y

  if(ground_navigation_mode_ == aerial_robot_navigation::ROLLING_STATE)
    {
      A.block(8, 0, 1, n_variables)  << 1, 0, 0, 0, 0, 0;
      A.block(9, 0, 1, n_variables)  << 0, 0, 1, 0, 0, 0;
      A.block(10, 0, 1, n_variables) << 0, 0, 0, 0, 1, 0;
    }

  Eigen::SparseMatrix<double> A_s;
  A_s = A.sparseView();
  Eigen::VectorXd gradient = Eigen::VectorXd::Ones(n_variables);

  Eigen::VectorXd lower_bound(n_constraints);
  Eigen::VectorXd upper_bound(n_constraints);

  lower_bound.head(standing_mode_n_constraints)
    <<
    target_wrench_acc_target_frame(ROLL) - epsilon,
    target_wrench_acc_target_frame(PITCH) - epsilon,
    target_wrench_acc_target_frame(YAW) - epsilon,
    -INFINITY,
    -steering_mu_ * robot_model_->getGravity()(Z),
    -INFINITY,
    -steering_mu_ * robot_model_->getGravity()(Z),
    -INFINITY;

  upper_bound.head(standing_mode_n_constraints)
    <<
    target_wrench_acc_target_frame(ROLL) + epsilon,
    target_wrench_acc_target_frame(PITCH) + epsilon,
    target_wrench_acc_target_frame(YAW) + epsilon,
    robot_model_->getGravity()(Z),
    INFINITY,
    steering_mu_ * robot_model_->getGravity()(Z),
    INFINITY,
    steering_mu_ * robot_model_->getGravity()(Z);

  if(ground_navigation_mode_ == aerial_robot_navigation::ROLLING_STATE)
    {
      for(int i = 0; i < motor_num_; i++)
        {
          lower_bound(standing_mode_n_constraints + i) = -robot_model_->getThrustUpperLimit();
          upper_bound(standing_mode_n_constraints + i) = -fabs(rolling_minimum_lateral_force_);
        }
    }

  OsqpEigen::Solver solver;

  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);
  solver.data()->setNumberOfVariables(n_variables);
  solver.data()->setNumberOfConstraints(n_constraints);
  solver.data()->setHessianMatrix(H_s);
  solver.data()->setLinearConstraintsMatrix(A_s);
  solver.data()->setGradient(gradient);
  solver.data()->setLowerBound(lower_bound);
  solver.data()->setUpperBound(upper_bound);

  if(!solver.initSolver())
    {
      ROS_ERROR("[control][OSQP] init solver error!");
    }

  if(!solver.solve())
    {
      ROS_WARN_STREAM("[control][OSQP] could not reach the solution.");
      // rolling_navigator_->setGroundNavigationMode(aerial_robot_navigation::STANDING_STATE);
    }
  else
    {
      auto solution = solver.getSolution();

      full_lambda_all_ = solution;
      full_lambda_trans_ = solution;
    }
}
