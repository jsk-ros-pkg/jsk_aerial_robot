//
// Created by lijinjie on 23/11/29.
//

#include "aerial_robot_control/nmpc/over_act_full/nmpc_controller.h"

using namespace aerial_robot_control;

nmpc_over_act_full::NMPCController::NMPCController()
{
}

void nmpc_over_act_full::NMPCController::initialize(
    ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du)
{
  ControlBase::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  ros::NodeHandle control_nh(nh_, "controller");

  // initialize nmpc solver
  mpc_solver_.initialize();

  getParam<double>(control_nh, "nmpc/T_samp", t_nmpc_samp_, 0.025);
  getParam<double>(control_nh, "nmpc/T_integ", t_nmpc_integ_, 0.1);

  getParam<double>(control_nh, "physical/mass", mass_, 0.5);
  getParam<double>(control_nh, "physical/gravity_const", gravity_const_, 9.81);

  getParam<bool>(control_nh, "nmpc/is_attitude_ctrl", is_attitude_ctrl_, true);
  getParam<bool>(control_nh, "nmpc/is_body_rate_ctrl", is_body_rate_ctrl_, false);
  getParam<bool>(control_nh, "nmpc/is_debug", is_debug_, false);

  /* timers */
  tmr_viz_ = nh_.createTimer(ros::Duration(0.05), &NMPCController::callbackViz, this);

  /* publishers */
  pub_viz_pred_ = nh_.advertise<geometry_msgs::PoseArray>("nmpc/viz_pred", 1);
  pub_viz_ref_ = nh_.advertise<geometry_msgs::PoseArray>("nmpc/viz_ref", 1);
  pub_flight_cmd_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  pub_gimbal_control_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);

  pub_rpy_gain_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);  // tmp
  pub_p_matrix_pseudo_inverse_inertia_ =
      nh_.advertise<spinal::PMatrixPseudoInverseWithInertia>("p_matrix_pseudo_inverse_inertia", 1);  // tmp

  /* services */
  srv_set_control_mode_ = nh_.serviceClient<spinal::SetControlMode>("set_control_mode");

  /* subscribers */
  sub_joint_states_ = nh_.subscribe("joint_states", 5, &NMPCController::callbackJointStates, this);
  sub_set_rpy_ = nh_.subscribe("set_rpy", 5, &NMPCController::callbackSetRPY, this);
  sub_set_ref_traj_ = nh_.subscribe("set_ref_traj", 5, &NMPCController::callbackSetRefTraj, this);

  reset();

  // set control mode
  spinal::SetControlMode set_control_mode_srv;
  set_control_mode_srv.request.is_attitude = is_attitude_ctrl_;
  set_control_mode_srv.request.is_body_rate = is_body_rate_ctrl_;
  while (!srv_set_control_mode_.call(set_control_mode_srv))
    ROS_WARN_THROTTLE(1,
                      "Waiting for set_control_mode service.... If you always see this message, the robot cannot fly.");

  ROS_INFO("Set control mode: attitude = %d and body rate = %d", set_control_mode_srv.request.is_attitude,
           set_control_mode_srv.request.is_body_rate);

  ROS_INFO("MPC Controller initialized!");

  // print physical parameters if needed
  bool is_print_physical_params;
  getParam(control_nh, "nmpc/is_print_physical_params", is_print_physical_params, false);
  if (is_print_physical_params)
    printPhysicalParams();
}

bool nmpc_over_act_full::NMPCController::update()
{
  if (!ControlBase::update())
    return false;

  /* TODO: these code should be initialized in init(). put here because of beetle's slow parameter init */
  if (!is_init_alloc_mat_)
  {
    is_init_alloc_mat_ = true;
    alloc_mat_.setZero();
    initAllocMat();
  }

  this->controlCore();
  this->SendCmd();

  return true;
}

void nmpc_over_act_full::NMPCController::reset()
{
  ControlBase::reset();

  sendRPYGain();                // tmp for angular gains  TODO: change to angular gains only
  sendRotationalInertiaComp();  // tmp for inertia

  /* reset controller using odom */
  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Vector3 vel = estimator_->getVel(Frame::COG, estimate_mode_);
  tf::Vector3 rpy = estimator_->getEuler(Frame::COG, estimate_mode_);
  tf::Vector3 omega = estimator_->getAngularVel(Frame::COG, estimate_mode_);
  tf::Quaternion q;
  q.setRPY(rpy.x(), rpy.y(), rpy.z());

  double x[NX] = {
    pos.x(),
    pos.y(),
    pos.z(),
    vel.x(),
    vel.y(),
    vel.z(),
    q.w(),
    q.x(),
    q.y(),
    q.z(),
    omega.x(),
    omega.y(),
    omega.z(),
    joint_angles_[0],
    joint_angles_[1],
    joint_angles_[2],
    joint_angles_[3],
  };
  double u[NU] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };  // initial guess = zero seems to be better!
  initPredXU(x_u_ref_);
  for (int i = 0; i < NN; i++)
  {
    std::copy(x, x + NX, x_u_ref_.x.data.begin() + NX * i);
    std::copy(u, u + NU, x_u_ref_.u.data.begin() + NU * i);
  }
  std::copy(x, x + NX, x_u_ref_.x.data.begin() + NX * NN);

  mpc_solver_.reset(x_u_ref_);

  /* reset flight_cmd, only focus on the thrust cmd to prevent sudden change when taking off */
  flight_cmd_.base_thrust = std::vector<float>(motor_num_, 0.0);
}

void nmpc_over_act_full::NMPCController::controlCore()
{
  if (!is_traj_tracking_)
  {
    /* point mode --> set target  */
    tf::Vector3 target_pos = navigator_->getTargetPos();
    tf::Vector3 target_vel = navigator_->getTargetVel();
    tf::Vector3 target_rpy = navigator_->getTargetRPY();
    tf::Vector3 target_omega = navigator_->getTargetOmega();

    /* calc target wrench in the body frame */
    Eigen::VectorXd fg_i = Eigen::VectorXd::Zero(3);
    fg_i(2) = mass_ * gravity_const_;

    // coordinate transformation
    tf::Quaternion q;
    q.setRPY(target_rpy.x(), target_rpy.y(), target_rpy.z());
    tf::Quaternion q_inv = q.inverse();

    Eigen::Matrix3d rot;
    tf::matrixTFToEigen(tf::Transform(q_inv).getBasis(), rot);
    Eigen::VectorXd fg_b = rot * fg_i;

    Eigen::VectorXd target_wrench(6);
    target_wrench << fg_b(0), fg_b(1), fg_b(2), 0, 0, 0;

    calXrUrRef(target_pos, target_vel, target_rpy, target_omega, target_wrench);
  }

  /* get odom information */
  nav_msgs::Odometry odom_now = getOdom();

  /* solve */
  mpc_solver_.solve(odom_now, joint_angles_, x_u_ref_, is_debug_);

  /* get result */
  // - body rates
  double target_roll_rate = (t_nmpc_samp_ * mpc_solver_.x_u_out_.x.data.at(10) +
                             (t_nmpc_integ_ - t_nmpc_samp_) * mpc_solver_.x_u_out_.x.data.at(10 + NX)) /
                            t_nmpc_integ_;
  double target_pitch_rate = (t_nmpc_samp_ * mpc_solver_.x_u_out_.x.data.at(11) +
                              (t_nmpc_integ_ - t_nmpc_samp_) * mpc_solver_.x_u_out_.x.data.at(11 + NX)) /
                             t_nmpc_integ_;
  double target_yaw_rate = (t_nmpc_samp_ * mpc_solver_.x_u_out_.x.data.at(12) +
                            (t_nmpc_integ_ - t_nmpc_samp_) * mpc_solver_.x_u_out_.x.data.at(12 + NX)) /
                           t_nmpc_integ_;
  flight_cmd_.body_rates[0] = static_cast<float>(target_roll_rate);
  flight_cmd_.body_rates[1] = static_cast<float>(target_pitch_rate);
  flight_cmd_.body_rates[2] = static_cast<float>(target_yaw_rate);

  // - thrust
  double ft1 = mpc_solver_.x_u_out_.u.data.at(0);
  double ft2 = mpc_solver_.x_u_out_.u.data.at(1);
  double ft3 = mpc_solver_.x_u_out_.u.data.at(2);
  double ft4 = mpc_solver_.x_u_out_.u.data.at(3);

  Eigen::VectorXd target_thrusts(4);
  target_thrusts << ft1, ft2, ft3, ft4;

  // constraint the change of thrust, preventing sudden thrust increasing during taking off
  if (navigator_->getNaviState() == aerial_robot_navigation::TAKEOFF_STATE)
  {
    double thrust_scale_ratio = 1.0;
    float permitted_max_thrust_change = 10.0 / 40.0;
    double real_max_thrust_change = 0.0;
    for (int i = 0; i < motor_num_; i++)
    {
      double thrust_change = fabs(target_thrusts(i) - flight_cmd_.base_thrust[i]);
      if (thrust_change > real_max_thrust_change)
        real_max_thrust_change = thrust_change;
    }
    if (real_max_thrust_change > permitted_max_thrust_change)
      thrust_scale_ratio = permitted_max_thrust_change / real_max_thrust_change;

    // scale the thrust
    for (int i = 0; i < motor_num_; i++)
    {
      flight_cmd_.base_thrust[i] = static_cast<float>(
          flight_cmd_.base_thrust[i] + (target_thrusts(i) - flight_cmd_.base_thrust[i]) * thrust_scale_ratio);
    }
  }
  else
  {
    for (int i = 0; i < motor_num_; i++)
    {
      flight_cmd_.base_thrust[i] = static_cast<float>(target_thrusts(i));
    }
  }

  // - servo angle
  double a1c = mpc_solver_.x_u_out_.u.data.at(4);
  double a2c = mpc_solver_.x_u_out_.u.data.at(5);
  double a3c = mpc_solver_.x_u_out_.u.data.at(6);
  double a4c = mpc_solver_.x_u_out_.u.data.at(7);

  gimbal_ctrl_cmd_.header.stamp = ros::Time::now();
  gimbal_ctrl_cmd_.position.clear();
  gimbal_ctrl_cmd_.position.push_back(a1c);
  gimbal_ctrl_cmd_.position.push_back(a2c);
  gimbal_ctrl_cmd_.position.push_back(a3c);
  gimbal_ctrl_cmd_.position.push_back(a4c);
}

void nmpc_over_act_full::NMPCController::SendCmd()
{
  pub_flight_cmd_.publish(flight_cmd_);
  pub_gimbal_control_.publish(gimbal_ctrl_cmd_);
}

nav_msgs::Odometry nmpc_over_act_full::NMPCController::getOdom()
{
  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Vector3 vel = estimator_->getVel(Frame::COG, estimate_mode_);
  tf::Vector3 rpy = estimator_->getEuler(Frame::COG, estimate_mode_);
  tf::Vector3 omega = estimator_->getAngularVel(Frame::COG, estimate_mode_);
  tf::Quaternion q;
  q.setRPY(rpy.x(), rpy.y(), rpy.z());

  nav_msgs::Odometry odom;
  odom.pose.pose.position.x = pos.x();
  odom.pose.pose.position.y = pos.y();
  odom.pose.pose.position.z = pos.z();
  odom.pose.pose.orientation.w = q.w();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.twist.twist.linear.x = vel.x();
  odom.twist.twist.linear.y = vel.y();
  odom.twist.twist.linear.z = vel.z();
  odom.twist.twist.angular.x = omega.x();
  odom.twist.twist.angular.y = omega.y();
  odom.twist.twist.angular.z = omega.z();

  return odom;
}

/**
 * @brief callbackViz: publish the predicted trajectory and reference trajectory
 * @param [ros::TimerEvent&] event
 */
void nmpc_over_act_full::NMPCController::callbackViz(const ros::TimerEvent& event)
{
  // from mpc_solver_.x_u_out to PoseArray
  geometry_msgs::PoseArray pred_poses;
  geometry_msgs::PoseArray ref_poses;

  for (int i = 0; i < NN; ++i)
  {
    geometry_msgs::Pose pred_pose;
    pred_pose.position.x = mpc_solver_.x_u_out_.x.data.at(i * NX);
    pred_pose.position.y = mpc_solver_.x_u_out_.x.data.at(i * NX + 1);
    pred_pose.position.z = mpc_solver_.x_u_out_.x.data.at(i * NX + 2);
    pred_pose.orientation.w = mpc_solver_.x_u_out_.x.data.at(i * NX + 6);
    pred_pose.orientation.x = mpc_solver_.x_u_out_.x.data.at(i * NX + 7);
    pred_pose.orientation.y = mpc_solver_.x_u_out_.x.data.at(i * NX + 8);
    pred_pose.orientation.z = mpc_solver_.x_u_out_.x.data.at(i * NX + 9);
    pred_poses.poses.push_back(pred_pose);

    geometry_msgs::Pose ref_pose;
    ref_pose.position.x = x_u_ref_.x.data.at(i * NX);
    ref_pose.position.y = x_u_ref_.x.data.at(i * NX + 1);
    ref_pose.position.z = x_u_ref_.x.data.at(i * NX + 2);
    ref_pose.orientation.w = x_u_ref_.x.data.at(i * NX + 6);
    ref_pose.orientation.x = x_u_ref_.x.data.at(i * NX + 7);
    ref_pose.orientation.y = x_u_ref_.x.data.at(i * NX + 8);
    ref_pose.orientation.z = x_u_ref_.x.data.at(i * NX + 9);
    ref_poses.poses.push_back(ref_pose);
  }

  pred_poses.header.frame_id = "world";
  pred_poses.header.stamp = ros::Time::now();
  pub_viz_pred_.publish(pred_poses);

  ref_poses.header.frame_id = "world";
  ref_poses.header.stamp = ros::Time::now();
  pub_viz_ref_.publish(ref_poses);
}

void nmpc_over_act_full::NMPCController::callbackJointStates(const sensor_msgs::JointStateConstPtr& msg)
{
  joint_angles_[0] = msg->position[0];
  joint_angles_[1] = msg->position[1];
  joint_angles_[2] = msg->position[2];
  joint_angles_[3] = msg->position[3];
}

/* TODO: this function is just for test. We may need a more general function to set all kinds of state */
void nmpc_over_act_full::NMPCController::callbackSetRPY(const spinal::DesireCoordConstPtr& msg)
{
  navigator_->setTargetRoll(msg->roll);
  navigator_->setTargetPitch(msg->pitch);
  navigator_->setTargetYaw(msg->yaw);
}

void nmpc_over_act_full::NMPCController::callbackSetRefTraj(const aerial_robot_msgs::PredXUConstPtr& msg)
{
  x_u_ref_ = *msg;
  is_traj_tracking_ = true;
}

void nmpc_over_act_full::NMPCController::sendRPYGain()
{
  spinal::RollPitchYawTerms rpy_gain_msg;
  rpy_gain_msg.motors.resize(motor_num_);

  ros::NodeHandle control_nh(nh_, "controller");
  double roll_rate_p_gain, pitch_rate_p_gain, yaw_rate_p_gain;
  getParam<double>(control_nh, "nmpc/roll_rate_p_gain", roll_rate_p_gain, 1.0);
  getParam<double>(control_nh, "nmpc/pitch_rate_p_gain", pitch_rate_p_gain, 1.0);
  getParam<double>(control_nh, "nmpc/yaw_rate_p_gain", yaw_rate_p_gain, 1.0);

  rpy_gain_msg.motors[0].roll_d = (short)(-roll_rate_p_gain * 1000);
  rpy_gain_msg.motors[0].pitch_d = (short)(-pitch_rate_p_gain * 1000);
  rpy_gain_msg.motors[0].yaw_d = (short)(-yaw_rate_p_gain * 1000);

  pub_rpy_gain_.publish(rpy_gain_msg);
}

void nmpc_over_act_full::NMPCController::sendRotationalInertiaComp()
{
  int lqi_mode_ = 4;

  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd p_mat_pseudo_inv_ = aerial_robot_model::pseudoinverse(P.middleRows(2, lqi_mode_));

  spinal::PMatrixPseudoInverseWithInertia p_pseudo_inverse_with_inertia_msg;  // to spinal
  p_pseudo_inverse_with_inertia_msg.pseudo_inverse.resize(motor_num_);

  for (int i = 0; i < motor_num_; ++i)
  {
    /* the p matrix pseudo inverse and inertia */
    p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].r = p_mat_pseudo_inv_(i, 1) * 1000;
    p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].p = p_mat_pseudo_inv_(i, 2) * 1000;
    if (lqi_mode_ == 4)
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].y = p_mat_pseudo_inv_(i, 3) * 1000;
    else
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].y = 0;
  }

  /* the articulated inertia */
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  p_pseudo_inverse_with_inertia_msg.inertia[0] = inertia(0, 0) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[1] = inertia(1, 1) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[2] = inertia(2, 2) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[3] = inertia(0, 1) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[4] = inertia(1, 2) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[5] = inertia(0, 2) * 1000;

  pub_p_matrix_pseudo_inverse_inertia_.publish(p_pseudo_inverse_with_inertia_msg);
}

void nmpc_over_act_full::NMPCController::initAllocMat()
{
  const auto& rotor_p = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  Eigen::Vector3d p1_b = rotor_p[0];
  Eigen::Vector3d p2_b = rotor_p[1];
  Eigen::Vector3d p3_b = rotor_p[2];
  Eigen::Vector3d p4_b = rotor_p[3];

  const map<int, int> rotor_dr = robot_model_->getRotorDirection();
  int dr1 = rotor_dr.find(1)->second;
  int dr2 = rotor_dr.find(2)->second;
  int dr3 = rotor_dr.find(3)->second;
  int dr4 = rotor_dr.find(4)->second;

  double kq_d_kt = robot_model_->getThrustWrenchUnits()[0][5];

  double sqrt_p1b_xy = sqrt(p1_b.x() * p1_b.x() + p1_b.y() * p1_b.y());
  double sqrt_p2b_xy = sqrt(p2_b.x() * p2_b.x() + p2_b.y() * p2_b.y());
  double sqrt_p3b_xy = sqrt(p3_b.x() * p3_b.x() + p3_b.y() * p3_b.y());
  double sqrt_p4b_xy = sqrt(p4_b.x() * p4_b.x() + p4_b.y() * p4_b.y());

  // - force
  alloc_mat_(0, 0) = p1_b.y() / sqrt_p1b_xy;
  alloc_mat_(1, 0) = -p1_b.x() / sqrt_p1b_xy;
  alloc_mat_(2, 1) = 1;

  alloc_mat_(0, 2) = p2_b.y() / sqrt_p2b_xy;
  alloc_mat_(1, 2) = -p2_b.x() / sqrt_p2b_xy;
  alloc_mat_(2, 3) = 1;

  alloc_mat_(0, 4) = p3_b.y() / sqrt_p3b_xy;
  alloc_mat_(1, 4) = -p3_b.x() / sqrt_p3b_xy;
  alloc_mat_(2, 5) = 1;

  alloc_mat_(0, 6) = p4_b.y() / sqrt_p4b_xy;
  alloc_mat_(1, 6) = -p4_b.x() / sqrt_p4b_xy;
  alloc_mat_(2, 7) = 1;

  // - torque
  alloc_mat_(3, 0) = -dr1 * kq_d_kt * p1_b.y() / sqrt_p1b_xy + p1_b.x() * p1_b.z() / sqrt_p1b_xy;
  alloc_mat_(4, 0) = dr1 * kq_d_kt * p1_b.x() / sqrt_p1b_xy + p1_b.y() * p1_b.z() / sqrt_p1b_xy;
  alloc_mat_(5, 0) = -p1_b.x() * p1_b.x() / sqrt_p1b_xy - p1_b.y() * p1_b.y() / sqrt_p1b_xy;

  alloc_mat_(3, 1) = p1_b.y();
  alloc_mat_(4, 1) = -p1_b.x();
  alloc_mat_(5, 1) = -dr1 * kq_d_kt;

  alloc_mat_(3, 2) = -dr2 * kq_d_kt * p2_b.y() / sqrt_p2b_xy + p2_b.x() * p2_b.z() / sqrt_p2b_xy;
  alloc_mat_(4, 2) = dr2 * kq_d_kt * p2_b.x() / sqrt_p2b_xy + p2_b.y() * p2_b.z() / sqrt_p2b_xy;
  alloc_mat_(5, 2) = -p2_b.x() * p2_b.x() / sqrt_p2b_xy - p2_b.y() * p2_b.y() / sqrt_p2b_xy;

  alloc_mat_(3, 3) = p2_b.y();
  alloc_mat_(4, 3) = -p2_b.x();
  alloc_mat_(5, 3) = -dr2 * kq_d_kt;

  alloc_mat_(3, 4) = -dr3 * kq_d_kt * p3_b.y() / sqrt_p3b_xy + p3_b.x() * p3_b.z() / sqrt_p3b_xy;
  alloc_mat_(4, 4) = dr3 * kq_d_kt * p3_b.x() / sqrt_p3b_xy + p3_b.y() * p3_b.z() / sqrt_p3b_xy;
  alloc_mat_(5, 4) = -p3_b.x() * p3_b.x() / sqrt_p3b_xy - p3_b.y() * p3_b.y() / sqrt_p3b_xy;

  alloc_mat_(3, 5) = p3_b.y();
  alloc_mat_(4, 5) = -p3_b.x();
  alloc_mat_(5, 5) = -dr3 * kq_d_kt;

  alloc_mat_(3, 6) = -dr4 * kq_d_kt * p4_b.y() / sqrt_p4b_xy + p4_b.x() * p4_b.z() / sqrt_p4b_xy;
  alloc_mat_(4, 6) = dr4 * kq_d_kt * p4_b.x() / sqrt_p4b_xy + p4_b.y() * p4_b.z() / sqrt_p4b_xy;
  alloc_mat_(5, 6) = -p4_b.x() * p4_b.x() / sqrt_p4b_xy - p4_b.y() * p4_b.y() / sqrt_p4b_xy;

  alloc_mat_(3, 7) = p4_b.y();
  alloc_mat_(4, 7) = -p4_b.x();
  alloc_mat_(5, 7) = -dr4 * kq_d_kt;

  alloc_mat_pinv_ = aerial_robot_model::pseudoinverse(alloc_mat_);
}

void nmpc_over_act_full::NMPCController::calXrUrRef(const tf::Vector3 target_pos, const tf::Vector3 target_vel,
                                                    const tf::Vector3 target_rpy, const tf::Vector3 target_omega,
                                                    const Eigen::VectorXd& target_wrench)
{
  Eigen::VectorXd x_lambda = alloc_mat_pinv_ * target_wrench;
  double a1_ref = atan2(x_lambda(0), x_lambda(1));
  double ft1_ref = sqrt(x_lambda(0) * x_lambda(0) + x_lambda(1) * x_lambda(1));
  double a2_ref = atan2(x_lambda(2), x_lambda(3));
  double ft2_ref = sqrt(x_lambda(2) * x_lambda(2) + x_lambda(3) * x_lambda(3));
  double a3_ref = atan2(x_lambda(4), x_lambda(5));
  double ft3_ref = sqrt(x_lambda(4) * x_lambda(4) + x_lambda(5) * x_lambda(5));
  double a4_ref = atan2(x_lambda(6), x_lambda(7));
  double ft4_ref = sqrt(x_lambda(6) * x_lambda(6) + x_lambda(7) * x_lambda(7));

  tf::Quaternion q;
  q.setRPY(target_rpy.x(), target_rpy.y(), target_rpy.z());

  double x[NX] = {
    target_pos.x(), target_pos.y(), target_pos.z(), target_vel.x(),   target_vel.y(),   target_vel.z(),   q.w(),
    q.x(),          q.y(),          q.z(),          target_omega.x(), target_omega.y(), target_omega.z(), a1_ref,
    a2_ref,         a3_ref,         a4_ref
  };
  double u[NU] = { ft1_ref, ft2_ref, ft3_ref, ft4_ref, 0.0, 0.0, 0.0, 0.0 };

  for (int i = 0; i < NN; i++)
  {
    // shift one step
    std::copy(x_u_ref_.x.data.begin() + NX * (i + 1), x_u_ref_.x.data.begin() + NX * (i + 2),
              x_u_ref_.x.data.begin() + NX * i);
    std::copy(x_u_ref_.u.data.begin() + NU * (i + 1), x_u_ref_.u.data.begin() + NU * (i + 2),
              x_u_ref_.u.data.begin() + NU * i);
  }
  std::copy(x, x + NX, x_u_ref_.x.data.begin() + NX * NN);
  std::copy(u, u + NU, x_u_ref_.u.data.begin() + NU * NN);
}

void nmpc_over_act_full::NMPCController::printPhysicalParams()
{
  cout << "mass: " << robot_model_->getMass() << endl;
  cout << "gravity: " << robot_model_->getGravity() << endl;
  cout << "inertia: " << robot_model_->getInertia<Eigen::Matrix3d>() << endl;
  cout << "rotor num: " << robot_model_->getRotorNum() << endl;
  for (const auto& dir : robot_model_->getRotorDirection())
  {
    std::cout << "Key: " << dir.first << ", Value: " << dir.second << std::endl;
  }
  for (const auto& vec : robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>())
  {
    std::cout << "rotor origin from cog: " << vec << std::endl;
  }
  cout << "thrust lower limit: " << robot_model_->getThrustLowerLimit() << endl;
  cout << "thrust upper limit: " << robot_model_->getThrustUpperLimit() << endl;
  robot_model_->getThrustWrenchUnits();
  for (const auto& vec : robot_model_->getThrustWrenchUnits())
  {
    std::cout << "thrust wrench units: " << vec << std::endl;
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc_over_act_full::NMPCController, aerial_robot_control::ControlBase);
