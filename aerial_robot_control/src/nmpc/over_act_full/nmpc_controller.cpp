//
// Created by lijinjie on 23/11/29.
//

#include "aerial_robot_control/nmpc/over_act_full/nmpc_controller.h"

using namespace aerial_robot_control;

void nmpc_over_act_full::NMPCController::initialize(
    ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du)
{
  ControlBase::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle nmpc_nh(control_nh, "nmpc");
  ros::NodeHandle physical_nh(control_nh, "physical");

  // initialize nmpc solver
  mpc_solver_.initialize();
  getParam<double>(physical_nh, "mass", mass_, 0.5);
  getParam<double>(physical_nh, "gravity_const", gravity_const_, 9.81);

  getParam<double>(nmpc_nh, "T_samp", t_nmpc_samp_, 0.025);
  getParam<double>(nmpc_nh, "T_integ", t_nmpc_integ_, 0.1);
  getParam<bool>(nmpc_nh, "is_attitude_ctrl", is_attitude_ctrl_, true);
  getParam<bool>(nmpc_nh, "is_body_rate_ctrl", is_body_rate_ctrl_, false);
  getParam<bool>(nmpc_nh, "is_debug", is_debug_, false);

  /* control parameters with dynamic reconfigure */
  double Qp_xy, Qp_z, Qv_xy, Qv_z, Qq_xy, Qq_z, Qw_xy, Qw_z, Qa, Rt, Rac_d;
  getParam<double>(nmpc_nh, "Qp_xy", Qp_xy, 300);
  getParam<double>(nmpc_nh, "Qp_z", Qp_z, 400);
  getParam<double>(nmpc_nh, "Qv_xy", Qv_xy, 10);
  getParam<double>(nmpc_nh, "Qv_z", Qv_z, 10);
  getParam<double>(nmpc_nh, "Qq_xy", Qq_xy, 300);
  getParam<double>(nmpc_nh, "Qq_z", Qq_z, 300);
  getParam<double>(nmpc_nh, "Qw_xy", Qw_xy, 5);
  getParam<double>(nmpc_nh, "Qw_z", Qw_z, 5);
  getParam<double>(nmpc_nh, "Qa", Qa, 1);
  getParam<double>(nmpc_nh, "Rt", Rt, 1);
  getParam<double>(nmpc_nh, "Rac_d", Rac_d, 250);

  // diagonal matrix
  mpc_solver_.setCostWDiagElement(0, Qp_xy);
  mpc_solver_.setCostWDiagElement(1, Qp_xy);
  mpc_solver_.setCostWDiagElement(2, Qp_z);
  mpc_solver_.setCostWDiagElement(3, Qv_xy);
  mpc_solver_.setCostWDiagElement(4, Qv_xy);
  mpc_solver_.setCostWDiagElement(5, Qv_z);
  mpc_solver_.setCostWDiagElement(6, 0);
  mpc_solver_.setCostWDiagElement(7, Qq_xy);
  mpc_solver_.setCostWDiagElement(8, Qq_xy);
  mpc_solver_.setCostWDiagElement(9, Qq_z);
  mpc_solver_.setCostWDiagElement(10, Qw_xy);
  mpc_solver_.setCostWDiagElement(11, Qw_xy);
  mpc_solver_.setCostWDiagElement(12, Qw_z);
  for (int i = 13; i < 17; ++i)
    mpc_solver_.setCostWDiagElement(i, Qa);
  for (int i = 17; i < 21; ++i)
    mpc_solver_.setCostWDiagElement(i, Rt, false);
  for (int i = 21; i < 25; ++i)
    mpc_solver_.setCostWDiagElement(i, Rac_d, false);
  mpc_solver_.setCostWeight(true, true);

  nmpc_reconf_servers_.push_back(boost::make_shared<NMPCControlDynamicConfig>(nmpc_nh));
  nmpc_reconf_servers_.back()->setCallback(boost::bind(&NMPCController::cfgNMPCCallback, this, _1, _2));

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
  bool res = ros::service::waitForService("set_control_mode", ros::Duration(5));

  /* subscribers */
  sub_joint_states_ = nh_.subscribe("joint_states", 5, &NMPCController::callbackJointStates, this);
  sub_set_rpy_ = nh_.subscribe("set_rpy", 5, &NMPCController::callbackSetRPY, this);
  sub_set_ref_traj_ = nh_.subscribe("set_ref_traj", 5, &NMPCController::callbackSetRefTraj, this);

  /* init some values */
  odom_ = nav_msgs::Odometry();
  odom_.pose.pose.orientation.w = 1;
  reset();

  /* set control mode */
  if (!res)
  {
    ROS_ERROR("cannot find service named set_control_mode");
  }
  ros::Duration(2.0).sleep();
  spinal::SetControlMode set_control_mode_srv;
  set_control_mode_srv.request.is_attitude = is_attitude_ctrl_;
  set_control_mode_srv.request.is_body_rate = is_body_rate_ctrl_;
  while (!srv_set_control_mode_.call(set_control_mode_srv))
    ROS_WARN_THROTTLE(1,
                      "Waiting for set_control_mode service.... If you always see this message, the robot cannot fly.");

  ROS_INFO("Set control mode: attitude = %d and body rate = %d", set_control_mode_srv.request.is_attitude,
           set_control_mode_srv.request.is_body_rate);

  ROS_INFO("MPC Controller initialized!");

  /* print physical parameters if needed */
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

  this->SendCmd();
  this->controlCore();

  return true;
}

void nmpc_over_act_full::NMPCController::reset()
{
  ControlBase::reset();

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

  /* reset control input */
  flight_cmd_.base_thrust = std::vector<float>(motor_num_, 0.0);

  gimbal_ctrl_cmd_.name.clear();
  gimbal_ctrl_cmd_.name.emplace_back("gimbal1");
  gimbal_ctrl_cmd_.name.emplace_back("gimbal2");
  gimbal_ctrl_cmd_.name.emplace_back("gimbal3");
  gimbal_ctrl_cmd_.name.emplace_back("gimbal4");
  gimbal_ctrl_cmd_.position.clear();
  gimbal_ctrl_cmd_.position.push_back(0.0);
  gimbal_ctrl_cmd_.position.push_back(0.0);
  gimbal_ctrl_cmd_.position.push_back(0.0);
  gimbal_ctrl_cmd_.position.push_back(0.0);
}

void nmpc_over_act_full::NMPCController::controlCore()
{
  if (!is_traj_tracking_)
  {
    /* point mode --> set target */
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
  else
  {
    /* tracking mode */
    if (ros::Time::now() - receive_time_ > ros::Duration(0.1))
    {
      ROS_INFO("Trajectory tracking mode is off!");
      is_traj_tracking_ = false;
      tf::Vector3 target_pos = estimator_->getPos(Frame::COG, estimate_mode_);
      tf::Vector3 target_rpy = navigator_->getTargetRPY();

      navigator_->setTargetPosX((float)target_pos.x());
      navigator_->setTargetPosY((float)target_pos.y());
      navigator_->setTargetPosZ((float)target_pos.z());
      navigator_->setTargetVelX(0.0);
      navigator_->setTargetVelY(0.0);
      navigator_->setTargetVelZ(0.0);
      navigator_->setTargetRoll((float)target_rpy.x());
      navigator_->setTargetPitch((float)target_rpy.y());
      navigator_->setTargetYaw((float)target_rpy.z());
      navigator_->setTargetZeroAngAcc();
    }
  }

  /* get odom information */
  nav_msgs::Odometry odom_now = getOdom();
  odom_ = odom_now;

  /* solve */
  mpc_solver_.solve(odom_, joint_angles_, x_u_ref_, is_debug_);

  /* get result */
  // - thrust
  double ft1 = getCommand(0, t_nmpc_samp_);
  double ft2 = getCommand(1, t_nmpc_samp_);
  double ft3 = getCommand(2, t_nmpc_samp_);
  double ft4 = getCommand(3, t_nmpc_samp_);

  Eigen::VectorXd target_thrusts(4);
  target_thrusts << ft1, ft2, ft3, ft4;

  for (int i = 0; i < motor_num_; i++)
  {
    flight_cmd_.base_thrust[i] = static_cast<float>(target_thrusts(i) * 1.28);
  }

  // - servo angle
  double a1c = getCommand(4, t_nmpc_samp_);
  double a2c = getCommand(5, t_nmpc_samp_);
  double a3c = getCommand(6, t_nmpc_samp_);
  double a4c = getCommand(7, t_nmpc_samp_);

  gimbal_ctrl_cmd_.header.stamp = ros::Time::now();
  gimbal_ctrl_cmd_.name.clear();
  gimbal_ctrl_cmd_.name.emplace_back("gimbal1");
  gimbal_ctrl_cmd_.name.emplace_back("gimbal2");
  gimbal_ctrl_cmd_.name.emplace_back("gimbal3");
  gimbal_ctrl_cmd_.name.emplace_back("gimbal4");
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

// TODO: should be moved to Estimator
nav_msgs::Odometry nmpc_over_act_full::NMPCController::getOdom()
{
  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Vector3 vel = estimator_->getVel(Frame::COG, estimate_mode_);
  tf::Vector3 rpy = estimator_->getEuler(Frame::COG, estimate_mode_);
  tf::Vector3 omega = estimator_->getAngularVel(Frame::COG, estimate_mode_);
  tf::Quaternion q;
  q.setRPY(rpy.x(), rpy.y(), rpy.z());

  // check the sign of the quaternion, avoid the flip of the quaternion
  double qe_c_w = q.w() * odom_.pose.pose.orientation.w + q.x() * odom_.pose.pose.orientation.x +
                  q.y() * odom_.pose.pose.orientation.y + q.z() * odom_.pose.pose.orientation.z;
  if (qe_c_w < 0)
  {
    q.setW(-q.w());
    q.setX(-q.x());
    q.setY(-q.y());
    q.setZ(-q.z());
  }

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

/* TODO: this function should be combined with the inner planning framework */
void nmpc_over_act_full::NMPCController::callbackSetRefTraj(const aerial_robot_msgs::PredXUConstPtr& msg)
{
  if (navigator_->getNaviState() == aerial_robot_navigation::TAKEOFF_STATE)
  {
    ROS_WARN_THROTTLE(1, "The robot is taking off, so the reference trajectory will be ignored!");
    return;
  }

  x_u_ref_ = *msg;
  receive_time_ = ros::Time::now();

  if (!is_traj_tracking_)
  {
    ROS_INFO("Trajectory tracking mode is on!");
    is_traj_tracking_ = true;
  }
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

double nmpc_over_act_full::NMPCController::getCommand(int idx_u, double t_pred)
{
  double u_predicted =
      mpc_solver_.x_u_out_.u.data.at(idx_u) +
      t_pred / t_nmpc_integ_ * (mpc_solver_.x_u_out_.u.data.at(idx_u + NU) - mpc_solver_.x_u_out_.u.data.at(idx_u));
  return u_predicted;
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

void nmpc_over_act_full::NMPCController::cfgNMPCCallback(NMPCConfig& config, uint32_t level)
{
  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;
  if (config.nmpc_flag)
  {
    switch (level)
    {
      case Levels::RECONFIGURE_NMPC_Q_P_XY: {
        mpc_solver_.setCostWDiagElement(0, config.Qp_xy);
        mpc_solver_.setCostWDiagElement(1, config.Qp_xy);

        ROS_INFO_STREAM("change Qp_xy for NMPC '" << config.Qp_xy << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_Q_P_Z: {
        mpc_solver_.setCostWDiagElement(2, config.Qp_z);
        ROS_INFO_STREAM("change Qp_z for NMPC '" << config.Qp_z << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_Q_V_XY: {
        mpc_solver_.setCostWDiagElement(3, config.Qv_xy);
        mpc_solver_.setCostWDiagElement(4, config.Qv_xy);
        ROS_INFO_STREAM("change Qv_xy for NMPC '" << config.Qv_xy << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_Q_V_Z: {
        mpc_solver_.setCostWDiagElement(5, config.Qv_z);
        ROS_INFO_STREAM("change Qv_z for NMPC '" << config.Qv_z << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_Q_Q_XY: {
        mpc_solver_.setCostWDiagElement(7, config.Qq_xy);
        mpc_solver_.setCostWDiagElement(8, config.Qq_xy);
        ROS_INFO_STREAM("change Qq_xy for NMPC '" << config.Qq_xy << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_Q_Q_Z: {
        mpc_solver_.setCostWDiagElement(9, config.Qq_z);
        ROS_INFO_STREAM("change Qq_z for NMPC '" << config.Qq_z << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_Q_W_XY: {
        mpc_solver_.setCostWDiagElement(10, config.Qw_xy);
        mpc_solver_.setCostWDiagElement(11, config.Qw_xy);
        ROS_INFO_STREAM("change Qw_xy for NMPC '" << config.Qw_xy << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_Q_W_Z: {
        mpc_solver_.setCostWDiagElement(12, config.Qw_z);
        ROS_INFO_STREAM("change Qw_z for NMPC '" << config.Qw_z << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_Q_A: {
        for (int i = 13; i < 17; ++i)
          mpc_solver_.setCostWDiagElement(i, config.Qa);
        ROS_INFO_STREAM("change Qa for NMPC '" << config.Qa << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_R_T: {
        for (int i = 17; i < 21; ++i)
          mpc_solver_.setCostWDiagElement(i, config.Rt, false);
        ROS_INFO_STREAM("change Rt for NMPC '" << config.Rt << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_R_AC_D: {
        for (int i = 21; i < 25; ++i)
          mpc_solver_.setCostWDiagElement(i, config.Rac_d, false);
        ROS_INFO_STREAM("change Rac_d for NMPC '" << config.Rac_d << "'");
        break;
      }
      default:
        break;
    }
    mpc_solver_.setCostWeight(true, true);
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc_over_act_full::NMPCController, aerial_robot_control::ControlBase);
