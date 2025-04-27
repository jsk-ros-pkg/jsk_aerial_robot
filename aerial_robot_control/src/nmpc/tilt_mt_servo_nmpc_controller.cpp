//
// Created by lijinjie on 23/11/29.
//

#include "aerial_robot_control/nmpc/tilt_mt_servo_nmpc_controller.h"

using namespace aerial_robot_control;

void nmpc::TiltMtServoNMPC::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                       boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                       boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                       boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                       double ctrl_loop_du)
{
  BaseMPC::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  /* init plugins */
  initPlugins();

  /* init general parameters */
  initGeneralParams();

  /* init cost weight parameters */
  initNMPCCostW();

  /* init constraints */
  initNMPCConstraints();

  /* init dynamic reconfigure */
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle nmpc_nh(control_nh, "nmpc");
  nmpc_reconf_servers_.push_back(boost::make_shared<NMPCControlDynamicConfig>(nmpc_nh));
  nmpc_reconf_servers_.back()->setCallback(boost::bind(&TiltMtServoNMPC::cfgNMPCCallback, this, _1, _2));

  /* set some ROS parameters */
  nmpc_nh.setParam("NN", mpc_solver_ptr_->NN_);
  nmpc_nh.setParam("NX", mpc_solver_ptr_->NX_);
  nmpc_nh.setParam("NU", mpc_solver_ptr_->NU_);

  /* timers */
  tmr_viz_ = nh_.createTimer(ros::Duration(0.05), &TiltMtServoNMPC::callbackViz, this);

  /* publishers */
  pub_viz_pred_ = nh_.advertise<geometry_msgs::PoseArray>("nmpc/viz_pred", 1);
  pub_viz_ref_ = nh_.advertise<geometry_msgs::PoseArray>("nmpc/viz_ref", 1);
  pub_flight_cmd_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  pub_gimbal_control_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  pub_flight_config_cmd_spinal_ = nh_.advertise<spinal::FlightConfigCmd>("flight_config_cmd", 1);

  /* services */
  srv_set_control_mode_ = nh_.serviceClient<spinal::SetControlMode>("set_control_mode");

  /* subscribers */
  sub_joint_states_ = nh_.subscribe("joint_states", 5, &TiltMtServoNMPC::callbackJointStates, this);
  sub_set_rpy_ = nh_.subscribe("set_rpy", 5, &TiltMtServoNMPC::callbackSetRPY, this);
  sub_set_ref_x_u_ = nh_.subscribe("set_ref_x_u", 5, &TiltMtServoNMPC::callbackSetRefXU, this);
  sub_set_traj_ = nh_.subscribe("set_ref_traj", 5, &TiltMtServoNMPC::callbackSetRefTraj, this);
  sub_set_fixed_rotor_ = nh_.subscribe("set_fixed_rotor", 5, &TiltMtServoNMPC::callbackSetFixedRotor, this);

  /* init some values */
  setControlMode();

  initActuatorStates();
  initPredXU(x_u_ref_, mpc_solver_ptr_->NN_, mpc_solver_ptr_->NX_, mpc_solver_ptr_->NU_);

  quat_prev_.setW(1.0);

  reset();
  ROS_INFO("MPC Controller initialized!");
}

void nmpc::TiltMtServoNMPC::activate()
{
  ControlBase::activate();

  initAllocMat();
  updateInertialParams();
  initNMPCParams();

  if (is_print_phys_params_)
    printPhysicalParams();

  /* also for some commands that should be sent after takeoff */
  // enable imu sending, only works in simulation. TODO: check its compatibility with real robot
  spinal::FlightConfigCmd flight_config_cmd;
  flight_config_cmd.cmd = spinal::FlightConfigCmd::INTEGRATION_CONTROL_ON_CMD;
  pub_flight_config_cmd_spinal_.publish(flight_config_cmd);
}

bool nmpc::TiltMtServoNMPC::update()
{
  if (!ControlBase::update())
    return false;

  this->controlCore();
  this->sendCmd();

  return true;
}

void nmpc::TiltMtServoNMPC::reset()
{
  ControlBase::reset();

  /* reset controller using odom */
  std::vector<double> x_vec = meas2VecX();
  std::vector<double> u_vec(mpc_solver_ptr_->NU_, 0);

  // reset x_u_ref_
  int &NX = mpc_solver_ptr_->NX_, &NU = mpc_solver_ptr_->NU_, &NN = mpc_solver_ptr_->NN_;
  for (int i = 0; i < mpc_solver_ptr_->NN_; i++)
  {
    std::copy(x_vec.begin(), x_vec.begin() + NX, x_u_ref_.x.data.begin() + NX * i);
    std::copy(u_vec.begin(), u_vec.begin() + NU, x_u_ref_.u.data.begin() + NU * i);
  }
  std::copy(x_vec.begin(), x_vec.begin() + NX, x_u_ref_.x.data.begin() + NX * NN);

  // reset mpc solver
  mpc_solver_ptr_->resetByX0U0(x_vec, u_vec);

  /* reset control input */
  flight_cmd_.base_thrust = std::vector<float>(motor_num_, 0.0);

  gimbal_ctrl_cmd_.name.clear();
  gimbal_ctrl_cmd_.position.clear();
  for (int i = 0; i < joint_num_; i++)
  {
    gimbal_ctrl_cmd_.name.emplace_back("gimbal" + std::to_string(i + 1));
    gimbal_ctrl_cmd_.position.push_back(0.0);
  }

  pub_gimbal_control_.publish(gimbal_ctrl_cmd_);
}

void nmpc::TiltMtServoNMPC::initGeneralParams()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle nmpc_nh(control_nh, "nmpc");
  ros::NodeHandle physical_nh(nh_, "physical");
  ros::NodeHandle alloc_nh(control_nh, "alloc");

  getParam<int>(alloc_nh, "type", alloc_type_, 0);
  getParam<double>(alloc_nh, "ft_thresh", ft_thresh_, 0.5);
  assert(ft_thresh_ > 0.0);  // used to calculate angle in singularity pose

  getParam<int>(physical_nh, "num_servos", joint_num_, 0);
  getParam<double>(physical_nh, "t_servo", t_servo_, 0.01);
  getParam<int>(physical_nh, "num_rotors", motor_num_, 0);
  getParam<double>(physical_nh, "t_rotor", t_rotor_, 0.01);

  getParam<double>(nmpc_nh, "T_samp", t_nmpc_samp_, 0.025);
  getParam<double>(nmpc_nh, "T_step", t_nmpc_step_, 0.1);
  getParam<double>(nmpc_nh, "T_horizon", t_nmpc_horizon_, 2.0);

  getParam<bool>(nmpc_nh, "is_attitude_ctrl", is_attitude_ctrl_, true);
  getParam<bool>(nmpc_nh, "is_body_rate_ctrl", is_body_rate_ctrl_, false);
  getParam<bool>(nmpc_nh, "is_print_phys_params", is_print_phys_params_, false);
  getParam<bool>(nmpc_nh, "is_debug", is_debug_, false);

  if (is_debug_)
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
}

void nmpc::TiltMtServoNMPC::initNMPCCostW()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle nmpc_nh(control_nh, "nmpc");

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
  mpc_solver_ptr_->setCostWDiagElement(0, Qp_xy);
  mpc_solver_ptr_->setCostWDiagElement(1, Qp_xy);
  mpc_solver_ptr_->setCostWDiagElement(2, Qp_z);
  mpc_solver_ptr_->setCostWDiagElement(3, Qv_xy);
  mpc_solver_ptr_->setCostWDiagElement(4, Qv_xy);
  mpc_solver_ptr_->setCostWDiagElement(5, Qv_z);
  mpc_solver_ptr_->setCostWDiagElement(6, 0);
  mpc_solver_ptr_->setCostWDiagElement(7, Qq_xy);
  mpc_solver_ptr_->setCostWDiagElement(8, Qq_xy);
  mpc_solver_ptr_->setCostWDiagElement(9, Qq_z);
  mpc_solver_ptr_->setCostWDiagElement(10, Qw_xy);
  mpc_solver_ptr_->setCostWDiagElement(11, Qw_xy);
  mpc_solver_ptr_->setCostWDiagElement(12, Qw_z);
  for (int i = 13; i < 13 + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qa);
  for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rt, false);
  for (int i = mpc_solver_ptr_->NX_ + motor_num_; i < mpc_solver_ptr_->NX_ + motor_num_ + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rac_d, false);
}

void nmpc::TiltMtServoNMPC::initNMPCConstraints()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle nmpc_nh(control_nh, "nmpc");

  double body_rate_max, body_rate_min, vel_max, vel_min;
  getParam<double>(nmpc_nh, "w_max", body_rate_max, 6.0);
  getParam<double>(nmpc_nh, "w_min", body_rate_min, -6.0);
  getParam<double>(nmpc_nh, "v_max", vel_max, 1.0);
  getParam<double>(nmpc_nh, "v_min", vel_min, -1.0);
  getParam<double>(nmpc_nh, "thrust_max", thrust_ctrl_max_, 0.0);
  getParam<double>(nmpc_nh, "thrust_min", thrust_ctrl_min_, 0.0);
  getParam<double>(nmpc_nh, "a_max", servo_angle_max_, 3.1416);
  getParam<double>(nmpc_nh, "a_min", servo_angle_min_, -3.1416);

  std::vector<int> idxbx = mpc_solver_ptr_->getConstraintsIdxbx();
  std::vector<int> idxbx_desired = { 3, 4, 5, 10, 11, 12 };
  idxbx_desired.resize(6 + joint_num_);
  for (int i = 0; i < joint_num_; i++)
  {
    idxbx_desired[6 + i] = 13 + i;
  }
  if (idxbx.size() != idxbx_desired.size() || !std::equal(idxbx.begin(), idxbx.end(), idxbx_desired.begin()))
  {
    ROS_ERROR("idxbx is not equal to idxbx_desired, we cannot set constraints lbx and ubx!");
  }

  std::vector<double> lbx = { vel_min, vel_min, vel_min, body_rate_min, body_rate_min, body_rate_min };
  std::vector<double> ubx = { vel_max, vel_max, vel_max, body_rate_max, body_rate_max, body_rate_max };
  lbx.resize(6 + joint_num_);
  ubx.resize(6 + joint_num_);
  for (int i = 0; i < joint_num_; i++)
  {
    lbx[6 + i] = servo_angle_min_;
    ubx[6 + i] = servo_angle_max_;
  }
  mpc_solver_ptr_->setConstraintsLbx(lbx);
  mpc_solver_ptr_->setConstraintsUbx(ubx);

  std::vector<int> idxbu = mpc_solver_ptr_->getConstraintsIdxbu();
  std::vector<int> idxbu_desired(motor_num_ + joint_num_);
  for (int i = 0; i < motor_num_; i++)
  {
    idxbu_desired[i] = i;
  }
  for (int i = 0; i < joint_num_; i++)
  {
    idxbu_desired[motor_num_ + i] = motor_num_ + i;
  }
  if (idxbu.size() != idxbu_desired.size() || !std::equal(idxbu.begin(), idxbu.end(), idxbu_desired.begin()))
  {
    ROS_ERROR("idxbu is not equal to idxbu_desired, we cannot set constraints lbu and ubu!");
  }

  std::vector<double> lbu(motor_num_ + joint_num_, 0.0);
  std::vector<double> ubu(motor_num_ + joint_num_, 0.0);
  for (int i = 0; i < motor_num_; i++)
  {
    lbu[i] = thrust_ctrl_min_;
    ubu[i] = thrust_ctrl_max_;
  }
  for (int i = 0; i < joint_num_; i++)
  {
    lbu[motor_num_ + i] = servo_angle_min_;
    ubu[motor_num_ + i] = servo_angle_max_;
  }
  mpc_solver_ptr_->setConstraintsLbu(lbu);
  mpc_solver_ptr_->setConstraintsUbu(ubu);
}

void nmpc::TiltMtServoNMPC::initAllocMat()
{
  /* get physical param */
  int rotor_num = robot_model_->getRotorNum();  // For tilt-rotor, rotor_num = servo_num
  const auto& rotor_p = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const map<int, int> rotor_dr = robot_model_->getRotorDirection();
  double kq_d_kt = abs(robot_model_->getMFRate());  // PAY ATTENTION: should be positive value

  /* alloc mat */
  alloc_mat_.resize(0, 0);
  alloc_mat_pinv_.resize(0, 0);

  // construct alloc_mat_
  alloc_mat_ = Eigen::MatrixXd::Zero(6, 2 * rotor_num);

  for (int i = 0; i < rotor_num; i++)
  {
    Eigen::Vector3d p_b = rotor_p[i];
    int dr = rotor_dr.find(i + 1)->second;  // PAY ATTENTION: the rotor index starts from 1!!!!!!!!!!!!!!!!!!!!!

    double sqrt_p_xy = sqrt(p_b.x() * p_b.x() + p_b.y() * p_b.y());

    // - force
    alloc_mat_(0, 2 * i) = p_b.y() / sqrt_p_xy;
    alloc_mat_(1, 2 * i) = -p_b.x() / sqrt_p_xy;
    alloc_mat_(2, 2 * i + 1) = 1;

    // - torque
    alloc_mat_(3, 2 * i) = -dr * kq_d_kt * p_b.y() / sqrt_p_xy + p_b.x() * p_b.z() / sqrt_p_xy;
    alloc_mat_(4, 2 * i) = dr * kq_d_kt * p_b.x() / sqrt_p_xy + p_b.y() * p_b.z() / sqrt_p_xy;
    alloc_mat_(5, 2 * i) = -p_b.x() * p_b.x() / sqrt_p_xy - p_b.y() * p_b.y() / sqrt_p_xy;

    alloc_mat_(3, 2 * i + 1) = p_b.y();
    alloc_mat_(4, 2 * i + 1) = -p_b.x();
    alloc_mat_(5, 2 * i + 1) = -dr * kq_d_kt;
  }

  alloc_mat_pinv_ = aerial_robot_model::pseudoinverse(alloc_mat_);
}

void nmpc::TiltMtServoNMPC::initNMPCParams()
{
  /* construct acados parameters */
  std::vector<double> acados_p(mpc_solver_ptr_->NP_, 0.0);

  acados_p[0] = 1.0;  // qw
  idx_p_quat_end_ = 3;

  int idx;
  // TODO: this condition is temporary for drones that don't pass in phys param (bi, tri, fix-qd)
  if (mpc_solver_ptr_->NP_ > 4 + 6)
  {
    ROS_INFO("Set physical parameters for NMPC solver");

    std::vector<double> phys_p = PhysToNMPCParams();
    std::copy(phys_p.begin(), phys_p.end(), acados_p.begin() + idx_p_quat_end_ + 1);
    idx = idx_p_quat_end_ + phys_p.size();
  }
  else
  {
    idx = idx_p_quat_end_;
  }

  // set idx_p_phys_end_ for setting other parameters later
  idx_p_phys_end_ = idx;

  /* set acados parameters */
  mpc_solver_ptr_->setParameters(acados_p);
}

void nmpc::TiltMtServoNMPC::updateInertialParams()
{
  mass_ = robot_model_->getMass();
  gravity_const_ = robot_model_->getGravity()[2];
  Eigen::Matrix3d inertia_mtx = robot_model_->getInertia<Eigen::Matrix3d>();
  inertia_.resize(3);
  inertia_[0] = inertia_mtx(0, 0);
  inertia_[1] = inertia_mtx(1, 1);
  inertia_[2] = inertia_mtx(2, 2);
}

std::vector<double> nmpc::TiltMtServoNMPC::PhysToNMPCParams() const
{
  int rotor_num = robot_model_->getRotorNum();  // For tilt-rotor, rotor_num = servo_num
  const auto& rotor_p = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const map<int, int> rotor_dr = robot_model_->getRotorDirection();
  double kq_d_kt = abs(robot_model_->getMFRate());  // PAY ATTENTION: should be positive value

  std::vector<double> phys_p(2 + 3 + 1 + 4 * rotor_num + 2, 0);
  // order: mass, gravity, Ixx, Iyy, Izz, kq_d_kt, dr1, p1_b, dr2, p2_b, dr3, p3_b, dr4, p4_b, t_rotor, t_servo
  phys_p[0] = mass_;
  phys_p[1] = gravity_const_;
  phys_p[2] = inertia_[0];
  phys_p[3] = inertia_[1];
  phys_p[4] = inertia_[2];
  phys_p[5] = kq_d_kt;
  int idx = 6;
  for (int i = 0; i < rotor_num; i++)
  {
    phys_p[idx] = rotor_dr.find(i + 1)->second;
    idx++;
    phys_p[idx] = rotor_p[i].x();
    idx++;
    phys_p[idx] = rotor_p[i].y();
    idx++;
    phys_p[idx] = rotor_p[i].z();
    idx++;
  }
  phys_p[idx] = t_rotor_;
  idx++;
  phys_p[idx] = t_servo_;

  return phys_p;
}

void nmpc::TiltMtServoNMPC::setControlMode()
{
  bool res = ros::service::waitForService("set_control_mode", ros::Duration(5));
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
}

void nmpc::TiltMtServoNMPC::controlCore()
{
  prepareNMPCRef();

  prepareNMPCParams();

  /* prepare initial value */
  std::vector<double> bx0 = meas2VecX();

  /* solve */
  try
  {
    mpc_solver_ptr_->solve(bx0, is_debug_);
  }
  catch (mpc_solver::AcadosSolveException& e)
  {
    ROS_FATAL("NMPC solver failed. Details: %s", e.what());
  }
  // The result is stored in mpc_solver_ptr_->uo_

  /* get result */
  // - thrust
  for (int i = 0; i < motor_num_; i++)
  {
    flight_cmd_.base_thrust[i] = (float)getCommand(i);
  }

  // - servo angle
  gimbal_ctrl_cmd_.header.stamp = ros::Time::now();
  gimbal_ctrl_cmd_.name.clear();
  gimbal_ctrl_cmd_.position.clear();
  for (int i = 0; i < joint_num_; i++)
  {
    gimbal_ctrl_cmd_.name.emplace_back("gimbal" + std::to_string(i + 1));
    gimbal_ctrl_cmd_.position.push_back(getCommand(motor_num_ + i));
  }
}

void nmpc::TiltMtServoNMPC::prepareNMPCRef()
{
  /* if in trajectory tracking mode, the ref is set by callbackSetRefXU.
   * So here we check if the traj info is still received. If not, we turn off the tracking mode */
  if (is_traj_tracking_)
  {
    if (ros::Time::now() - x_u_ref_.header.stamp > ros::Duration(0.5))
    {
      ROS_INFO("No traj msg for 0.5s. Trajectory tracking mode is off! Return to the hovering!");
      is_traj_tracking_ = false;
      tf::Vector3 current_pos = estimator_->getPos(Frame::COG, estimate_mode_);
      tf::Vector3 current_rpy = estimator_->getEuler(Frame::COG, estimate_mode_);

      navigator_->setTargetPosX((float)current_pos.x());
      navigator_->setTargetPosY((float)current_pos.y());
      navigator_->setTargetPosZ((float)current_pos.z());
      navigator_->setTargetVelX(0.0);
      navigator_->setTargetVelY(0.0);
      navigator_->setTargetVelZ(0.0);
      navigator_->setTargetRoll(0.0);
      navigator_->setTargetPitch(0.0);
      navigator_->setTargetYaw((float)current_rpy.z());
      navigator_->setTargetOmegaX(0.0);
      navigator_->setTargetOmegaY(0.0);
      navigator_->setTargetOmegaZ(0.0);
    }
    else if (ros::Time::now() - x_u_ref_.header.stamp > ros::Duration(0.1))
    {
      ROS_INFO_THROTTLE(1, "No traj msg for 0.1s. Try to track current pose.");
      tf::Vector3 current_pos = estimator_->getPos(Frame::COG, estimate_mode_);
      tf::Vector3 current_rpy = estimator_->getEuler(Frame::COG, estimate_mode_);

      navigator_->setTargetPosX((float)current_pos.x());
      navigator_->setTargetPosY((float)current_pos.y());
      navigator_->setTargetPosZ((float)current_pos.z());
      navigator_->setTargetVelX(0.0);
      navigator_->setTargetVelY(0.0);
      navigator_->setTargetVelZ(0.0);
      navigator_->setTargetRoll((float)current_rpy.x());
      navigator_->setTargetPitch((float)current_rpy.y());
      navigator_->setTargetYaw((float)current_rpy.z());
      navigator_->setTargetOmegaX(0.0);
      navigator_->setTargetOmegaY(0.0);
      navigator_->setTargetOmegaZ(0.0);
    }

    return;
  }

  /* if not in tracking mode, we use point mode --> set target */
  tf::Vector3 target_pos = navigator_->getTargetPos();
  tf::Vector3 target_vel = navigator_->getTargetVel();
  tf::Vector3 target_rpy = navigator_->getTargetRPY();
  tf::Quaternion target_quat;
  target_quat.setRPY(target_rpy.x(), target_rpy.y(), target_rpy.z());
  tf::Vector3 target_omega = navigator_->getTargetOmega();

  setXrUrRef(target_pos, target_vel, tf::Vector3(0, 0, 0), target_quat, target_omega, tf::Vector3(0, 0, 0), -1);
  rosXU2VecXU(x_u_ref_, mpc_solver_ptr_->xr_, mpc_solver_ptr_->ur_);
  mpc_solver_ptr_->setReference(mpc_solver_ptr_->xr_, mpc_solver_ptr_->ur_, true);
}

void nmpc::TiltMtServoNMPC::prepareNMPCParams()
{
  updateInertialParams();

  // TODO: this condition is temporary for drones that don't pass in phys param (bi, tri, fix-qd)
  if (mpc_solver_ptr_->NP_ > 4 + 6)
  {
    std::vector<double> phys_p = PhysToNMPCParams();

    std::vector<int> idx(phys_p.size());
    std::iota(idx.begin(), idx.end(), idx_p_quat_end_ + 1);

    mpc_solver_ptr_->setParamSparseAllStages(idx, phys_p);
  }
}

void nmpc::TiltMtServoNMPC::sendCmd()
{
  /* publish */
  if (motor_num_ > 0)
    pub_flight_cmd_.publish(flight_cmd_);
  if (joint_num_ > 0)
    pub_gimbal_control_.publish(gimbal_ctrl_cmd_);
}

/**
 * @brief callbackViz: publish the predicted trajectory and reference trajectory
 * @param [ros::TimerEvent&] event
 */
void nmpc::TiltMtServoNMPC::callbackViz(const ros::TimerEvent& event)
{
  // from mpc_solver_ptr_->x_u_out to PoseArray
  geometry_msgs::PoseArray pred_poses;
  geometry_msgs::PoseArray ref_poses;

  int& NN = mpc_solver_ptr_->NN_;
  int& NX = mpc_solver_ptr_->NX_;

  for (int i = 0; i < NN; ++i)
  {
    geometry_msgs::Pose pred_pose;
    pred_pose.position.x = mpc_solver_ptr_->xo_[i][0];
    pred_pose.position.y = mpc_solver_ptr_->xo_[i][1];
    pred_pose.position.z = mpc_solver_ptr_->xo_[i][2];
    pred_pose.orientation.w = mpc_solver_ptr_->xo_[i][6];
    pred_pose.orientation.x = mpc_solver_ptr_->xo_[i][7];
    pred_pose.orientation.y = mpc_solver_ptr_->xo_[i][8];
    pred_pose.orientation.z = mpc_solver_ptr_->xo_[i][9];
    pred_poses.poses.push_back(pred_pose);

    geometry_msgs::Pose ref_pose;
    ref_pose.position.x = mpc_solver_ptr_->xr_[i][0];
    ref_pose.position.y = mpc_solver_ptr_->xr_[i][1];
    ref_pose.position.z = mpc_solver_ptr_->xr_[i][2];
    ref_pose.orientation.w = mpc_solver_ptr_->xr_[i][6];
    ref_pose.orientation.x = mpc_solver_ptr_->xr_[i][7];
    ref_pose.orientation.y = mpc_solver_ptr_->xr_[i][8];
    ref_pose.orientation.z = mpc_solver_ptr_->xr_[i][9];
    ref_poses.poses.push_back(ref_pose);
  }

  pred_poses.header.frame_id = "world";
  pred_poses.header.stamp = ros::Time::now();
  pub_viz_pred_.publish(pred_poses);

  ref_poses.header.frame_id = "world";
  ref_poses.header.stamp = ros::Time::now();
  pub_viz_ref_.publish(ref_poses);
}

void nmpc::TiltMtServoNMPC::callbackJointStates(const sensor_msgs::JointStateConstPtr& msg)
{
  for (int i = 0; i < joint_num_; i++)
    joint_angles_[i] = msg->position[i];
}

/* TODO: this function is just for test. We may need a more general function to set all kinds of state */
void nmpc::TiltMtServoNMPC::callbackSetRPY(const spinal::DesireCoordConstPtr& msg)
{
  // add a check to avoid the singular point for euler angle
  if (msg->pitch == M_PI / 2.0 or msg->pitch == -M_PI / 2.0)
  {
    ROS_WARN(
        "The pitch angle is set to PI/2 or -PI/2, which is a singular point for euler angle."
        " Please set other values for the pitch angle.");
    return;
  }

  navigator_->setTargetRoll(msg->roll);
  navigator_->setTargetPitch(msg->pitch);
  navigator_->setTargetYaw(msg->yaw);
}

/* TODO: this function should be combined with the inner planning framework */
void nmpc::TiltMtServoNMPC::callbackSetRefXU(const aerial_robot_msgs::PredXUConstPtr& msg)
{
  /* failsafe check */
  if (navigator_->getNaviState() == aerial_robot_navigation::TAKEOFF_STATE)
  {
    ROS_WARN_THROTTLE(1, "The robot is taking off, so the reference trajectory will be ignored!");
    return;
  }

  /* switch tracking mode */
  if (!is_traj_tracking_)
  {
    ROS_INFO("Trajectory tracking mode is on!");
    is_traj_tracking_ = true;
  }

  /* receive info */
  x_u_ref_ = *msg;

  /* set reference */
  rosXU2VecXU(x_u_ref_, mpc_solver_ptr_->xr_, mpc_solver_ptr_->ur_);
  mpc_solver_ptr_->setReference(mpc_solver_ptr_->xr_, mpc_solver_ptr_->ur_, true);
}

void nmpc::TiltMtServoNMPC::callbackSetRefTraj(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
  if (msg->points.size() != mpc_solver_ptr_->NN_ + 1)
    ROS_WARN("The length of the trajectory is not equal to the prediction horizon! Cannot use the trajectory!");

  if (navigator_->getNaviState() == aerial_robot_navigation::TAKEOFF_STATE)
  {
    ROS_WARN_THROTTLE(1, "The robot is taking off, so the reference trajectory will be ignored!");
    return;
  }

  for (int i = 0; i < mpc_solver_ptr_->NN_ + 1; i++)
  {
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& point = msg->points[i];
    geometry_msgs::Vector3 pos = point.transforms[0].translation;
    geometry_msgs::Vector3 vel = point.velocities[0].linear;
    geometry_msgs::Vector3 acc = point.accelerations[0].linear;
    geometry_msgs::Quaternion quat = point.transforms[0].rotation;
    geometry_msgs::Vector3 omega = point.velocities[0].angular;
    geometry_msgs::Vector3 ang_acc = point.accelerations[0].angular;
    setXrUrRef(tf::Vector3(pos.x, pos.y, pos.z), tf::Vector3(vel.x, vel.y, vel.z), tf::Vector3(acc.x, acc.y, acc.z),
               tf::Quaternion(quat.x, quat.y, quat.z, quat.w), tf::Vector3(omega.x, omega.y, omega.z),
               tf::Vector3(ang_acc.x, ang_acc.y, ang_acc.z), i);
  }

  x_u_ref_.header.stamp = msg->header.stamp;
  callbackSetRefXU(boost::make_shared<const aerial_robot_msgs::PredXU>(x_u_ref_));
}

void nmpc::TiltMtServoNMPC::callbackSetFixedRotor(const aerial_robot_msgs::FixRotorConstPtr& msg)
{
  // failsafe
  if (msg->rotor_id < 0 || msg->rotor_id >= motor_num_)
  {
    ROS_WARN_STREAM("The rotor_id " << static_cast<int>(msg->rotor_id)
                                    << " is incorrect. Note that the id starts from 0.");
    return;
  }

  if (msg->fix_ft < thrust_ctrl_min_ || msg->fix_ft > thrust_ctrl_max_)
  {
    ROS_WARN_STREAM("The fix_ft value " << msg->fix_ft << " is out of range. It should be between " << thrust_ctrl_min_
                                        << " and " << thrust_ctrl_max_ << ".");
    return;
  }

  if (msg->fix_alpha < servo_angle_min_ || msg->fix_alpha > servo_angle_max_)
  {
    ROS_WARN_STREAM("The fix_alpha value " << msg->fix_alpha << " is out of range. It should be between "
                                           << servo_angle_min_ << " and " << servo_angle_max_ << ".");
    return;
  }

  // set values
  is_set_fix_rotor_ = true;
  fix_rotor_msg_ = *msg;
}

void nmpc::TiltMtServoNMPC::cfgNMPCCallback(NMPCConfig& config, uint32_t level)
{
  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;
  if (config.nmpc_flag)
  {
    try
    {
      switch (level)
      {
        case Levels::RECONFIGURE_NMPC_Q_P_XY: {
          mpc_solver_ptr_->setCostWDiagElement(0, config.Qp_xy);
          mpc_solver_ptr_->setCostWDiagElement(1, config.Qp_xy);

          ROS_INFO_STREAM("change Qp_xy for NMPC '" << config.Qp_xy << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_P_Z: {
          mpc_solver_ptr_->setCostWDiagElement(2, config.Qp_z);
          ROS_INFO_STREAM("change Qp_z for NMPC '" << config.Qp_z << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_V_XY: {
          mpc_solver_ptr_->setCostWDiagElement(3, config.Qv_xy);
          mpc_solver_ptr_->setCostWDiagElement(4, config.Qv_xy);
          ROS_INFO_STREAM("change Qv_xy for NMPC '" << config.Qv_xy << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_V_Z: {
          mpc_solver_ptr_->setCostWDiagElement(5, config.Qv_z);
          ROS_INFO_STREAM("change Qv_z for NMPC '" << config.Qv_z << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_Q_XY: {
          mpc_solver_ptr_->setCostWDiagElement(7, config.Qq_xy);
          mpc_solver_ptr_->setCostWDiagElement(8, config.Qq_xy);
          ROS_INFO_STREAM("change Qq_xy for NMPC '" << config.Qq_xy << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_Q_Z: {
          mpc_solver_ptr_->setCostWDiagElement(9, config.Qq_z);
          ROS_INFO_STREAM("change Qq_z for NMPC '" << config.Qq_z << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_W_XY: {
          mpc_solver_ptr_->setCostWDiagElement(10, config.Qw_xy);
          mpc_solver_ptr_->setCostWDiagElement(11, config.Qw_xy);
          ROS_INFO_STREAM("change Qw_xy for NMPC '" << config.Qw_xy << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_W_Z: {
          mpc_solver_ptr_->setCostWDiagElement(12, config.Qw_z);
          ROS_INFO_STREAM("change Qw_z for NMPC '" << config.Qw_z << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_A: {
          for (int i = 13; i < 13 + joint_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Qa);
          ROS_INFO_STREAM("change Qa for NMPC '" << config.Qa << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_R_T: {
          for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Rt, false);
          ROS_INFO_STREAM("change Rt for NMPC '" << config.Rt << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_R_AC_D: {
          for (int i = mpc_solver_ptr_->NX_ + motor_num_; i < mpc_solver_ptr_->NX_ + motor_num_ + joint_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Rac_d, false);
          ROS_INFO_STREAM("change Rac_d for NMPC '" << config.Rac_d << "'");
          break;
        }
        default: {
          ROS_INFO_STREAM("The setting variable is not in the list!");
          break;
        }
      }
    }
    catch (std::invalid_argument& e)
    {
      ROS_ERROR_STREAM("NMPC config failed: " << e.what());
    }
  }
}

double nmpc::TiltMtServoNMPC::getCommand(int idx_u, double T_horizon)
{
  if (T_horizon == 0)
    return mpc_solver_ptr_->uo_.at(0).at(idx_u);

  return mpc_solver_ptr_->uo_.at(0).at(idx_u) +
         T_horizon / t_nmpc_step_ * (mpc_solver_ptr_->uo_.at(1).at(idx_u) - mpc_solver_ptr_->uo_.at(0).at(idx_u));
}

std::vector<double> nmpc::TiltMtServoNMPC::meas2VecX()
{
  vector<double> bx0(mpc_solver_ptr_->NBX0_, 0);

  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Vector3 vel = estimator_->getVel(Frame::COG, estimate_mode_);
  tf::Quaternion quat = estimator_->getQuat(Frame::COG, estimate_mode_);
  tf::Vector3 ang_vel = estimator_->getAngularVel(Frame::COG, estimate_mode_);

  // === check the sign of the quaternion, avoid the flip of the quaternion. ===
  // This is quite important because of the warm-starting of the NMPC solver. The quaternion should be continuous.
  double qe_c_w =
      quat.w() * quat_prev_.w() + quat.x() * quat_prev_.x() + quat.y() * quat_prev_.y() + quat.z() * quat_prev_.z();
  if (qe_c_w < 0)
  {
    quat = quat.operator-();
  }

  quat_prev_ = quat;
  // =========================

  bx0[0] = pos.x();
  bx0[1] = pos.y();
  bx0[2] = pos.z();
  bx0[3] = vel.x();
  bx0[4] = vel.y();
  bx0[5] = vel.z();
  bx0[6] = quat.w();
  bx0[7] = quat.x();
  bx0[8] = quat.y();
  bx0[9] = quat.z();
  bx0[10] = ang_vel.x();
  bx0[11] = ang_vel.y();
  bx0[12] = ang_vel.z();
  for (int i = 0; i < joint_num_; i++)
    bx0[13 + i] = joint_angles_[i];
  return bx0;
}

void nmpc::TiltMtServoNMPC::printPhysicalParams()
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

  cout << "kq_kt_rate" << robot_model_->getMFRate() << endl;
  cout << "abs(kq_kt_rate)" << abs(robot_model_->getMFRate()) << endl;
}

/**
 * @brief calXrUrRef: calculate the reference state and control input
 * @param ref_pos_i
 * @param ref_vel_i
 * @param ref_acc_i - the acceleration is in the inertial frame, no including the gravity
 * @param ref_quat_ib
 * @param ref_omega_b
 * @param ref_ang_acc_b
 * @param horizon_idx - set -1 for adding the target point to the end of the reference trajectory, 0 ~ NN for adding
 * the target point to the horizon_idx interval
 */
void nmpc::TiltMtServoNMPC::setXrUrRef(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i,
                                       const tf::Vector3& ref_acc_i, const tf::Quaternion& ref_quat_ib,
                                       const tf::Vector3& ref_omega_b, const tf::Vector3& ref_ang_acc_b,
                                       const int& horizon_idx)
{
  int& NX = mpc_solver_ptr_->NX_;
  int& NU = mpc_solver_ptr_->NU_;
  int& NN = mpc_solver_ptr_->NN_;

  /* calculate the reference wrench in the body frame */
  Eigen::VectorXd acc_with_g_i(3);
  acc_with_g_i(0) = ref_acc_i.x();
  acc_with_g_i(1) = ref_acc_i.y();
  acc_with_g_i(2) = ref_acc_i.z() + gravity_const_;  // add gravity

  // coordinate transformation
  tf::Quaternion q_bi = ref_quat_ib.inverse();
  Eigen::Matrix3d rot_bi;
  tf::matrixTFToEigen(tf::Transform(q_bi).getBasis(), rot_bi);
  Eigen::VectorXd ref_acc_b = rot_bi * acc_with_g_i;

  Eigen::VectorXd ref_wrench_b(6);
  ref_wrench_b(0) = ref_acc_b(0) * mass_;
  ref_wrench_b(1) = ref_acc_b(1) * mass_;
  ref_wrench_b(2) = ref_acc_b(2) * mass_;
  ref_wrench_b(3) = ref_ang_acc_b.x() * inertia_.at(0);
  ref_wrench_b(4) = ref_ang_acc_b.y() * inertia_.at(1);
  ref_wrench_b(5) = ref_ang_acc_b.z() * inertia_.at(2);

  /* calculate X U from ref, aka. control allocation */
  std::vector<double> x(NX);
  std::vector<double> u(NU);
  allocateToXU(ref_pos_i, ref_vel_i, ref_quat_ib, ref_omega_b, ref_wrench_b, x, u);

  /* set values */
  if (horizon_idx == -1)
  {
    // Aim: gently add the target point to the end of the reference trajectory
    // - x: NN + 1, u: NN
    // - for 0 ~ NN-2 x and u, shift
    // - copy x to x: NN-1 and NN, copy u to u: NN-1
    for (int i = 0; i < NN - 1; i++)
    {
      // shift one step
      std::copy(x_u_ref_.x.data.begin() + NX * (i + 1), x_u_ref_.x.data.begin() + NX * (i + 2),
                x_u_ref_.x.data.begin() + NX * i);
      std::copy(x_u_ref_.u.data.begin() + NU * (i + 1), x_u_ref_.u.data.begin() + NU * (i + 2),
                x_u_ref_.u.data.begin() + NU * i);
    }
    std::copy(x.begin(), x.begin() + NX, x_u_ref_.x.data.begin() + NX * (NN - 1));
    std::copy(u.begin(), u.begin() + NU, x_u_ref_.u.data.begin() + NU * (NN - 1));

    std::copy(x.begin(), x.begin() + NX, x_u_ref_.x.data.begin() + NX * NN);

    return;
  }

  if (horizon_idx < 0 || horizon_idx > NN)
  {
    ROS_WARN("horizon_idx is out of range! CalXrUrRef failed!");
    return;
  }

  std::copy(x.begin(), x.begin() + NX, x_u_ref_.x.data.begin() + NX * horizon_idx);
  if (horizon_idx < NN)
    std::copy(u.begin(), u.begin() + NU, x_u_ref_.u.data.begin() + NU * horizon_idx);
}

void nmpc::TiltMtServoNMPC::allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i,
                                         const tf::Quaternion& ref_quat_ib, const tf::Vector3& ref_omega_b,
                                         const VectorXd& ref_wrench_b, vector<double>& x, vector<double>& u)
{
  x.at(0) = ref_pos_i.x();
  x.at(1) = ref_pos_i.y();
  x.at(2) = ref_pos_i.z();
  x.at(3) = ref_vel_i.x();
  x.at(4) = ref_vel_i.y();
  x.at(5) = ref_vel_i.z();
  x.at(6) = ref_quat_ib.w();
  x.at(7) = ref_quat_ib.x();
  x.at(8) = ref_quat_ib.y();
  x.at(9) = ref_quat_ib.z();
  x.at(10) = ref_omega_b.x();
  x.at(11) = ref_omega_b.y();
  x.at(12) = ref_omega_b.z();

  // ========= 0) if one rotor is fixed, do it and finish. ======
  if (is_set_fix_rotor_)
  {
    if (ros::Time::now() - fix_rotor_msg_.header.stamp > ros::Duration(0.1))
    {
      ROS_INFO_THROTTLE(1, "No FixRotor msg for 0.1s. Recover to the normal allocation state.");
      is_set_fix_rotor_ = false;
    }

    allocateToXUwOneFixedRotor(fix_rotor_msg_.rotor_id, fix_rotor_msg_.fix_ft, fix_rotor_msg_.fix_alpha, ref_wrench_b,
                               x, u);
    return;
  }
  // =============================================================

  // 1) do one allocation
  Eigen::VectorXd x_lambda = alloc_mat_pinv_ * ref_wrench_b;
  std::vector<double> ft_ref_vec(motor_num_);
  std::vector<double> a_ref_vec(joint_num_);
  // assert(motor_num_ == joint_num_);
  for (int i = 0; i < motor_num_; i++)
  {
    ft_ref_vec[i] = sqrt(x_lambda(2 * i) * x_lambda(2 * i) + x_lambda(2 * i + 1) * x_lambda(2 * i + 1));
    a_ref_vec[i] = atan2(x_lambda(2 * i), x_lambda(2 * i + 1));

    u.at(i) = ft_ref_vec[i];
    x.at(13 + i) = a_ref_vec[i];
  }

  if (alloc_type_ == 0)
    return;

  // 2) check if one rotor's thrust is less than threshold and flip backwards
  std::vector<int> rotor_idx_vec;
  for (int i = 0; i < motor_num_; i++)
  {
    if (ft_ref_vec[i] > ft_thresh_)
      continue;

    if (a_ref_vec[i] >= -M_PI_2 && a_ref_vec[i] <= M_PI_2)
      continue;

    rotor_idx_vec.push_back(i);
  }

  if (rotor_idx_vec.empty())
    return;

  if (rotor_idx_vec.size() > 1)
  {
    ROS_ERROR("More than one rotor is below threshold and flip backwards! Cannot allocate!");
    return;
  }

  int rotor_idx = rotor_idx_vec.at(0);

  // 3) if rotor_idx is not empty, maintain the thrust and modify the angle
  double ft_stop_rotor = ft_ref_vec[rotor_idx];
  double alpha_stop_rotor = M_PI_2 - acos(x_lambda(2 * rotor_idx) / ft_thresh_);

  // 4) re-alloc
  allocateToXUwOneFixedRotor(rotor_idx, ft_stop_rotor, alpha_stop_rotor, ref_wrench_b, x, u);
}

void nmpc::TiltMtServoNMPC::allocateToXUwOneFixedRotor(int fix_rotor_idx, double fix_ft, double fix_alpha,
                                                       const VectorXd& ref_wrench_b, vector<double>& x,
                                                       vector<double>& u)
{
  double fix_ft_x = fix_ft * sin(fix_alpha);
  double fix_ft_y = fix_ft * cos(fix_alpha);

  // 1) construct tgt_wrench from z_from_rotor
  Eigen::VectorXd z_from_rotor = Eigen::VectorXd::Zero(motor_num_ * 2);
  z_from_rotor(2 * fix_rotor_idx) = fix_ft_x;
  z_from_rotor(2 * fix_rotor_idx + 1) = fix_ft_y;
  Eigen::VectorXd tgt_wrench_from_rotor = alloc_mat_ * z_from_rotor;

  // 2) calculate alloc_mat with this rotor's contribution
  Eigen::VectorXd tgt_wrench_modified = ref_wrench_b - tgt_wrench_from_rotor;

  // 3) calculate the allocation matrix without this rotor, which is 6*6
  if (fix_rotor_idx != rotor_idx_prev_)
  {
    Eigen::MatrixXd alloc_mat_del_rotor(alloc_mat_.rows(), alloc_mat_.cols() - 2);
    int j = 0;
    for (int k = 0; k < alloc_mat_.cols(); ++k)
    {
      if (k == 2 * fix_rotor_idx || k == 2 * fix_rotor_idx + 1)
        continue;
      alloc_mat_del_rotor.col(j++) = alloc_mat_.col(k);
    }
    alloc_mat_del_rotor_inv_ = alloc_mat_del_rotor.inverse();
  }

  // 4) reconstruct the z output
  Eigen::VectorXd z_except_rotor = alloc_mat_del_rotor_inv_ * tgt_wrench_modified;

  // 5) at the place of 2*fix_rotor_idx, insert 2 numbers to z_except_rotor
  Eigen::VectorXd z_final(motor_num_ * 2);
  z_final.head(2 * fix_rotor_idx) = z_except_rotor.head(2 * fix_rotor_idx);
  z_final(2 * fix_rotor_idx) = fix_ft_x;
  z_final(2 * fix_rotor_idx + 1) = fix_ft_y;
  z_final.tail(z_except_rotor.size() - 2 * fix_rotor_idx) =
      z_except_rotor.tail(z_except_rotor.size() - 2 * fix_rotor_idx);

  // 6) reconstruct the thrust and servo angle
  for (int i = 0; i < motor_num_; i++)
  {
    u.at(i) = sqrt(z_final(2 * i) * z_final(2 * i) + z_final(2 * i + 1) * z_final(2 * i + 1));
    x.at(13 + i) = atan2(z_final(2 * i), z_final(2 * i + 1));
  }

  // if the fixed rotor is the same with previous one, no need to recalculate the allocation matrix.
  rotor_idx_prev_ = fix_rotor_idx;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoNMPC, aerial_robot_control::ControlBase)
