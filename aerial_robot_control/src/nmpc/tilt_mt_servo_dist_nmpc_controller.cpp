//
// Created by lijinjie on 24/07/18.
//

#include "aerial_robot_control/nmpc/tilt_mt_servo_dist_nmpc_controller.h"

using namespace aerial_robot_control;

void nmpc::TiltMtServoDistNMPC::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_du)
{
  TiltMtServoNMPC::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "if_use_est_wrench_4_control", if_use_est_wrench_4_control_, false);
  ros::NodeHandle disturb_nh(control_nh, "disturb");
  getParam<double>(disturb_nh, "thresh_force", thresh_force_, 0.1);
  getParam<double>(disturb_nh, "thresh_torque", thresh_torque_, 0.01);
  getParam<double>(disturb_nh, "steepness_force", steepness_force_, 1);
  getParam<double>(disturb_nh, "steepness_torque", steepness_torque_, 1);

  pub_disturb_wrench_ = nh_.advertise<geometry_msgs::WrenchStamped>("disturbance_wrench", 1);
  pub_disturb_wrench_coefficient_ = nh_.advertise<geometry_msgs::Vector3Stamped>("disturbance_wrench/coefficient", 1);
}

bool nmpc::TiltMtServoDistNMPC::update()
{
  updateDisturbWrench();

  // Note that the meas2VecX() function is called in the update() function. And since it always get the latest info
  // from the estimator, the NMPC result should not be influenced by the disturbance wrench.
  if (!TiltMtServoNMPC::update())
    return false;

  // pub the est. wrench used by the controller. put here to shorten the delay caused by calcDisturbWrench();
  pubDisturbWrench();

  return true;
}

void nmpc::TiltMtServoDistNMPC::reset()
{
  TiltMtServoNMPC::reset();

  wrench_est_i_term_.reset();

  if (wrench_est_ptr_ != nullptr)
    wrench_est_ptr_->reset();
}

void nmpc::TiltMtServoDistNMPC::initPlugins()
{
  /* I Term is always loaded  */
  wrench_est_i_term_.initialize(nh_, robot_model_, estimator_, ctrl_loop_du_);

  /* plugin: wrench estimator */
  wrench_est_loader_ptr_ = boost::make_shared<pluginlib::ClassLoader<aerial_robot_control::WrenchEstActuatorMeasBase>>(
      "aerial_robot_control", "aerial_robot_control::WrenchEstActuatorMeasBase");
  try
  {
    // 1. read the plugin name from the parameter server
    std::string wrench_estimator_name;
    nh_.param("wrench_estimator_name", wrench_estimator_name, std::string("aerial_robot_control::WrenchEstNone"));

    // 2. load the plugin
    wrench_est_ptr_ = wrench_est_loader_ptr_->createInstance(wrench_estimator_name);
    wrench_est_ptr_->initialize(nh_, robot_model_, estimator_, ctrl_loop_du_);
    ROS_INFO("load wrench estimator plugin: %s", wrench_estimator_name.c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("wrench_est_plugin: The plugin failed to load for some reason. Error: %s", ex.what());
  }
}

void nmpc::TiltMtServoDistNMPC::updateITerm()
{
  /* HANDLING MODEL ERROR */
  /* get the current state */
  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Quaternion q = estimator_->getQuat(Frame::COG, estimate_mode_);

  /* get the target state */
  // Note that the target state is the estimated state from NMPC solver.
  tf::Vector3 pos_x0(mpc_solver_ptr_->xo_.at(0).at(0), mpc_solver_ptr_->xo_.at(0).at(1),
                     mpc_solver_ptr_->xo_.at(0).at(2));
  tf::Vector3 pos_x1(mpc_solver_ptr_->xo_.at(1).at(0), mpc_solver_ptr_->xo_.at(1).at(1),
                     mpc_solver_ptr_->xo_.at(1).at(2));
  tf::Vector3 target_pos = pos_x0 + (pos_x1 - pos_x0) * t_nmpc_samp_ / t_nmpc_step_;

  tf::Quaternion quat_x0(mpc_solver_ptr_->xo_.at(0).at(7), mpc_solver_ptr_->xo_.at(0).at(8),
                         mpc_solver_ptr_->xo_.at(0).at(9), mpc_solver_ptr_->xo_.at(0).at(6));
  quat_x0.normalize();
  tf::Quaternion quat_x1(mpc_solver_ptr_->xo_.at(1).at(7), mpc_solver_ptr_->xo_.at(1).at(8),
                         mpc_solver_ptr_->xo_.at(1).at(9), mpc_solver_ptr_->xo_.at(1).at(6));
  quat_x1.normalize();
  tf::Quaternion target_q = quat_x0.slerp(quat_x1, t_nmpc_samp_ / t_nmpc_step_);

  /* update I term */
  wrench_est_i_term_.update(target_pos, target_q, pos, q);
}

void nmpc::TiltMtServoDistNMPC::initNMPCParams()
{
  TiltMtServoNMPC::initNMPCParams();
  idx_p_dist_end_ = idx_p_phys_end_ + 6;
}

void nmpc::TiltMtServoDistNMPC::prepareNMPCParams()
{
  TiltMtServoNMPC::prepareNMPCParams();

  // This form of I Term can always be activated, no need to be shut down when the external wrench appears.
  updateITerm();
  auto mdl_error_force_w = wrench_est_i_term_.getDistForceW();
  auto mdl_error_torque_cog = wrench_est_i_term_.getDistTorqueCOG();

  vector<int> idx = { idx_p_phys_end_ + 1, idx_p_phys_end_ + 2, idx_p_phys_end_ + 3,
                      idx_p_phys_end_ + 4, idx_p_phys_end_ + 5, idx_p_phys_end_ + 6 };
  vector<double> p = { mdl_error_force_w.x,    mdl_error_force_w.y,    mdl_error_force_w.z,
                       mdl_error_torque_cog.x, mdl_error_torque_cog.y, mdl_error_torque_cog.z };
  mpc_solver_ptr_->setParamSparseAllStages(idx, p);
}

std::vector<double> nmpc::TiltMtServoDistNMPC::meas2VecX(bool is_ee_centric)
{
  /* disturbance rejection */
  geometry_msgs::Vector3 external_force_w;     // default: 0, 0, 0
  geometry_msgs::Vector3 external_torque_cog;  // default: 0, 0, 0

  auto nav_state = navigator_->getNaviState();
  if (if_use_est_wrench_4_control_ && nav_state == aerial_robot_navigation::HOVER_STATE)
  {
    if (!wrench_est_ptr_->getOffsetFlag())
      wrench_est_ptr_->toggleOffsetFlag();

    // the external wrench is only added when the robot is in the hover state
    external_force_w = wrench_est_ptr_->getDistForceW();
    external_torque_cog = wrench_est_ptr_->getDistTorqueCOG();

    updateWrenchImpactCoeff(external_force_w, external_torque_cog);
  }

  vector<double> bx0 = TiltMtServoNMPC::meas2VecX(is_ee_centric);

  bx0[13 + joint_num_ + 0] = external_force_w.x * impact_coeff_force_;
  bx0[13 + joint_num_ + 1] = external_force_w.y * impact_coeff_force_;
  bx0[13 + joint_num_ + 2] = external_force_w.z * impact_coeff_force_;
  bx0[13 + joint_num_ + 3] = external_torque_cog.x * impact_coeff_torque_;
  bx0[13 + joint_num_ + 4] = external_torque_cog.y * impact_coeff_torque_;
  bx0[13 + joint_num_ + 5] = external_torque_cog.z * impact_coeff_torque_;

  return bx0;
}

void nmpc::TiltMtServoDistNMPC::updateDisturbWrench() const
{
  /* update the external wrench estimator based on the Nav State */
  auto nav_state = navigator_->getNaviState();

  if (nav_state != aerial_robot_navigation::TAKEOFF_STATE && nav_state != aerial_robot_navigation::HOVER_STATE &&
      nav_state != aerial_robot_navigation::LAND_STATE)
    return;

  if (estimator_->getPos(Frame::COG, estimate_mode_).z() < 0.3)  // TODO: change to a state: IN_AIR
    return;

  /* get external wrench */
  if (wrench_est_ptr_ != nullptr)
    wrench_est_ptr_->update();
  else
    ROS_ERROR("wrench_est_ptr_ is nullptr");
}

/* We use sigmoid function right now */
void nmpc::TiltMtServoDistNMPC::updateWrenchImpactCoeff(const geometry_msgs::Vector3& external_force_w,
                                                        const geometry_msgs::Vector3& external_torque_cog)
{
  const double external_force_norm =
      sqrt(external_force_w.x * external_force_w.x + external_force_w.y * external_force_w.y +
           external_force_w.z * external_force_w.z);
  const double external_torque_norm =
      sqrt(external_torque_cog.x * external_torque_cog.x + external_torque_cog.y * external_torque_cog.y +
           external_torque_cog.z * external_torque_cog.z);

  // use sigmoid function to limit the external force
  impact_coeff_force_ = 1.0 / (1.0 + exp(-steepness_force_ * (external_force_norm - thresh_force_)));
  impact_coeff_torque_ = 1.0 / (1.0 + exp(-steepness_torque_ * (external_torque_norm - thresh_torque_)));
}

void nmpc::TiltMtServoDistNMPC::pubDisturbWrench() const
{
  geometry_msgs::WrenchStamped dist_wrench_;
  dist_wrench_.header.frame_id = "beetle1/cog";

  auto ext_force_w = wrench_est_ptr_->getDistForceW();
  dist_wrench_.wrench.torque = wrench_est_ptr_->getDistTorqueCOG();

  tf::Matrix3x3 rot_mtx_cog2w = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 tf_dist_force_w = tf::Vector3(ext_force_w.x, ext_force_w.y, ext_force_w.z);
  tf::Vector3 tf_dist_force_cog = rot_mtx_cog2w.inverse() * tf_dist_force_w;
  dist_wrench_.wrench.force.x = tf_dist_force_cog.x();
  dist_wrench_.wrench.force.y = tf_dist_force_cog.y();
  dist_wrench_.wrench.force.z = tf_dist_force_cog.z();

  dist_wrench_.header.stamp = ros::Time::now();
  pub_disturb_wrench_.publish(dist_wrench_);

  // publish the disturbance wrench coefficient
  geometry_msgs::Vector3Stamped dist_wrench_coefficient;
  dist_wrench_coefficient.header.stamp = ros::Time::now();
  dist_wrench_coefficient.vector.x = impact_coeff_force_;
  dist_wrench_coefficient.vector.y = impact_coeff_torque_;
  pub_disturb_wrench_coefficient_.publish(dist_wrench_coefficient);
}

void nmpc::TiltMtServoDistNMPC::initAllocMat()
{
  TiltMtServoNMPC::initAllocMat();

  if (wrench_est_ptr_ != nullptr)
    wrench_est_ptr_->init_alloc_mtx(alloc_mat_, alloc_mat_pinv_);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoDistNMPC, aerial_robot_control::ControlBase);
