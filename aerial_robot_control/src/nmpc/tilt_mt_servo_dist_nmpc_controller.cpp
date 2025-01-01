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

  pub_disturb_wrench_ = nh_.advertise<geometry_msgs::WrenchStamped>("disturbance_wrench", 1);
}

bool nmpc::TiltMtServoDistNMPC::update()
{
  if (!TiltMtServoNMPC::update())
    return false;

  // pub the est. wrench used by the controller, not the latest one. so this line is put before calcDisturbWrench()
  pubDisturbWrench();

  calcDisturbWrench();

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

void nmpc::TiltMtServoDistNMPC::prepareNMPCParams()
{
  /* HANDLING MODEL ERROR */
  // TODO: wrap this part as a function
  /* get the current state */
  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Quaternion q = estimator_->getQuat(Frame::COG, estimate_mode_);

  /* get the target state */
  tf::Vector3 target_pos;
  tf::Quaternion target_q;
  if (is_traj_tracking_)
  {
    target_pos.setX(x_u_ref_.x.data.at(0));
    target_pos.setY(x_u_ref_.x.data.at(1));
    target_pos.setZ(x_u_ref_.x.data.at(2));

    target_q.setW(x_u_ref_.x.data.at(6));
    target_q.setX(x_u_ref_.x.data.at(7));
    target_q.setY(x_u_ref_.x.data.at(8));
    target_q.setZ(x_u_ref_.x.data.at(9));
  }
  else
  {
    target_pos = navigator_->getTargetPos();

    tf::Vector3 target_rpy = navigator_->getTargetRPY();
    target_q.setRPY(target_rpy.x(), target_rpy.y(), target_rpy.z());
  }

  /* update I term */
  tf::Vector3 vel = estimator_->getVel(Frame::COG, estimate_mode_);
  // if the norm of vel is less than 0.2 m/s, than the I term is updated.
  if (vel.length() < 0.1)
  {
    wrench_est_i_term_.update(target_pos, target_q, pos, q);
  }

  auto mdl_error_force_w = wrench_est_i_term_.getDistForceW();
  auto mdl_error_torque_cog = wrench_est_i_term_.getDistTorqueCOG();

  vector<int> idx = { 4, 5, 6, 7, 8, 9 };
  vector<double> p = { mdl_error_force_w.x,    mdl_error_force_w.y,    mdl_error_force_w.z,
                       mdl_error_torque_cog.x, mdl_error_torque_cog.y, mdl_error_torque_cog.z };
  mpc_solver_ptr_->setParamSparseAllStages(idx, p);
}

std::vector<double> nmpc::TiltMtServoDistNMPC::meas2VecX()
{
  vector<double> bx0 = TiltMtServoNMPC::meas2VecX();

  /* disturbance rejection */
  geometry_msgs::Vector3 external_force_w;  // default: 0, 0, 0
  geometry_msgs::Vector3 external_torque_cog;  // default: 0, 0, 0

  if (if_use_est_wrench_4_control_)
  {
    external_force_w = wrench_est_ptr_->getDistForceW();
    external_torque_cog = wrench_est_ptr_->getDistTorqueCOG();
  }

  bx0[13 + joint_num_ + 0] = external_force_w.x;
  bx0[13 + joint_num_ + 1] = external_force_w.y;
  bx0[13 + joint_num_ + 2] = external_force_w.z;
  bx0[13 + joint_num_ + 3] = external_torque_cog.x;
  bx0[13 + joint_num_ + 4] = external_torque_cog.y;
  bx0[13 + joint_num_ + 5] = external_torque_cog.z;

  return bx0;
}

void nmpc::TiltMtServoDistNMPC::calcDisturbWrench()
{
  /* update the external wrench estimator based on the Nav State */
  auto nav_state = navigator_->getNaviState();

  if (nav_state == aerial_robot_navigation::TAKEOFF_STATE || nav_state == aerial_robot_navigation::HOVER_STATE ||
      nav_state == aerial_robot_navigation::LAND_STATE)
  {
    if (estimator_->getPos(Frame::COG, estimate_mode_).z() > 0.3)  // TODO: change to a state: IN_AIR
    {
      /* get external wrench */
      if (wrench_est_ptr_ != nullptr)
        wrench_est_ptr_->update();
      else
        ROS_ERROR("wrench_est_ptr_ is nullptr");
    }

    // the external wrench is only added when the robot is in the hover state
    if (if_use_est_wrench_4_control_ && nav_state == aerial_robot_navigation::HOVER_STATE)
    {
      if (!wrench_est_ptr_->getOffsetFlag())
        wrench_est_ptr_->toggleOffsetFlag();
    }
  }
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
