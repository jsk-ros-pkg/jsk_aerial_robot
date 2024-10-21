//
// Created by lijinjie on 24/07/25.
//

#include "aerial_robot_control/nmpc/tilt_qd_servo_dist_mdl/nmpc_controller_indi.h"

using namespace aerial_robot_control;

void nmpc::TiltQdServoNMPCwINDI::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                            boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                            boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                            boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                            double ctrl_loop_du)
{
  TiltQdServoDistNMPC::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  sub_imu_ = nh_.subscribe("imu", 1, &TiltQdServoNMPCwINDI::callbackImu, this);
}

void nmpc::TiltQdServoNMPCwINDI::calcDisturbWrench()
{
}

void nmpc::TiltQdServoNMPCwINDI::callbackImu(const spinal::ImuConstPtr& msg)
{
  // calculate fBu
  const auto& rotor_p = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  Eigen::Vector3d p1_b = rotor_p[0];
  Eigen::Vector3d p2_b = rotor_p[1];
  Eigen::Vector3d p3_b = rotor_p[2];
  Eigen::Vector3d p4_b = rotor_p[3];

  double sqrt_p1b_xy = sqrt(p1_b.x() * p1_b.x() + p1_b.y() * p1_b.y());
  double sqrt_p2b_xy = sqrt(p2_b.x() * p2_b.x() + p2_b.y() * p2_b.y());
  double sqrt_p3b_xy = sqrt(p3_b.x() * p3_b.x() + p3_b.y() * p3_b.y());
  double sqrt_p4b_xy = sqrt(p4_b.x() * p4_b.x() + p4_b.y() * p4_b.y());

  double ft1 = getCommand(0);
  double ft2 = getCommand(1);
  double ft3 = getCommand(2);
  double ft4 = getCommand(3);
  double alpha1 = getCommand(4);
  double alpha2 = getCommand(5);
  double alpha3 = getCommand(6);
  double alpha4 = getCommand(7);

  Eigen::Vector3d fBu = Eigen::Vector3d::Zero();
  fBu(0) = ft1 * p1_b.y() * sin(alpha1) / sqrt_p1b_xy + ft2 * p2_b.y() * sin(alpha2) / sqrt_p2b_xy +
           ft3 * p3_b.y() * sin(alpha3) / sqrt_p3b_xy + ft4 * p4_b.y() * sin(alpha4) / sqrt_p4b_xy;
  fBu(1) = -ft1 * p1_b.x() * sin(alpha1) / sqrt_p1b_xy - ft2 * p2_b.x() * sin(alpha2) / sqrt_p2b_xy -
           ft3 * p3_b.x() * sin(alpha3) / sqrt_p3b_xy - ft4 * p4_b.x() * sin(alpha4) / sqrt_p4b_xy;
  fBu(2) = ft1 * cos(alpha1) + ft2 * cos(alpha2) + ft3 * cos(alpha3) + ft4 * cos(alpha4);

  // calculate RIB
  tf::Quaternion q = estimator_->getQuat(Frame::COG, estimate_mode_);
  double qw = q.w();
  double qx = q.x();
  double qy = q.y();
  double qz = q.z();

  Eigen::Matrix3d RIB = Eigen::Matrix3d::Zero();
  RIB(0, 0) = 1 - 2 * qy * qy - 2 * qz * qz;
  RIB(0, 1) = 2 * qx * qy - 2 * qz * qw;
  RIB(0, 2) = 2 * qx * qz + 2 * qy * qw;
  RIB(1, 0) = 2 * qx * qy + 2 * qz * qw;
  RIB(1, 1) = 1 - 2 * qx * qx - 2 * qz * qz;
  RIB(1, 2) = 2 * qy * qz - 2 * qx * qw;
  RIB(2, 0) = 2 * qx * qz - 2 * qy * qw;
  RIB(2, 1) = 2 * qy * qz + 2 * qx * qw;
  RIB(2, 2) = 1 - 2 * qx * qx - 2 * qy * qy;

  Eigen::Vector3d sfBIMU = Eigen::Vector3d(msg->acc_data[0], msg->acc_data[1], msg->acc_data[2]);

  // fId = RIB.(m * sfBIMU - fBu)
  Eigen::Vector3d fId = RIB * (mass_ * sfBIMU - fBu);

  if (estimator_->getPos(Frame::COG, estimate_mode_).z() > 0.05)
  {
    //    dist_force_w_.x = fId(0);
    //    dist_force_w_.y = fId(1);
    dist_force_w_.z = fId(2);
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltQdServoNMPCwINDI, aerial_robot_control::ControlBase);
