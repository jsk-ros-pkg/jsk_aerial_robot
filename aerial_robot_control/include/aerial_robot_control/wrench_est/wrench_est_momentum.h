//
// Created by li-jinjie on 24-10-25.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_MOMENTUM_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_MOMENTUM_H

#include "aerial_robot_control/wrench_est/wrench_est_base.h"
#include "aerial_robot_estimation/sensor/imu_4_wrench_est.h"

#include "sensor_msgs/JointState.h"
#include "spinal/ESCTelemetryArray.h"

namespace aerial_robot_control
{

class WrenchEstMomentum : public aerial_robot_control::WrenchEstBase
{
public:
  WrenchEstMomentum() = default;

  void initialize(ros::NodeHandle& nh, boost::shared_ptr<aerial_robot_model::RobotModel>& robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator>& estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator>& navigator, double ctrl_loop_du) override
  {
    WrenchEstBase::initialize(nh, robot_model, estimator, navigator, ctrl_loop_du);

    // initialize the matrix
    momentum_observer_matrix_ = Eigen::MatrixXd::Identity(6, 6);
    double force_weight, torque_weight;
    ros::NodeHandle momentum_nh(nh_, "controller/momentum_observer");
    getParam<double>(momentum_nh, "momentum_observer_force_weight", force_weight, 3.0);
    getParam<double>(momentum_nh, "momentum_observer_torque_weight", torque_weight, 2.0);
    momentum_observer_matrix_.topRows(3) *= force_weight;
    momentum_observer_matrix_.bottomRows(3) *= torque_weight;

    // TODO: combine this part with the controller. especially the subscriber part.
    ros::NodeHandle motor_nh(nh_, "motor_info");
    getParam<double>(motor_nh, "krpm_rate", krpm2_d_thrust_, 0.0);

    joint_angles_.resize(robot_model_->getJointNum(), 0.0);
    sub_joint_states_ = nh_.subscribe("joint_states", 1, &WrenchEstMomentum::callbackJointStates, this);

    thrust_meas_.resize(robot_model_->getRotorNum(), 0.0);
    sub_esc_telem_ = nh_.subscribe("esc_telem", 1, &WrenchEstMomentum::callbackESCTelem, this);
  }

  void update(const tf::Vector3& pos_ref, const tf::Quaternion& q_ref, const tf::Vector3& pos,
              const tf::Quaternion& q) override
  {
    // this function comes from Dragon:
    // https://github.com/jsk-ros-pkg/jsk_aerial_robot/blob/master/robots/dragon/src/control/full_vectoring_control.cpp#L783C6-L783C35
    if (navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE &&
        navigator_->getNaviState() != aerial_robot_navigation::LAND_STATE)
    {
      prev_est_wrench_timestamp_ = 0;
      integrate_term_ = Eigen::VectorXd::Zero(6);
      return;
    }

    Eigen::Vector3d vel_w, omega_cog;
    auto imu_handler = boost::dynamic_pointer_cast<sensor_plugin::Imu4WrenchEst>(estimator_->getImuHandler(0));
    tf::vectorTFToEigen(imu_handler->getFilteredVelCog(), vel_w);        // the vel of CoG point in world frame
    tf::vectorTFToEigen(imu_handler->getFilteredOmegaCog(), omega_cog);  // the omega of CoG point in CoG frame
    Eigen::Matrix3d cog_rot;
    tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimator_->getEstimateMode()), cog_rot);

    Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
    double mass = robot_model_->getMass();

    Eigen::VectorXd sum_momentum = Eigen::VectorXd::Zero(6);
    sum_momentum.head(3) = mass * vel_w;
    sum_momentum.tail(3) = inertia * omega_cog;

    Eigen::MatrixXd J_t = Eigen::MatrixXd::Identity(6, 6);
    J_t.topLeftCorner(3, 3) = cog_rot;

    Eigen::VectorXd N = mass * robot_model_->getGravity();                    // mg
    N.tail(3) = aerial_robot_model::skew(omega_cog) * (inertia * omega_cog);  // omega x (I omega)

    Eigen::VectorXd target_wrench_cog = calWrenchFromActuatorMeas();  // The wrench is from the actuator measurement

    if (prev_est_wrench_timestamp_ == 0)
    {
      prev_est_wrench_timestamp_ = ros::Time::now().toSec();
      init_sum_momentum_ = sum_momentum;  // not good
    }

    double dt = ros::Time::now().toSec() - prev_est_wrench_timestamp_;

    integrate_term_ += (J_t * target_wrench_cog - N + est_external_wrench_) * dt;

    // TODO: check: for est_external_wrench_, force is in world frame, torque is in CoG frame
    est_external_wrench_ = momentum_observer_matrix_ * (sum_momentum - init_sum_momentum_ - integrate_term_);

    /* set value */
    setDistForceW(est_external_wrench_(0), est_external_wrench_(1), est_external_wrench_(2));
    setDistTorqueCOG(est_external_wrench_(3), est_external_wrench_(4), est_external_wrench_(5));
  }

  Eigen::VectorXd calWrenchFromActuatorMeas()
  {
    Eigen::VectorXd z = Eigen::VectorXd::Zero(2 * robot_model_->getRotorNum());
    for (int i = 0; i < robot_model_->getRotorNum(); i++)
    {
      z(2 * i) = thrust_meas_[i] * sin(joint_angles_[i]);
      z(2 * i + 1) = thrust_meas_[i] * cos(joint_angles_[i]);
    }

    // allocate * z
    Eigen::VectorXd est_wrench_b = alloc_mat_ * z;
    return est_wrench_b;
  }

private:
  Eigen::VectorXd init_sum_momentum_;
  Eigen::VectorXd est_external_wrench_;
  Eigen::MatrixXd momentum_observer_matrix_;
  Eigen::VectorXd integrate_term_;
  double prev_est_wrench_timestamp_;

  // for servo angles
  std::vector<double> joint_angles_;

  ros::Subscriber sub_joint_states_;
  void callbackJointStates(const sensor_msgs::JointStateConstPtr& msg)
  {
    for (int i = 0; i < robot_model_->getJointNum(); i++)
      joint_angles_[i] = msg->position[i];
  }

  // for actual thrust
  std::vector<double> thrust_meas_;

  ros::Subscriber sub_esc_telem_;
  double krpm2_d_thrust_;
  void callbackESCTelem(const spinal::ESCTelemetryArrayConstPtr& msg)
  {
    double krpm;
    krpm = (double)msg->esc_telemetry_1.rpm * 0.001;
    thrust_meas_[0] = krpm * krpm / krpm2_d_thrust_;

    krpm = (double)msg->esc_telemetry_2.rpm * 0.001;
    thrust_meas_[1] = krpm * krpm / krpm2_d_thrust_;

    krpm = (double)msg->esc_telemetry_3.rpm * 0.001;
    thrust_meas_[2] = krpm * krpm / krpm2_d_thrust_;

    krpm = (double)msg->esc_telemetry_4.rpm * 0.001;
    thrust_meas_[3] = krpm * krpm / krpm2_d_thrust_;
  }
};

};  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_MOMENTUM_H
