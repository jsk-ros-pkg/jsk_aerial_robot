//
// Created by li-jinjie on 24-10-25.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_MOMENTUM_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_MOMENTUM_H

#include "aerial_robot_control/wrench_est/wrench_est_base.h"
#include "aerial_robot_estimation/sensor/imu_4_wrench_est.h"

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
    ROS_INFO("WrenchEstMomentum initialize");
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

    Eigen::Vector3d vel_w, omega_cog;  // workaround: use the filtered value
    auto imu_handler = boost::dynamic_pointer_cast<sensor_plugin::Imu4WrenchEst>(estimator_->getImuHandler(0));
    tf::vectorTFToEigen(imu_handler->getFilteredVelCog(), vel_w);  // TODO: ?? I think all value is the CoG point in world frame
    tf::vectorTFToEigen(imu_handler->getFilteredOmegaCog(), omega_cog);
    Eigen::Matrix3d cog_rot;
    tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimator_->getEstimateMode()), cog_rot);

    Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
    double mass = robot_model_->getMass();

    Eigen::VectorXd sum_momentum = Eigen::VectorXd::Zero(6);
    sum_momentum.head(3) = mass * vel_w;
    sum_momentum.tail(3) = inertia * omega_cog;  // TODO: expressed in CoG frame or world frame?

    Eigen::MatrixXd J_t = Eigen::MatrixXd::Identity(6, 6);
    J_t.topLeftCorner(3, 3) = cog_rot;

    Eigen::VectorXd N = mass * robot_model_->getGravity();  // mg
    N.tail(3) = aerial_robot_model::skew(omega_cog) * (inertia * omega_cog);  // omega x (I omega)

    const Eigen::VectorXd target_wrench_acc_cog = getTargetWrenchAccCog();  // calculated from the control input or motor command
    Eigen::VectorXd target_wrench_cog = Eigen::VectorXd::Zero(6);
    target_wrench_cog.head(3) = mass * target_wrench_acc_cog.head(3);  // world frame
    target_wrench_cog.tail(3) = inertia * target_wrench_acc_cog.tail(3);  // CoG frame or world frame?

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

private:
  Eigen::VectorXd init_sum_momentum_;
  Eigen::VectorXd est_external_wrench_;
  Eigen::MatrixXd momentum_observer_matrix_;
  Eigen::VectorXd integrate_term_;
  double prev_est_wrench_timestamp_;
};

};  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_MOMENTUM_H
