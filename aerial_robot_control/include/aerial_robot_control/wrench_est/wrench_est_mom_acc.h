//
// Created by jinjie on 24/11/04.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_MOM_ACC_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_MOM_ACC_H

#include "aerial_robot_control/wrench_est/wrench_est_actuator_meas_base.h"
#include "aerial_robot_estimation/sensor/imu_4_wrench_est.h"

namespace aerial_robot_control
{

class WrenchEstMomAcc : public WrenchEstActuatorMeasBase
{
public:
  WrenchEstMomAcc() = default;

  void initialize(ros::NodeHandle& nh, boost::shared_ptr<aerial_robot_model::RobotModel>& robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator>& estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator>& navigator, double ctrl_loop_du)
  {
    WrenchEstActuatorMeasBase::initialize(nh, robot_model, estimator, navigator, ctrl_loop_du);

    // acceleration-based method for the force estimation
    force_acc_matrix_ = Eigen::MatrixXd::Identity(3, 3);
    double force_alpha_weight;
    ros::NodeHandle acceleration_nh(nh_, "controller/acceleration_observer");
    getParam<double>(acceleration_nh, "force_alpha_weight", force_alpha_weight, 0.5);
    force_acc_matrix_ *= force_alpha_weight;

    // momentum-based method for the torque estimation
    torque_mom_matrix_ = Eigen::MatrixXd::Identity(3, 3);

    double torque_weight;
    ros::NodeHandle momentum_nh(nh_, "controller/momentum_observer");
    getParam<double>(momentum_nh, "momentum_observer_torque_weight", torque_weight, 2.0);
    torque_mom_matrix_ *= torque_weight;

    est_external_torque_ = Eigen::VectorXd::Zero(3);
    init_sum_momentum_ = Eigen::VectorXd::Zero(3);
    integrate_term_ = Eigen::VectorXd::Zero(3);

    prev_est_wrench_timestamp_ = 0;
  }

  void update(const tf::Vector3& pos_ref, const tf::Quaternion& q_ref, const tf::Vector3& pos,
              const tf::Quaternion& q) override
  {
    // this function comes from Dragon:
    // https://github.com/jsk-ros-pkg/jsk_aerial_robot/blob/master/robots/dragon/src/control/full_vectoring_control.cpp#L783C6-L783C35
    if (navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE &&
        navigator_->getNaviState() != aerial_robot_navigation::LAND_STATE)   // TODO: move this part to outside
    {
      prev_est_wrench_timestamp_ = 0;
      integrate_term_ = Eigen::VectorXd::Zero(3);
      return;
    }

    auto imu_handler = boost::dynamic_pointer_cast<sensor_plugin::Imu4WrenchEst>(estimator_->getImuHandler(0));

    Eigen::VectorXd target_wrench_cog = calWrenchFromActuatorMeas();
    Eigen::VectorXd target_force_cog = target_wrench_cog.head(3);
    Eigen::VectorXd target_torque_cog = target_wrench_cog.tail(3);

    // torque estimation
    Eigen::Vector3d omega_cog;
    tf::vectorTFToEigen(imu_handler->getFilteredOmegaCogInCog(), omega_cog);  // the omega of CoG point in CoG frame
    Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
    Eigen::VectorXd sum_momentum = inertia * omega_cog;

    Eigen::MatrixXd J_t = Eigen::MatrixXd::Identity(3, 3);

    Eigen::VectorXd N = aerial_robot_model::skew(omega_cog) * (inertia * omega_cog);  // omega x (I omega)

    if (prev_est_wrench_timestamp_ == 0)
    {
      prev_est_wrench_timestamp_ = ros::Time::now().toSec();
      init_sum_momentum_ = sum_momentum;  // not good
    }

    double dt = ros::Time::now().toSec() - prev_est_wrench_timestamp_;

    integrate_term_ += (J_t * target_torque_cog - N + est_external_torque_) * dt;

    est_external_torque_ = torque_mom_matrix_ * (sum_momentum - init_sum_momentum_ - integrate_term_);

    prev_est_wrench_timestamp_ = ros::Time::now().toSec();

    setDistTorqueCOG(est_external_torque_(0), est_external_torque_(1), est_external_torque_(2));

  }

private:
  Eigen::MatrixXd force_acc_matrix_;

  // momentum-based method for the torque estimation
  Eigen::MatrixXd torque_mom_matrix_;
  Eigen::VectorXd init_sum_momentum_;
  Eigen::VectorXd est_external_torque_;
  Eigen::VectorXd integrate_term_;
  double prev_est_wrench_timestamp_;
};

}  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_MOM_ACC_H
