//
// Created by li-jinjie on 24-12-8.
//

#ifndef WRENCH_EST_MHE_ACC_MOM_H
#define WRENCH_EST_MHE_ACC_MOM_H

#include "aerial_robot_control/wrench_est/wrench_est_actuator_meas_base.h"
#include "aerial_robot_control/wrench_est/mhe_wrench_est_acc_mom_mdl/mhe_solver.h"

namespace aerial_robot_control
{

class WrenchEstMHEAccMom : public WrenchEstActuatorMeasBase
{
public:
  WrenchEstMHEAccMom() = default;

  void initialize(ros::NodeHandle& nh, boost::shared_ptr<aerial_robot_model::RobotModel>& robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator>& estimator, double ctrl_loop_du) override
  {
    WrenchEstActuatorMeasBase::initialize(nh, robot_model, estimator, ctrl_loop_du);

    mhe_solver_.initialize();
  }

  void reset() override
  {
    WrenchEstActuatorMeasBase::reset();
    mhe_solver_.reset();
  }

  void update() override
  {
    auto imu_handler = boost::dynamic_pointer_cast<sensor_plugin::Imu4WrenchEst>(estimator_->getImuHandler(0));

    Eigen::VectorXd meas_wrench_cog = calcWrenchFromActuatorMeas();
    Eigen::VectorXd meas_force_cog = meas_wrench_cog.head(3);
    Eigen::VectorXd meas_torque_cog = meas_wrench_cog.tail(3);

    // IMU measurement
    Eigen::Vector3d omega_cog;
    tf::vectorTFToEigen(imu_handler->getFilteredOmegaCogInCog(), omega_cog);

    Eigen::Vector3d specific_force_cog;  // the specific force of CoG point in CoG frame, i.e., acceleration - gravity
    tf::vectorTFToEigen(imu_handler->getFilteredAccCogInCog(), specific_force_cog);

    // force estimation
    double mass = robot_model_->getMass();

    Eigen::Matrix3d cog_rot;
    tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimator_->getEstimateMode()), cog_rot);

    auto est_external_force_now = cog_rot * (mass * specific_force_cog - meas_force_cog);  // body frame

    // parameters for MHE
    std::vector<double> meas_torque_cog_vec(meas_torque_cog.data(), meas_torque_cog.data() + meas_torque_cog.size());

    // measurement for MHE
    std::vector<double> input = { est_external_force_now(0),
                                  est_external_force_now(1),
                                  est_external_force_now(2),
                                  omega_cog(0),
                                  omega_cog(1),
                                  omega_cog(2) };

    // update MHE
    mhe_solver_.setMeasurement(input, meas_torque_cog_vec);

    try
    {
      mhe_solver_.solve();
    }
    catch (mhe_solver::AcadosSolveException& e)
    {
      ROS_ERROR("MHE solver failed. Details: %s", e.what());
    }

    auto x_est = mhe_solver_.getEstimatedState(mhe_solver_.NN_);

    setDistForceW(x_est.at(3), x_est.at(4), x_est.at(5));
    setDistTorqueCOG(x_est.at(6), x_est.at(7), x_est.at(8));

    WrenchEstActuatorMeasBase::update();
  }

private:
  mhe_solver::MHEWrenchEstAccMom mhe_solver_;
};

}  // namespace aerial_robot_control

#endif  // WRENCH_EST_MHE_ACC_MOM_H
