//
// Created by li-jinjie on 24-10-25.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_ACCELERATION_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_ACCELERATION_H

#include "aerial_robot_control/wrench_est/wrench_est_actuator_meas_base.h"

namespace aerial_robot_control
{

class WrenchEstAcceleration : public aerial_robot_control::WrenchEstActuatorMeasBase
{
public:
  WrenchEstAcceleration() = default;

  void initialize(ros::NodeHandle& nh, boost::shared_ptr<aerial_robot_model::RobotModel>& robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator>& estimator, double ctrl_loop_du) override
  {
    WrenchEstActuatorMeasBase::initialize(nh, robot_model, estimator, ctrl_loop_du);

    // read IIR parameters
    ros::NodeHandle accel_nh(nh_, "controller/accel_observer");

    std::vector<double> iir_general_num(3), iir_general_den(3);
    double iir_general_gain;
    accel_nh.getParam("iir_general/num", iir_general_num);
    accel_nh.getParam("iir_general/den", iir_general_den);
    accel_nh.getParam("iir_general/gain", iir_general_gain);
    for (int i = 0; i < 4; i++)
    {
      thrust_lpf_[i].setCoeffs(iir_general_num, iir_general_den, iir_general_gain);
    }
    for (int i = 0; i < 4; i++)
    {
      servo_lpf_[i].setCoeffs(iir_general_num, iir_general_den, iir_general_gain);
    }
    for (int i = 0; i < 3; i++)
    {
      acc_lpf_[i].setCoeffs(iir_general_num, iir_general_den, iir_general_gain);
    }
    for (int i = 0; i < 3; i++)
    {
      omega_lpf_[i].setCoeffs(iir_general_num, iir_general_den, iir_general_gain);
    }

    std::vector<double> iir_omega_dot_num(3), iir_omega_dot_den(3);
    double iir_omega_dot_gain;
    accel_nh.getParam("iir_omega_dot/num", iir_omega_dot_num);
    accel_nh.getParam("iir_omega_dot/den", iir_omega_dot_den);
    accel_nh.getParam("iir_omega_dot/gain", iir_omega_dot_gain);
    for (int i = 0; i < 3; i++)
    {
      omega_dot_lpf_[i].setCoeffs(iir_omega_dot_num, iir_omega_dot_den, iir_omega_dot_gain);
    }
  }

  void reset() override
  {
    WrenchEstActuatorMeasBase::reset();
    est_ext_force_cog_ = Eigen::VectorXd::Zero(3);
    est_ext_torque_cog_ = Eigen::VectorXd::Zero(3);
    for (auto& f : thrust_lpf_)
      f.reset();
    for (auto& f : servo_lpf_)
      f.reset();
    for (auto& f : acc_lpf_)
      f.reset();
    for (auto& f : omega_lpf_)
      f.reset();
    for (auto& f : omega_dot_lpf_)
      f.reset();
  }

  Eigen::VectorXd calDistWrench()
  {
    /* filter */
    auto imu_handler = boost::dynamic_pointer_cast<sensor_plugin::Imu4WrenchEst>(estimator_->getImuHandler(0));

    // acc
    Eigen::Vector3d specific_force_cog;  // the specific force of CoG point in CoG frame, i.e., acceleration - gravity
    tf::vectorTFToEigen(imu_handler->getAccCogInCog(), specific_force_cog);
    specific_force_cog(0) = acc_lpf_[0].filter(specific_force_cog(0));
    specific_force_cog(1) = acc_lpf_[1].filter(specific_force_cog(1));
    specific_force_cog(2) = acc_lpf_[2].filter(specific_force_cog(2));

    // ang_vel (omega)
    Eigen::Vector3d omega_cog;
    tf::vectorTFToEigen(imu_handler->getOmegaCogInCog(), omega_cog);
    omega_cog(0) = omega_lpf_[0].filter(omega_cog(0));
    omega_cog(1) = omega_lpf_[1].filter(omega_cog(1));
    omega_cog(2) = omega_lpf_[2].filter(omega_cog(2));

    // ang_acc (omega_dot)
    Eigen::Vector3d omega_dot_cog;
    tf::vectorTFToEigen(imu_handler->getOmegaDotCogInCog(), omega_dot_cog);
    omega_dot_cog(0) = omega_dot_lpf_[0].filter(omega_dot_cog(0));
    omega_dot_cog(1) = omega_dot_lpf_[1].filter(omega_dot_cog(1));
    omega_dot_cog(2) = omega_dot_lpf_[2].filter(omega_dot_cog(2));

    // servo_angle
    std::vector<double> joint_angles_filtered;
    joint_angles_filtered.reserve(joint_angles_.size());
    for (size_t i = 0; i < joint_angles_.size(); ++i)
    {
      joint_angles_filtered.push_back(servo_lpf_[i].filter(joint_angles_[i]));
    }

    // thrust --- NOTE: here we use thrust_cmd_ instead of thrust_meas_ to decrease noise!!!
    std::vector<double> thrust_filtered;
    thrust_filtered.reserve(thrust_cmd_.size());
    for (size_t i = 0; i < thrust_cmd_.size(); ++i)
    {
      thrust_filtered.push_back(thrust_lpf_[i].filter(thrust_cmd_[i]));
    }

    /* calculation */
    // nominal wrench in CoG frame
    Eigen::VectorXd target_wrench_cog = calcWrenchFromActuatorMeas(thrust_filtered, joint_angles_filtered);
    Eigen::VectorXd target_force_cog = target_wrench_cog.head(3);
    Eigen::VectorXd target_torque_cog = target_wrench_cog.tail(3);

    // force estimation
    double mass = robot_model_->getMass();
    auto external_force_cog = mass * specific_force_cog - target_force_cog;

    // torque estimation
    Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
    Eigen::VectorXd torque_imu_cog =
        inertia * omega_dot_cog + aerial_robot_model::skew(omega_cog) * (inertia * omega_cog);
    auto external_torque_cog = torque_imu_cog - target_torque_cog;

    // combine the force and torque
    Eigen::VectorXd external_wrench_cog(6);
    external_wrench_cog << external_force_cog, external_torque_cog;
    return external_wrench_cog;
  }

  void update() override
  {
    /* calculate the external wrench in the CoG frame */
    Eigen::VectorXd external_wrench_cog = calDistWrench();
    Eigen::VectorXd est_ext_force_cog_ = external_wrench_cog.head(3);
    Eigen::VectorXd est_ext_torque_cog_ = external_wrench_cog.tail(3);

    // coordinate transformation
    Eigen::Matrix3d cog_rot;
    tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimator_->getEstimateMode()), cog_rot);
    auto est_ext_force_w = cog_rot * est_ext_force_cog_;

    setDistForceW(est_ext_force_w(0), est_ext_force_w(1), est_ext_force_w(2));
    setDistTorqueCOG(est_ext_torque_cog_(0), est_ext_torque_cog_(1), est_ext_torque_cog_(2));

    WrenchEstActuatorMeasBase::update();
  }

private:
  // lpf filters
  std::array<digital_filter::IIRFilter<3, 3>, 4> thrust_lpf_;
  std::array<digital_filter::IIRFilter<3, 3>, 4> servo_lpf_;
  std::array<digital_filter::IIRFilter<3, 3>, 3> acc_lpf_;
  std::array<digital_filter::IIRFilter<3, 3>, 3> omega_lpf_;
  std::array<digital_filter::IIRFilter<3, 3>, 3> omega_dot_lpf_;

  // acceleration-based method
  Eigen::VectorXd est_ext_force_cog_;  // TODO: combine these two variables and matrices to a single one
  Eigen::VectorXd est_ext_torque_cog_;
};

};  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_ACCELERATION_H
