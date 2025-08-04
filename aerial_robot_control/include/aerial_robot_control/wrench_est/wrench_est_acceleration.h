//
// Created by li-jinjie on 24-10-25.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_ACCELERATION_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_ACCELERATION_H

#include "aerial_robot_control/wrench_est/wrench_est_actuator_meas_base.h"
#include "aerial_robot_control/wrench_est/utils.h"

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

    ros::NodeHandle accel_nh(nh_, "controller/accel_observer");

    accel_nh.getParam("thrust_type", thrust_type_);

    // IIR of sensors
    std::vector<double> iir_sensor_general_num(3), iir_sensor_general_den(3);
    double iir_sensor_general_gain;
    accel_nh.getParam("iir_sensor_general/num", iir_sensor_general_num);
    accel_nh.getParam("iir_sensor_general/den", iir_sensor_general_den);
    accel_nh.getParam("iir_sensor_general/gain", iir_sensor_general_gain);
    for (int i = 0; i < 4; i++)
    {
      thrust_lpf_[i].setCoeffs(iir_sensor_general_num, iir_sensor_general_den, iir_sensor_general_gain);
    }
    for (int i = 0; i < 4; i++)
    {
      servo_lpf_[i].setCoeffs(iir_sensor_general_num, iir_sensor_general_den, iir_sensor_general_gain);
    }
    for (int i = 0; i < 3; i++)
    {
      acc_lpf_[i].setCoeffs(iir_sensor_general_num, iir_sensor_general_den, iir_sensor_general_gain);
    }
    for (int i = 0; i < 3; i++)
    {
      omega_lpf_[i].setCoeffs(iir_sensor_general_num, iir_sensor_general_den, iir_sensor_general_gain);
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

    // IIR of wrench
    std::vector<double> iir_wrench_num(3), iir_wrench_den(3);
    double iir_wrench_gain;
    accel_nh.getParam("iir_wrench/num", iir_wrench_num);
    accel_nh.getParam("iir_wrench/den", iir_wrench_den);
    accel_nh.getParam("iir_wrench/gain", iir_wrench_gain);
    for (int i = 0; i < 3; i++)
    {
      ext_force_lpf_[i].setCoeffs(iir_wrench_num, iir_wrench_den, iir_wrench_gain);
      ext_torque_lpf_[i].setCoeffs(iir_wrench_num, iir_wrench_den, iir_wrench_gain);
    }

    // get the rotor time constant
    ros::NodeHandle physical_nh(nh_, "physical");
    getParam<double>(physical_nh, "t_rotor", ts_rotor_, 0.09);
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
    for (auto& f : ext_force_lpf_)
      f.reset();
    for (auto& f : ext_torque_lpf_)
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
    std::vector<double> joint_angles_filtered(joint_angles_.size());
    for (size_t i = 0; i < joint_angles_.size(); ++i)
    {
      joint_angles_filtered[i] = servo_lpf_[i].filter(joint_angles_[i]);
    }

    // thrust --- NOTE: using thrust_cmd_ instead of thrust_meas_ can decrease noise, but it introduces delay.
    std::vector<double> thrust_filtered(robot_model_->getRotorNum());
    if (thrust_type_ == 0)  // thrust type 0: use thrust command
    {
      double alpha = getCtrlLoopDu() / (getCtrlLoopDu() + ts_rotor_);
      for (size_t i = 0; i < thrust_cmd_.size(); ++i)
      {
        // Note: the thrust command is delayed by ts_rotor_ seconds to have effect, so we need to compensate for it.
        double thrust_cmd_delayed = thrust_cmd_[i] * alpha + (1 - alpha) * thrust_lpf_[i].getLastOutput();
        thrust_filtered[i] = thrust_lpf_[i].filter(thrust_cmd_delayed);
      }
    }
    else if (thrust_type_ == 1)  // thrust type 1: use actual thrust
    {
      for (size_t i = 0; i < thrust_meas_.size(); ++i)
      {
        thrust_filtered[i] = thrust_lpf_[i].filter(thrust_meas_[i]);
      }
    }
    else
    {
      ROS_ERROR("Unknown thrust type: %d", thrust_type_);
      return Eigen::VectorXd::Zero(6);
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

    // filter the external force and torque
    for (int i = 0; i < 3; ++i)
    {
      est_ext_force_cog_(i) = ext_force_lpf_[i].filter(est_ext_force_cog_(i));
      est_ext_torque_cog_(i) = ext_torque_lpf_[i].filter(est_ext_torque_cog_(i));
    }

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
  std::array<BiquadIIR, 4> thrust_lpf_;
  std::array<BiquadIIR, 4> servo_lpf_;
  std::array<BiquadIIR, 3> acc_lpf_;
  std::array<BiquadIIR, 3> omega_lpf_;
  std::array<BiquadIIR, 3> omega_dot_lpf_;

  std::array<BiquadIIR, 3> ext_force_lpf_;
  std::array<BiquadIIR, 3> ext_torque_lpf_;

  int thrust_type_;
  double ts_rotor_;  // time step for rotor, compensate for the delay of thrust command

  // acceleration-based method
  Eigen::VectorXd est_ext_force_cog_;  // TODO: combine these two variables and matrices to a single one
  Eigen::VectorXd est_ext_torque_cog_;
};

};  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_ACCELERATION_H
