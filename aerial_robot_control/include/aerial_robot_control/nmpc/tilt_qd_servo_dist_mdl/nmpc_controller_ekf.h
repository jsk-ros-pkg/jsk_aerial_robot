//
// Created by lijinjie on 24/07/18.
//

#ifndef TILT_QD_SERVO_NMPC_W_EKF_CONTROLLER_H
#define TILT_QD_SERVO_NMPC_W_EKF_CONTROLLER_H

#include "nmpc_controller.h"
#include "nsim_solver.h"
#include "aerial_robot_control/nmpc/ekf_estimator.h"

#include "geometry_msgs/WrenchStamped.h"
#include "spinal/Imu.h"

using NMPCControlDynamicConfig = dynamic_reconfigure::Server<aerial_robot_control::NMPCConfig>;

namespace aerial_robot_control
{

namespace nmpc
{

class TiltQdServoNMPCwEKF : public nmpc::TiltQdServoDistNMPC
{
public:
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;

protected:
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_mocap_;

  std::unique_ptr<mpc_sim_solver::TiltQdServoDistMdlSimSolver> sim_solver_ptr_;
  EKFEstimator ekf_;

  inline void initMPCSolverPtr() override
  {
    mpc_solver_ptr_ = std::make_unique<mpc_solver::TiltQdServoDistMdlMPCSolver>();
    sim_solver_ptr_ = std::make_unique<mpc_sim_solver::TiltQdServoDistMdlSimSolver>();
  }

  void initParams() override;

  void calcDisturbWrench() override;

  std::vector<double> meas2VecX() override;

  void callbackImu(const spinal::ImuConstPtr& msg);
  void callbackMoCap(const geometry_msgs::PoseStampedConstPtr& msg);
  void callbackJointStates(const sensor_msgs::JointStateConstPtr& msg) override;

  void cfgNMPCCallback(NMPCConfig& config, uint32_t level) override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_QD_SERVO_NMPC_W_EKF_CONTROLLER_H