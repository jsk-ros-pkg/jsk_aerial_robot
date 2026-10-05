//
// Created by li-jinjie on 23-11-25.
//

#ifndef BASE_MPC_CONTROLLER_H
#define BASE_MPC_CONTROLLER_H

#include "aerial_robot_control/control/base/base.h"
#include "aerial_robot_control/nmpc/base_mpc_solver.h"
#include "aerial_robot_msgs/PredXU.h"

namespace aerial_robot_control
{
namespace nmpc
{

/**
 * @brief The BaseMPC class defines the common interface of NMPC controllers, especially for the ROS communication.
 */
class BaseMPC : public ControlBase
{
public:
  BaseMPC() = default;
  ~BaseMPC() override = default;

  // ----------- lifecycle -----------
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;
  void activate() override;
  bool update() override;
  void reset() override;

protected:
  // ----------- MPC solver -----------
  boost::shared_ptr<pluginlib::ClassLoader<aerial_robot_control::mpc_solver::BaseMPCSolver>> mpc_solver_loader_ptr_;
  boost::shared_ptr<aerial_robot_control::mpc_solver::BaseMPCSolver> mpc_solver_ptr_;

  // ----------- initialization steps -----------
  virtual void initPlugins() {};
  virtual void initGeneralParams() = 0;
  virtual void initNMPCCostW() = 0;
  virtual void initNMPCConstraints() = 0;

  // ----------- NMPC functions -----------
  virtual void initNMPCParams() = 0;
  virtual void prepareNMPCRef() = 0;
  virtual void prepareNMPCParams() {};

  virtual void callbackSetRefXU(const aerial_robot_msgs::PredXUConstPtr& msg) = 0;
  virtual std::vector<double> meas2VecX() = 0;
  virtual void controlCore(bool is_warmup = false) = 0;  // is_warmup: tell the function whether this is a warmup phase
  virtual void sendCmd() = 0;
  virtual void callbackViz(const ros::TimerEvent& event) = 0;

  // ----------- reset functions -----------
  virtual void resetPlugins() {};

  // ----------- Helper functions -----------
  static void initPredXU(aerial_robot_msgs::PredXU& x_u, int nn, int nx, int nu);
  static void rosXU2VecXU(const aerial_robot_msgs::PredXU& x_u, std::vector<std::vector<double>>& x_vec,
                          std::vector<std::vector<double>>& u_vec);

private:
  double dt_last_ = 0.0;
  ros::Time t_last_;
};

}  // namespace nmpc
}  // namespace aerial_robot_control

#endif  // BASE_MPC_CONTROLLER_H
