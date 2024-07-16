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
 * @brief The BaseMPC class to define the common interface of NMPC controllers, especially for the ROS communication.
 */
class BaseMPC : public ControlBase
{
public:
protected:
  /* initialize() */
  // define the ROS msg related to MPC
  inline static void initPredXU(aerial_robot_msgs::PredXU& x_u, int nn, int nx, int nu)
  {
    x_u.x.layout.dim.emplace_back();
    x_u.x.layout.dim.emplace_back();
    x_u.x.layout.dim[0].label = "horizon";
    x_u.x.layout.dim[0].size = nn + 1;
    x_u.x.layout.dim[0].stride = (nn + 1) * nx;
    x_u.x.layout.dim[1].label = "state";
    x_u.x.layout.dim[1].size = nx;
    x_u.x.layout.dim[1].stride = nx;
    x_u.x.layout.data_offset = 0;
    x_u.x.data.resize((nn + 1) * nx);
    std::fill(x_u.x.data.begin(), x_u.x.data.end(), 0.0);
    // quaternion
    for (int i = 6; i < (nn + 1) * nx; i += nx)
      x_u.x.data[i] = 1.0;

    x_u.u.layout.dim.emplace_back();
    x_u.u.layout.dim.emplace_back();
    x_u.u.layout.dim[0].label = "horizon";
    x_u.u.layout.dim[0].size = nn;
    x_u.u.layout.dim[0].stride = nn * nu;
    x_u.u.layout.dim[1].label = "input";
    x_u.u.layout.dim[1].size = nu;
    x_u.u.layout.dim[1].stride = nu;
    x_u.u.layout.data_offset = 0;
    x_u.u.data.resize(nn * nu);
    std::fill(x_u.u.data.begin(), x_u.u.data.end(), 0.0);
  }
  // define MPC solver
  std::unique_ptr<mpc_solver::BaseMPCSolver> mpc_solver_ptr_;
  virtual inline void initMPCSolverPtr() = 0;

  /* update() */
  // 1. set reference for NMPC
  virtual void callbackSetRefXU(const aerial_robot_msgs::PredXUConstPtr& msg) = 0;
  static void rosXU2VecXU(const aerial_robot_msgs::PredXU& x_u, std::vector<std::vector<double>>& x_vec,
                          std::vector<std::vector<double>>& u_vec)
  {
    int NN = (int)u_vec.size();  // x_vec is NN+1
    int NX = (int)x_vec[0].size();
    int NU = (int)u_vec[0].size();

    if (x_u.x.layout.dim[1].stride != NX || x_u.u.layout.dim[1].stride != NU)
      ROS_ERROR("The dimension of x_u: nx, nu is not correct!");

    if (x_u.x.layout.dim[0].size != NN + 1 || x_u.u.layout.dim[0].size != NN)
      ROS_ERROR("The dimension of x_u: nn for x, nn for u is not correct!");

    // convert x_u to xr_ and ur_
    for (int i = 0; i < NN; i++)
    {
      std::copy(x_u.x.data.begin() + i * NX, x_u.x.data.begin() + (i + 1) * NX, x_vec[i].begin());
      std::copy(x_u.u.data.begin() + i * NU, x_u.u.data.begin() + (i + 1) * NU, u_vec[i].begin());
    }
    std::copy(x_u.x.data.begin() + NN * NX, x_u.x.data.begin() + (NN + 1) * NX, x_vec[NN].begin());
  }
  // 2. get the current state from the measurement
  virtual std::vector<double> meas2VecX() = 0;
  // 3. solve NMPC
  virtual void controlCore() = 0;
  // 4. send the command to the robot
  virtual void sendCmd() = 0;
  // 5. viz the result in RVIZ
  virtual void callbackViz(const ros::TimerEvent& event) = 0;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // BASE_MPC_CONTROLLER_H
