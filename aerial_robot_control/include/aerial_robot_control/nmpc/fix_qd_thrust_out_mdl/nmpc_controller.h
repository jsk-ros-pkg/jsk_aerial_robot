//
// Created by li-jinjie on 23-11-25.
//

#ifndef Fix_QD_NMPC_CONTROLLER_H
#define Fix_QD_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_qd_servo_mdl/nmpc_controller.h"
#include "nmpc_solver.h"

namespace aerial_robot_control
{

namespace nmpc
{

class FixQdNMPC : public nmpc::TiltQdServoNMPC
{
protected:
  void initAllocMat() override;

  void allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i, const tf::Quaternion& ref_quat_ib,
                    const tf::Vector3& ref_omega_b, const VectorXd& ref_wrench_b, vector<double>& x,
                    vector<double>& u) const override;
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // Fix_QD_NMPC_CONTROLLER_H
