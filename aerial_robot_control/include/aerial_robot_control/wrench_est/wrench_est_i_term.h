//
// Created by li-jinjie on 24-10-24.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_I_TERM_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_I_TERM_H

#include "aerial_robot_control/wrench_est/wrench_est_base.h"
#include "i_term.h"

/* dynamic reconfigure */
#include <dynamic_reconfigure/server.h>
#include "aerial_robot_msgs/DynamicReconfigureLevels.h"
#include "aerial_robot_control/ITermConfig.h"

using ITermDynamicConfig = dynamic_reconfigure::Server<aerial_robot_control::ITermConfig>;

namespace aerial_robot_control
{

class WrenchEstITerm : public WrenchEstBase
{
public:
  WrenchEstITerm() = default;

  void initialize(ros::NodeHandle& nh, boost::shared_ptr<aerial_robot_model::RobotModel>& robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator>& estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator>& navigator, double ctrl_loop_du) override;
  void update(const tf::Vector3& pos_ref, const tf::Quaternion& q_ref, const tf::Vector3& pos,
              const tf::Quaternion& q) override;

private:
  ITerm pos_i_term_[6];  // x, y, z, roll, pitch, yaw

  std::vector<boost::shared_ptr<ITermDynamicConfig>> reconf_servers_;

  void cfgCallback(ITermConfig& config, uint32_t level);
};

}  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_I_TERM_H
