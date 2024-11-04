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
    ROS_INFO("WrenchEstMomAcc initialize");
  }

  void update(const tf::Vector3& pos_ref, const tf::Quaternion& q_ref, const tf::Vector3& pos,
              const tf::Quaternion& q) override
  {
    // do nothing
  }
};

}

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_MOM_ACC_H
