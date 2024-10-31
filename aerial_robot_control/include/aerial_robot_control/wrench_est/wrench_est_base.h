//
// Created by li-jinjie on 24-10-23.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_BASE_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_BASE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <aerial_robot_control/control/base/base.h>
#include <aerial_robot_control/flight_navigation.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <aerial_robot_model/model/aerial_robot_model_ros.h>

namespace aerial_robot_control
{
class WrenchEstBase
{
public:
  virtual inline void initialize(ros::NodeHandle& nh, boost::shared_ptr<aerial_robot_model::RobotModel>& robot_model,
                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator>& estimator,
                                 boost::shared_ptr<aerial_robot_navigation::BaseNavigator>& navigator,
                                 double ctrl_loop_du)
  {
    nh_ = nh;

    robot_model_ = robot_model;
    estimator_ = estimator;
    navigator_ = navigator;

    setCtrlLoopDu(ctrl_loop_du);
  };

  void inline init_alloc_mtx(Eigen::MatrixXd& alloc_mat, Eigen::MatrixXd& alloc_mat_pinv)
  {
    alloc_mat_ = alloc_mat;
    alloc_mat_pinv_ = alloc_mat_pinv;
  }

  virtual void update(const tf::Vector3& pos_ref, const tf::Quaternion& q_ref, const tf::Vector3& pos,
                      const tf::Quaternion& q) = 0;
  virtual ~WrenchEstBase() = default;

  /* getter */
  inline geometry_msgs::Vector3 getDistForceW()
  {
    return dist_force_w_;
  }
  inline geometry_msgs::Vector3 getDistTorqueCOG()
  {
    return dist_torque_cog_;
  }
  inline double getCtrlLoopDu() const
  {
    return ctrl_loop_du_;
  }

  /* setter */
  inline void setDistForceW(double x, double y, double z)
  {
    dist_force_w_.x = x;
    dist_force_w_.y = y;
    dist_force_w_.z = z;
  }
  inline void setDistTorqueCOG(double x, double y, double z)
  {
    dist_torque_cog_.x = x;
    dist_torque_cog_.y = y;
    dist_torque_cog_.z = z;
  }
  inline void setParamVerbose(bool param_verbose)
  {
    param_verbose_ = param_verbose;
  }
  inline void setCtrlLoopDu(double ctrl_loop_du)
  {
    ctrl_loop_du_ = ctrl_loop_du;
  }

protected:
  WrenchEstBase() = default;

  template <class T>
  void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
  {
    nh.param<T>(param_name, param, default_value);

    if (param_verbose_)
      ROS_INFO_STREAM("[" << nh.getNamespace() << "] " << param_name << ": " << param);
  }

protected:
  ros::NodeHandle nh_;

  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator_;
  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator_;

  Eigen::MatrixXd alloc_mat_;
  Eigen::MatrixXd alloc_mat_pinv_;

private:
  double ctrl_loop_du_;
  bool param_verbose_ = false;

  geometry_msgs::Vector3 dist_force_w_;     // disturbance force in world frame
  geometry_msgs::Vector3 dist_torque_cog_;  // disturbance torque in cog frame
};
};  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_BASE_H
