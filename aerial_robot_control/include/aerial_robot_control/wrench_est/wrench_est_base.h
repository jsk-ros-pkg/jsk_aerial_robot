//
// Created by li-jinjie on 24-10-23.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_BASE_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_BASE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <aerial_robot_control/control/base/base.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <aerial_robot_model/model/aerial_robot_model_ros.h>

namespace aerial_robot_control
{
class WrenchEstBase
{
public:
  virtual ~WrenchEstBase() = default;

  virtual void initialize(ros::NodeHandle& nh, boost::shared_ptr<aerial_robot_model::RobotModel>& robot_model,
                          boost::shared_ptr<aerial_robot_estimation::StateEstimator>& estimator, double ctrl_loop_du)
  {
    nh_ = nh;

    robot_model_ = robot_model;
    estimator_ = estimator;

    setCtrlLoopDu(ctrl_loop_du);

    tmr_pub_dist_wrench_ = nh_.createTimer(ros::Duration(0.1), &WrenchEstBase::cbPubDistWrench, this);
    initWrenchPub();  // TODO: this function is only used to specify the topic name. Try a more elegant method later

    reset();
  };

  virtual void initWrenchPub() = 0;

  virtual void reset()
  {
    dist_force_w_ = Eigen::Vector3d::Zero();
    dist_torque_cog_ = Eigen::Vector3d::Zero();
  }

  void init_alloc_mtx(Eigen::MatrixXd& alloc_mat, Eigen::MatrixXd& alloc_mat_pinv)
  {
    alloc_mat_ = alloc_mat;
    alloc_mat_pinv_ = alloc_mat_pinv;
  }

  /* getter */
  virtual geometry_msgs::Vector3 getDistForceW() const
  {
    geometry_msgs::Vector3 dist_force_w_ros;
    dist_force_w_ros.x = dist_force_w_(0);
    dist_force_w_ros.y = dist_force_w_(1);
    dist_force_w_ros.z = dist_force_w_(2);

    return dist_force_w_ros;
  }
  virtual geometry_msgs::Vector3 getDistTorqueCOG() const
  {
    geometry_msgs::Vector3 dist_torque_cog_ros;
    dist_torque_cog_ros.x = dist_torque_cog_(0);
    dist_torque_cog_ros.y = dist_torque_cog_(1);
    dist_torque_cog_ros.z = dist_torque_cog_(2);

    return dist_torque_cog_ros;
  }
  double getCtrlLoopDu() const
  {
    return ctrl_loop_du_;
  }

  /* setter */
  void setDistForceW(double x, double y, double z)
  {
    dist_force_w_ << x, y, z;
  }
  void setDistTorqueCOG(double x, double y, double z)
  {
    dist_torque_cog_ << x, y, z;
  }
  void setParamVerbose(bool param_verbose)
  {
    param_verbose_ = param_verbose;
  }
  void setCtrlLoopDu(double ctrl_loop_du)
  {
    ctrl_loop_du_ = ctrl_loop_du;
  }

protected:
  ros::NodeHandle nh_;

  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator_;

  Eigen::MatrixXd alloc_mat_;
  Eigen::MatrixXd alloc_mat_pinv_;

  ros::Timer tmr_pub_dist_wrench_;
  ros::Publisher pub_disturb_wrench_;

  Eigen::Vector3d dist_force_w_;     // disturbance force in world frame
  Eigen::Vector3d dist_torque_cog_;  // disturbance torque in cog frame

  WrenchEstBase() = default;

  // Note that force is in world frame, and torque is in CoG frame. Only for debug.
  virtual void cbPubDistWrench(const ros::TimerEvent& event) const
  {
    geometry_msgs::WrenchStamped dist_wrench_;
    dist_wrench_.header.stamp = ros::Time::now();

    dist_wrench_.wrench.force = getDistForceW();
    dist_wrench_.wrench.torque = getDistTorqueCOG();

    pub_disturb_wrench_.publish(dist_wrench_);
  }

  template <class T>
  void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
  {
    nh.param<T>(param_name, param, default_value);

    if (param_verbose_)
      ROS_INFO_STREAM("[" << nh.getNamespace() << "] " << param_name << ": " << param);
  }

private:
  double ctrl_loop_du_;
  bool param_verbose_ = false;
};
};  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_BASE_H
