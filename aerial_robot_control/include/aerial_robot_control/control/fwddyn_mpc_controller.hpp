// -*- mode: c++ -*-
#pragma once

#include <pinocchio/fwd.hpp>

#include <aerial_robot_control/control/base/base.h>
#include <aerial_robot_control/control/fwddyn_mpc_control_problem.hpp>
#include <aerial_robot_dynamics/robot_model.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/JointState.h>
#include <spinal/FourAxisCommand.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Geometry>

#include <map>

namespace aerial_robot_control
{

class FwddynMpcController : public ControlBase
{
public:
  FwddynMpcController();
  virtual ~FwddynMpcController() = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_rate) override;

  void activate() override;
  bool update() override;
  void reset() override;

protected:
  void controlCore();
  void sendCmd();

private:
  ros::Publisher four_axis_command_pub_;
  ros::Publisher joints_ctrl_pub_;
  ros::Publisher target_cog_pos_pub_;
  ros::Publisher thrust_rate_pub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber target_joint_state_sub_;
  ros::Subscriber target_root_rpy_sub_;
  tf::TransformBroadcaster optimized_root_tf_broadcaster_;

  sensor_msgs::JointState joint_state_;
  bool joint_state_received_;

  Eigen::VectorXd curr_q_;
  Eigen::VectorXd curr_dq_;

  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model_;
  std::shared_ptr<pinocchio::Model> pin_model_;
  pinocchio::FrameIndex baselink_frame_id_;

  std::map<std::string, int> joint_name_to_q_idx_;
  std::map<std::string, int> joint_name_to_v_idx_;
  int n_joints_;  // non-floating-base DOF

  FwddynMpcControlProblem mpc_problem_;
  FwddynMpcControlProblem::Parameters mpc_parameters_;

  double mpc_elapsed_time_;

  Eigen::VectorXd x_ref_;
  Eigen::VectorXd x_ref_default_;

  // per-joint target for interpolation; initialized from x_ref_default_, updated by subscriber
  struct JointTarget
  {
    double position = 0.0;
    double velocity = 0.0;  // interpolation speed; 0 means use default_joint_vel_
  };
  std::map<std::string, JointTarget> joint_targets_;

  double default_joint_vel_ = 0.1;  // [rad/s] used when velocity not specified in JointTarget

  // root-link attitude target for interpolation; the reference quaternion in x_ref_ is slerped toward this target
  struct RootAttitudeTarget
  {
    Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();  // world -> root, target orientation
    double angular_velocity = 0.0;  // [rad/s] interpolation speed; 0 means use default_root_angular_vel_
  };
  RootAttitudeTarget root_attitude_target_;

  double default_root_angular_vel_ = 0.1;  // [rad/s] used when angular_velocity not specified in RootAttitudeTarget

  // max look-ahead displacement [m] of the CoM reference trajectory from the target position
  double com_ref_max_offset_ = 1.0;

  // for debugging
  double target_cog_pos_pub_duration_ = 1.0;
  double last_target_cog_pos_pub_time_ = 0.0;
  double tf_broadcast_duration_ = 0.1;
  double last_tf_broadcast_time_ = 0.0;

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void targetJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void targetRootRpyCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
  void updateBaseRefInterpolation();
  void syncRootAttitudeTarget();

  Eigen::VectorXd buildCurrentState();
  std::vector<Eigen::Vector3d> buildComRefTrajectory();
  std::vector<Eigen::VectorXd> buildStateRefTrajectory();
  void broadcastOptimizedRootTransforms();
  void publishJointsCtrl();
};

}  // namespace aerial_robot_control
