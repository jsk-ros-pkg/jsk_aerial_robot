#pragma once

#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <spinal/ServoControlCmd.h>
#include <spinal/ServoStates.h>

namespace aerial_robot_control
{
class SoftAirframeController : public aerial_robot_control::PoseLinearController
{
public:
  SoftAirframeController();
  virtual ~SoftAirframeController() = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_rate);

  virtual void reset() override;

protected:
  ros::Publisher flight_cmd_pub_; //for spinal
  ros::Publisher rpy_gain_pub_; //for spinal
  ros::Publisher torque_allocation_matrix_inv_pub_; //for spinal
  ros::Publisher gimbal_control_pub_;
  ros::Subscriber joint_state_sub_;
  double torque_allocation_matrix_inv_pub_stamp_;

  Eigen::MatrixXd q_mat_;
  Eigen::MatrixXd q_mat_inv_;

  double target_roll_, target_pitch_; // under-actuated
  double candidate_yaw_term_;
  std::vector<float> target_base_thrust_;

  double torque_allocation_matrix_inv_pub_interval_;

  // double z_limit_;
  bool hovering_approximate_;

  double gimbal_angle_diff_ = 0.0;
  double gimbal_current_angle;
  ros::Time gimbal_update_time;

  int virtual_motor_num_ = 6;

  void setAttitudeGains();
  virtual void rosParamInit();
  virtual void controlCore() override;
  virtual Eigen::MatrixXd getFullQMat();
  virtual Eigen::MatrixXd getQMat();
  virtual void sendCmd() override;
  virtual void sendFourAxisCommand();
  virtual void jointStateCallback(const sensor_msgs::JointState& msg);
  virtual void sendGimbalCommand();
  virtual void sendTorqueAllocationMatrixInv();

};
}  // namespace aerial_robot_control
