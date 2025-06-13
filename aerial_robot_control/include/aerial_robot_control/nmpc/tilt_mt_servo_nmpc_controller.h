//
// Created by lijinjie on 23/11/29.
//

#ifndef TILT_MT_SERVO_NMPC_CONTROLLER_H
#define TILT_MT_SERVO_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/base_mpc_controller.h"

#include <angles/angles.h>
#include <tf_conversions/tf_eigen.h>
#include <numeric>

/* dynamic reconfigure */
#include "aerial_robot_msgs/DynamicReconfigureLevels.h"
#include "aerial_robot_control/NMPCConfig.h"
#include <dynamic_reconfigure/server.h>

/* protocol */
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "geometry_msgs/PoseArray.h"
#include "aerial_robot_msgs/PredXU.h"
#include "aerial_robot_msgs/FixRotor.h"
#include "spinal/FourAxisCommand.h"
#include "spinal/SetControlMode.h"
#include "spinal/FlightConfigCmd.h"
#include "spinal/DesireCoord.h"

/* action */
#include "actionlib/server/simple_action_server.h"
#include "aerial_robot_msgs/PredXU.h"
#include "aerial_robot_msgs/TrackTrajAction.h"
#include "aerial_robot_msgs/TrackTrajFeedback.h"
#include "aerial_robot_msgs/TrackTrajGoal.h"
#include "aerial_robot_msgs/TrackTrajResult.h"

using NMPCControlDynamicConfig = dynamic_reconfigure::Server<aerial_robot_control::NMPCConfig>;

namespace aerial_robot_control
{

namespace nmpc
{

class TiltMtServoNMPC : public BaseMPC
{
public:
  TiltMtServoNMPC() = default;  // note that constructor should not have arguments as the rule of rospluginlib
  ~TiltMtServoNMPC() override = default;
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;
  void activate() override;
  bool update() override;
  void reset() override;

protected:
  ros::Timer tmr_viz_;

  ros::Publisher pub_viz_pred_;                  // for viz predictions
  ros::Publisher pub_viz_ref_;                   // for viz reference
  ros::Publisher pub_flight_cmd_;                // for spinal
  ros::Publisher pub_gimbal_control_;            // for gimbal control
  ros::Publisher pub_flight_config_cmd_spinal_;  // for spinal, enable the gyro measurement after the takeoff

  ros::ServiceClient srv_set_control_mode_;
  std::vector<boost::shared_ptr<NMPCControlDynamicConfig>> nmpc_reconf_servers_;

  ros::Subscriber sub_joint_states_;
  ros::Subscriber sub_set_rpy_;
  ros::Subscriber sub_set_ref_x_u_;
  ros::Subscriber sub_set_traj_;
  ros::Subscriber sub_set_fixed_rotor_;

  bool is_attitude_ctrl_;
  bool is_body_rate_ctrl_;
  bool is_print_phys_params_;
  bool is_debug_;

  double mass_;
  double gravity_const_;
  std::vector<double> inertia_;
  int motor_num_;
  double t_rotor_;
  double thrust_ctrl_max_;
  double thrust_ctrl_min_;
  int joint_num_;
  double t_servo_;
  double servo_angle_max_;
  double servo_angle_min_;

  double t_nmpc_samp_;
  double t_nmpc_step_;
  double t_nmpc_horizon_;

  int idx_p_quat_end_ = 0;
  int idx_p_phys_end_ = 0;

  std::vector<double> joint_angles_;

  Eigen::MatrixXd alloc_mat_;
  Eigen::MatrixXd alloc_mat_pinv_;

  bool is_traj_tracking_ = false;  // TODO: tmp value. should be combined with inner traj. tracking in the future
  trajectory_msgs::MultiDOFJointTrajectory last_traj_msg_;

  aerial_robot_msgs::PredXU x_u_ref_;  // TODO: maybe we should remove x_u_ref_ and use xr_ & ur_ inside mpc_solver_ptr_
  spinal::FourAxisCommand flight_cmd_;
  sensor_msgs::JointState gimbal_ctrl_cmd_;

  // For singularity points
  int alloc_type_ = 0;
  double ft_thresh_;
  int rotor_idx_prev_ = -1;
  Eigen::MatrixXd alloc_mat_del_rotor_inv_;

  // For fixing rotor
  bool is_set_fix_rotor_ = false;
  aerial_robot_msgs::FixRotor fix_rotor_msg_;

  /* initialize() */
  virtual void initPlugins() {};
  virtual void initGeneralParams();
  virtual void initNMPCCostW();
  virtual void initNMPCConstraints();
  void setControlMode();
  virtual inline void initActuatorStates()
  {
    joint_angles_.resize(joint_num_, 0.0);
  }

  /* activate() */
  virtual void initAllocMat();
  virtual void initNMPCParams();
  void updateInertialParams();
  std::vector<double> PhysToNMPCParams() const;

  /* update() */
  void controlCore() override;
  void sendCmd() override;

  // controlCore()
  void prepareNMPCRef();
  virtual void prepareNMPCParams();
  void setXrUrRef(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i, const tf::Vector3& ref_acc_i,
                  const tf::Quaternion& ref_quat_ib, const tf::Vector3& ref_omega_b, const tf::Vector3& ref_ang_acc_b,
                  const int& horizon_idx);
  virtual void allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i,
                            const tf::Quaternion& ref_quat_ib, const tf::Vector3& ref_omega_b,
                            const VectorXd& ref_wrench_b, vector<double>& x, vector<double>& u);
  void allocateToXUwOneFixedRotor(int fix_rotor_idx, double fix_ft, double fix_alpha, const VectorXd& ref_wrench_b,
                                  vector<double>& x, vector<double>& u);

  /* callback functions */
  void callbackViz(const ros::TimerEvent& event) override;
  virtual void callbackJointStates(const sensor_msgs::JointStateConstPtr& msg);
  void callbackSetRPY(const spinal::DesireCoordConstPtr& msg);
  void callbackSetRefXU(const aerial_robot_msgs::PredXUConstPtr& msg) override;
  void callbackSetRefTraj(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);
  void callbackSetFixedRotor(const aerial_robot_msgs::FixRotorConstPtr& msg);
  virtual void cfgNMPCCallback(NMPCConfig& config, uint32_t level);

  /* utils */
  // get functions
  double getCommand(int idx_u, double T_horizon = 0.0);

  // conversion functions
  std::vector<double> meas2VecX() override;

  // debug functions
  void printPhysicalParams();

  // check functions
  inline bool isAlmostEqual(const double a, const double b, const double epsilon = 1e-6)
  {
    return std::fabs(a - b) < epsilon;
  }

  bool isMulDOFJointTrajPtEqual(const trajectory_msgs::MultiDOFJointTrajectoryPoint& a,
                                const trajectory_msgs::MultiDOFJointTrajectoryPoint& b, bool if_compare_time = true,
                                double epsilon = 1e-6);

private:
  tf::Quaternion quat_prev_;  // To deal with the discontinuity of the quaternion.
};

}  // namespace nmpc

}  // namespace aerial_robot_control

#endif  // TILT_MT_SERVO_NMPC_CONTROLLER_H