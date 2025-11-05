// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <aerial_robot_msgs/WrenchAllocationMatrix.h>
#include <delta/model/delta_robot_model.h>
#include <delta/control/delta_wrench_allocation.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nlopt.hpp>
#include <numeric>
#include <OsqpEigen/OsqpEigen.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/ServoTorqueCmd.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>

namespace aerial_robot_control
{
class DeltaController : public PoseLinearController
{
public:
  DeltaController();
  ~DeltaController() = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_rate) override;

  boost::shared_ptr<aerial_robot_model::RobotModel> getRobotModel()
  {
    return robot_model_;
  }
  boost::shared_ptr<DeltaRobotModel> getDeltaRobotModel()
  {
    return delta_robot_model_;
  }
  boost::shared_ptr<DeltaRobotModel> getRobotModelForControl()
  {
    return robot_model_for_control_;
  }
  const std::vector<double>& getRotorTilt()
  {
    return rotor_tilt_;
  }
  Eigen::VectorXd getTargetAccCog()
  {
    return target_acc_cog_;
  }
  const int getNLOptResult()
  {
    return nlopt_result_;
  }
  const int getNLOptIterations()
  {
    return nlopt_iterations_;
  }
  void setNLOptIterations(int iterations)
  {
    nlopt_iterations_ = iterations;
  }
  const std::vector<float> getNLOptLog()
  {
    return nlopt_log_;
  }

  /* ros callbacks */
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);

private:
  ros::Publisher rpy_gain_pub_;                      // for spinal
  ros::Publisher flight_cmd_pub_;                    // for spinal
  ros::Publisher gimbal_control_pub_;                // for servo bridge
  ros::Publisher servo_torque_command_pub_;          // for spinal
  ros::Publisher torque_allocation_matrix_inv_pub_;  // for spinal
  ros::Publisher target_vectoring_force_pub_;        // for debug
  ros::Publisher target_acc_cog_pub_;                // for debug
  ros::Publisher gravity_compensate_term_pub_;       // for debug
  ros::Publisher wrench_allocation_matrix_pub_;      // for debug
  ros::Publisher full_q_mat_pub_;                    // for debug
  ros::Publisher exerted_wrench_cog_pub_;            // for debug
  ros::Publisher nlopt_log_pub_;                     // for debug
  ros::Publisher rotor_origin_pub_;                  // for debug
  ros::Publisher rotor_normal_pub_;                  // for debug

  boost::shared_ptr<DeltaRobotModel> delta_robot_model_;
  boost::shared_ptr<DeltaRobotModel> robot_model_for_control_;

  /* common part */
  bool first_run_;
  int nlopt_result_;
  int nlopt_iterations_ = 0;
  std::vector<float> nlopt_log_;
  std::vector<double> rotor_tilt_;
  std::vector<float> lambda_all_;
  std::vector<double> target_gimbal_angles_;
  Eigen::MatrixXd q_mat_;
  Eigen::MatrixXd q_mat_inv_;
  double candidate_yaw_term_;
  bool hovering_approximate_;
  double torque_allocation_matrix_inv_pub_stamp_;
  double torque_allocation_matrix_inv_pub_interval_;

  /* flight mode */
  Eigen::VectorXd target_acc_cog_;

  /* common part */
  bool update() override;
  void reset() override;
  void controlCore() override;
  void activate() override;
  void rosParamInit();
  void processGimbalAngles();
  void calcYawTerm();

  /* aerial mode */
  void calcAccFromCog();
  void forceLandingProcess();
  void calcFlightFullLambda();
  void wrenchAllocation();
  void nonlinearWrenchAllocation();

  /* send command */
  void sendCmd();
  void sendGimbalAngles();
  void sendFourAxisCommand();
  void sendTorqueAllocationMatrixInv();
  void setAttitudeGains();
};
};  // namespace aerial_robot_control
