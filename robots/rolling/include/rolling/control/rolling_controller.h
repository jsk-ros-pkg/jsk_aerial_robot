// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_control/control/pose_linear_controller.h>
#include <aerial_robot_msgs/WrenchAllocationMatrix.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <std_msgs/Float32MultiArray.h>
#include <rolling/model/rolling_robot_model.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>

namespace aerial_robot_control
{
  class RollingController : public PoseLinearController
  {
  public:
    RollingController();
    ~RollingController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate) override;

  private:
    ros::Publisher rpy_gain_pub_;
    ros::Publisher flight_cmd_pub_;
    ros::Publisher gimbal_control_pub_;
    ros::Publisher torque_allocation_matrix_inv_pub_; //for spinal
    ros::Publisher target_vectoring_force_pub_;
    ros::Publisher wrench_allocation_matrix_pub_;
    ros::Subscriber ground_mode_sub_;
    ros::Subscriber joint_state_sub_;
    tf2_ros::TransformBroadcaster br_;

    boost::shared_ptr<RollingRobotModel> rolling_robot_model_;
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control_;

    std::vector<float> target_base_thrust_;
    std::vector<double> target_gimbal_angles_;
    Eigen::VectorXd full_lambda_trans_;
    Eigen::VectorXd full_lambda_rot_;
    Eigen::VectorXd full_lambda_all_;
    Eigen::MatrixXd full_q_mat_;
    Eigen::MatrixXd full_q_mat_inv_;
    Eigen::MatrixXd q_mat_;
    Eigen::MatrixXd q_mat_inv_;
    double candidate_yaw_term_;
    double torque_allocation_matrix_inv_pub_stamp_;
    double torque_allocation_matrix_inv_pub_interval_;
    double allocation_refine_threshold_;
    int allocation_refine_max_iteration_;
    int ground_mode_;
    double circle_radius_;
    std::string tf_prefix_;

    void controlCore() override;
    void reset() override;
    void sendCmd();
    void rosParamInit();
    void sendFourAxisCommand();
    void sendTorqueAllocationMatrixInv();
    void setAttitudeGains();
    void groundModeCallback(const std_msgs::Int16Ptr & msg);
    void jointStateCallback(const sensor_msgs::JointStateConstPtr & msg);
  };
};
