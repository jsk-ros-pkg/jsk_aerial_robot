// -*- mode: c++ -*-

#pragma once

#include <numeric>
#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <aerial_robot_msgs/WrenchAllocationMatrix.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <spinal/DesireCoord.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <rolling/model/rolling_robot_model.h>
#include <rolling/rolling_navigation.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <OsqpEigen/OsqpEigen.h>
#include <nlopt.hpp>

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
    ros::Publisher desire_coordinate_pub_;
    ros::Publisher target_vectoring_force_pub_;
    ros::Publisher target_wrench_acc_cog_pub_;
    ros::Publisher wrench_allocation_matrix_pub_;
    ros::Publisher full_q_mat_pub_;
    ros::Publisher full_q_mat_inv_pub_;
    ros::Publisher under_q_mat_pub_;
    ros::Publisher under_q_mat_inv_pub_;
    ros::Publisher operability_pub_;
    ros::Publisher target_acc_cog_pub_;
    ros::Publisher target_acc_dash_pub_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber z_i_control_flag_sub_;
    ros::Subscriber z_i_term_sub_;
    ros::Subscriber reset_attitude_gains_sub_;
    ros::Subscriber control_mode_sub_;
    ros::Subscriber stay_current_sub_;

    tf2_ros::TransformBroadcaster br_;

    boost::shared_ptr<aerial_robot_navigation::RollingNavigator> rolling_navigator_;
    boost::shared_ptr<RollingRobotModel> rolling_robot_model_;
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control_;

    std::vector<double> rotor_tilt_;
    std::vector<float> target_base_thrust_;
    std::vector<double> target_gimbal_angles_;
    std::vector<double> prev_target_gimbal_angles_;
    std::vector<double> target_acc_cog_;
    std::vector<double> target_acc_dash_;
    Eigen::VectorXd target_wrench_acc_cog_;
    Eigen::VectorXd full_lambda_trans_;
    Eigen::VectorXd full_lambda_rot_;
    Eigen::VectorXd full_lambda_all_;
    Eigen::MatrixXd full_q_mat_;
    Eigen::MatrixXd full_q_mat_inv_;
    Eigen::MatrixXd under_q_mat_;
    Eigen::MatrixXd under_q_mat_inv_;
    Eigen::MatrixXd q_mat_;
    Eigen::MatrixXd q_mat_inv_;

    double candidate_yaw_term_;
    double torque_allocation_matrix_inv_pub_stamp_;
    double torque_allocation_matrix_inv_pub_interval_;
    double allocation_refine_threshold_;
    int allocation_refine_max_iteration_;
    double circle_radius_;
    double initial_roll_tilt_;
    std::string tf_prefix_;
    bool gain_updated_;
    bool use_sr_inv_;
    double sr_inv_weight_;
    bool fully_actuated_;
    double z_limit_;
    Eigen::VectorXd target_thrust_z_term_;
    double target_roll_, target_pitch_; //for under actuated control
    bool hovering_approximate_;
    double gimbal_lpf_factor_;
    int ground_navigation_mode_;
    std::vector<int> controlled_axis_;
    int control_dof_;

    double standing_target_phi_;
    double standing_converged_baselink_roll_thresh_;
    double standing_converged_z_i_term_;
    double standing_baselink_ref_pitch_last_update_time_;
    double standing_baselink_ref_pitch_update_thresh_;

    void controlCore() override;
    void reset() override;
    void sendCmd();
    void rosParamInit();
    void sendGimbalAngles();
    void sendFourAxisCommand();
    void sendTorqueAllocationMatrixInv();
    void setAttitudeGains();
    void jointStateCallback(const sensor_msgs::JointStateConstPtr & msg);
    void setZIControlFlagCallback(const std_msgs::BoolPtr & msg);
    void setZITermCallback(const std_msgs::Float32Ptr & msg);
    void stayCurrentXYPosition(const std_msgs::Empty & msg);
    void resetAttitudeGains();
    void resetAttitudeGainsCallback(const std_msgs::Empty & msg);
    void setControlModeCallback(const std_msgs::Int16Ptr & msg);
    void targetStatePlan();
    void calcAccFromCog();
    void calcWrenchAllocationMatrix();
    void wrenchAllocation();
    void calcYawTerm();
    void setControlAxis(int axis, int mode)
    {
      int prev_mode = controlled_axis_.at(axis);
      if(axis < 0 || 6 <= axis) return;
      if(mode) controlled_axis_.at(axis) = 1;
      else controlled_axis_.at(axis) = 0;
      if(prev_mode != mode)
        {
          ROS_WARN_STREAM("[control] set control axis about " << axis << " to " << mode << ". The controlled axis is ["
                          << controlled_axis_.at(0) << " "
                          << controlled_axis_.at(1) << " "
                          << controlled_axis_.at(2) << " "
                          << controlled_axis_.at(3) << " "
                          << controlled_axis_.at(4) << " "
                          << controlled_axis_.at(5) << "]"
                          );
        }
    }
  };
};
