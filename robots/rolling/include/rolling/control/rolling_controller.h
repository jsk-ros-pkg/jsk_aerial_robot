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
#include <osqp_slsqp/slsqp.h>
#include <rolling/control/osqp_solver.h>

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

    boost::shared_ptr<aerial_robot_model::RobotModel> getRobotModel() {return robot_model_;}
    boost::shared_ptr<RollingRobotModel> getRollingRobotModel() {return rolling_robot_model_;}
    boost::shared_ptr<RollingRobotModel> getRollingRobotModelForOpt() {return rolling_robot_model_for_opt_;}

    Eigen::VectorXd getTargetWrenchAccCog() {return target_wrench_acc_cog_;}
    Eigen::MatrixXd getFullQ() {return full_q_mat_;}
    Eigen::MatrixXd getFullQTrans() {return full_q_trans_;}
    Eigen::MatrixXd getFullQRot() {return full_q_rot_;}
    Eigen::MatrixXd getFullQTargetFrame() {return full_q_mat_target_frame_;}
    Eigen::VectorXd getTargetWrenchAccTargetFrame() {return target_wrench_acc_target_frame_;}
    double getRobotMassForOpt() {return robot_mass_for_opt_;}
    double getGravityForOpt() {return gravity_for_opt_;}
    double getSteeringMu() {return steering_mu_;}

  private:
    ros::Publisher rpy_gain_pub_;                     // for spinal
    ros::Publisher flight_cmd_pub_;                   // for spinal
    ros::Publisher gimbal_control_pub_;               // for servo bridge
    ros::Publisher torque_allocation_matrix_inv_pub_; // for spinal
    ros::Publisher desire_coordinate_pub_;            // for spinal
    ros::Publisher target_vectoring_force_pub_;       // for debug
    ros::Publisher target_wrench_acc_cog_pub_;        // for debug
    ros::Publisher wrench_allocation_matrix_pub_;     // for debug
    ros::Publisher full_q_mat_pub_;                   // for debug
    ros::Publisher operability_pub_;                  // for debug
    ros::Publisher target_acc_cog_pub_;               // for debug
    ros::Publisher target_acc_dash_pub_;              // for debug
    ros::Publisher exerted_wrench_pub_;               // for debug
    ros::Subscriber joint_state_sub_;

    tf2_ros::TransformBroadcaster br_;
    KDL::Frame contact_point_alined_;

    std::mutex contact_point_alined_mutex_;
    std::mutex current_gimbal_angles_mutex_;

    boost::shared_ptr<aerial_robot_navigation::RollingNavigator> rolling_navigator_;
    boost::shared_ptr<RollingRobotModel> rolling_robot_model_;
    boost::shared_ptr<RollingRobotModel> rolling_robot_model_for_opt_;
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control_;

    std::vector<double> rotor_tilt_;
    std::vector<float> target_base_thrust_;
    std::vector<double> target_gimbal_angles_;
    std::vector<double> prev_target_gimbal_angles_;
    std::vector<double> current_gimbal_angles_;
    std::vector<double> target_acc_cog_;
    std::vector<double> target_acc_dash_;
    Eigen::VectorXd target_wrench_acc_cog_;
    Eigen::VectorXd target_wrench_acc_target_frame_;
    Eigen::VectorXd controlled_wrench_acc_cog_;
    Eigen::VectorXd full_lambda_trans_;
    Eigen::VectorXd full_lambda_rot_;
    Eigen::VectorXd full_lambda_all_;
    Eigen::MatrixXd full_q_mat_;
    Eigen::MatrixXd full_q_trans_;
    Eigen::MatrixXd full_q_rot_;
    Eigen::MatrixXd full_q_mat_target_frame_;
    Eigen::MatrixXd full_q_trans_target_frame_;
    Eigen::MatrixXd full_q_rot_target_frame_;
    Eigen::MatrixXd controlled_q_mat_;
    Eigen::MatrixXd controlled_q_mat_inv_;
    Eigen::MatrixXd q_mat_;
    Eigen::MatrixXd q_mat_inv_;

    double candidate_yaw_term_;
    double torque_allocation_matrix_inv_pub_stamp_;
    double torque_allocation_matrix_inv_pub_interval_;
    double circle_radius_;
    std::string tf_prefix_;
    bool use_sr_inv_;
    double sr_inv_weight_;
    Eigen::VectorXd target_thrust_z_term_;
    double target_roll_, target_pitch_; //for under actuated control
    bool hovering_approximate_;
    double gimbal_lpf_factor_;
    int ground_navigation_mode_;
    std::vector<int> controlled_axis_;
    int control_dof_;
    double default_z_i_gain_;
    double rolling_control_timestamp_;
    bool realtime_gimbal_allocation_;

    double robot_mass_for_opt_;
    double gravity_for_opt_;

    double standing_converged_baselink_roll_thresh_;
    double standing_converged_z_i_term_min_;
    double standing_converged_z_i_term_descend_ratio_;
    double standing_baselink_roll_restart_z_i_control_thresh_;
    double standing_baselink_ref_pitch_last_update_time_;
    double standing_baselink_ref_pitch_update_thresh_;
    double standing_minimum_z_i_term_;
    double standing_target_phi_;
    double standing_target_baselink_pitch_;
    double standing_feed_forward_z_compensate_roll_thresh_;
    double steering_z_acc_min_;
    double steering_mu_;

    void reset() override;
    void controlCore() override;
    void rosParamInit();
    void targetStatePlan();
    void calcAccFromCog();
    void calcAccFromTargetFrame();
    void calcWrenchAllocationMatrix();
    void calcWrenchAllocationMatrixFromTargetFrame();
    void calcFullLambda();
    void wrenchAllocation();
    void calcYawTerm();
    void sendCmd();
    void sendGimbalAngles();
    void sendFourAxisCommand();
    void sendTorqueAllocationMatrixInv();
    void setAttitudeGains();
    void jointStateCallback(const sensor_msgs::JointStateConstPtr & msg);
    void setControllerParams(std::string ns);
    void rosoutControlParams(std::string ns);
    void setControlAxisWithNameSpace(std::string ns);
    void rosoutControlAxis(std::string ns);
    void calcStandingFullLambda();
    void standingPlanning();
    void slsqpSolve();

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
