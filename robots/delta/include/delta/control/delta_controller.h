// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <aerial_robot_msgs/WrenchAllocationMatrix.h>
#include <delta/model/delta_robot_model.h>
#include <delta/navigation/delta_navigation.h>
#include <geometry_msgs/WrenchStamped.h>
#include <numeric>
#include <OsqpEigen/OsqpEigen.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <spinal/DesireCoord.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
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

    boost::shared_ptr<aerial_robot_model::RobotModel> getRobotModel() {return robot_model_;}
    boost::shared_ptr<RollingRobotModel> getRollingRobotModel() {return rolling_robot_model_;}

  private:
    ros::Publisher rpy_gain_pub_;                     // for spinal
    ros::Publisher flight_cmd_pub_;                   // for spinal
    ros::Publisher gimbal_control_pub_;               // for servo bridge
    ros::Publisher torque_allocation_matrix_inv_pub_; // for spinal
    ros::Publisher gimbal_dof_pub_;                   // for spinal
    ros::Publisher gimbal_indices_pub_;               // for spinal
    ros::Publisher target_vectoring_force_pub_;       // for debug
    ros::Publisher target_wrench_acc_cog_pub_;        // for debug
    ros::Publisher gravity_compensate_term_pub_;      // for debug
    ros::Publisher wrench_allocation_matrix_pub_;     // for debug
    ros::Publisher full_q_mat_pub_;                   // for debug
    ros::Publisher operability_pub_;                  // for debug
    ros::Publisher target_acc_cog_pub_;               // for debug
    ros::Publisher exerted_wrench_cog_pub_;               // for debug
    ros::Subscriber joint_state_sub_;

    tf2_ros::TransformBroadcaster br_;
    KDL::Frame contact_point_alined_;
    KDL::Frame cog_alined_;

    std::mutex contact_point_alined_mutex_;
    std::mutex current_gimbal_angles_mutex_;
    std::mutex cog_alined_mutex_;

    boost::shared_ptr<aerial_robot_navigation::RollingNavigator> rolling_navigator_;
    boost::shared_ptr<RollingRobotModel> rolling_robot_model_;
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control_;

    /* common part */
    std::vector<double> rotor_tilt_;
    std::vector<float> lambda_trans_;
    std::vector<float> lambda_all_;
    std::vector<double> target_gimbal_angles_;
    Eigen::VectorXd full_lambda_all_;
    Eigen::VectorXd full_lambda_trans_;
    Eigen::VectorXd full_lambda_rot_;
    Eigen::MatrixXd q_mat_;
    Eigen::MatrixXd q_mat_inv_;
    double candidate_yaw_term_;
    bool use_sr_inv_;
    double sr_inv_weight_;
    std::string tf_prefix_;
    int ground_navigation_mode_;
    double torque_allocation_matrix_inv_pub_stamp_;
    double torque_allocation_matrix_inv_pub_interval_;

    /* flight mode */
    Eigen::VectorXd target_wrench_acc_cog_;
    std::vector<double> target_acc_cog_;
    bool hovering_approximate_;

    /* ground mode */
    Eigen::Vector3d gravity_compensate_term_;
    double steering_mu_;
    Eigen::MatrixXd gravity_compensate_weights_;
    double rolling_minimum_lateral_force_;

    /* common part */
    bool update() override;
    void reset() override;
    void controlCore() override;
    void activate() override;
    void rosParamInit();
    void fullLambdaToActuatorInputs();
    void resolveGimbalOffset();
    void processGimbalAngles();
    void calcYawTerm();

    /* flight mode */
    void calcAccFromCog();
    void calcFlightFullLambda();

    /* rolling mode */
    void standingPlanning();
    void calcStandingFullLambda();

    /* send command */
    void sendCmd();
    void sendGimbalAngles();
    void sendFourAxisCommand();
    void sendTorqueAllocationMatrixInv();
    void setAttitudeGains();

    /* ros callbacks */
    void jointStateCallback(const sensor_msgs::JointStateConstPtr & msg);

    /* utils */
    void setControllerParams(std::string ns);
    void rosoutControlParams(std::string ns);
    void printDebug();
  };
};
