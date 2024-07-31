// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <aerial_robot_control/control/fully_actuated_controller.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <gimbalrotor/model/gimbalrotor_robot_model.h>

namespace aerial_robot_control
{
  class GimbalrotorController: public PoseLinearController
  {
  public:
    GimbalrotorController();
    ~GimbalrotorController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate
                    ) override;
    const Eigen::MatrixXd getIntegratedMapInvTrans() {
      std::lock_guard<std::mutex> lock(trans_map_inv_mutex_);
      return integrated_map_inv_trans_;
    }
    const Eigen::MatrixXd getIntegratedMapInvRot() {
      std::lock_guard<std::mutex> lock(rot_map_inv_mutex_);
      return integrated_map_inv_rot_;
    }

  private:
    std::mutex wrench_mutex_;
    std::mutex trans_map_inv_mutex_;
    std::mutex rot_map_inv_mutex_;
    
    ros::Publisher flight_cmd_pub_;
    ros::Publisher gimbal_control_pub_;
    ros::Publisher gimbal_state_pub_;
    ros::Publisher target_vectoring_force_pub_;
    ros::Publisher torque_allocation_matrix_inv_pub_; //for spinal
    ros::Publisher gimbal_dof_pub_; //for spinal

    boost::shared_ptr<GimbalrotorRobotModel> gimbalerotor_robot_model_;
    std::vector<float> target_base_thrust_;
    std::vector<float> target_full_thrust_;
    std::vector<double> target_gimbal_angles_;
    bool hovering_approximate_;
    Eigen::VectorXd target_vectoring_f_;
    Eigen::VectorXd target_vectoring_f_trans_;
    Eigen::VectorXd target_vectoring_f_rot_;
    Eigen::MatrixXd integrated_map_inv_trans_;
    Eigen::MatrixXd integrated_map_inv_rot_;
    double candidate_yaw_term_;
    int gimbal_dof_;

    bool update() override;
    void sendCmd() override;
    void sendFourAxisCommand();
    void sendGimbalCommand();
    void sendTorqueAllocationMatrixInv();
  protected:
    bool gimbal_calc_in_fc_;
    bool i_term_rp_calc_in_pc_;
    ros::Publisher rpy_gain_pub_; //for spinal
    virtual void reset() override;
    virtual void setAttitudeGains();
    virtual void rosParamInit();
    virtual void controlCore() override;
    Eigen::VectorXd additional_wrench_acc_cog_term_;
  };
};
