// -*- mode: c++ -*-

#pragma once
#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <twin_hammer/model/twin_hammer_model.h>
#include <nlopt.hpp>

namespace aerial_robot_control
{
  class TwinHammerController: public PoseLinearController
  {
  public:
    TwinHammerController();
    ~TwinHammerController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate
                    ) override;

    inline boost::shared_ptr<TwinHammerModel> getHammerModel() { return twin_hammer_model_;}
    const Eigen::Vector3d getHapticsForce() { return haptics_force_; }
    const Eigen::Vector3d getHapticsTorque() { return haptics_torque_; }
    const Eigen::VectorXd getTargetWrencAccCog() { return target_wrench_acc_cog_; }

  private:
    boost::shared_ptr<TwinHammerModel> twin_hammer_model_;
    double gimbal_roll_vel_limit_;
    double gimbal_roll_acc_limit_;
    double counter_torque_limit_;
    double gravity_acc_;
    double time_prev_;

    bool use_polynominal_;
    bool use_gaussian_;
    double filtered_gimbal_1_roll_;
    double filtered_gimbal_2_roll_;
    double delay_param_;
    double gimbal_round_range_;
    double gaussian_exp_;

    boost::shared_ptr<nlopt::opt> nl_solver_;
    std::vector<double> opt_x_;
    std::vector<double> prev_opt_x_;

    ros::Publisher test_counter_torque_pub_;
    ros::Publisher flight_cmd_pub_;
    ros::Publisher gimbal_control_pub_;
    ros::Subscriber gimbal_states_sub_;
    ros::Subscriber haptics_switch_sub_;
    ros::Subscriber haptics_wrench_sub_;
    std::vector<float> target_base_thrust_;
    std::vector<double> target_gimbal_angles_;
    std::vector<double> prev_gimbal_angles_;
    std::vector<double> gimbal_states_angles_;
    Eigen::VectorXd target_vectoring_f_;
    Eigen::VectorXd target_wrench_acc_cog_;
    Eigen::Vector2d prev_gimbal_roll_vel_;

    bool use_haptics_flag_;
    bool haptics_switch_;
    Eigen::Vector3d haptics_force_;
    Eigen::Vector3d haptics_torque_;

    void sendCmd() override;
    void GimbalStatesCallback(sensor_msgs::JointState);
    void HapticsSwitchCallback(std_msgs::Int8 msg);
    void HapticsWrenchCallback(geometry_msgs::WrenchStamped msg);

  protected:
    void controlCore() override;
    void rosParamInit() ;
  };
};
