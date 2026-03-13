// -*- mode: c++ -*-

#pragma once
#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <twin_hammer/model/twin_hammer_model.h>
#include <nlopt.hpp>

namespace aerial_robot_control
{
  class TwinHammerOptimizer: public PoseLinearController
  {
  public:
    TwinHammerOptimizer();
    ~TwinHammerOptimizer() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate
                    ) override;

    inline boost::shared_ptr<TwinHammerModel> getHammerModel() { return twin_hammer_model_;}
    const Eigen::Vector3d getHapticsForce() { return haptics_force_; }
    const Eigen::Vector3d getHapticsTorque() { return haptics_torque_; }
    const Eigen::VectorXd getTargetWrenchAccCog() { return target_wrench_acc_cog_; }

  private:
    boost::shared_ptr<TwinHammerModel> twin_hammer_model_;
    double gimbal_roll_delta_angle_;
    double gimbal_pitch_delta_angle_;
    double gravity_acc_;

    boost::shared_ptr<nlopt::opt> nl_solver_;
    std::vector<double> opt_x_;
    std::vector<double> prev_opt_x_;

    ros::Publisher flight_cmd_pub_;
    ros::Publisher gimbal_control_pub_;
    ros::Subscriber haptics_switch_sub_;
    ros::Subscriber haptics_wrench_sub_;
    std::vector<float> target_base_thrust_;
    std::vector<double> target_gimbal_angles_;
    std::vector<double> prev_gimbal_angles_;
    Eigen::VectorXd target_vectoring_f_;
    Eigen::VectorXd target_wrench_acc_cog_;

    bool use_haptics_flag_;
    bool haptics_switch_;
    Eigen::Vector3d haptics_force_;
    Eigen::Vector3d haptics_torque_;

    void sendCmd() override;
    void HapticsSwitchCallback(std_msgs::Int8 msg);
    void HapticsWrenchCallback(geometry_msgs::WrenchStamped msg);

  protected:
    void controlCore() override;
    void rosParamInit() ;
  };
};
