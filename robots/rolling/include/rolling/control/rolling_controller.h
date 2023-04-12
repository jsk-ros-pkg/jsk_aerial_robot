// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_control/control/pose_linear_controller.h>
#include <spinal/FourAxisCommand.h>
#include <std_msgs/Float32MultiArray.h>

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
    bool rolling_mode_;
    bool hovering_approximate_;
    ros::Publisher flight_cmd_pub_;
    ros::Publisher gimbal_control_pub_;
    ros::Publisher target_vectoring_force_pub_;
    std::vector<float> target_base_thrust_;
    std::vector<double> target_gimbal_angles_;
    Eigen::VectorXd target_vectoring_f_;
    void controlCore() override;
    void sendCmd();
    void rosParamInit();

  };
};
