// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_control/control/under_actuated_controller.h>
#include <aerial_robot_estimation/state_estimation.h>

namespace aerial_robot_control
{
  class BirotorController: public PoseLinearController
  {
  public:
    BirotorController();
    ~BirotorController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate
                    ) override;

  private:
    ros::Publisher gimbal_control_pub_;
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_;

    std::vector<float> target_base_thrust_;
    std::vector<double> target_gimbal_angles_;
    bool hovering_approximate_;

    void controlCore() override;
    void sendCmd();
    void rosParamInit();

  };
};
