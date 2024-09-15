// -*- mode: c++ -*-

#pragma once
#include <beetle/control/beetle_controller.h>
#include <ninja/ninja_navigation.h>
#include <ninja/model/ninja_robot_model.h>

namespace aerial_robot_control
{
  enum
    {
     JOINT_PITCH = TZ +1,
     JOINT_YAW,
    };
  
  class NinjaController: public BeetleController
  {
  public:
    NinjaController();
    ~NinjaController() = default;
    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate
                    ) override;
  private:
    boost::shared_ptr<aerial_robot_navigation::NinjaNavigator> ninja_navigator_;
    boost::shared_ptr<NinjaRobotModel> ninja_robot_model_;

    double joint_p_gain_;
    double joint_i_gain_;
    double joint_d_gain_;

    double joint_control_timestamp_;
  protected:
    void externalWrenchEstimate() override;
    void rosParamInit() override;
    void controlCore() override;
    bool update() override;
    void reset() override;
  };
};
