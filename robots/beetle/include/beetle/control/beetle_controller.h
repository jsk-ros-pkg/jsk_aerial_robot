// -*- mode: c++ -*-

#pragma once
#include <beetle/model/beetle_robot_model.h>
#include <gimbalrotor/control/gimbalrotor_controller.h>
#include <beetle/sensor/imu.h>

namespace aerial_robot_control
{
  class BeetleController: public GimbalrotorController
  {
  public:
    BeetleController();
    ~BeetleController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate
                    ) override;
  private:
    boost::shared_ptr<BeetleRobotModel> beetle_robot_model_;
    
    ros::Publisher external_wrench_compensation_pub_;

    /* external wrench compensation */
    bool pd_wrench_comp_mode_;
    Eigen::VectorXd external_wrench_upper_limit_;
    Eigen::VectorXd external_wrench_lower_limit_;

    int pre_module_state_;
    double ErrI_X_;
    double ErrI_Y_;
    double ErrI_Z_;
    double ErrI_ROLL_;
    double ErrI_PITCH_;
    double ErrI_YAW_;
    
    void controlCore() override;

  protected:
    void rosParamInit() override; 
  };
};
