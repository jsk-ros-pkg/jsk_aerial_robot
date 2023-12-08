// -*- mode: c++ -*-

#pragma once
#include <beetle/model/beetle_robot_model.h>
#include <beetle/TaggedWrench.h>
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
    ros::Publisher tagged_external_wrench_pub_;
    ros::Publisher whole_external_wrench_pub_;
    ros::Publisher internal_wrench_pub_;
    map<string, ros::Subscriber> est_wrench_subs_;

    /* external wrench compensation */
    bool pd_wrench_comp_mode_;
    Eigen::VectorXd external_wrench_upper_limit_;
    Eigen::VectorXd external_wrench_lower_limit_;

    int pre_module_state_;

    std::map<int, Eigen::VectorXd> est_wrench_list_;
    std::map<int, Eigen::VectorXd> inter_wrench_list_;
    std::map<int, Eigen::VectorXd> wrench_comp_list_;

    double comp_term_update_freq_;
    double prev_comp_update_time_;
    double wrench_comp_gain_;
    double I_comp_Fx_;
    double I_comp_Fy_;
    double I_comp_Fz_;
    double I_comp_Tx_;
    double I_comp_Ty_;
    double I_comp_Tz_;
    
    void controlCore() override;
    void calcInteractionWrench();
    void estExternalWrenchCallback(const beetle::TaggedWrench & msg);

  protected:
    void rosParamInit() override;
    void externalWrenchEstimate() override;
  };
};
