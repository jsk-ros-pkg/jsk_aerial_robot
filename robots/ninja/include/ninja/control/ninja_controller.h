// -*- mode: c++ -*-

#pragma once
#include <beetle/control/beetle_controller.h>
#include <ninja/ninja_navigation.h>
#include <ninja/model/ninja_robot_model.h>

namespace aerial_robot_control
{
  enum
    {
     JOINT_TY = TZ +1,
     JOINT_TZ,
    };

  enum joint_rot
    {
     PITCH_JOINT,
     YAW_JOINT
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
    void pseudoAsmCallback(const std_msgs::BoolConstPtr & msg);
    boost::shared_ptr<aerial_robot_navigation::NinjaNavigator> ninja_navigator_;
    boost::shared_ptr<NinjaRobotModel> ninja_robot_model_;

    double joint_p_gain_;
    double joint_i_gain_;
    double joint_d_gain_;

    double joint_control_timestamp_;

    KDL::Tree module_tree_for_control_;

    ros::Subscriber pseudo_assembly_flag_sub_;
    ros::Publisher com_motion_pid_pub_;

    aerial_robot_msgs::PoseControlPid com_motion_pid_msg_;

  protected:
    void calcInteractionWrench() override;
    void externalWrenchEstimate() override;
    void rosParamInit() override;
    void controlCore() override;
    bool update() override;
    void reset() override;
  };
};
