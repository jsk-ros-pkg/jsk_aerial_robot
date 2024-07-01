// -*- mode: c++ -*-

#pragma once

#include <beetle/beetle_navigation.h>
#include <ninja/model/ninja_robot_model.h>

namespace aerial_robot_navigation
{
  enum module_joint
    {
     PITCH,
     YAW
    };
  
  class ModuleData
  {
  public:
    ModuleData(int id): id_(id), joint_pos_(), module_tree_()
    {}
    int id_;
    KDL::JntArray joint_pos_;
    KDL::Tree module_tree_;
  };

  class NinjaNavigator : public BeetleNavigator
  {
  public:
    NinjaNavigator();
    ~NinjaNavigator(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    double loop_du) override;
    void update() override;

  protected:
    void calcCenterOfMoving() override;
    void updateEntSysState();
    void updateAssemblyTree();

  private:
    void convertTargetPosFromCoG2CoM() override;

    boost::shared_ptr<NinjaRobotModel> ninja_robot_model_;
    ros::Subscriber entire_structure_sub_;
    ros::Publisher joint_control_pub_;

    std::map<int, ModuleData> assembled_modules_data_;
    KDL::Rotation target_com_rot_;
  };
};
