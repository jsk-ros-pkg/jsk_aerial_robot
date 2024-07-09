// -*- mode: c++ -*-

#pragma once

#include <beetle/beetle_navigation.h>
#include <ninja/model/ninja_robot_model.h>
#include <geometry_msgs/Pose.h>
#include <regex>

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
    ModuleData(int id): id_(id), joint_pos_(), module_tree_(){}
    ModuleData(): id_(1), joint_pos_(), module_tree_(){}
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
    void setTargetComRot(KDL::Rotation target_com_rot){ target_com_rot_ = target_com_rot;}
    void setTargetCoMPoseFromCurrState();
    void setCoM2Base(const KDL::Frame com2base){com2base_ = com2base;}

    template<class T> T getCom2Base();
  protected:
    std::mutex mutex_com2base_;
    
    void calcCenterOfMoving() override;
    void updateEntSysState();
    void updateAssemblyTree();

  private:
    void convertTargetPosFromCoG2CoM() override;
    void setTargetCoMRotCallback(const spinal::DesireCoordConstPtr & msg);
    void assemblyJointPosCallback(const sensor_msgs::JointStateConstPtr& msg);

    ros::Publisher target_com_pose_pub_;
    boost::shared_ptr<NinjaRobotModel> ninja_robot_model_;
    ros::Subscriber target_com_rot_sub_;
    ros::Subscriber target_joints_pos_sub_;
    ros::Publisher joint_control_pub_;

    std::map<int, ModuleData> assembled_modules_data_;
    KDL::Rotation target_com_rot_;

    KDL::Frame com2base_;

    KDL::Frame test_frame_;
    int module_joint_num_;
  };
  template<> inline KDL::Frame NinjaNavigator::getCom2Base()
  {
    std::lock_guard<std::mutex> lock(mutex_com2base_);
    return com2base_;
  }
};
