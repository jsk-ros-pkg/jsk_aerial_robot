// -*- mode: c++ -*-

#pragma once

#include <beetle/beetle_navigation.h>
#include <ninja/model/ninja_robot_model.h>
#include <geometry_msgs/Pose.h>
#include <regex>
#include <aerial_robot_control/control/utils/pid.h>

namespace aerial_robot_navigation
{
  enum module_joint
    {
     PITCH,
     YAW
    };

    enum convergence_func
    {
     CONSTANT,
     FRAC,
     EXP
    };
  
  class ModuleData
  {
  public:
    ModuleData(int id): id_(id), joint_pos_(), goal_joint_pos_(), start_joint_pos_(), module_tree_(), first_joint_processed_time_(), joint_process_coef_(){}
    ModuleData(): id_(1), joint_pos_(), goal_joint_pos_(), start_joint_pos_(), module_tree_(), first_joint_processed_time_(), joint_process_coef_(){}
    int id_;
    KDL::JntArray joint_pos_;
    KDL::Tree module_tree_;

    std::vector<double> first_joint_processed_time_;
    std::vector<double> joint_process_coef_;
    KDL::JntArray goal_joint_pos_;
    KDL::JntArray start_joint_pos_;


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
    void morphingProcess();

    bool getFreeJointFlag(){return free_joint_flag_;}
    std::vector<double> getJointPosErr(){return joint_pos_errs_;}
    template<class T> T getCom2Base();
  protected:
    std::mutex mutex_com2base_;
    
    void calcCenterOfMoving() override;
    void rosParamInit() override;
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
    std::vector<double> joint_pos_errs_;

    KDL::Rotation target_com_rot_;
    KDL::Frame com2base_;
    KDL::Frame test_frame_;

    int module_joint_num_;
    double morphing_vel_;
    double joint_pos_chnage_thresh_;
    int joint_process_func_;

    bool free_joint_flag_;
    
  };
  template<> inline KDL::Frame NinjaNavigator::getCom2Base()
  {
    std::lock_guard<std::mutex> lock(mutex_com2base_);
    return com2base_;
  }

  template<> inline Eigen::Affine3d NinjaNavigator::getCom2Base()
  {
    return aerial_robot_model::kdlToEigen(NinjaNavigator::getCom2Base<KDL::Frame>());
  }  
};
