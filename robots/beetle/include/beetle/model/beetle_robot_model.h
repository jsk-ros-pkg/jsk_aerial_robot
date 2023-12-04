// -*- mode: c++ -*-

#pragma once

#include <gimbalrotor/model/gimbalrotor_robot_model.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <algorithm>
using namespace aerial_robot_model;

enum module_state
  {
   SEPARATED,
   FOLLOWER,
   LEADER
  };

class BeetleRobotModel : public GimbalrotorRobotModel{
public:
  BeetleRobotModel(bool init_with_rosparam = true,
                    bool verbose = false,
                    double fc_t_min_thre = 0,
                    double epsilon = 10);
  virtual ~BeetleRobotModel() = default;

  template<class T> T getContactFrame();
  template<class T> T getCog2Cp();
  template<class T> T getCog2CoM();
  bool getCurrentAssembled(){return current_assembled_;}
  int getModuleState(){return module_state_;}
  int getReconfigFlag(){return reconfig_flag_;}
  int getMyID(){return my_id_;}
  int getLeaderID(){return leader_id_;}
  std::vector<int> getModuleIDs(){return assembled_modules_ids_;}
  bool getControlFlag(){return control_flag_;}
  

  void setContactFrame(const KDL::Frame contact_frame){contact_frame_ = contact_frame;}
  void setCog2Cp(const KDL::Frame Cog2Cp){Cog2Cp_ = Cog2Cp;}
  void setCog2CoM(const KDL::Frame Cog2CoM){Cog2CoM_ = Cog2CoM;}
  void setAssemblyFlag(const int key, const bool value){
    assembly_flags_[key] = value;
  }
  void setControlFlag(const bool control_flag){control_flag_ = control_flag;}

  std::map<int, bool> getAssemblyFlags(){return assembly_flags_;}
  int getMaxModuleNum(){return max_modules_num_;}

  void calcCenterOfMoving();

private:
  ros::NodeHandle nh_;
  ros::Publisher cog_com_dist_pub_;
  KDL::Frame contact_frame_;
  KDL::Frame Cog2Cp_;
  KDL::Frame Cog2CoM_;
  std::mutex mutex_contact_frame_;
  std::mutex mutex_cog2cp_;
  std::mutex mutex_cog2com_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformBroadcaster br_;
  int max_modules_num_ = 4; //TODO: get the value from rosparam
  int pre_assembled_modules_;
  int my_id_;
  int leader_id_;
  std::map<int, bool> assembly_flags_;
  bool reconfig_flag_;
  bool current_assembled_;
  bool control_flag_;
  std::vector<int> assembled_modules_ids_;
  int module_state_;
  

protected:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
};

template<> inline KDL::Frame BeetleRobotModel::getContactFrame()
{
  std::lock_guard<std::mutex> lock(mutex_contact_frame_);
  return contact_frame_;
}

template<> inline KDL::Frame BeetleRobotModel::getCog2Cp()
{
  std::lock_guard<std::mutex> lock(mutex_cog2cp_);
  return Cog2Cp_;
}

template<> inline KDL::Frame BeetleRobotModel::getCog2CoM()
{
  std::lock_guard<std::mutex> lock(mutex_cog2com_);
  return Cog2CoM_;
}

template<> inline geometry_msgs::TransformStamped BeetleRobotModel::getContactFrame()
{
  return aerial_robot_model::kdlToMsg(BeetleRobotModel::getContactFrame<KDL::Frame>());
}
