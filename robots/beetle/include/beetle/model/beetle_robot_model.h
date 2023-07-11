// -*- mode: c++ -*-

#pragma once

#include <gimbalrotor/model/gimbalrotor_robot_model.h>

using namespace aerial_robot_model;

class BeetleRobotModel : public GimbalrotorRobotModel{
public:
  BeetleRobotModel(bool init_with_rosparam = true,
                    bool verbose = false,
                    double fc_t_min_thre = 0,
                    double epsilon = 10);
  virtual ~BeetleRobotModel() = default;

  template<class T> T getContactFrame();
  template<class T> T getCog2Cp();

  void setContactFrame(const KDL::Frame contact_frame){contact_frame_ = contact_frame;}
  void setCog2Cp(const KDL::Frame Cog2Cp){Cog2Cp_ = Cog2Cp;}

private:
  KDL::Frame contact_frame_;
  KDL::Frame Cog2Cp_;
  std::mutex mutex_contact_frame_;
  std::mutex mutex_cog2cp_;

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

template<> inline geometry_msgs::TransformStamped BeetleRobotModel::getContactFrame()
{
  return aerial_robot_model::kdlToMsg(BeetleRobotModel::getContactFrame<KDL::Frame>());
}
