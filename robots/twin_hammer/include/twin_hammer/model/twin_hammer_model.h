// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <spinal/FourAxisCommand.h>
using namespace aerial_robot_model;

class TwinHammerModel : public aerial_robot_model::RobotModel{
public:
  TwinHammerModel(bool init_with_rosparam = true,
                    bool verbose = false,
                    double fc_t_min_thre = 0,
                    double epsilon = 10);
  virtual ~TwinHammerModel() = default;
  const Eigen::Matrix3d& getGimbal1Inertia() const {return inertia_gimbal1_total_eigen_;}
  const Eigen::Matrix3d& getGimbal2Inertia() const {return inertia_gimbal2_total_eigen_;}

private:
  ros::NodeHandle nh_;
  Eigen::Matrix3d inertia_gimbal1_total_eigen_;
  Eigen::Matrix3d inertia_gimbal2_total_eigen_;

protected:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
};