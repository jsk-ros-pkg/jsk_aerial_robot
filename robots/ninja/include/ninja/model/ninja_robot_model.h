// -*- mode: c++ -*-

#pragma once

#include <beetle/model/beetle_robot_model.h>

using namespace aerial_robot_model;

class NinjaRobotModel : public BeetleRobotModel{
public:
  NinjaRobotModel(bool init_with_rosparam = true,
                  bool verbose = false,
                  double fc_t_min_thre = 0,
                  double epsilon = 10);
  virtual ~NinjaRobotModel() = default;

  void calcCenterOfMoving() override;
protected:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
private:
  KDL::Tree entire_structure_;
};
