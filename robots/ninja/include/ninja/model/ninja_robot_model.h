// -*- mode: c++ -*-

#pragma once

#include <beetle/model/beetle_robot_model.h>
#include <unordered_set>

using namespace aerial_robot_model;

class NinjaRobotModel : public BeetleRobotModel{
public:
  NinjaRobotModel(bool init_with_rosparam = true,
                  bool verbose = false,
                  double fc_t_min_thre = 0,
                  double epsilon = 10);
  virtual ~NinjaRobotModel() = default;
  const KDL::Tree& getInitModuleTree(){ return init_module_tree_; }
  const KDL::Vector& getInitCog2BaseVec(){ return cog2baselink_vector_; }

  void copyTreeStructure(const KDL::Tree& source_tree, KDL::Tree& destination_tree);

protected:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
  bool addSegmentsRecursively(const KDL::Tree& source_tree, KDL::Tree& destination_tree, const std::string& segment_name, std::unordered_set<std::string>& added_segments);
private:
  KDL::Tree init_module_tree_;
  KDL::Vector cog2baselink_vector_;
};
