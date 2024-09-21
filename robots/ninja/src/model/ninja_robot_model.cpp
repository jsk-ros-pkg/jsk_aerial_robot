#include <ninja/model/ninja_robot_model.h>

NinjaRobotModel::NinjaRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  BeetleRobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
  /*Create copy tree model of single module*/
  init_module_tree_ = KDL::Tree("root");
  copyTreeStructure(getTree(), init_module_tree_);
  cog2baselink_vector_ = (getCog2Baselink<KDL::Frame>()).p;
}

void NinjaRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  GimbalrotorRobotModel::updateRobotModelImpl(joint_positions);
}

bool NinjaRobotModel::addSegmentsRecursively(const KDL::Tree& source_tree, KDL::Tree& destination_tree, const std::string& segment_name, std::unordered_set<std::string>& added_segments)
{
  const KDL::SegmentMap::const_iterator it = source_tree.getSegment(segment_name);
  if (it == source_tree.getSegments().end()) {
    std::cerr << "Segment " << segment_name << " not found in source tree!" << std::endl;
    return false;
  }

  const std::string parent_name = it->second.parent->second.segment.getName();
    
  if (added_segments.find(parent_name) == added_segments.end()) {
    if (!addSegmentsRecursively(source_tree, destination_tree, parent_name, added_segments)) {
      return false;
    }
  }

  if (!destination_tree.addSegment(it->second.segment, parent_name)) {
    std::cerr << "Failed to add segment " << segment_name << " to tree!" << std::endl;
    return false;
  }
    
  added_segments.insert(segment_name);
  return true;
}

void NinjaRobotModel::copyTreeStructure(const KDL::Tree& source_tree, KDL::Tree& destination_tree)
{
  std::unordered_set<std::string> added_segments;    
  const std::string& root_name = source_tree.getRootSegment()->first;
  destination_tree = KDL::Tree(root_name);
  added_segments.insert(root_name);

  for (const auto& segment_pair : source_tree.getSegments()) {
    const std::string& segment_name = segment_pair.second.segment.getName();
    if (added_segments.find(segment_name) == added_segments.end()) {
      if (!addSegmentsRecursively(source_tree, destination_tree, segment_name, added_segments)) {
        std::cerr << "Failed to copy segment " << segment_name << " to destination tree!" << std::endl;
        return;
      }
    }
  }
}

void NinjaRobotModel::setJointFree(KDL::Chain& source_chain, const std::string joint_name, const int rot_direction)
{
  KDL::Chain new_chain;
    
  for (unsigned int i = 0; i < source_chain.getNrOfSegments(); ++i)
    {
      KDL::Segment seg = source_chain.getSegment(i);        
      if (seg.getJoint().getName() == joint_name)
        {
          KDL::Segment modified_segment;
          switch(rot_direction)
            {
            case PITCH:
              modified_segment = KDL::Segment(seg.getName(), KDL::Joint(KDL::Joint::RotY), seg.getFrameToTip(), seg.getInertia());
              break;
            case YAW:
              modified_segment = KDL::Segment(seg.getName(), KDL::Joint(KDL::Joint::RotZ), seg.getFrameToTip(), seg.getInertia());
              break;
            default:
              ROS_ERROR("This joint direction is unsupported");
              return;
              break;
            }
          new_chain.addSegment(modified_segment);
        } else {
        new_chain.addSegment(seg);
      }
    }
  source_chain = new_chain;
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(NinjaRobotModel, aerial_robot_model::RobotModel);
