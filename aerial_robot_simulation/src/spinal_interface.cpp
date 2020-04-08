///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2017, JSK.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/* Author: Moju Zhao
   Desc:   Hardware Interface for propeller rotor in Gazebo
*/

#include <aerial_robot_simulation/spinal_interface.h>

namespace hardware_interface
{
  SpinalInterface::SpinalInterface(): q_(), joint_num_(0)
  {
  }

  bool SpinalInterface::init(const urdf::Model& model)
  {
    kdl_parser::treeFromUrdfModel(model, tree_);

    /* get baselink from robot model */
    auto robot_model_xml = aerial_robot_model::RobotModel::getRobotModelXml("robot_description");
    TiXmlElement* baselink_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("baselink");
    if(!baselink_attr)
      {
        ROS_ERROR_STREAM_NAMED("spianl interface", "Failed to find baselink attribute from urdf model, please add '<baselink name=\"fc\" \/>' to your urdf file");
        return false;
      }
    baselink_ = std::string(baselink_attr->Attribute("name"));

    KDL::SegmentMap::const_iterator it = tree_.getSegment(baselink_);
    if(it == tree_.getSegments().end())
      {
        ROS_ERROR_STREAM_NAMED("spianl interface", "Failed to find baselink '" << baselink_ << "' in urdf from robot_description");
        return false;
      }

    std::function<KDL::Frame (const KDL::SegmentMap::const_iterator& ) > recursiveFindParent = [&recursiveFindParent, this](const KDL::SegmentMap::const_iterator& it)
      {
        const KDL::TreeElementType& currentElement = it->second;
        KDL::Frame currentFrame = GetTreeElementSegment(currentElement).pose(0);

        KDL::SegmentMap::const_iterator parentIt = GetTreeElementParent(currentElement);

        if(GetTreeElementSegment(parentIt->second).getJoint().getType() != KDL::Joint::None ||
           parentIt == tree_.getRootSegment())
          {
            baselink_parent_ = parentIt->first;
            return currentFrame;
          }
        else
          {
            return recursiveFindParent(parentIt) * currentFrame;
          }
      };

    baselink_offset_ = recursiveFindParent(it);
    if(baselink_parent_ == std::string("none"))
      {
        ROS_ERROR_STREAM_NAMED("spianl interface", "Can not find the parent of the baselink '" << baselink_);
        return false;
      }

    ROS_DEBUG_NAMED("aerial_robot_hw", "Find the parent link for the baselink %s:  %s, offset is [%f, %f, %f]", baselink_.c_str(), baselink_parent_.c_str(), baselink_offset_.p.x(), baselink_offset_.p.y(), baselink_offset_.p.z());
    return true;
  }
};

namespace rotor_limits_interface
{
  EffortRotorSaturationHandle::EffortRotorSaturationHandle(const hardware_interface::RotorHandle& jh, urdf::JointConstSharedPtr urdf_joint)
    : jh_(jh)
  {
    if (!urdf_joint)
      {
        throw joint_limits_interface::JointLimitsInterfaceException("Cannot laod param for rotor '" + getName() +
                                                                    "'. It has no urdf for this rotor.");
      }
    min_force_ = urdf_joint->limits->lower;
    max_force_ = urdf_joint->limits->upper;
    ROS_DEBUG_STREAM_NAMED("robot_imits_interface","Loading joint '" << jh_.getName()
                           << "' of max force  '" << max_force_ << "' and min force '" << min_force_);
  }

};
