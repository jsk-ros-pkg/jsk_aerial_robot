// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* ros */
#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf/tf.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/static_transform_broadcaster.h>

using namespace std;

class SegmentPair
{
public:
  SegmentPair(const KDL::Segment& p_segment, const std::string& p_root, const std::string& p_tip):
    segment(p_segment), root(p_root), tip(p_tip){}

  KDL::Segment segment;
  std::string root, tip;
};

class RotorTfPublisher {
public:
  RotorTfPublisher(ros::NodeHandle nh, ros::NodeHandle nhp, const KDL::Tree& tree, const urdf::Model& model): nh_(nh), nhp_(nhp)
  {
    nhp_.param("rotor_joint_name", rotor_joint_name_, string("rotor"));
    nhp_.param("tf_prefix", tf_prefix_, string(""));

    addChildren(tree.getRootSegment());
    timer_ = nhp_.createTimer(0.1, &RotorTfPublisher::callbackFixedJoint, this, true);
  }

  ~RotorTfPublisher(){}

private:
  void callbackFixedJoint(const ros::TimerEvent& e)
  {
    std::vector<geometry_msgs::TransformStamped> tf_transforms;
    geometry_msgs::TransformStamped tf_transform;

    // loop over all fixed segments
    for (map<string, SegmentPair>::const_iterator seg=segments_rotor_.begin(); seg != segments_rotor_.end(); seg++) {
      geometry_msgs::TransformStamped tf_transform = tf2::kdlToTransform(seg->second.segment.pose(0));
      tf_transform.header.stamp = ros::Time::now();
      tf_transform.header.frame_id = tf::resolve(tf_prefix_, seg->second.root);
      tf_transform.child_frame_id =  tf::resolve(tf_prefix_, seg->second.tip);
      tf_transforms.push_back(tf_transform);
    }
    static_tf_broadcaster_.sendTransform(tf_transforms);
  }

  ros::Timer timer_;
  ros::NodeHandle nh_, nhp_;
  string rotor_joint_name_;
  string tf_prefix_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  std::map<std::string, SegmentPair> segments_rotor_;

  void addChildren(const KDL::SegmentMap::const_iterator segment)
  {
    const std::string& root = GetTreeElementSegment(segment->second).getName();

    const std::vector<KDL::SegmentMap::const_iterator>& children = GetTreeElementChildren(segment->second);
    for (unsigned int i=0; i<children.size(); i++)
      {
        const KDL::Segment& child = GetTreeElementSegment(children[i]->second);
        SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
        std::string::size_type pos = child.getJoint().getName().find(rotor_joint_name_);

        if(pos != std::string::npos)
            segments_rotor_.insert(make_pair(child.getJoint().getName(), s));

        addChildren(children[i]);
      }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rotor_tf_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  urdf::Model model;
  if (!model.initParam("robot_description"))
    return -1;

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return -1;
  }

  RotorTfPublisher rotor_tf_publisher(nh, nhp, tree, model);
  ros::spin();

  return 0;
}

