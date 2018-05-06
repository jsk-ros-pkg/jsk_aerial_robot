// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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


#ifndef DRAGON_TRANSFORM_CONTROL_H
#define DRAGON_TRANSFORM_CONTROL_H

/* ros */
#include <ros/ros.h>

/* basic transform control */
#include <hydrus/transform_control.h>
#include <unordered_map>

using namespace std;

class DragonTransformController : public TransformController
{
public:
  DragonTransformController(ros::NodeHandle nh, ros::NodeHandle nh_private, bool callback_flag = true);
  ~DragonTransformController(){}

  void gimbalProcess(sensor_msgs::JointState& state);

  void getLinksOrientation(std::vector<KDL::Rotation>& links_frame_from_cog)
  {
    links_frame_from_cog = links_frame_from_cog_;
  }

  void forwardKinematics(sensor_msgs::JointState& state);

  void setEdfsFromCog(const std::vector<Eigen::Vector3d>& edfs_origin_from_cog)
  {
    boost::lock_guard<boost::mutex> lock(origins_mutex_);
    assert(edfs_origin_from_cog_.size() == edfs_origin_from_cog.size());
    edfs_origin_from_cog_ = edfs_origin_from_cog;
  }

  void getEdfsFromCog(std::vector<Eigen::Vector3d>& edfs_origin_from_cog)
  {
    boost::lock_guard<boost::mutex> lock(origins_mutex_);
    int size = edfs_origin_from_cog_.size();
    for(int i=0; i< size; i++)
      edfs_origin_from_cog = edfs_origin_from_cog_;
  }

  bool overlapCheck(bool verbose = false);

  std::vector<double>& getGimbalNominalAngles() {return gimbal_nominal_angles_;}
private:
  ros::Publisher gimbal_control_pub_;

  bool gimbal_control_;

  std::vector<KDL::Rotation> links_frame_from_cog_;
  std::vector<Eigen::Vector3d> edfs_origin_from_cog_;

  /* check the relative horizontal distance between propellers */
  double edf_radius_; // the radius of EDF
  double edf_max_tilt_;

  void initParam();

  std::vector<double> gimbal_nominal_angles_;
};

#endif
