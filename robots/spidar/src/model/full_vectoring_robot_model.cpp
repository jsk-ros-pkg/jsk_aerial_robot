// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, JSK Lab
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

#include <spidar/model/full_vectoring_robot_model.h>

using namespace Spider;


FullVectoringRobotModel::FullVectoringRobotModel(bool init_with_rosparam, bool verbose, double edf_radius, double edf_max_tilt) :
  Dragon::FullVectoringRobotModel(init_with_rosparam, verbose, edf_radius, edf_max_tilt)
{
}

void FullVectoringRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  Dragon::FullVectoringRobotModel::updateRobotModelImpl(joint_positions);

  calcJointTorque(true);
  const auto& joint_torque = getJointTorque();
  // ROS_INFO_STREAM_THROTTLE(1.0, "hovering thrust is: " << getHoverVectoringF().transpose());
  // ROS_INFO_THROTTLE(1.0, "joint static torque for Limb1 is: [%f, %f, %f, %f]", joint_torque(0), joint_torque(1), joint_torque(4), joint_torque(5));
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(Spider::FullVectoringRobotModel, aerial_robot_model::RobotModel);
