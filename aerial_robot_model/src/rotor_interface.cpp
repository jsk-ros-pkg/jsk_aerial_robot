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

#include <aerial_robot_model/rotor_interface.h>

namespace hardware_interface
{

  /** A handle used to read the state of a single joint. */
  RotorHandle::RotorHandle(ros::NodeHandle nh,  urdf::JointConstSharedPtr urdf_joint):  force_(new double(0)), max_pwm_(2000)
  {
    name_ = urdf_joint->name;
    direction_ = urdf_joint->axis.z;
    ROS_WARN("[%s], direction: %d", name_.c_str(), direction_);

    std::string full_name;
    ros::NodeHandle motor_nh("/motor_info");
    if (motor_nh.searchParam("f_pwm_rate", full_name))
      {
        motor_nh.getParam(full_name, f_pwm_rate_);
        ROS_INFO(" f_pwm_rate: %f", f_pwm_rate_);
      }
    else
      ROS_ERROR("Cannot get f_pwm_rate from ros nodehandle: %s", nh.getNamespace().c_str());

    if (motor_nh.searchParam("f_pwm_offset", full_name))
      {
        motor_nh.getParam(full_name, f_pwm_offset_);
        ROS_INFO(" f_pwm_offset: %f", f_pwm_offset_);
      }
    else
      ROS_ERROR("Cannot get f_pwm_offset from ros nodehandle: %s", nh.getNamespace().c_str());

    if (motor_nh.searchParam("m_f_rate", full_name))
      {
        motor_nh.getParam(full_name, m_f_rate_);
        ROS_INFO(" m_f_rate: %f", m_f_rate_);
      }
    else
      ROS_ERROR("Cannot get m_f_rate from ros nodehandle: %s", nh.getNamespace().c_str());

    if (motor_nh.searchParam("pwm_rate", full_name))
      {
        motor_nh.getParam(full_name, pwm_rate_);
        ROS_INFO(" pwm_rate: %f", pwm_rate_);
      }
    else
      ROS_ERROR("Cannot get pwm_rate from ros nodehandle: %s", nh.getNamespace().c_str());
  }

  RotorInterface::RotorInterface(): baselink_name_("baselink"), q_(), joint_num_(0), found_baselink_(false) {}
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
