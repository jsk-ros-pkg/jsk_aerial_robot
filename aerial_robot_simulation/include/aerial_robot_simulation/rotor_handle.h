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

#ifndef HARDWARE_INTERFACE_ROTOR_HANDLE_H
#define HARDWARE_INTERFACE_ROTOR_HANDLE_H


#include <ros/ros.h>
#include <urdf_model/joint.h>

namespace hardware_interface
{

  /** A handle used to read the state of a single joint. */
  class RotorHandle
  {
  public:
    RotorHandle():force_(new double(0)) {}
    RotorHandle(ros::NodeHandle nh,  urdf::JointConstSharedPtr urdf_joint):  force_(new double(0)), max_pwm_(2000)
    {
      name_ = urdf_joint->name;
      direction_ = urdf_joint->axis.z;

      std::string full_name;
      ros::NodeHandle motor_nh("/motor_info");

      if (motor_nh.searchParam("m_f_rate", full_name))
        {
          motor_nh.getParam(full_name, m_f_rate_);
          ROS_DEBUG_STREAM("m_f_rate: " <<  m_f_rate_);
        }
      else
        ROS_ERROR_STREAM_NAMED("RotorHandle", "Cannot get m_f_rate from ros nodehandle");
    }

    inline std::string getName() const {return name_;}
    inline double getForce()    const {return *force_;}
    inline double setForce(double force)    {*force_ = force;}
    inline double getTorque()    const {return getForce() * direction_ * m_f_rate_;}
    inline void setCommand(double command); //no implement here

  private:
    std::string name_;
    boost::shared_ptr<double> force_;
    int direction_;
    double f_pwm_rate_;
    double f_pwm_offset_;
    double m_f_rate_;
    double pwm_rate_;
    double max_pwm_;
  };
}

#endif
