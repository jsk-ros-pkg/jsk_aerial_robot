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
#include <tf/LinearMath/Vector3.h>
#include <aerial_robot_simulation/noise_model.h>

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

      auto robot_model_xml = aerial_robot_model::RobotModel::getRobotModelXml("robot_description", nh);
      TiXmlElement* m_f_rate_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("m_f_rate");
      if(!m_f_rate_attr)
        ROS_ERROR_STREAM_NAMED("RotorHandle", "Cannot get m_f_rate from ros nodehandle");
      else
        {
          m_f_rate_attr->Attribute("value", &m_f_rate_);
          ROS_DEBUG_STREAM("m_f_rate: " <<  m_f_rate_);
        }


      ros::NodeHandle motor_nh(nh, "motor_info");
      motor_nh.param("rotor_damping_rate", rotor_damping_rate_, 1.0); // s
      if(rotor_damping_rate_ > 1.0)
        {
          ROS_ERROR_NAMED("rotor handler", "invalid rotor damping rate %f, which should be less than 1, set to 1", rotor_damping_rate_);
          rotor_damping_rate_ = 1.0;
        }
      if(rotor_damping_rate_ <= 0)
        {
          ROS_ERROR_NAMED("rotor handler", "invalid rotor damping rate %f, which should be larger than 0, set to 0.1", rotor_damping_rate_);
          rotor_damping_rate_ = 0.1;
        }

      motor_nh.param("rotor_force_noise", rotor_force_noise_, 0.0); // N
      motor_nh.param("dual_rotor_moment_noise", dual_rotor_moment_noise_, 0.0);
      motor_nh.param("speed_rate", speed_rate_, 1.0); // rad/s/N , this is a virtual linear rate of speed-f
    }

    inline std::string getName() const {return name_;}
    double getForce()   const  { return *force_; }

    inline void setForce(double target_force, bool direct = false)
    {
      if(direct || target_force < 1e-6)
        {
          *force_ = target_force;
        }
      else
        {
          /* set the damping function */
          double current_force = *force_;
          *force_ =  (1 - rotor_damping_rate_)  * current_force + rotor_damping_rate_ * target_force + gazebo::gaussianKernel(rotor_force_noise_);
        }
    }

    inline tf::Vector3 getTorque()   const
    {
      return tf::Vector3(gazebo::gaussianKernel(dual_rotor_moment_noise_), 0, getForce() * direction_ * m_f_rate_);
    }
    inline void setCommand(double command); //no implement here

    inline double getSpeed() const
    {
      return *force_ * speed_rate_;
    }

  private:
    std::string name_;
    boost::shared_ptr<double> force_;
    int direction_;
    double f_pwm_rate_;
    double f_pwm_offset_;
    double m_f_rate_;
    double speed_rate_;
    double pwm_rate_;
    double max_pwm_;

    double rotor_damping_rate_;
    double rotor_force_noise_;
    double dual_rotor_moment_noise_;
  };
}

#endif
