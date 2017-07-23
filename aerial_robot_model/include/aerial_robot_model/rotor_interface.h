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

#ifndef HARDWARE_INTERFACE_ROTOR_INTERFACE_H
#define HARDWARE_INTERFACE_ROTOR_INTERFACE_H

#include <cassert>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>

#include <ros/ros.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <urdf_model/joint.h>
#include <urdf/urdfdom_compatibility.h>
#include <ros/common.h>
#include <tf/LinearMath/Transform.h>
#include <joint_limits_interface/joint_limits_interface_exception.h>

namespace hardware_interface
{

  /** A handle used to read the state of a single joint. */
  class RotorHandle
  {
  public:
    RotorHandle():force_(new double(0)) {}
    RotorHandle(ros::NodeHandle nh, const std::string& name);

    inline std::string getName() const {return name_;}
    inline double getForce()    const {return *force_;}
    inline double setForce(double force)    {*force_ = force;}
    inline double getTorque()    const {return getForce() * direction_ * m_f_rate_;}
    inline void setCommand(double command) {
      *force_ = (command * max_pwm_ / pwm_rate_ * f_pwm_rate_ + f_pwm_offset_);
    } // pwm to force

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

  class RotorInterface : public HardwareResourceManager<RotorHandle, ClaimResources>
  {

  public:
    RotorInterface();

    inline std::string getBaseLinkName() {return baselink_name_;}
    inline void setBaseLinkName(std::string baselink_name) { baselink_name_ = baselink_name; }
    inline void setBaseLinkOrientation(double qx, double qy, double qz, double qw) { q_.setValue(qx, qy, qz, qw); }
    inline void setBaseLinkAngular(double wx, double wy, double wz) { w_.setValue(wx, wy, wz); }
    inline void setBaseLink() {found_baselink_ = true; }
    inline bool foundBaseLink() {return found_baselink_; }

    inline void setJointNum(uint8_t joint_num) {joint_num_ = joint_num; }
    inline uint8_t getJointNum() {return joint_num_; }
    inline tf::Quaternion getBaseLinkOrientation() {return q_; }
    inline tf::Vector3 getBaseLinkAngular() {return w_;}

    /* temporary implementation */
    uint8_t getBaseLinkNo()
    {
      for(uint8_t i = 0; i < getNames().size(); i++)
        {
          std::stringstream link_no;
          link_no << i + 1;
          if(baselink_name_ == std::string("link") + link_no.str())
            return i;
        }
      return 255;
    }

  private:
    uint8_t joint_num_;
    bool found_baselink_;
    std::string baselink_name_;
    tf::Quaternion q_;
    tf::Vector3 w_;
  };

};
namespace rotor_limits_interface
{
  namespace internal
  {

    template<typename T>
    T saturate(const T val, const T min_val, const T max_val)
    {
      return std::min(std::max(val, min_val), max_val);
    }

  }

  /** \brief A handle used to enforce position, velocity, and effort limits of an effort-controlled rotor that does not
      have soft limits. */
  class EffortRotorSaturationHandle
  {
  public:
    EffortRotorSaturationHandle(const hardware_interface::RotorHandle& jh, urdf::JointConstSharedPtr urdf_joint);

    /** \return Rotor name. */
    std::string getName() const {return jh_.getName();}

    /**
     * \brief Enforce position, velocity, and effort limits for a rotor that is not subject to soft limits. // only force
     */
    void enforceLimits(const ros::Duration& /* period */)
    {
      jh_.setForce(internal::saturate(jh_.getForce(), min_force_, max_force_));
    }

  private:
    hardware_interface::RotorHandle jh_;

    double max_force_;
    double min_force_;
  };


  /**
   * \brief Interface for enforcing rotor limits.
   *
   * \tparam HandleType %Handle type. Must implement the following methods:
   *  \code
   *   void enforceLimits();
   *   std::string getName() const;
   *  \endcode
   */
  template <class HandleType>
  class RotorLimitsInterface : public hardware_interface::ResourceManager<HandleType>
  {
  public:
    HandleType getHandle(const std::string& name)
    {
      // Rethrow exception with a meaningful type
      try
        {
          return this->hardware_interface::ResourceManager<HandleType>::getHandle(name);
        }
      catch(const std::logic_error& e)
        {
          throw joint_limits_interface::JointLimitsInterfaceException(e.what());
        }
    }

    void enforceLimits(const ros::Duration& period)
    {
      typedef typename hardware_interface::ResourceManager<HandleType>::ResourceMap::iterator ItratorType;
      for (ItratorType it = this->resource_map_.begin(); it != this->resource_map_.end(); ++it)
        {
          it->second.enforceLimits(period);
        }
    }
  };

  class EffortRotorSaturationInterface : public RotorLimitsInterface<EffortRotorSaturationHandle> {};

};

#endif
