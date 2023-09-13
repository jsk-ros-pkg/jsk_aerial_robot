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

#ifndef HARDWARE_INTERFACE_SPINAL_INTERFACE_H
#define HARDWARE_INTERFACE_SPINAL_INTERFACE_H

#include <aerial_robot_estimation/state_estimation.h>
#include <aerial_robot_simulation/rotor_handle.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <joint_limits_interface/joint_limits_interface_exception.h>
#include <limits>
#include <ros/ros.h>
#include <ros/common.h>
#include <state_estimate/state_estimate.h>
#include <tf/LinearMath/Transform.h>
#include <urdf_model/joint.h>
#include <urdf/urdfdom_compatibility.h>

namespace hardware_interface
{
  class SpinalInterface : public HardwareResourceManager<RotorHandle, ClaimResources>
  {

  public:
    SpinalInterface();
    bool init(ros::NodeHandle& nh, int joint_num);

    inline uint8_t getJointNum() {return joint_num_; }

    void setImuValue(double acc_x, double acc_y, double acc_z, double gyro_x, double gyro_y, double gyro_z);
    void setMagValue(double mag_x, double mag_y, double mag_z);
    void setTrueBaselinkOrientation(double q_x, double q_y, double q_z, double q_w);
    void setTrueBaselinkAngular(double w_x, double w_y, double w_z);

    tf::Vector3 getTrueBaselinkRPY();
    inline tf::Vector3 getTrueBaselinkAngular() { return baselink_angular_;}
    tf::Vector3 getTrueCogRPY();
    tf::Vector3 getTrueCogAngular();

    void stateEstimate();
    inline void onGround(bool flag) { on_ground_ = flag; }

    StateEstimate* getEstimatorPtr() {return &spinal_state_estimator_;}
  private:
    uint8_t joint_num_;

    tf::Matrix3x3 baselink_rot_;
    tf::Vector3 baselink_angular_;

    /* attitude estimator */
    bool on_ground_;
    StateEstimate spinal_state_estimator_;
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
      if(jh_.getForce() == 0) return;

      /* because of "inline double setForce(double force)    {*force_ = force;}", we can change the value with same address */
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
