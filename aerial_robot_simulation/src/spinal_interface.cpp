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
  SpinalInterface::SpinalInterface()
  {
    on_ground_ = true; // freeze attitude estimate, since the init state is on the ground, the contact force between ground is very unstable
  }

  bool SpinalInterface::init(ros::NodeHandle& nh, int joint_num)
  {
    joint_num_ = joint_num;
    spinal_state_estimator_.init(&nh);

    return true;
  }

  void SpinalInterface::stateEstimate()
  {
    if(on_ground_)
      {
        /* assume the robot is static, acc: [0, 0, g] */
        setImuValue(0, 0, aerial_robot_estimation::G, 0, 0, 0);
      }

    spinal_state_estimator_.update();
  }

  void SpinalInterface::setImuValue(double acc_x, double acc_y, double acc_z, double gyro_x, double gyro_y, double gyro_z)
  {
    spinal_state_estimator_.getAttEstimator()->setAcc(acc_x, acc_y, acc_z);
    spinal_state_estimator_.getAttEstimator()->setGyro(gyro_x, gyro_y, gyro_z);
  }

  void SpinalInterface::setMagValue(double mag_x, double mag_y, double mag_z)
  {
    spinal_state_estimator_.getAttEstimator()->setMag(mag_x, mag_y, mag_z);
  }

  void SpinalInterface::setGroundTruthStates(double q_x, double q_y, double q_z, double q_w,
                                             double w_x, double w_y, double w_z)
  {
    /* directly overwrite the state in attitude estimation */
    ap::Quaternion q(q_w, q_x, q_y, q_z);
    ap::Matrix3f rot; q.rotation_matrix(rot);
    ap::Vector3f ang_vel(w_x, w_y, w_z);

    spinal_state_estimator_.getAttEstimator()->setGroundTruthStates(rot, ang_vel);
  }

  void SpinalInterface::useGroundTruth(bool flag)
  {
    spinal_state_estimator_.getAttEstimator()->useGroundTruth(flag);
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
