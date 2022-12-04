// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, JSK Lab
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

#pragma once

#include <dragon/model/full_vectoring_robot_model.h>

namespace Tiger
{
  class GroundRobotModel : public Dragon::FullVectoringRobotModel
  {
  public:
    GroundRobotModel(bool init_with_rosparam = true,
                            bool verbose = false,
                            double edf_radius = 0,
                            double edf_max_tilt = 0);
    virtual ~GroundRobotModel() = default;

    inline const Eigen::VectorXd& getStaticVectoringF() const {return static_vectoring_f_;}
    inline const Eigen::VectorXd& getStaticJointT() const {return static_joint_t_;}
    inline void setStaticVectoringF(Eigen::VectorXd f) { static_vectoring_f_ = f; }
    inline void setStaticJointT(Eigen::VectorXd t) { static_joint_t_ = t; }

    inline void setFreeleg(int id) { free_leg_id_ = id; }
    inline const int getFreeleg() const { return free_leg_id_; }
    inline void resetFreeleg() { free_leg_id_ = -1; }

  protected:
    void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;

    bool calculate_walk_statics_;

    double joint_torque_limit_;
    int free_leg_id_; // start from 0: [0, leg_num -1]; -1: non touched leg

    Eigen::VectorXd static_vectoring_f_;
    Eigen::VectorXd static_joint_t_;
  };
};

