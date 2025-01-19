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

#include <dragon/model/hydrus_like_robot_model.h>
#include <nlopt.hpp>
#include <numeric>
#include <ros/console.h>
#include <sstream>
#include <tf_conversions/tf_kdl.h>

namespace Dragon
{

  class FullVectoringRobotModel : public HydrusLikeRobotModel
  {
  public:
    FullVectoringRobotModel(bool init_with_rosparam = true,
                            bool verbose = false,
                            double edf_radius = 0,
                            double edf_max_tilt = 0);
    virtual ~FullVectoringRobotModel() = default;


    inline boost::shared_ptr<aerial_robot_model::transformable::RobotModel> getRobotModelForPlan() { return robot_model_for_plan_;}
    inline const Eigen::VectorXd& getHoverVectoringF() const {return hover_vectoring_f_;}

    const std::vector<int> getRollLockedGimbal()
    {
      std::lock_guard<std::mutex> lock(roll_locked_gimbal_mutex_);
      return roll_locked_gimbal_;
    }

    double getMinForceNormalizedWeight() const {return min_force_normalized_weight_;}
    double getMinTorqueNormalizedWeight() const {return min_torque_normalized_weight_;}
    template <class T> std::vector<T> getGimbalRollOriginFromCog() const ; // only for gimbal lock planning
    const std::vector<int>& getRollLockedGimbalForPlan() const { return roll_locked_gimbal_for_plan_; } // only for gimbal lock planning
    void setRollLockedGimbalForPlan(const std::vector<int> roll_locked_gimbal_for_plan) { roll_locked_gimbal_for_plan_ = roll_locked_gimbal_for_plan; } // only for gimbal lock planning

    void setRollLockedGimbal(const std::vector<int> roll_locked_gimbal)
    {
      std::lock_guard<std::mutex> lock(roll_locked_gimbal_mutex_);
      roll_locked_gimbal_ = roll_locked_gimbal;
    }

    // TODO: overwrite the implementation about the Jacobian, since the gimbal processing is different from the hydrus-like model.
    // right now, we approximate to that  one.


    // rewrite
    Eigen::VectorXd calcFeasibleControlFxyDists(const std::vector<int>& gimbal_roll_lock, const std::vector<double>& locked_roll_angles, int rotor_num, const std::vector<Eigen::Matrix3d>& link_rot);
    Eigen::VectorXd calcFeasibleControlTDists(const std::vector<int>& gimbal_roll_lock, const std::vector<double>& locked_roll_angles, int rotor_num, const std::vector<Eigen::Vector3d>& rotor_pos, const std::vector<Eigen::Matrix3d>& link_rot);

    bool stabilityCheck(bool verbose) override;

  private:

    boost::shared_ptr<aerial_robot_model::transformable::RobotModel> robot_model_for_plan_;

    Eigen::VectorXd hover_vectoring_f_;

    bool debug_verbose_;
    double gimbal_lock_threshold_;
    std::vector<double> locked_angles_;
    std::vector<int> roll_locked_gimbal_, prev_roll_locked_gimbal_;
    std::vector<int> roll_locked_gimbal_for_plan_;
    std::vector<int> roll_lock_status_accumulator_;
    std::vector<int> roll_lock_angle_smooth_;
    std::vector<KDL::Vector> gimbal_roll_origin_from_cog_;
    double gimbal_delta_angle_;
    double link_att_change_threshold_;
    int lock_status_change_threshold_;
    double min_force_weight_;
    double min_torque_weight_;
    double min_force_normalized_weight_;
    double min_torque_normalized_weight_;
    std::vector<KDL::Rotation> prev_links_rotation_from_cog_;
    int robot_model_refine_max_iteration_;
    double robot_model_refine_threshold_;
    std::mutex roll_locked_gimbal_mutex_;
    double gimbal_roll_change_threshold_;

    std::vector<Eigen::Vector3d> overlap_positions_;
    std::vector<double> overlap_magnitudes_;

    //private functions
    void getParamFromRos();
    void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;

    /* gimbal roll angle optimization problem */
    std::vector<double>  calcBestLockGimbalRoll(const std::vector<int>& gimbal_roll_lock, const std::vector<int>& prev_gimbal_roll_lock, const std::vector<double>& prev_opt_locked_roll_angles);

    void calcStaticThrust() override {}; // do nothing
    void calcFeasibleControlFDists() {}; // do nothing
    void calcFeasibleControlTDists() {}; // do nothing
  };

  template<> inline std::vector<KDL::Vector> FullVectoringRobotModel::getGimbalRollOriginFromCog() const
  {
    return gimbal_roll_origin_from_cog_;
  }

  template<> inline std::vector<Eigen::Vector3d> FullVectoringRobotModel::getGimbalRollOriginFromCog() const
  {
    return aerial_robot_model::kdlToEigen(getGimbalRollOriginFromCog<KDL::Vector>());
  }
};

