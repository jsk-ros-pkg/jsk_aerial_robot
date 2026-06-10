// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, JSK Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/o2r other materials provided
 * with the distribution.
 * * Neither the name of the JSK Lab nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <algorithm>
#include <hydrus/hydrus_tilted_robot_model.h>
#include <nlopt.hpp>
#include <OsqpEigen/OsqpEigen.h>
// ===== 【新規追加】内部モーメント指令用 =====
#include <std_msgs/Float64MultiArray.h> 

namespace aerial_robot_navigation
{
  class HydrusXiUnderActuatedNavigator : public BaseNavigator
  {
  public:
    HydrusXiUnderActuatedNavigator();
    ~HydrusXiUnderActuatedNavigator();

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    double loop_du) override;

    inline boost::shared_ptr<HydrusTiltedRobotModel> getRobotModelForPlan() { return robot_model_for_plan_;}
    inline OsqpEigen::Solver& getYawRangeLPSolver() { return yaw_range_lp_solver_;}

    inline KDL::JntArray& getJointPositionsForPlan()  {return joint_positions_for_plan_;}
    inline const double& getMaxMinYaw() const { return max_min_yaw_;}

    inline const double& getForceNormWeight() const { return force_norm_weight_;}
    inline const double& getForceVariantWeight() const { return force_variant_weight_;}
    inline const double& getYawTorqueWeight() const { return yaw_torque_weight_;}
    inline const double& getFCTMinWeight() const { return fc_t_min_weight_;}
    inline const double& getBaselinkRotThresh() const { return baselink_rot_thresh_;}
    inline const double& getFCTMinThresh() const { return fc_t_min_thresh_;}

    const std::vector<std::string>& getControlNames() const { return control_gimbal_names_; }
    const std::vector<int>& getControlIndices() const { return control_gimbal_indices_; }

    const bool getPlanVerbose() const { return plan_verbose_; }

    void setMaxMinYaw(const double max_min_yaw) { max_min_yaw_ = max_min_yaw;}

    // ===== 【新規追加】内部モーメント制御用パブリックアクセッサー & メソッド =====
    inline int getTargetJointIndex() const { return target_joint_index_; }
    inline double getTauDesTarget() const { return tau_des_target_; }
    inline bool hasMomentCommand() const { return has_moment_command_; }
    inline double getTargetMomentWeight() const { return target_moment_weight_; }

    // ★ エラー解消：外部の最適化関数からアクセスできるように public に移動
    double computeInternalMomentZ(
        const std::vector<double>& x,
        const boost::shared_ptr<HydrusTiltedRobotModel>& robot_model_ptr);

  private:
    ros::Publisher gimbal_ctrl_pub_;
    std::thread plan_thread_;
    boost::shared_ptr<HydrusTiltedRobotModel> robot_model_for_plan_;
    OsqpEigen::Solver yaw_range_lp_solver_;
    boost::shared_ptr<nlopt::opt> vectoring_nl_solver_;

    KDL::JntArray joint_positions_for_plan_;
    std::vector<std::string> control_gimbal_names_;
    std::vector<int> control_gimbal_indices_;
    double max_min_yaw_;

    bool plan_verbose_;
    bool maximize_yaw_;
    double force_norm_weight_; // cost func
    double force_variant_weight_; // cost func
    double yaw_torque_weight_; // cost func
    double fc_t_min_weight_; // cost func
    double baselink_rot_thresh_; // constraint func
    double fc_t_min_thresh_; // constraint func
    double gimbal_delta_angle_; // configuration state

    std::vector<double> opt_gimbal_angles_, prev_opt_gimbal_angles_;

    void threadFunc();
    bool plan();

    void rosParamInit() override;

    // ===== 【新規追加】内部モーメント制御用メンバ変数・内部メソッド =====
    int target_joint_index_;              // 対象関節インデックス (0=Joint1, 1=Joint2, 2=Joint3, -1=なし)
    double tau_des_target_;               // 目標内部モーメント [N⋅m]
    bool has_moment_command_;             // コマンド受信フラグ
    double target_moment_weight_;         // ペナルティ重み（ROS パラメータから読み込み）

    ros::Subscriber moment_command_sub_;  // /hydrus_xi/target_internal_moment の Subscriber

    void momentCommandCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    
    std::vector<double> extractThrustsFromOptVars(
        const std::vector<double>& x,
        const boost::shared_ptr<HydrusTiltedRobotModel>& robot_model_ptr);
        
    std::vector<double> extractGimbalsFromOptVars(
        const std::vector<double>& x);
  };
};