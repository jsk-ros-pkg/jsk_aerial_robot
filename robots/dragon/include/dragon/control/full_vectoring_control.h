// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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

#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <dragon/model/full_vectoring_robot_model.h>
#include <geometry_msgs/WrenchStamped.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerm.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf_conversions/tf_eigen.h>
#include <dragon/sensor/imu.h>
#include <visualization_msgs/MarkerArray.h>

namespace aerial_robot_control
{
  class DragonFullVectoringController: public PoseLinearController
  {
  public:
    DragonFullVectoringController();
    ~DragonFullVectoringController()
    {
      wrench_estimate_thread_.interrupt();
      wrench_estimate_thread_.join();
    }

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate) override;

  private:

    ros::Publisher flight_cmd_pub_; //for spinal
    ros::Publisher gimbal_control_pub_;
    ros::Publisher target_vectoring_force_pub_;
    ros::Publisher estimate_external_wrench_pub_;
    ros::Publisher rotor_interfere_wrench_pub_;
    ros::Publisher interfrence_marker_pub_;

    boost::shared_ptr<Dragon::FullVectoringRobotModel> dragon_robot_model_;
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control_;
    std::vector<float> target_base_thrust_;
    std::vector<double> target_gimbal_angles_;
    Eigen::VectorXd target_vectoring_f_;
    bool decoupling_;
    bool gimbal_vectoring_check_flag_;
    double allocation_refine_threshold_;
    int allocation_refine_max_iteration_;
    Eigen::VectorXd target_wrench_acc_cog_;

    /* external wrench */
    std::mutex wrench_mutex_;
    boost::thread wrench_estimate_thread_;
    Eigen::VectorXd init_sum_momentum_;
    Eigen::VectorXd est_external_wrench_;
    Eigen::MatrixXd momentum_observer_matrix_;
    Eigen::VectorXd integrate_term_;
    double prev_est_wrench_timestamp_;

    bool rotor_interfere_compensate_;
    double fz_bias_;
    double tx_bias_;
    double ty_bias_;
    double wrench_lpf_rate_;
    double fz_bias_thresh_;
    double comp_wrench_lpf_rate_;
    double rotor_interfere_torque_xy_weight_;
    double rotor_interfere_force_dev_weight_;
    Eigen::VectorXd rotor_interfere_force_;
    Eigen::VectorXd rotor_interfere_comp_wrench_;
    std::vector<Eigen::VectorXd> overlap_positions_;
    std::vector<double> overlap_weights_;
    std::vector<std::string> overlap_segments_;
    std::vector<std::string> overlap_rotors_;
    double overlap_dist_rotor_thresh_;
    double overlap_dist_rotor_relax_thresh_;
    double overlap_dist_link_thresh_;
    double overlap_dist_link_relax_thresh_;
    double overlap_dist_inter_joint_thresh_;


    void externalWrenchEstimate();
    const Eigen::VectorXd getTargetWrenchAccCog()
    {
      std::lock_guard<std::mutex> lock(wrench_mutex_);
      return target_wrench_acc_cog_;
    }
    void setTargetWrenchAccCog(const Eigen::VectorXd target_wrench_acc_cog)
    {
      std::lock_guard<std::mutex> lock(wrench_mutex_);
      target_wrench_acc_cog_ = target_wrench_acc_cog;
    }

    void controlCore() override;
    void rotorInterfereCompensation();
    void rosParamInit();
    void sendCmd();
  };
};
