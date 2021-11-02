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

#include <aerial_robot_msgs/ApplyWrench.h>
#include <aerial_robot_msgs/ForceList.h>
#include <aerial_robot_control/control/pose_linear_controller.h>
#include <dragon/model/full_vectoring_robot_model.h>
#include <dragon/dragon_navigation.h>
#include <geometry_msgs/WrenchStamped.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerm.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <tf_conversions/tf_eigen.h>
#include <dragon/sensor/imu.h>
#include <visualization_msgs/MarkerArray.h>
#include <nlopt.hpp>

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

    const boost::shared_ptr<Dragon::FullVectoringRobotModel> getDragonRobotModel() const { return dragon_robot_model_;}
    const boost::shared_ptr<aerial_robot_model::RobotModel> getRobotModelForControl() const { return robot_model_for_control_;}

    const Eigen::VectorXd getTargetWrench()
    {
      std::lock_guard<std::mutex> lock(wrench_mutex_);
      return target_wrench_cog_;
    }

    const Eigen::VectorXd getTargetAcc() { return target_acc_cog_; }

    void reset() override;

    // only for read
    const std::vector<int>& getRollLockedGimbal() const { return roll_locked_gimbal_; }
    const std::vector<double>& getGimbalNominalAngles() const { return gimbal_nominal_angles_; }


  private:

    ros::Publisher flight_cmd_pub_; //for spinal
    ros::Publisher gimbal_control_pub_;
    ros::Publisher target_vectoring_force_pub_;
    ros::Publisher estimate_external_wrench_pub_;
    ros::Publisher rotor_interfere_wrench_pub_;
    ros::Publisher interfrence_marker_pub_;

    boost::shared_ptr<Dragon::FullVectoringRobotModel> dragon_robot_model_;
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control_;
    std::vector<double> target_base_thrust_;
    std::vector<double> target_gimbal_angles_;
    Eigen::VectorXd target_vectoring_f_;

    bool gimbal_vectoring_check_flag_;
    Eigen::VectorXd target_acc_cog_; // 6DoF acc (only represent the dynamic motion)
    Eigen::VectorXd target_wrench_cog_; // 6DoF wrench (target_acc (DoF) * inertia + external wrench compensate term)

    /* control method */
    bool integral_vectoring_allocation_; // use gimbal roll and ptich for all control terms.

    /* allocation method */
    bool enable_static_allocation_method_;
    bool enable_nonlinear_allocation_method_;
    bool enable_gradient_allocation_method_;

    /* static iterative allocation */
    double allocation_refine_threshold_;
    int allocation_refine_max_iteration_;


    /* SR inverse for allocation */
    double sr_inverse_sigma_;
    double sr_inverse_acc_diff_thresh_;
    double sr_inverse_thrust_diff_thresh_;

    /* modification of gimbal roll method to enhance FC Tmin */
    double gimbal_roll_target_lin_vel_thresh_, gimbal_roll_target_ang_vel_thresh_; // threshold to decide whether uses gimbal roll angles to handle velocity control term.
    bool low_fctmin_;
    double fctmin_thresh_; //, fctmin_hard_thresh_;
    int low_fctmin_cnt_, normal_fctmin_cnt_;
    int fctmin_status_change_thresh_;
    double fixed_gimbal_roll_offset_; // heuristic
    bool smooth_change_;
    double gimbal_roll_change_thresh_;
    double gimbal_roll_offset_;

    /* nonlinear allocation for full vectoring  */
    std::vector<double> prev_target_gimbal_angles_;
    std::vector<double> gimbal_nominal_angles_;
    std::vector<int> roll_locked_gimbal_;

    /* external wrench */
    std::mutex wrench_mutex_;
    boost::thread wrench_estimate_thread_;
    Eigen::VectorXd init_sum_momentum_;
    Eigen::VectorXd est_external_wrench_;
    Eigen::MatrixXd momentum_observer_matrix_;
    Eigen::VectorXd integrate_term_;
    double prev_est_wrench_timestamp_;

    /* rotor aerodynamic interference */
    bool rotor_interfere_compensate_;
    bool disable_torque_compensate_;
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

    /* external (static) wrench compensation */
    ros::Subscriber add_external_wrench_sub_, clear_external_wrench_sub_;
    void addExternalWrenchCallback(const aerial_robot_msgs::ApplyWrench::ConstPtr& msg);
    void clearExternalWrenchCallback(const std_msgs::String::ConstPtr& msg);

    /* extra vectoring force (i.e., for grasping) */
    ros::Subscriber extra_vectoring_force_sub_;
    std::vector<Eigen::Vector3d> extra_vectoring_forces_;
    void extraVectoringForceCallback(const aerial_robot_msgs::ForceListConstPtr& msg);

    void externalWrenchEstimate();

    /* allocation method */
    bool staticIterativeAllocation(const int iterative_cnt, const double iterative_threshold, const Eigen::VectorXd target_acc, const std::map<std::string, Dragon::ExternalWrench>& external_wrench_map, const std::vector<Eigen::Vector3d>& extra_vectoring_forces, KDL::JntArray& gimbal_processed_joint, const std::vector<Eigen::Matrix3d>& links_rotation_from_cog, std::vector<double>& thrust_forces, std::vector<double>& gimbal_angles, Eigen::VectorXd& vectoring_forces);
    bool strictNonlinearAllocation(const Eigen::VectorXd target_acc, const std::map<std::string, Dragon::ExternalWrench>& external_wrench_map, const std::vector<Eigen::Vector3d>& extra_vectoring_forces, KDL::JntArray& gimbal_processed_joint, const std::vector<Eigen::Matrix3d>& links_rotation_from_cog);
    bool gradientDescentAllocation(const int iterative_cnt, const Eigen::VectorXd target_acc, const std::map<std::string, Dragon::ExternalWrench>& external_wrench_map, const std::vector<Eigen::Vector3d>& extra_vectoring_forces, KDL::JntArray& gimbal_processed_joint, const std::vector<Eigen::Matrix3d>& links_rotation_from_cog, std::vector<double>& thrust_forces, std::vector<double>& gimbal_angles);

    bool srInverseAllocation(const Eigen::MatrixXd& q_wrench, const Eigen::VectorXd& nominal_f, const Eigen::VectorXd& target_wrench, const Eigen::VectorXd& nominal_thrust_force, std::vector<double>& thrust_force, std::vector<double>& gimbal_angles);

    Eigen::VectorXd calcExternalWrenchSum(const std::map<std::string, Dragon::ExternalWrench>& external_wrench_map);

    void setTargetAcc(const Eigen::VectorXd target_acc_cog) { target_acc_cog_ = target_acc_cog;}
    void setTargetWrench(const Eigen::VectorXd target_wrench_cog)
    {
      std::lock_guard<std::mutex> lock(wrench_mutex_);
      target_wrench_cog_ = target_wrench_cog;
    }

    void controlCore() override;
    void rotorInterfereCompensation();
    void rosParamInit();
    void sendCmd();
  };
};
