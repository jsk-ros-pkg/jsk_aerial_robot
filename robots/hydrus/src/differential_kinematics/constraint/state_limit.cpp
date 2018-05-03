// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
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

#include <hydrus/differential_kinematics/constraint/base_plugin.h>

namespace differential_kinematics
{
  namespace constraint
  {
    template <class motion_planner>
    class StateLimit :public Base<motion_planner>
    {
    public:
      StateLimit() {}
      ~StateLimit(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<motion_planner> planner, std::string constraint_name,
                              bool orientation, bool full_body)
      {
        Base<motion_planner>::initialize(nh, nhp, planner, constraint_name, orientation, full_body);
        Base<motion_planner>::nc_ = 6 + Base<motion_planner>::j_ndof_;

        joint_angle_min_ = Base<motion_planner>::planner_->getRobotModelPtr()->joint_angle_min_;
        joint_angle_max_ = Base<motion_planner>::planner_->getRobotModelPtr()->joint_angle_max_;

        Base<motion_planner>::nhp_.param ("root_translational_vel_thre", root_translational_vel_thre_, 0.05);
        if(Base<motion_planner>::verbose_) std::cout << "root_translational_vel_thre: " << std::setprecision(3) << root_translational_vel_thre_ << std::endl;
        Base<motion_planner>::nhp_.param ("root_rotational_vel_thre", root_rotational_vel_thre_, 0.1);
        if(Base<motion_planner>::verbose_) std::cout << "root_rotational_vel_thre: " << std::setprecision(3) << root_rotational_vel_thre_ << std::endl;
        Base<motion_planner>::nhp_.param ("joint_vel_thre", joint_vel_thre_, 0.1);
        if(Base<motion_planner>::verbose_) std::cout << "joint_vel_thre: " << std::setprecision(3) << joint_vel_thre_ << std::endl;
        Base<motion_planner>::nhp_.param ("joint_vel_constraint_range", joint_vel_constraint_range_, 0.2);
        if(Base<motion_planner>::verbose_) std::cout << "joint_vel_constraint_range: " << std::setprecision(3) << joint_vel_constraint_range_ << std::endl;
        Base<motion_planner>::nhp_.param ("joint_vel_forbidden_range", joint_vel_forbidden_range_, 0.1);
        if(Base<motion_planner>::verbose_) std::cout << "joint_vel_forbidden_range: " << std::setprecision(3) << joint_vel_forbidden_range_ << std::endl;
      }

      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
        A = Eigen::MatrixXd::Zero(Base<motion_planner>::nc_, Base<motion_planner>::nc_);
        lb = Eigen::VectorXd::Constant(Base<motion_planner>::nc_, 1);
        ub = Eigen::VectorXd::Constant(Base<motion_planner>::nc_, 1);

        /* root */
        if(Base<motion_planner>::full_body_)
          {
            lb.segment(0, 3) *= -root_translational_vel_thre_;
            ub.segment(0, 3) *= root_translational_vel_thre_;
            lb.segment(3, 3) *= -root_rotational_vel_thre_;
            ub.segment(3, 3) *= root_rotational_vel_thre_;

            if(Base<motion_planner>::planner_->getMultilinkType() == motion_planner::MULTILINK_TYPE_SE2)
              {
                lb.segment(2, 3) *= 0;
                ub.segment(2, 3) *= 0;
              }
          }
        else
          {
            lb.head(6) *= 0;
            ub.head(6) *= 0;
          }

        /* joint */
        Eigen::VectorXd joint_vector = Base<motion_planner>::planner_->getTargetJointVector();

        lb.tail(Base<motion_planner>::j_ndof_) *= -joint_vel_thre_;
        ub.tail(Base<motion_planner>::j_ndof_) *= joint_vel_thre_;
        for(int i = 0; i < Base<motion_planner>::j_ndof_; i ++)
          {
            /* min */
            if(joint_vector(i) - joint_angle_min_ < joint_vel_constraint_range_)
              lb(i + 6) *= (joint_vector(i) - joint_angle_min_ - joint_vel_forbidden_range_) / (joint_vel_constraint_range_ - joint_vel_forbidden_range_);
            /* max */
            if(joint_angle_max_ - joint_vector(i)  < joint_vel_constraint_range_)
              ub(i + 6) *=
                (joint_angle_max_ - joint_vector(i) - joint_vel_forbidden_range_) / (joint_vel_constraint_range_ - joint_vel_forbidden_range_);
          }

        if(debug)
          {
            std::cout << "constraint name: " << Base<motion_planner>::constraint_name_ << ", lb: \n" << lb.transpose() << std::endl;
            std::cout << "constraint name: " << Base<motion_planner>::constraint_name_ << ", ub: \n" << ub.transpose() << std::endl;
          }
      }

      bool directConstraint(){return true;}

    protected:
      double root_translational_vel_thre_;
      double root_rotational_vel_thre_;
      double joint_vel_thre_;
      double joint_vel_constraint_range_;
      double joint_vel_forbidden_range_;
      double joint_angle_min_;
      double joint_angle_max_;
    };
  };
};

#include <pluginlib/class_list_macros.h>
#include <hydrus/differential_kinematics/planner_core.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::StateLimit<differential_kinematics::Planner>, differential_kinematics::constraint::Base<differential_kinematics::Planner>);



