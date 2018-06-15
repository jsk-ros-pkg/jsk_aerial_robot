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

#include <hydrus/differential_kinematics/cost/base_plugin.h>

namespace differential_kinematics
{
  namespace cost
  {
    template <class motion_planner>
    class StateVel :public Base<motion_planner>
    {
    public:
      StateVel() {}
      ~StateVel(){}

      void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                      boost::shared_ptr<motion_planner> planner, std::string cost_name,
                      bool orientation, bool full_body)
      {
        Base<motion_planner>::initialize(nh, nhp, planner, cost_name, orientation, full_body);

        Base<motion_planner>::nhp_.param ("w_joint_vel", w_joint_vel_, 0.001);
        if(Base<motion_planner>::verbose_) std::cout << "w_joint_vel: " << std::setprecision(3) << w_joint_vel_ << std::endl;
        Base<motion_planner>::nhp_.param ("w_root_translational_vel", w_root_translational_vel_, 0.001);
        if(Base<motion_planner>::verbose_) std::cout << "w_root_translational_vel: " << std::setprecision(3) << w_root_translational_vel_ << std::endl;
        Base<motion_planner>::nhp_.param ("w_root_rotational_vel", w_root_rotational_vel_, 0.001);
        if(Base<motion_planner>::verbose_) std::cout << "w_root_rotational_vel: " << std::setprecision(3) << w_root_rotational_vel_ << std::endl;
      }

      bool getHessianGradient(bool& convergence, Eigen::MatrixXd& H, Eigen::VectorXd& g, bool debug = false)
      {
        H = Eigen::MatrixXd::Identity(Base<motion_planner>::j_ndof_ + 6, Base<motion_planner>::j_ndof_ + 6);
        if(Base<motion_planner>::full_body_)
          {
            H.block(0, 0, 3, 3) *= w_root_translational_vel_;
            H.block(3, 3, 3, 3) *= w_root_rotational_vel_;
          }
        else
          H.block(0, 0, 6, 6) *= 0;

        H.block(6, 6, Base<motion_planner>::j_ndof_, Base<motion_planner>::j_ndof_) *= w_joint_vel_;

        g = Eigen::VectorXd::Zero(Base<motion_planner>::j_ndof_ + 6);

        if(debug)
          {
            std::cout << "cost name: " << Base<motion_planner>::cost_name_ << ", H: \n" << H << std::endl;
            std::cout << "cost name: " << Base<motion_planner>::cost_name_ << ", g: \n" << g.transpose() << std::endl;
          }

        convergence = true;
        return true;
      }

    protected:

      double w_joint_vel_;
      double w_root_translational_vel_;
      double w_root_rotational_vel_;
    };
  };
};

#include <pluginlib/class_list_macros.h>
#include <hydrus/differential_kinematics/planner_core.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::cost::StateVel<differential_kinematics::Planner>, differential_kinematics::cost::Base<differential_kinematics::Planner>);
