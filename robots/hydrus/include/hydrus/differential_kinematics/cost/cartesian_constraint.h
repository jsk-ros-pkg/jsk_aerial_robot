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

#ifndef DIFFERENTIA_KINEMATICS_CARTESIAN_CONSTRAINT_PLUGIN_H
#define DIFFERENTIA_KINEMATICS_CARTESIAN_CONSTRAINT_PLUGIN_H

#include <hydrus/differential_kinematics/cost/base_plugin.h>

/* kinemtiacs */
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

/* linear math */
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

namespace differential_kinematics
{
  namespace cost
  {
    template <class motion_planner>
    class CartersianConstraint :public Base<motion_planner>
    {
    public:
      CartersianConstraint() {}
      ~CartersianConstraint(){}

      void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                      boost::shared_ptr<motion_planner>, std::string cost_name,
                      bool orientation, bool full_body);

      bool getHessianGradient(bool& convergence, Eigen::MatrixXd& H, Eigen::VectorXd& g, bool debug = false);

      /* special process */
      void updateChain(std::string root_link, std::string parent_link, KDL::Segment referece_frame)
      {
        Base<motion_planner>::planner_->getRobotModelPtr()->getModelTree().getChain(root_link, parent_link, chain_);
        chain_.addSegment(referece_frame);
      }

      void updateTargetFrame(tf::Transform target_reference_frame)
      {
        target_reference_frame_ = target_reference_frame;
      }

    protected:
      double pos_convergence_thre_;
      double rot_convergence_thre_;
      double pos_err_max_;
      double rot_err_max_;
      double w_pos_err_constraint_;
      double w_att_err_constraint_;

      KDL::Chain chain_;
      Eigen::MatrixXd W_cartesian_err_constraint_;
      tf::Transform target_reference_frame_;

      bool calcJointJacobian(Eigen::MatrixXd& jacobian, bool debug);
    };
  };
};

#endif
