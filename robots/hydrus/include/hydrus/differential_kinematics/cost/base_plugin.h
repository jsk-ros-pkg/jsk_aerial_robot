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

#ifndef DIFFERENTIA_KINEMATICS_COST_PLUGIN_H
#define DIFFERENTIA_KINEMATICS_COST_PLUGIN_H

#include <ros/ros.h>

/* Linear Math */
#include <Eigen/Dense>

namespace differential_kinematics
{
  namespace cost
  {
    template <class motion_planner>
    class Base
    {
    public:
      Base() {}
      ~Base(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<motion_planner> planner, std::string cost_name,
                              bool orientation, bool full_body)
      {
        nh_ = ros::NodeHandle(nh, cost_name);
        nhp_ = ros::NodeHandle(nhp, cost_name);

        planner_ = planner;

        //std::cout << nhp_.getNamespace() << std::endl;
        nhp_.param("verbose", verbose_, false);

        cost_name_ = cost_name;
        orientation_ = orientation;
        full_body_ = full_body;
        j_ndof_ = planner_->getRobotModelPtr()->getActuatorJointMap().size();
      }

      virtual bool getHessianGradient(bool& convergence, Eigen::MatrixXd& H, Eigen::VectorXd& g, bool debug = false) = 0;

      std::string getCostName() {return cost_name_;}
    protected:

      ros::NodeHandle nh_;
      ros::NodeHandle nhp_;

      boost::shared_ptr<motion_planner> planner_;

      bool verbose_;
      std::string cost_name_;

      int j_ndof_;
      bool orientation_;
      bool full_body_;

    };
  };
};

#endif
