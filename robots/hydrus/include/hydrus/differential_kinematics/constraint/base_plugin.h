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

#ifndef DIFFERENTIA_KINEMATICS_CONSTRAINT_PLUGIN_H
#define DIFFERENTIA_KINEMATICS_CONSTRAINT_PLUGIN_H

#include <ros/ros.h>

/* Linear Math */
#include <Eigen/Dense>

namespace differential_kinematics
{
  namespace constraint
  {
    template <class motion_planner>
    class Base
    {
    public:
      Base() {}
      ~Base(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<motion_planner> planner, std::string constraint_name,
                              bool orientation, bool full_body)
      {
        nh_ = ros::NodeHandle(nh, constraint_name);
        nhp_ = ros::NodeHandle(nhp, constraint_name);

        planner_ = planner;
        nhp_.param("verbose", verbose_, false);

        constraint_name_ = constraint_name;
        orientation_ = orientation;
        full_body_ = full_body;
        j_ndof_ = planner_->getRobotModelPtr()->getActuatorJointMap().size();
      }
      virtual bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false) = 0;
      virtual bool directConstraint(){return false;}

      int getNc() {return nc_;}
      std::string getConstraintName() {return constraint_name_;}

      virtual void result() = 0;
    protected:

      ros::NodeHandle nh_;
      ros::NodeHandle nhp_;

      boost::shared_ptr<motion_planner> planner_;

      bool verbose_;
      std::string constraint_name_;
      int nc_;

      int j_ndof_;
      bool orientation_;
      bool full_body_;

    };
  };
};

#endif
