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


/* robot model */
#include <urdf/model.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <sensor_msgs/JointState.h>

#include <tf_conversions/tf_eigen.h>
#include <tf_conversions/tf_kdl.h>
#include <sensor_msgs/JointState.h>

namespace differential_kinematics
{
  namespace constraint
  {
    template <class motion_planner>
    class LinkAttitude :public Base<motion_planner>
    {
    public:
      LinkAttitude(): result_roll_max_(0), result_pitch_max_(0), joint_map_()
      {}
      ~LinkAttitude(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<motion_planner> planner, std::string constraint_name,
                              bool orientation, bool full_body)
      {
        Base<motion_planner>::initialize(nh, nhp, planner, constraint_name, orientation, full_body);
        rotor_num_ = Base<motion_planner>::planner_->getRobotModelPtr()->getRotorNum();

        Base<motion_planner>::nc_ = 2 * rotor_num_; //roll + pitch

        for(int i = 0; i < Base<motion_planner>::planner_->getRobotModelPtr()->getModelTree().getNrOfJoints(); i++)
          {
            for(auto it : Base<motion_planner>::planner_->getRobotModelPtr()->getActuatorMap())
              {
                if(it.second == i && it.first.find("joint")  != std::string::npos)
                  {
                    std::cout << Base<motion_planner>::constraint_name_ <<  ", " << it.first << std::endl;
                    joint_map_.push_back(it.second);
                  }
              }
          }

        Base<motion_planner>::nhp_.param ("roll_angle_thre", roll_angle_thre_, 1.);
        if(Base<motion_planner>::verbose_) std::cout << "roll_angle_thre: " << std::setprecision(3) << roll_angle_thre_ << std::endl;
        Base<motion_planner>::nhp_.param ("pitch_angle_thre", pitch_angle_thre_, 1.0);
        if(Base<motion_planner>::verbose_) std::cout << "pitch_angle_thre: " << std::setprecision(3) << pitch_angle_thre_ << std::endl;

        Base<motion_planner>::nhp_.param ("attitude_change_vel_thre", attitude_change_vel_thre_, 0.1);
        if(Base<motion_planner>::verbose_) std::cout << "attitude_change_vel_thre: " << std::setprecision(3) << attitude_change_vel_thre_ << std::endl;
        Base<motion_planner>::nhp_.param ("attitude_constraint_range", attitude_constraint_range_, 0.17);
        if(Base<motion_planner>::verbose_) std::cout << "attitude_constraint_range: " << std::setprecision(3) << attitude_constraint_range_ << std::endl;
        Base<motion_planner>::nhp_.param ("attitude_forbidden_range", attitude_forbidden_range_, 0.02);
        if(Base<motion_planner>::verbose_) std::cout << "attitude_forbidden_range: " << std::setprecision(3) << attitude_forbidden_range_ << std::endl;

      }

      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
        //debug = true;
        A = Eigen::MatrixXd::Zero(Base<motion_planner>::nc_, Base<motion_planner>::j_ndof_ + 6);
        lb = Eigen::VectorXd::Constant(Base<motion_planner>::nc_, -attitude_change_vel_thre_);
        ub = Eigen::VectorXd::Constant(Base<motion_planner>::nc_, attitude_change_vel_thre_);

        /* 1. calculate the matrix converting from euler to angular velocity */
        Eigen::Matrix3d r_root;
        tf::matrixTFToEigen(Base<motion_planner>::planner_->getTargetRootPose().getBasis(), r_root);
        double r, p, y;
        Base<motion_planner>::planner_->getTargetRootPose().getBasis().getRPY(r, p, y);

        /* 2. get jacobian w.r.t root link */

        /* 2.1 get joint angles */
        KDL::JntArray joint_positions(Base<motion_planner>::planner_->getRobotModelPtr()->getModelTree().getNrOfJoints());
        sensor_msgs::JointState actuator_state = Base<motion_planner>::planner_->getTargetActuatorVector();
        auto actuator_map = Base<motion_planner>::planner_->getRobotModelPtr()->getActuatorMap();
        for(size_t i = 0; i < actuator_state.position.size(); i++)
          {
            auto itr = actuator_map.find(actuator_state.name.at(i));

            if(itr != actuator_map.end())  joint_positions(actuator_map.find(actuator_state.name.at(i))->second) = actuator_state.position.at(i);
          }

        /* 2.2 get full-body jacobian */
        Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, Base<motion_planner>::j_ndof_ + 6);

        /* calculate the jacobian */
        KDL::TreeJntToJacSolver jac_solver(Base<motion_planner>::planner_->getRobotModelPtr()->getModelTree());
        KDL::Jacobian jac(Base<motion_planner>::planner_->getRobotModelPtr()->getModelTree().getNrOfJoints());
        std::stringstream ss_end_link; ss_end_link << rotor_num_;
        if(jac_solver.JntToJac(joint_positions, jac, std::string("link") + ss_end_link.str() ) == KDL::SolverI::E_NOERROR)
          {
            if(debug) std::cout << "raw jacobian: \n" << jac.data << std::endl;
            /* joint reassignment  */
            for(size_t i = 0; i < joint_map_.size(); i++)
              jacobian.block(0, 6 + i , 3, 1) = jac.data.block(3, joint_map_.at(i), 3, 1);

            /* full body */
            if(Base<motion_planner>::full_body_)
              jacobian.block(0, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

            if(debug) std::cout << "jacobian: \n" << jacobian << std::endl;
          }
        else
          {
            ROS_WARN("constraint (%s) can not calculate the jacobian", Base<motion_planner>::constraint_name_.c_str());
            return false;
          }


        /* set A and lb/ub */
        KDL::Rotation root_att;
        tf::quaternionTFToKDL(Base<motion_planner>::planner_->getTargetRootPose().getRotation(), root_att);

        KDL::TreeFkSolverPos_recursive fk_solver(Base<motion_planner>::planner_->getRobotModelPtr()->getModelTree());
        int joint_offset = Base<motion_planner>::j_ndof_ / (rotor_num_ - 1 );
        std::vector<double> roll_vec; // for debug
        std::vector<double> pitch_vec; // for debug
        std::vector<KDL::Frame> f_link_vec; // for debug
        for(int index = 0; index < rotor_num_; index++)
          {
            /* lb/ub */
            std::stringstream ss; ss << index + 1;
            KDL::Frame f_link;
            int status = fk_solver.JntToCart(joint_positions, f_link, std::string("link") + ss.str());
            f_link_vec.push_back(f_link);
            (root_att * f_link.M).GetRPY(r, p, y);
            if(debug) std::cout << std::string("link") + ss.str() << ": roll: " << r << ", pitch: " << p << std::endl;
            Eigen::Matrix3d r_convert;
            r_convert << 0, - sin(y), cos(y) * cos(p), 0, cos(y), sin(y) * cos(p), 1, 0, -sin(p);
            if(debug) std::cout << "constraint (" << Base<motion_planner>::constraint_name_.c_str()  << "): r_convert inverse: \n" << r_convert.inverse() << std::endl;
            Eigen::MatrixXd jacobian_euler = r_convert.inverse() * r_root * jacobian;
            //if(debug) std::cout << "constraint (" << Base<motion_planner>::constraint_name_.c_str()  << "): jacobian_euler: \n" << jacobian_euler << std::endl;

            /* yaw -> pitch -> roll => pitch -> roll */
            /* pitch -> roll */
            A.block(index * 2, 0, 2, 6 + index * joint_offset) = jacobian_euler.block(1, 0, 2, 6 + index * joint_offset);

            /* pitch */
            if(p - (-pitch_angle_thre_) < attitude_constraint_range_)
              lb(index * 2 ) *= ((p - (-pitch_angle_thre_) - attitude_forbidden_range_ ) / (attitude_constraint_range_ - attitude_forbidden_range_));
            if(pitch_angle_thre_ - p < attitude_constraint_range_)
              ub(index * 2 ) *= ((pitch_angle_thre_ - p - attitude_forbidden_range_ ) / (attitude_constraint_range_ - attitude_forbidden_range_));
            /* roll */
            if(r - (-roll_angle_thre_) < attitude_constraint_range_)
              lb(index * 2 + 1) *= ((r - (-roll_angle_thre_) - attitude_forbidden_range_ ) / (attitude_constraint_range_ - attitude_forbidden_range_));
            if(roll_angle_thre_ - r < attitude_constraint_range_)
              ub(index * 2 + 1) *= ((roll_angle_thre_ - r - attitude_forbidden_range_ ) / (attitude_constraint_range_ - attitude_forbidden_range_));

            /* update result */
            if(result_roll_max_ < fabs(r))
              {
                result_roll_max_ = fabs(r);
                result_roll_max_link_ = index + 1;
              }
            if(result_pitch_max_ < fabs(p))
              {
                result_pitch_max_ = fabs(p);
                result_pitch_max_link_ = index + 1;
              }

            /* for debug */
            roll_vec.push_back(r);
            pitch_vec.push_back(p);
          }

#if 0 //numerial calculation
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(Base<motion_planner>::nc_, Base<motion_planner>::j_ndof_ + 6);
        double delta_angle = 0.001; // [rad]
        for(int j = 0; j < Base<motion_planner>::j_ndof_; j++)
          {
            sensor_msgs::JointState actuator_state_temp = Base<motion_planner>::planner_->getTargetActuatorVector();
            actuator_state_temp.position.at(Base<motion_planner>::planner_->getRobotModelPtr()->getActuatorJointMap().at(j)) += delta_angle;
            auto actuator_map = Base<motion_planner>::planner_->getRobotModelPtr()->getActuatorMap();
            for(size_t i = 0; i < actuator_state.position.size(); i++)
              {
                auto itr = actuator_map.find(actuator_state_temp.name.at(i));

                if(itr != actuator_map.end())  joint_positions(actuator_map.find(actuator_state_temp.name.at(i))->second) = actuator_state_temp.position.at(i);
              }

            for(int index = 0; index < rotor_num_; index++)
              {
                std::stringstream ss; ss << index + 1;
                KDL::Frame f_link;
                fk_solver.JntToCart(joint_positions, f_link, std::string("link") + ss.str());
                (root_att * f_link.M).GetRPY(r, p, y);
                B(index * 2, 6 + j) = (p - pitch_vec.at(index)) / delta_angle;
                B(index * 2 + 1, 6 + j) = (r - roll_vec.at(index)) / delta_angle;
              }
          }

        for(int index = 0; index < rotor_num_; index++)
          {
            /* roll */
            (root_att * KDL::Rotation::RPY(delta_angle, 0, 0) * f_link_vec.at(index).M).GetRPY(r,p,y);
            B(index * 2, 3) = (p - pitch_vec.at(index)) / delta_angle;
            B(index * 2 + 1, 3) = (r - roll_vec.at(index)) / delta_angle;
            /* pitch */
            (root_att * KDL::Rotation::RPY(0, delta_angle, 0) * f_link_vec.at(index).M).GetRPY(r,p,y);
            B(index * 2, 4) = (p - pitch_vec.at(index)) / delta_angle;
            B(index * 2 + 1, 4) = (r - roll_vec.at(index)) / delta_angle;
            /* yaw */
            (root_att * KDL::Rotation::RPY(0, 0, delta_angle) * f_link_vec.at(index).M).GetRPY(r,p,y);
            B(index * 2, 5) = (p - pitch_vec.at(index)) / delta_angle;
            B(index * 2 + 1, 5) = (r - roll_vec.at(index)) / delta_angle;
          }
        std::cout << "constraint (" << Base<motion_planner>::constraint_name_.c_str()  << "): matrix B: \n" << B << std::endl;

#endif

        if(debug)
          {
            std::cout << "constraint (" << Base<motion_planner>::constraint_name_.c_str()  << "): matrix A: \n" << A << std::endl;
            std::cout << "constraint name: " << Base<motion_planner>::constraint_name_ << ", lb: \n" << lb.transpose() << std::endl;
            std::cout << "constraint name: " << Base<motion_planner>::constraint_name_ << ", ub: \n" << ub.transpose() << std::endl;
          }

        return true;
      }

      void result()
      {
        std::cout << Base<motion_planner>::constraint_name_ << "\n"
                  << "max roll angle: " << result_roll_max_ << " at link" << result_roll_max_link_ << "\n"
                  << "max pitch angle: " << result_pitch_max_ << " at link" << result_pitch_max_link_
                  << std::endl;
      }

      bool directConstraint(){return false;}

    protected:
      double rotor_num_;

      double roll_angle_thre_;
      double pitch_angle_thre_;
      double attitude_change_vel_thre_;
      double attitude_constraint_range_;
      double attitude_forbidden_range_;

      double result_roll_max_;
      double result_pitch_max_;
      int result_roll_max_link_;
      int result_pitch_max_link_;

      /* not good varible */
      std::vector<int> joint_map_;
    };
  };
};

#include <pluginlib/class_list_macros.h>
#include <hydrus/differential_kinematics/planner_core.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::LinkAttitude<differential_kinematics::Planner>, differential_kinematics::constraint::Base<differential_kinematics::Planner>);

