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

/* specialized for DRAGON model */

#include <hydrus/differential_kinematics/constraint/base_plugin.h>

/* dragon model */
#include <dragon/transform_control.h>


namespace differential_kinematics
{
  namespace constraint
  {
    template <class motion_planner>
    class Overlap :public Base<motion_planner>
    {
    public:
      Overlap(): result_overlap_min_(1e6)
      {}
      ~Overlap(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<motion_planner> planner, std::string constraint_name,
                              bool orientation, bool full_body)
      {
        Base<motion_planner>::initialize(nh, nhp, planner, constraint_name, orientation, full_body);
        robot_model_ptr_ = boost::dynamic_pointer_cast<DragonTransformController>(planner->getRobotModelPtr());
        rotor_num_ = Base<motion_planner>::planner_->getRobotModelPtr()->getRotorNum();

        Base<motion_planner>::nc_ = 1; // temporary, using one dimension

        Base<motion_planner>::nhp_.param ("overlap_change_vel_thre", overlap_change_vel_thre_, 0.05);
        if(Base<motion_planner>::verbose_) std::cout << "overlap_change_vel_thre: " << std::setprecision(3) << overlap_change_vel_thre_ << std::endl;
        Base<motion_planner>::nhp_.param ("overlap_constraint_range", overlap_constraint_range_, 0.05); // [m]
        if(Base<motion_planner>::verbose_) std::cout << "overlap_constraint_range: " << std::setprecision(3) << overlap_constraint_range_ << std::endl;
        Base<motion_planner>::nhp_.param ("overlap_forbidden_range", overlap_forbidden_range_, 0.01); // [m]
        if(Base<motion_planner>::verbose_) std::cout << "overlap_forbidden_range: " << std::setprecision(3) << overlap_forbidden_range_ << std::endl;

      }

      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
        //debug = true;
        A = Eigen::MatrixXd::Zero(Base<motion_planner>::nc_, Base<motion_planner>::j_ndof_ + 6);
        lb = Eigen::VectorXd::Constant(Base<motion_planner>::nc_, -overlap_change_vel_thre_);
        ub = Eigen::VectorXd::Constant(Base<motion_planner>::nc_, 1e6);

        std::vector<Eigen::Vector3d> edfs_origin_from_cog(rotor_num_ * 2);
        robot_model_ptr_->getEdfsFromCog(edfs_origin_from_cog);

        /* Note: we only choose the minimum diff, which is not feasible for N-multilink */
        double min_dist = 1e6;
        int overlap_link1, overlap_link2;
        for(int i = 0; i < rotor_num_ * 2; i++)
          {
            for(int j =  i + 1; j < rotor_num_ * 2; j++)
              {
                Eigen::Vector3d diff3d = edfs_origin_from_cog[i] - edfs_origin_from_cog[j]; //dual
                double projected_dist2d = sqrt(diff3d(0) * diff3d(0) + diff3d(1) * diff3d(1));
                /* Note: approximated, the true one should be (edf_radius_ / cos(tilt) + diff(2) * tan(tilt) */
                double dist2d_thre = 2 * robot_model_ptr_->edf_radius_  + fabs(diff3d(2)) * tan(robot_model_ptr_->edf_max_tilt_);

                /* special for dual rotor */
                if(i / 2 == j / 2) continue;

                if(dist2d_thre > projected_dist2d)
                  {
                    if(Base<motion_planner>::verbose_)ROS_ERROR("constraint: %s, overlap!: %d and %d, projectd_dist: %f, thre: %f", Base<motion_planner>::constraint_name_.c_str(), i + 1, j + 1, projected_dist2d, dist2d_thre);
                    return false;
                  }

                if(projected_dist2d - dist2d_thre < min_dist)
                  {
                    min_dist = projected_dist2d - dist2d_thre;
                    overlap_link1 = i;
                    overlap_link2 = j;
                  }

              }
          }

        if(debug)ROS_INFO("constraint: %s, overlap min dist %f between %d and %d", Base<motion_planner>::constraint_name_.c_str(), min_dist, overlap_link1 + 1, overlap_link2 + 1);

        /* update matrix A */
        double delta_angle = 0.001; // [rad]
        for(int j = 0; j < Base<motion_planner>::j_ndof_; j++)
          {
            sensor_msgs::JointState actuator_state = Base<motion_planner>::planner_->getTargetActuatorVector();
            actuator_state.position.at(robot_model_ptr_->getActuatorJointMap().at(j)) += delta_angle;
            robot_model_ptr_->forwardKinematics(actuator_state);
            robot_model_ptr_->getEdfsFromCog(edfs_origin_from_cog);

            Eigen::Vector3d diff3d = edfs_origin_from_cog[overlap_link1] - edfs_origin_from_cog[overlap_link2]; //dual
            double projected_dist2d = sqrt(diff3d(0) * diff3d(0) + diff3d(1) * diff3d(1));
            double dist2d_thre = 2 * robot_model_ptr_->edf_radius_  + fabs(diff3d(2)) * tan(robot_model_ptr_->edf_max_tilt_);

            A(0, 6 + j) = (projected_dist2d - dist2d_thre - min_dist) / delta_angle;
          }

        sensor_msgs::JointState actuator_state = Base<motion_planner>::planner_->getTargetActuatorVector();
        KDL::Rotation root_att;
        tf::quaternionTFToKDL(Base<motion_planner>::planner_->getTargetRootPose().getRotation(), root_att);
        /* roll */
        robot_model_ptr_->setCogDesireOrientation(root_att * KDL::Rotation::RPY(delta_angle, 0, 0));
        robot_model_ptr_->forwardKinematics(actuator_state);
        robot_model_ptr_->getEdfsFromCog(edfs_origin_from_cog);
        Eigen::Vector3d diff3d = edfs_origin_from_cog[overlap_link1] - edfs_origin_from_cog[overlap_link2]; //dual
        double projected_dist2d = sqrt(diff3d(0) * diff3d(0) + diff3d(1) * diff3d(1));
        double dist2d_thre = 2 * robot_model_ptr_->edf_radius_  + fabs(diff3d(2)) * tan(robot_model_ptr_->edf_max_tilt_);
        A(0, 3) = (projected_dist2d - dist2d_thre - min_dist) / delta_angle;

        /* pitch */
        robot_model_ptr_->setCogDesireOrientation(root_att * KDL::Rotation::RPY(0, delta_angle, 0));
        robot_model_ptr_->forwardKinematics(actuator_state);
        robot_model_ptr_->getEdfsFromCog(edfs_origin_from_cog);
        diff3d = edfs_origin_from_cog[overlap_link1] - edfs_origin_from_cog[overlap_link2]; //dual
        projected_dist2d = sqrt(diff3d(0) * diff3d(0) + diff3d(1) * diff3d(1));
        dist2d_thre = 2 * robot_model_ptr_->edf_radius_  + fabs(diff3d(2)) * tan(robot_model_ptr_->edf_max_tilt_);
        A(0, 4) = (projected_dist2d - dist2d_thre - min_dist) / delta_angle;

        /* yaw */
        robot_model_ptr_->setCogDesireOrientation(root_att * KDL::Rotation::RPY(0, 0, delta_angle));
        robot_model_ptr_->forwardKinematics(actuator_state);
        robot_model_ptr_->getEdfsFromCog(edfs_origin_from_cog);
        diff3d = edfs_origin_from_cog[overlap_link1] - edfs_origin_from_cog[overlap_link2]; //dual
        projected_dist2d = sqrt(diff3d(0) * diff3d(0) + diff3d(1) * diff3d(1));
        dist2d_thre = 2 * robot_model_ptr_->edf_radius_  + fabs(diff3d(2)) * tan(robot_model_ptr_->edf_max_tilt_);
        A(0, 5) = (projected_dist2d - dist2d_thre - min_dist) / delta_angle;

        /* set lb */
        if(min_dist < overlap_constraint_range_)
            lb(0) *= ((min_dist - overlap_forbidden_range_) / (overlap_constraint_range_ - overlap_forbidden_range_));

        //lb(0) = -1e6;
        if(debug)
          {
            std::cout << "constraint (" << Base<motion_planner>::constraint_name_ << "): matrix A: \n" << A <<  "\n lb: \n" << lb.transpose() << "\n, ub: \n" << ub.transpose() << std::endl;
          }

        /* update reuslt */
        if(result_overlap_min_ > min_dist)
          {
            result_overlap_min_ = min_dist;
            result_overlap_link1_ = overlap_link1;
            result_overlap_link2_ = overlap_link2;
          }

        return true;
      }

      void result()
      {
        std::cout << Base<motion_planner>::constraint_name_ << "\n"
                  << "result overlap min: " << result_overlap_min_
                  << " at rotor" << result_overlap_link1_
                  << " at rotor" << result_overlap_link2_
                  << std::endl;
      }

      bool directConstraint(){return false;}

    protected:
      double rotor_num_;

      double overlap_change_vel_thre_;
      double overlap_constraint_range_;
      double overlap_forbidden_range_;

      double result_overlap_min_;
      int result_overlap_link1_;
      int result_overlap_link2_;

      boost::shared_ptr<DragonTransformController> robot_model_ptr_;
    };
  };
};

#include <pluginlib/class_list_macros.h>
#include <hydrus/differential_kinematics/planner_core.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::Overlap<differential_kinematics::Planner>, differential_kinematics::constraint::Base<differential_kinematics::Planner>);

