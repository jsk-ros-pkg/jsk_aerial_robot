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

#include <tf_conversions/tf_kdl.h>
#include <sensor_msgs/JointState.h>

namespace differential_kinematics
{
  namespace constraint
  {
    template <class motion_planner>
    class Stability :public Base<motion_planner>
    {
    public:
      Stability():
        result_f_min_(1e6), result_f_max_(0),
        result_p_det_min_(1e6), result_stability_margin_min_(1e6)
      {}
      ~Stability(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<motion_planner> planner, std::string constraint_name,
                              bool orientation, bool full_body)
      {
        Base<motion_planner>::initialize(nh, nhp, planner, constraint_name, orientation, full_body);
        rotor_num_ = Base<motion_planner>::planner_->getRobotModelPtr()->getRotorNum();
        Base<motion_planner>::nc_ = 2 + rotor_num_;

        stability_margin_thre_ = Base<motion_planner>::planner_->getRobotModelPtr()->stability_margin_thre_;
        p_det_thre_ = Base<motion_planner>::planner_->getRobotModelPtr()->p_det_thre_;
        f_min_ = Base<motion_planner>::planner_->getRobotModelPtr()->f_min_;
        f_max_ = Base<motion_planner>::planner_->getRobotModelPtr()->f_max_;

        Base<motion_planner>::nhp_.param ("stability_margin_decrease_vel_thre", stability_margin_decrease_vel_thre_, -0.01);
        if(Base<motion_planner>::verbose_) std::cout << "stability_margin_decrease_vel_thre: " << std::setprecision(3) << stability_margin_decrease_vel_thre_ << std::endl;
        Base<motion_planner>::nhp_.param ("stability_margin_constraint_range", stability_margin_constraint_range_, 0.02);
        if(Base<motion_planner>::verbose_) std::cout << "stability_margin_constraint_range: " << std::setprecision(3) << stability_margin_constraint_range_ << std::endl;
        Base<motion_planner>::nhp_.param ("stability_margin_forbidden_range", stability_margin_forbidden_range_, 0.005);
        if(Base<motion_planner>::verbose_) std::cout << "stability_margin_forbidden_range: " << std::setprecision(3) << stability_margin_forbidden_range_ << std::endl;

      }

      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
        A = Eigen::MatrixXd::Zero(Base<motion_planner>::nc_, Base<motion_planner>::j_ndof_ + 6);
        lb = Eigen::VectorXd::Constant(Base<motion_planner>::nc_, 1);
        ub = Eigen::VectorXd::Constant(Base<motion_planner>::nc_, 1);

        /* 1. stability margin */
        double nominal_stability_margin = Base<motion_planner>::planner_->getRobotModelPtr()->getStabilityMargin();
        /* fill lb */
        lb(0) = stability_margin_decrease_vel_thre_;
        if(nominal_stability_margin - stability_margin_thre_   < stability_margin_constraint_range_)
          lb(0) *=  ((nominal_stability_margin - stability_margin_thre_ - stability_margin_forbidden_range_) / (stability_margin_constraint_range_ - stability_margin_forbidden_range_));
        //ub(0) = 1e6;

        /* 2. singularity */
        double nominal_p_det = Base<motion_planner>::planner_->getRobotModelPtr()->getPdeterminant();
        /* fill ub */
        lb(1) =  p_det_thre_ - nominal_p_det;
        //ub(1) = 1e6;

        /*************************************************************************************
        3. optimal hovering thrust constraint (including singularity check)
           f_min < F + delta_f < f_max =>   delta_f =  (f(q + d_q) - f(q)) / d_q * delta_q
           TODO: use virutal state (beta) to maximize the flight stability
        ****************************************************************************************/

        Eigen::VectorXd nominal_hovering_f =  Base<motion_planner>::planner_->getRobotModelPtr()->getOptimalHoveringThrust();
        /* fill the lb/ub */
        lb.segment(2, rotor_num_) = Eigen::VectorXd::Constant(rotor_num_, f_min_) - nominal_hovering_f;
        ub.segment(2, rotor_num_) = Eigen::VectorXd::Constant(rotor_num_, f_max_) - nominal_hovering_f;

        if(debug)
          {
            std::cout << "constraint name: " << Base<motion_planner>::constraint_name_ << ", nominal stability margin : \n" << nominal_stability_margin << std::endl;
            std::cout << "constraint name: " << Base<motion_planner>::constraint_name_ << ", nominal p det : \n" << nominal_p_det << std::endl;
            std::cout << "constraint name: " << Base<motion_planner>::constraint_name_ << ", nominal f: \n" << nominal_hovering_f.transpose() << std::endl;
          }

        /* update the result */
        if(result_p_det_min_ > nominal_p_det) result_p_det_min_ = nominal_p_det;
        if(result_stability_margin_min_ > nominal_stability_margin) result_stability_margin_min_ = nominal_stability_margin;
        if(result_f_min_ > nominal_hovering_f.minCoeff())
          result_f_min_ = nominal_hovering_f.minCoeff(&result_f_min_rotor_);
        if(result_f_max_ < nominal_hovering_f.maxCoeff())
          result_f_max_ = nominal_hovering_f.maxCoeff(&result_f_max_rotor_);

        /* joint */
        double delta_angle = 0.001; // [rad]
        for(int index = 0; index < Base<motion_planner>::j_ndof_; index++)
          {
            sensor_msgs::JointState actuator_state = Base<motion_planner>::planner_->getTargetActuatorVector();
            actuator_state.position.at(Base<motion_planner>::planner_->getRobotModelPtr()->getActuatorJointMap().at(index)) += delta_angle;
            Base<motion_planner>::planner_->getRobotModelPtr()->forwardKinematics(actuator_state);
            Base<motion_planner>::planner_->getRobotModelPtr()->stabilityMarginCheck();
            Base<motion_planner>::planner_->getRobotModelPtr()->modelling();

            /* stability margin */
            A(0, 6 + index) = (Base<motion_planner>::planner_->getRobotModelPtr()->getStabilityMargin() - nominal_stability_margin) /delta_angle;
            /* singularity */
            A(1, 6 + index) = (Base<motion_planner>::planner_->getRobotModelPtr()->getPdeterminant() - nominal_p_det) /delta_angle;
            /* hovering thrust */
            A.block(2, 6 + index, rotor_num_, 1) = (Base<motion_planner>::planner_->getRobotModelPtr()->getOptimalHoveringThrust() - nominal_hovering_f) / delta_angle;
          }

        if(debug)
          std::cout << "constraint (" << Base<motion_planner>::constraint_name_.c_str()  << "): matrix A: \n" << A << std::endl;
#if 0

        /* debug */
        delta_angle = -0.001; // [rad]
        for(int index = 0; index < Base<motion_planner>::j_ndof_; index++)
          {
            sensor_msgs::JointState actuator_state = Base<motion_planner>::planner_->getTargetActuatorVector();
            actuatorstate.position.at(Base<motion_planner>::planner_->getRobotModelPtr()->getActuatorJointMap().at(index)) += delta_angle;
            Base<motion_planner>::planner_->getRobotModelPtr()->forwardKinematics(actuatorstate);
            Base<motion_planner>::planner_->getRobotModelPtr()->stabilityMarginCheck();
            Base<motion_planner>::planner_->getRobotModelPtr()->modelling();

            /* stability margin */
            A(0, index) = (Base<motion_planner>::planner_->getRobotModelPtr()->getStabilityMargin() - stability_margin) /delta_angle;
            /* singularity */
            A(1, index) = (Base<motion_planner>::planner_->getRobotModelPtr()->getPdeterminant() - p_det) /delta_angle;
            /* hovering thrust */
            A.block(2, index, rotor_num_, 1) = (Base<motion_planner>::planner_->getRobotModelPtr()->getOptimalHoveringThrust() - hovering_f) / delta_angle;
          }

        if(debug)
          std::cout << "constraint (" << Base<motion_planner>::constraint_name_.c_str()  << "): matrix A: \n" << A << std::endl;
        delta_angle = 0.001; // revert

#endif

        /* root */
        sensor_msgs::JointState actuator_state = Base<motion_planner>::planner_->getTargetActuatorVector();
        KDL::Rotation root_att;
        tf::quaternionTFToKDL(Base<motion_planner>::planner_->getTargetRootPose().getRotation(), root_att);
        /* roll */
        Base<motion_planner>::planner_->getRobotModelPtr()->setCogDesireOrientation(root_att * KDL::Rotation::RPY(delta_angle, 0, 0));
        Base<motion_planner>::planner_->getRobotModelPtr()->forwardKinematics(actuator_state);
        Base<motion_planner>::planner_->getRobotModelPtr()->stabilityMarginCheck();
        Base<motion_planner>::planner_->getRobotModelPtr()->modelling();

        /* stability margin */
        A(0, 0) = (Base<motion_planner>::planner_->getRobotModelPtr()->getStabilityMargin() - nominal_stability_margin) / delta_angle;
        /* singularity */
        A(1, 0) = (Base<motion_planner>::planner_->getRobotModelPtr()->getPdeterminant() - nominal_p_det) / delta_angle;
        /* hovering thrust */
        A.block(2, 0, rotor_num_, 1) = (Base<motion_planner>::planner_->getRobotModelPtr()->getOptimalHoveringThrust() - nominal_hovering_f) / delta_angle;

        /*  pitch */
        Base<motion_planner>::planner_->getRobotModelPtr()->setCogDesireOrientation(root_att * KDL::Rotation::RPY(0, delta_angle, 0));
        Base<motion_planner>::planner_->getRobotModelPtr()->forwardKinematics(actuator_state);
        Base<motion_planner>::planner_->getRobotModelPtr()->stabilityMarginCheck();
        Base<motion_planner>::planner_->getRobotModelPtr()->modelling();

        /* stability margin */
        A(0, 1) = (Base<motion_planner>::planner_->getRobotModelPtr()->getStabilityMargin() - nominal_stability_margin) / delta_angle;
        /* singularity */
        A(1, 1) = (Base<motion_planner>::planner_->getRobotModelPtr()->getPdeterminant() - nominal_p_det) / delta_angle;
        /* hovering thrust */
        A.block(2, 1, rotor_num_, 1) = (Base<motion_planner>::planner_->getRobotModelPtr()->getOptimalHoveringThrust() - nominal_hovering_f) / delta_angle;

        if(debug)
          std::cout << "constraint (" << Base<motion_planner>::constraint_name_.c_str()  << "): matrix A: \n" << A << std::endl;

#if 0
        delta_angle = -0.001;
        /* roll */
        Base<motion_planner>::planner_->getRobotModelPtr()->setCogDesireOrientation(root_att * KDL::Rotation::RPY(delta_angle, 0, 0));
        Base<motion_planner>::planner_->getRobotModelPtr()->forwardKinematics(actuator_state);
        Base<motion_planner>::planner_->getRobotModelPtr()->stabilityMarginCheck();
        Base<motion_planner>::planner_->getRobotModelPtr()->modelling();

        /* stability margin */
        A(0, 0) = (Base<motion_planner>::planner_->getRobotModelPtr()->getStabilityMargin() - nominal_stability_margin) / delta_angle;
        /* singularity */
        A(1, 0) = (Base<motion_planner>::planner_->getRobotModelPtr()->getPdeterminant() - nominal_p_det) / delta_angle;
        /* hovering thrust */
        A.block(2, 0, rotor_num_, 1) = (Base<motion_planner>::planner_->getRobotModelPtr()->getOptimalHoveringThrust() - nominal_hovering_f) / delta_angle;

        /*  pitch */
        Base<motion_planner>::planner_->getRobotModelPtr()->setCogDesireOrientation(root_att * KDL::Rotation::RPY(0, delta_angle, 0));
        Base<motion_planner>::planner_->getRobotModelPtr()->forwardKinematics(actuator_state);
        Base<motion_planner>::planner_->getRobotModelPtr()->stabilityMarginCheck();
        Base<motion_planner>::planner_->getRobotModelPtr()->modelling();

        /* stability margin */
        A(0, 1) = (Base<motion_planner>::planner_->getRobotModelPtr()->getStabilityMargin() - nominal_stability_margin) / delta_angle;
        /* singularity */
        A(1, 1) = (Base<motion_planner>::planner_->getRobotModelPtr()->getPdeterminant() - nominal_p_det) / delta_angle;
        /* hovering thrust */
        A.block(2, 1, rotor_num_, 1) = (Base<motion_planner>::planner_->getRobotModelPtr()->getOptimalHoveringThrust() - nominal_hovering_f) / delta_angle;

        if(debug)
          std::cout << "constraint (" << Base<motion_planner>::constraint_name_.c_str()  << "): matrix A: \n" << A << std::endl;

#endif
        if(debug)
          {
            std::cout << "constraint name: " << Base<motion_planner>::constraint_name_ << ", lb: \n" << lb << std::endl;
            std::cout << "constraint name: " << Base<motion_planner>::constraint_name_ << ", ub: \n" << ub.transpose() << std::endl;
          }

        return true;
      }

      void result()
      {
        std::cout << Base<motion_planner>::constraint_name_
                  << "min p determinant: " << result_p_det_min_ << "\n"
                  << "min stability margin: " << result_stability_margin_min_ << "\n"
                  << "min f: " << result_f_min_ << " at rotor" << result_f_min_rotor_ << "\n"
                  << "max f: " << result_f_max_ << " at rotor" << result_f_max_rotor_ << std::endl;
      }

      bool directConstraint(){return false;}

    protected:
      double rotor_num_;
      double stability_margin_thre_;
      double p_det_thre_;
      double f_min_, f_max_;

      double stability_margin_decrease_vel_thre_;
      double stability_margin_constraint_range_;
      double stability_margin_forbidden_range_;


      double result_f_min_;
      double result_f_max_;
      Eigen::MatrixXd::Index result_f_min_rotor_;
      Eigen::MatrixXd::Index result_f_max_rotor_;

      double result_p_det_min_;
      double result_stability_margin_min_;
    };
  };
};

#include <pluginlib/class_list_macros.h>
#include <hydrus/differential_kinematics/planner_core.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::Stability<differential_kinematics::Planner>, differential_kinematics::constraint::Base<differential_kinematics::Planner>);

