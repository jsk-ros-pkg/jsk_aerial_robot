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

#include <hydrus/differential_kinematics/planner_core.h>

namespace differential_kinematics
{
  using RobotModel = TransformController;
  using CostContainer = std::vector<boost::shared_ptr<cost::Base<Planner> > >;
  using ConstraintContainer = std::vector<boost::shared_ptr<constraint::Base<Planner> > >;

  Planner::Planner(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<RobotModel> robot_model_ptr): nh_(nh), nhp_(nhp), robot_model_ptr_(robot_model_ptr), multilink_type_(MULTILINK_TYPE_SE2), motion_func_vector_(), solved_(false)
  {
    for(auto itr = robot_model_ptr_->getRobotModel().joints_.begin(); itr != robot_model_ptr_->getRobotModel().joints_.end(); itr++)
      {
        if(itr->second->type != urdf::Joint::FIXED && itr->second->axis.z == 0)
          multilink_type_ = MULTILINK_TYPE_SE3;
      }

    nhp_.param ("differential_kinematics_count", differential_kinematics_count_, 100);

    /* publisher for joint(actuator) steate */
    std::string actuator_state_pub_name;
    nhp_.param("actuator_state_pub_name", actuator_state_pub_name, std::string("joint_state"));
    actuator_state_pub_ = nh_.advertise<sensor_msgs::JointState>(actuator_state_pub_name, 1);

    /* motion timer */
    double rate;
    nhp_.param("motion_func_rate", rate, 10.0);
    motion_timer_ = nhp_.createTimer(ros::Duration(1.0 / rate), &Planner::motionFunc, this);
  }

  bool Planner::solver(CostContainer cost_container, ConstraintContainer constraint_container, bool debug)
  {
    target_actuator_vector_sequence_.resize(0);
    target_root_pose_sequence_.resize(0);
    sequence_ = 0;
    double start_time = ros::Time::now().toSec();

    int total_nc = 0;
    for(auto itr = constraint_container.begin(); itr != constraint_container.end(); itr ++)
      {
        if(!(*itr)->directConstraint()) total_nc += (*itr)->getNc();
      }

    if(total_nc == 0)
      {
        ROS_ERROR("no propoer consttaint for QP, since total_nc = 0");
        return false;
      }

    /* QP */
    /*
      1. QP cost function in qp Oases is : 1/2 * x^T H x + g^T x
      - so the hessian Matrix H shoube be multiplied by 2, or gradian g shoud be half.
      2. n_wrs is a total number in whole sequnece process, the value decrease every loop.
      3. the matrix in Eigen library is colunm-major, so use transpose to get row-major data.
      Plus, (A.transpose()).data is still column-mojar, please assign to a new matrix
    */
    boost::shared_ptr<SQProblem> qp_solver(new SQProblem(6 + robot_model_ptr_->getActuatorJointMap().size(), total_nc));
    bool qp_init_flag = true;

    int n_wsr = 100;
    Options qp_options;
    qp_options.enableEqualities = BT_TRUE;
    qp_options.printLevel = PL_LOW;
    qp_solver->setOptions(qp_options);

    /* inverse kinematics loop */
    for(int l = 0; l < differential_kinematics_count_; l++)
      {
        if(debug) ROS_WARN("loop: %d", l);
        Eigen::MatrixXd qp_H = Eigen::MatrixXd::Zero(qp_solver->getNV(), qp_solver->getNV());
        Eigen::MatrixXd qp_g = Eigen::VectorXd::Zero(qp_solver->getNV());
        Eigen::VectorXd qp_lb = Eigen::VectorXd::Constant(qp_solver->getNV(), -INFTY);
        Eigen::VectorXd qp_ub = Eigen::VectorXd::Constant(qp_solver->getNV(),  INFTY);

        Eigen::MatrixXd qp_A = Eigen::MatrixXd::Zero(qp_solver->getNC(), qp_solver->getNV());
        Eigen::VectorXd qp_lA = Eigen::VectorXd::Constant(qp_solver->getNC(), -INFTY);
        Eigen::VectorXd qp_uA = Eigen::VectorXd::Constant(qp_solver->getNC(), INFTY);

        // if(debug) std::cout << "the init qp lb is: \n" << qp_lb << std::endl;
        // if(debug) std::cout << "the init qp ub is: \n" << qp_ub << std::endl;

        /* step1: update the kinematics by forward kinemtiacs, along with the modelling with current kinematics  */
        KDL::Rotation root_att;
        tf::quaternionTFToKDL(target_root_pose_.getRotation(), root_att);
        robot_model_ptr_->setCogDesireOrientation(root_att);
        robot_model_ptr_->forwardKinematics(target_actuator_vector_);
        if(!robot_model_ptr_->stabilityMarginCheck()) ROS_ERROR("[differential kinematics] update modelling, bad stability margin: %f (thre: %f)", robot_model_ptr_->getStabilityMargin(), robot_model_ptr_->stability_margin_thre_);
        if(!robot_model_ptr_->modelling()) ROS_ERROR("[differential kinematics] update modelling, bad stability from force");
        if(!robot_model_ptr_->overlapCheck()) ROS_ERROR("[differential kinematics] update overlap check, detect overlap with this form");

        /* considering the non-joint modules such as gimbal are updated after forward-kinemtics */
        /* the correct target_actuator vector should be added here */
        target_root_pose_sequence_.push_back(target_root_pose_);
        target_actuator_vector_sequence_.push_back(target_actuator_vector_);

        /* step2: check convergence & update Hessian and Gradient */
        bool convergence = true;
        for(auto itr = cost_container.begin(); itr != cost_container.end(); itr++)
          {
            bool single_convergence;
            Eigen::MatrixXd single_H;
            Eigen::VectorXd single_g;
            /* hesian and gradient should be linear sum */
            (*itr)->getHessianGradient(single_convergence, single_H, single_g, debug);
            qp_H += single_H;
            qp_g += single_g;

            convergence &= single_convergence;
          }

        if(convergence)
          {
            ROS_INFO("convergence to target state with %d times with %f[sec]", l, ros::Time::now().toSec() - start_time);
            for(auto itr: constraint_container) itr->result();
            solved_ = true;
            return true;
          }

        /* step3: update Constaint Matrix and Bounds */
        size_t offset = 0;
        for(auto itr = constraint_container.begin(); itr != constraint_container.end(); itr++)
          {
            Eigen::MatrixXd single_A;
            Eigen::VectorXd single_lb;
            Eigen::VectorXd single_ub;

            if(!(*itr)->getConstraint(single_A, single_lb, single_ub, debug))
              {
                ROS_ERROR("constraint: %s is invlid", (*itr)->getConstraintName().c_str());
                return false;
              }

            if((*itr)->directConstraint()) /* without constraint matrix */
              {
                if(qp_solver->getNV() != (*itr)->getNc())
                  {
                    ROS_ERROR("the constiant number from %s is %d, which is not equal with the state number %d",  (*itr)->getConstraintName().c_str(), (*itr)->getNc(), qp_solver->getNV());
                    return false;
                  }

                for(size_t i = 0; i < qp_solver->getNV(); i++)
                  {
                    if(single_lb(i) > qp_lb(i)) qp_lb(i) = single_lb(i);
                    if(single_ub(i) < qp_ub(i)) qp_ub(i) = single_ub(i);
                  }
              }
            else /* with constraint matrix */
              {
                qp_lA.segment(offset, (*itr)->getNc()) = single_lb;
                qp_uA.segment(offset, (*itr)->getNc()) = single_ub;
                qp_A.block(offset, 0, single_A.rows(), single_A.cols()) = single_A;
                offset += (*itr)->getNc();
              }
          }
        if(debug)
          {
            std::cout << "qp H \n" << qp_H << std::endl;
            std::cout << "qp g \n" << qp_g.transpose() << std::endl;
            std::cout << "qp A \n" << qp_A << std::endl;
            std::cout << "qp lA \n" << qp_lA.transpose() << std::endl;
            std::cout << "qp uA \n" << qp_uA.transpose() << std::endl;
          }

        /* step4: calculate the QP using qp-oases  */
        int solver_result;
        n_wsr = 100; /* this value have to be updated every time, otherwise it will decrease every loop */
        Eigen::MatrixXd qp_At = qp_A.transpose();
        if(qp_init_flag)
          { /* first time */
            qp_init_flag = false;

            solver_result = qp_solver->init(qp_H.data(), qp_g.data(),
                                            qp_At.data(), /* CAUTION: eigen is col-major order array in default! */
                                            qp_lb.data(), qp_ub.data(),
                                            qp_lA.data(), qp_uA.data(), n_wsr);
          }
        else
          {
            solver_result = qp_solver->hotstart(qp_H.data(), qp_g.data(),
                                                qp_At.data(),
                                                qp_lb.data(), qp_ub.data(),
                                                qp_lA.data(), qp_uA.data(), n_wsr);
          }

        if(solver_result != 0)
          {
            ROS_ERROR("can not solve QP the solver_result is %d", solver_result);
            solved_ = true; //debug
            return false;
          }

        Eigen::VectorXd delta_state_vector = Eigen::VectorXd::Zero(qp_solver->getNV());
        qp_solver->getPrimalSolution(delta_state_vector.data());
        if(debug)
          {
            std::cout << "delta state vector: \n" << delta_state_vector.transpose() << std::endl;
            std::cout << "qp_A * delta state vector: \n" << (qp_A *  delta_state_vector).transpose() << std::endl;
          }

        /* step5: update the state (root link & joint state) */
        /* root link incremental transformation: */
        tf::Vector3 delta_pos_from_root_link;
        tf::vectorEigenToTF(delta_state_vector.head(3), delta_pos_from_root_link);
        tf::Vector3 delta_rot_from_root_link;
        tf::vectorEigenToTF(delta_state_vector.segment(3, 3), delta_rot_from_root_link);

        if(delta_rot_from_root_link.length() == 0)
          target_root_pose_ *= tf::Transform(tf::Quaternion(0, 0, 0, 1), delta_pos_from_root_link);
        else
          target_root_pose_ *= tf::Transform(tf::Quaternion(delta_rot_from_root_link, delta_rot_from_root_link.length()), delta_pos_from_root_link);

        /* udpate the joint angles */
        for(size_t i = 0; i < robot_model_ptr_->getActuatorJointMap().size(); i++)
          target_actuator_vector_.position.at(robot_model_ptr_->getActuatorJointMap().at(i)) += delta_state_vector(i + 6);

        /* step6: update each cost or constraint (e.g. changable ik), if necessary */
        for(auto func_itr = update_func_vector_.begin(); func_itr != update_func_vector_.end(); func_itr++)
          {
          if(!(*func_itr)())
            {
              ROS_ERROR("update function fail");
              //solved_ = true; //debug
              return false;
            }
          }

        if(debug)
          {
            ROS_WARN("finish loop %d", l);
            std::cout << "target joint vector: \n" << getTargetJointVector() << std::endl;
          }
      }

    ROS_WARN("can not solve the differential kinematics based planning with %d", differential_kinematics_count_);
    solved_ = true; //debug
    return false;
  }

  void Planner::registerMotionFunc(std::function<void(void)> new_func)
  {
    motion_func_vector_.push_back(new_func);
  }

  void Planner::registerUpdateFunc(std::function<bool(void)> new_func)
  {
    update_func_vector_.push_back(new_func);
  }

  const Eigen::VectorXd Planner::getTargetJointVector()
  {
    Eigen::VectorXd target_joint_vector = Eigen::VectorXd::Zero(robot_model_ptr_->getActuatorJointMap().size());
    for(size_t i = 0; i < target_joint_vector.size(); i++)
      {
        target_joint_vector(i) = target_actuator_vector_.position.at(robot_model_ptr_->getActuatorJointMap().at(i));
      }
    return target_joint_vector;
  }

  void Planner::motionFunc(const ros::TimerEvent & e)
  {
    if(solved_)
      {
        /* publish joint angle and root tf */
        ros::Time now_time = ros::Time::now();
        br_.sendTransform(tf::StampedTransform(target_root_pose_sequence_.at(sequence_), now_time, "world", "root"));
        target_actuator_vector_sequence_.at(sequence_).header.stamp = now_time;
        actuator_state_pub_.publish(target_actuator_vector_sequence_.at(sequence_));
        sequence_++;
        if(sequence_ == target_actuator_vector_sequence_.size()) sequence_ = 0;

        /* additional function from external module */
        for(auto func_itr = motion_func_vector_.begin(); func_itr != motion_func_vector_.end(); func_itr++)
          (*func_itr)();

      }
  }

};
