// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, JSK Lab
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

#include <dragon/maximum_single_contact_force.h>

MaximumSingleContact::MaximumSingleContact(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<DragonRobotModel> robot_model_ptr): nh_(nh), nhp_(nhp), robot_model_ptr_(robot_model_ptr)
{
  nhp_.param("verbose", verbose_, false);

  /* reset the baselink to link1, which is easy to solve the problem */
  ROS_WARN_STREAM("change the baselink of dragon from " << robot_model_ptr_->getBaselinkName() << " to link1");
  robot_model_ptr_->setBaselinkName(std::string("link1"));

  max_thrust_force_ = robot_model_ptr_->getThrustUpperLimit();
  rotor_num_ = robot_model_ptr_->getRotorNum();
  inertia_map_ = robot_model_ptr_->getInertiaMap();
}

bool MaximumSingleContact::getMaximumSingleContactForce(const std::string& contact_frame, const double& root_roll, const double& root_pitch, const double& root_yaw, const sensor_msgs::JointState& joint_state, double& max_upward_contact_force, double& max_downward_contact_force, double& max_horizontal_contact_force)
{
  /* CoG Frame == World Frame */
  robot_model_ptr_->setCogDesireOrientation(KDL::Rotation::RPY(root_roll, root_pitch, root_yaw));
  robot_model_ptr_->updateRobotModel(joint_state);

  /* overwrite the joint state becuase of the speical process for gimbal */
  auto joint_state_with_gimbal = robot_model_ptr_->getGimbalProcessedJoint<sensor_msgs::JointState>();

  /* check the validity of the robot pose */
  if(!robot_model_ptr_->stabilityMarginCheck())
    {
      if(verbose_) ROS_ERROR_STREAM("bad stability margin: " << robot_model_ptr_->getStabilityMargin());
      return false;
    }

  if(!robot_model_ptr_->modelling())
    {
      if(verbose_) ROS_ERROR("bad stability from force, %f", max_thrust_force_);
      return false;
    }

  if(!robot_model_ptr_->overlapCheck())
    {
      if(verbose_) ROS_ERROR("detect overlap with this form");
      return false;
    }

  Eigen::VectorXd hovering_thrust_vector = robot_model_ptr_->getOptimalHoveringThrust();

  /* verbose */
  std::cout << "hover force from modeling: \n" <<  robot_model_ptr_->getOptimalHoveringThrust().transpose() << std::endl; //debug

  /* calculate the maximum contact force */
  auto segments_tf = robot_model_ptr_->getSegmentsTf();
  auto wrench_tf_root_link = segments_tf.find(contact_frame);

  std::vector<Eigen::Vector3d> rotors_origin_from_cog = robot_model_ptr_->getRotorsOriginFromCog<Eigen::Vector3d>();

#if 1 /* calculate the hovering thrust force from Jacobian */

  /* J_rotor_root_motion_combined: the jacobian regarding rotor force w.r.t. root, in terms of the up root motion balance (i.e., hovering)  */
  Eigen::MatrixXd J_rotor_root_motion_combined = Eigen::MatrixXd::Zero(3 * rotor_num_, 6);
  for(int i = 0; i < rotor_num_; i++)
    {
      /* TODO: for hovering thrust, we have to use "thrustx" frame, but for external wrench compensation, we have to use "gimbalx_roll_module" */
      Eigen::MatrixXd J_rotor_i_root_motion = getJacobian(std::string("root"), std::string("thrust") + std::to_string(i+1), joint_state_with_gimbal, true, robot_model_ptr_->getCogDesireOrientation<Eigen::Matrix3d>());
      J_rotor_root_motion_combined.block(3 * i, 0, 3 , 6) = J_rotor_i_root_motion.block(0, 0, 3, 6);
    }

  /* debug */
  Eigen::VectorXd hover_thrust_3d = Eigen::VectorXd::Zero(12);
  for(int i = 0; i < rotor_num_; i++)
    hover_thrust_3d(3 * i + 2) = hovering_thrust_vector(i);
  std::cout << "virtual root wrench from modeling: \n" << (J_rotor_root_motion_combined.transpose() * hover_thrust_3d).transpose() / robot_model_ptr_->getMass() / 9.8  << std::endl;

  /* calcualte the gravity influence on root motion */
  Eigen::VectorXd virtual_root_wrench = Eigen::VectorXd::Zero(6);
  for(const auto& seg: inertia_map_)
    {
      Eigen::MatrixXd J_gravity_i_root_motion = getJacobian(std::string("root"), seg.first, joint_state_with_gimbal, true, robot_model_ptr_->getCogDesireOrientation<Eigen::Matrix3d>(), KDL::Segment(std::string("cog_point"), KDL::Joint(), KDL::Frame(seg.second.getCOG())));
      virtual_root_wrench += J_gravity_i_root_motion.block(0, 0, 3, 6).transpose() * seg.second.getMass() * Eigen::Vector3d(0, 0, -9.8);
    }
  std::cout << "virtual root wrench from jacobian statics: \n" << virtual_root_wrench.transpose() / robot_model_ptr_->getMass() / 9.8 << std::endl;

  // better result than only z
  Eigen::FullPivLU<Eigen::MatrixXd> lu_solver(J_rotor_root_motion_combined.transpose() * J_rotor_root_motion_combined);
  Eigen::VectorXd hovering_force_statics = J_rotor_root_motion_combined * lu_solver.solve(-virtual_root_wrench);
  std::cout << "hover force from jacobian statics: \n" << hovering_force_statics.transpose() << std::endl;
  //std::cout << hovering_thrust_vector.squaredNorm() << " vs " << hovering_force_statics.squaredNorm() << std::endl;

  /* only z */
  Eigen::MatrixXd J_rotor_root_motion_only_z_combined = Eigen::MatrixXd::Zero(rotor_num_, 3);
  for(int i = 0 ; i < rotor_num_; i++)
    J_rotor_root_motion_only_z_combined.block(i, 0, 1, 3) = J_rotor_root_motion_combined.block(3 * i + 2, 2, 1, 3);
  std::cout << "hover force from jacobian statics only z: \n" << (aerial_robot_model::pseudoinverse(J_rotor_root_motion_only_z_combined.transpose()) * (-virtual_root_wrench).segment(2,3)).transpose() << std::endl;

#endif

  return true;

  if(wrench_tf_root_link != segments_tf.end())
    {
      Eigen::Vector3d wrench_point_cog_frame = aerial_robot_model::kdlToEigen((robot_model_ptr_->getCog<KDL::Frame>().Inverse() * wrench_tf_root_link->second).p);

      Eigen::MatrixXd allocation_mat = Eigen::MatrixXd::Zero(6, 3 * rotor_num_);
      for(int i = 0; i < rotor_num_; i++)
        {
          allocation_mat.block(0, 3 * i, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
          allocation_mat.block(3, 3 * i, 3, 3) = aerial_robot_model::skewSymmetricMatrix(rotors_origin_from_cog[i]);
          //std::cout << "Rotor" << i+1 << "\n" << rotors_origin_from_cog[i] << std::endl; //debug
        }

      std::vector<Eigen::Vector3d> unit_force_list;
      /* 1. upward: +z */
      unit_force_list.push_back(Eigen::Vector3d(0,0,1));
      /* 2. downward: -z */
      unit_force_list.push_back(Eigen::Vector3d(0,0,-1));
      /* 3. horizontal: +x */
      unit_force_list.push_back(Eigen::Vector3d(1,0,0));

      std::vector<double> max_contact_force_list;
      for(const auto& unit_force: unit_force_list)
        {
          if(verbose_) ROS_INFO("for unit force [%f, %f, %f]", unit_force(0), unit_force(1), unit_force(2));
          /* -- calculuation total wrench w.r.t CoG frame -- */
          /* -- Note: CoG frame == World frame -- */
          Eigen::VectorXd wrench_cog_frame = Eigen::VectorXd::Zero(6);
          wrench_cog_frame.head(3) = unit_force;
          wrench_cog_frame.tail(3) = wrench_point_cog_frame.cross(unit_force);
          if(verbose_) std::cout << " external wrench w.r.t cog frame: " << wrench_cog_frame.transpose() << std::endl;

          /* -- allocation -- */
          Eigen::VectorXd ex_force_vec = aerial_robot_model::pseudoinverse(allocation_mat) * (-wrench_cog_frame);
          //std::cout << "ex_force_vec: \n" <<  ex_force_vec.transpose() << std::endl; //debug

          /* -- find the minimum max contact force (i.e., lambda) for all rotor -- */
          double min_lambda = 1e6;
          for(int i = 0; i < rotor_num_; i++)
            {
              Eigen::Vector3d ex_force_i = ex_force_vec.segment(3*i, 3);
              double lambda = 0;
               // || f_ex + hovering_f || = f_max
              double coeff_a = ex_force_i.squaredNorm();
              double coeff_b = 2 * ex_force_i(2) * hovering_thrust_vector(i);
              double coeff_c = hovering_thrust_vector(i) * hovering_thrust_vector(i) - max_thrust_force_ * max_thrust_force_;

              lambda = (-coeff_b + sqrt(coeff_b * coeff_b - 4 * coeff_a * coeff_c)) / (2 * coeff_a);

              if(ex_force_i(2) < 0)
                { // consideration: fz + hovering_f > 0
                  double lambda2 =  hovering_thrust_vector(i) / -ex_force_i(2);
                  if(lambda2 < lambda) lambda = lambda2;
                }

              if(verbose_) ROS_INFO_STREAM(" No." << i+1 << " rotor, the max force: "<< lambda);

              if(min_lambda > lambda) min_lambda = lambda;
            }

          max_contact_force_list.push_back(min_lambda);
          if(verbose_) ROS_INFO_STREAM(" the minimum max contact force is " << min_lambda);
        }

      max_upward_contact_force = max_contact_force_list.at(0);
      max_downward_contact_force = max_contact_force_list.at(1);
      max_horizontal_contact_force = max_contact_force_list.at(2);
    }
  else
    {
      ROS_ERROR_STREAM("undefined segment: " <<  contact_frame);
      return false;
    }


  return true;
}


/* arrange from https://github.com/tongtybj/motion_planning/blob/force_grasping_by_vectoring_thrust/aerial_manipulation/vectoring_thrust_grasp/src/vectoring_thrust_planner.cpp#L495-L568 */
const Eigen::MatrixXd MaximumSingleContact::getJacobian(std::string root_link, std::string tip_link, const sensor_msgs::JointState& joint_angles, bool full_body, Eigen::Matrix3d root_att, KDL::Segment additional_frame)
{
  /* get chain from root_link to tip_link */
  KDL::Chain chain;
  if(!robot_model_ptr_->getTree().getChain(root_link, tip_link, chain))
    {
      ROS_ERROR_STREAM("can not get proper kdl chain from " << root_link << " to " << tip_link);
    }
  if(additional_frame.getName() != "NoName") chain.addSegment(additional_frame);

  /* get jacobian */
  /* assign the joint angle */
  KDL::JntArray joint_positions(chain.getNrOfJoints());

  std::vector<int> whitelist(0);
  int j = 0;
  for(auto seg : chain.segments)
    {
      if(seg.getJoint().getType() != KDL::Joint::JointType::None)
        {
          /* gimbal + joint  */
          if(seg.getJoint().getName().find("rotor") == std::string::npos)
            {
              auto itr = std::find(joint_angles.name.begin(), joint_angles.name.end(), seg.getJoint().getName());
              if(itr != joint_angles.name.end())
                {
                  joint_positions(j) = joint_angles.position.at(std::distance(joint_angles.name.begin(), itr));
                  //std::cout << "add " << seg.getJoint().getName() << ": " << joint_positions(j)  << " to joint posiiton" << j  << std::endl;
                }

              /* joint only */
              if(seg.getJoint().getName().find("joint") != std::string::npos)
                {
                  whitelist.push_back(j);
                  //std::cout << "add " << seg.getJoint().getName() << " to whitelist" << std::endl;
                }
            }
          j++;
        }
    }

  if(verbose_) std::cout << "joint array from " << root_link << " to " << tip_link << ": \n " << joint_positions.data.transpose() << std::endl;

  KDL::ChainJntToJacSolver jac_solver(chain);
  KDL::Jacobian jac(chain.getNrOfJoints());

  if(jac_solver.JntToJac(joint_positions, jac) != KDL::SolverI::E_NOERROR)
    ROS_ERROR_STREAM("Can not calculate the jacobian for " << tip_link << " w.r.t. " << root_link);

  Eigen::MatrixXd joint_jacobian = Eigen::MatrixXd::Zero(6, whitelist.size());
  for(size_t i = 0; i < whitelist.size(); i++)
    joint_jacobian.col(i) = jac.data.col(whitelist.at(i));

  if(full_body)
    {
      Eigen::MatrixXd fullbody_jacobian = Eigen::MatrixXd::Zero(6, joint_jacobian.cols() + 6);

      KDL::Frame end_frame;
      KDL::ChainFkSolverPos_recursive fk_solver(chain);
      fk_solver.JntToCart(joint_positions, end_frame);

      if(verbose_) std::cout << "end pose for full body: [" << end_frame.p.x() << ", " << end_frame.p.y() << ", " << end_frame.p.z() << "]" << std::endl;

      if(joint_jacobian.cols() > 0)
        {
          fullbody_jacobian.block(0, 6, 3, joint_jacobian.cols()) = root_att * joint_jacobian.block(0, 6, 3, joint_jacobian.cols());
          fullbody_jacobian.block(3, 6, 3, joint_jacobian.cols()) = root_att * joint_jacobian.block(3, 6, 3, joint_jacobian.cols());
        }

      /* root link */
      fullbody_jacobian.block(0, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
      Eigen::Vector3d end_frame_wrt_world = root_att * Eigen::Vector3d(end_frame.p.data);
      fullbody_jacobian.block(0, 3, 3, 1) = (Eigen::Vector3d(1, 0, 0)).cross(end_frame_wrt_world);
      fullbody_jacobian.block(0, 4, 3, 1) = (Eigen::Vector3d(0, 1, 0)).cross(end_frame_wrt_world);
      fullbody_jacobian.block(0, 5, 3, 1) = (Eigen::Vector3d(0, 0, 1)).cross(end_frame_wrt_world);


      return fullbody_jacobian;
    }

  return joint_jacobian;
}
