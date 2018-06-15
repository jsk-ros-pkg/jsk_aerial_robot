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

#include <hydrus/differential_kinematics/constraint/collision_avoidance.h>

namespace differential_kinematics
{
  namespace constraint
  {
    template <class motion_planner>
    CollisionAvoidance<motion_planner>::CollisionAvoidance():
      collision_manager_(new fcl::DynamicAABBTreeCollisionManager<double>()),
      joint_map_(), min_dist_(1e6)
    {
    }

    template <class motion_planner>
    void CollisionAvoidance<motion_planner>::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                                        boost::shared_ptr<motion_planner> planner,
                                                        std::string constraint_name,
                                                        bool orientation, bool full_body)
    {
      Base<motion_planner>::initialize(nh, nhp, planner, constraint_name, orientation, full_body);

      /* collision model conversion */
      for(auto link: planner->getRobotModelPtr()->getRobotModel().links_)
        {
          if(link.second->collision)
            {

              robot_collision_model_.push_back(new fcl::CollisionObject<double>(createGeometryObject(link.second)));
              /* add additional data */
              robot_collision_model_.back()->setUserData(link.second.get());
              /* debug */
              // ROS_ERROR("link %s has collision model, pose: [%f, %f, %f]", link.first.c_str(),
              //           link.second->collision->origin.position.x,
              //           link.second->collision->origin.position.y, link.second->collision->origin.position.z);
            }
        }

      Base<motion_planner>::nc_ = robot_collision_model_.size();
      /* temporary mapping method for hydrusx and dragon, which has "rotor" and "gimbal" as joints */
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
      // if(joint_map_.size() != Base<motion_planner>::j_ndof_)
      //   ROS_FATAL("the joint_map_ size %d is not equal with jdof %d", joint_map_.size(), Base<motion_planner>::j_ndof_);

      Base<motion_planner>::nhp_.param ("collision_damping_gain", collision_damping_gain_, 0.01);
      if(Base<motion_planner>::verbose_) std::cout << "collision_damping_gain: " << std::setprecision(3) << collision_damping_gain_ << std::endl;
      Base<motion_planner>::nhp_.param ("collision_distance_constraint_range", collision_distance_constraint_range_, 0.03);
      if(Base<motion_planner>::verbose_) std::cout << "collision_distance_constraint_range: " << std::setprecision(3) << collision_distance_constraint_range_ << std::endl;
      Base<motion_planner>::nhp_.param ("collision_distance_forbidden_range", collision_distance_forbidden_range_, 0.01);
      if(Base<motion_planner>::verbose_) std::cout << "collision_distance_forbidden_range: " << std::setprecision(3) << collision_distance_forbidden_range_ << std::endl;
    }

    template <class motion_planner>
    std::shared_ptr<fcl::CollisionGeometry<double>> CollisionAvoidance<motion_planner>::createGeometryObject(boost::shared_ptr<urdf::Link> link)
    {
      boost::shared_ptr<urdf::Geometry> geom = link->collision->geometry;
      ROS_ASSERT(geom);

      if(geom->type == urdf::Geometry::BOX)
        {
          //ROS_INFO("%s: BOX", link->name.c_str());
          urdf::Vector3 dim = boost::dynamic_pointer_cast<urdf::Box>(geom)->dim;
          return std::shared_ptr<fcl::CollisionGeometry<double> >(new fcl::Box<double>(dim.x, dim.y, dim.z));
        }
      else if(geom->type == urdf::Geometry::SPHERE)
        {
          //ROS_INFO("%s: SPHERE", link->name.c_str());
          return std::shared_ptr<fcl::CollisionGeometry<double> >(new fcl::Sphere<double>(boost::dynamic_pointer_cast<urdf::Sphere>(geom)->radius));
        }
      else if(geom->type == urdf::Geometry::CYLINDER)
        {
          //ROS_INFO("CYLINDER: %f, %f", boost::dynamic_pointer_cast<urdf::Cylinder>(geom)->radius, boost::dynamic_pointer_cast<urdf::Cylinder>(geom)->length);
          return std::shared_ptr<fcl::CollisionGeometry<double> >(new fcl::Cylinder<double>(boost::dynamic_pointer_cast<urdf::Cylinder>(geom)->radius, boost::dynamic_pointer_cast<urdf::Cylinder>(geom)->length));
        }
      else if(geom->type == urdf::Geometry::MESH)
        {
          ROS_WARN("MESH --- currently not supported");
          /* TODO implement the mesh type */
        }
      else
        {
          ROS_ERROR("Unknown geometry type: %d", (int)geom->type);
        }

      return nullptr;
    }

    template <class motion_planner>
    bool CollisionAvoidance<motion_planner>::getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug)
    {
      //debug = true;
      A = Eigen::MatrixXd::Zero(Base<motion_planner>::nc_, Base<motion_planner>::j_ndof_ + 6);
      lb = Eigen::VectorXd::Constant(Base<motion_planner>::nc_, -1e6);
      ub = Eigen::VectorXd::Constant(Base<motion_planner>::nc_, 1e6);

      KDL::TreeFkSolverPos_recursive fk_solver(Base<motion_planner>::planner_->getRobotModelPtr()->getModelTree());
      KDL::JntArray joint_positions(Base<motion_planner>::planner_->getRobotModelPtr()->getModelTree().getNrOfJoints());

      sensor_msgs::JointState actuator_state = Base<motion_planner>::planner_->getTargetActuatorVector();

      auto actuator_map = Base<motion_planner>::planner_->getRobotModelPtr()->getActuatorMap();
      for(size_t i = 0; i < actuator_state.position.size(); i++)
        {
          auto itr = actuator_map.find(actuator_state.name.at(i));

          if(itr != actuator_map.end())  joint_positions(actuator_map.find(actuator_state.name.at(i))->second) = actuator_state.position.at(i);
          //else ROS_FATAL("collision avoidance: no matching joint called %s", actuator_state.name.at(i).c_str());
        }

        /* check the distance */
      for(auto it = robot_collision_model_.begin(); it != robot_collision_model_.end(); it++)
        {
          /* get root tf in world frame */
          KDL::Frame f_root;
          tf::transformTFToKDL(Base<motion_planner>::planner_->getTargetRootPose(), f_root);

          /* get collision-included link tf in root frame */
          urdf::Link link_info = *(reinterpret_cast<urdf::Link *>((*it)->getUserData())); /* OK */
          KDL::Frame f_link;
          int status = fk_solver.JntToCart(joint_positions, f_link, link_info.name);
          /* get collision model offset tf in collision-included link frame */
          KDL::Frame f_collision_obj_offset(KDL::Rotation::Quaternion(link_info.collision->origin.rotation.x,
                                                                      link_info.collision->origin.rotation.y,
                                                                      link_info.collision->origin.rotation.z,
                                                                      link_info.collision->origin.rotation.w),
                                            KDL::Vector(link_info.collision->origin.position.x,
                                                        link_info.collision->origin.position.y,
                                                        link_info.collision->origin.position.z));

          /* get collision model offset tf in world frame */
          KDL::Frame f_collision_obj = f_root * f_link * f_collision_obj_offset;
          double qx, qy, qz, qw;
          f_collision_obj.M.GetQuaternion(qx, qy, qz, qw);
          //ROS_INFO("link name: %s, collision position: [%f, %f, %f], orientation: [%f, %f, %f, %f]", link_info.name.c_str(), f_collision_obj.p.x(), f_collision_obj.p.y(), f_collision_obj.p.z(), qx, qy, qz, qw);

          /* convert from KDL to FCL and update collision object */
          Eigen::Affine3d tf_obj;
          tf::transformKDLToEigen(f_collision_obj, tf_obj);

          (*it)->setTransform(tf_obj);
          (*it)->computeAABB();

          /* start nearest points calculation */
          fcl::DistanceResult<double> result;
          result.clear();
          fcl::DistanceRequest<double> request(true);
          DistanceData distance_data;
          distance_data.request = request;
          collision_manager_->distance((*it), &distance_data, CollisionAvoidance::defaultDistanceFunction);

          if(boost::math::isnan(distance_data.result.nearest_points[0](0)) ||
             boost::math::isnan(distance_data.result.nearest_points[0](1)) ||
             boost::math::isnan(distance_data.result.nearest_points[0](2)))
            {
              ROS_WARN("broadphase distance process causes  nan");
              //ROS_ERROR("distance: %f is nan, link name: %s, collision position: [%f, %f, %f], orientation: [%f, %f, %f, %f]", distance_data.result.min_distance, link_info.name.c_str(), f_collision_obj.p.x(), f_collision_obj.p.y(), f_collision_obj.p.z(), qx, qy, qz, qw);
              //std::cout << " point of env in local frame: x = " << distance_data.result.nearest_points[0](0) << " y = " << distance_data.result.nearest_points[0](1) << " z = " << distance_data.result.nearest_points[0](2) << std::endl;
              //std::cout << " point on robot in local frame: x = " << distance_data.result.nearest_points[1](0) << " y = " << distance_data.result.nearest_points[1](1) << " z = " << distance_data.result.nearest_points[1](2) << std::endl;

              /* special process */
              std::vector< fcl::CollisionObject<double> * > env_objs;
              collision_manager_->getObjects (env_objs);

              fcl::DistanceResult<double> result;
              fcl::DistanceRequest<double> request(true);
              double distance = 1e6;
              for(auto env_obj: env_objs)
                {
                  result.clear();
                  double distance_temp = fcl::distance(env_obj, (*it), request, result);
                  // std::cout << "\n" << "distance is: " << distance << std::endl;
                  // std::cout << " point of env in local frame: x = " << result.nearest_points[0](0) << " y = " << result.nearest_points[0](1) << " z = " << result.nearest_points[0](2) << std::endl;
                  // std::cout << " point on robot in local frame: x = " << result.nearest_points[1](0) << " y = " << result.nearest_points[1](1) << " z = " << result.nearest_points[1](2) << std::endl;

                  if(boost::math::isnan(result.nearest_points[0](0)))
                    {
                      std::cout << "\n" << "object type1: " << env_obj->getNodeType() <<  "; object type2: " << (*it)->getNodeType() << std::endl;

                      if(env_obj->getNodeType()  == fcl::GEOM_BOX && (*it)->getNodeType()  == fcl::GEOM_BOX)
                        {
                          const fcl::Box<double>* obj1_ptr = dynamic_cast<const fcl::Box<double>* >((env_obj->getCollisionGeometry()));
                          const fcl::Box<double>* obj2_ptr = dynamic_cast<const fcl::Box<double>* >(((*it)->getCollisionGeometry()));
                          ROS_INFO("distance: %f", distance_temp);
                          ROS_INFO("obj1: size: [%f, %f, %f], pos: [%f, %f, %f], rot: [%f %f, %f, %f]",
                                   obj1_ptr->side[0], obj1_ptr->side[1], obj1_ptr->side[2],
                                   env_obj->getTranslation()[0], env_obj->getTranslation()[1], env_obj->getTranslation()[2],
                                   env_obj->getQuatRotation().x(), env_obj->getQuatRotation().y(), env_obj->getQuatRotation().z(), env_obj->getQuatRotation().w());
                          ROS_INFO("obj2: size: [%f, %f, %f], pos: [%f, %f, %f], rot: [%f, %f, %f, %f]",
                                   obj2_ptr->side[0], obj2_ptr->side[1], obj2_ptr->side[2],
                                   (*it)->getTranslation()[0], (*it)->getTranslation()[1], (*it)->getTranslation()[2],
                                   (*it)->getQuatRotation().x(), (*it)->getQuatRotation().y(), (*it)->getQuatRotation().z(), (*it)->getQuatRotation().w());

                          //std::cout << " point of env in local frame: x = " << result.nearest_points[0](0) << " y = " << result.nearest_points[0](1) << " z = " << result.nearest_points[0](2) << std::endl;
                          //std::cout << " point on robot in local frame: x = " << result.nearest_points[1](0) << " y = " << result.nearest_points[1](1) << " z = " << result.nearest_points[1](2) << std::endl;
                        }
                    }

                  if(!boost::math::isnan(result.nearest_points[0](0)) &&
                     !boost::math::isnan(result.nearest_points[0](1)) &&
                     !boost::math::isnan(result.nearest_points[0](2)) &&
                     !boost::math::isnan(result.nearest_points[1](0)) &&
                     !boost::math::isnan(result.nearest_points[1](1)) &&
                     !boost::math::isnan(result.nearest_points[1](2)))
                    {
                      if(distance_temp < distance && distance_temp > 0)
                        {
                          distance_data.result = result;
                          distance_data.result.min_distance = distance_temp;
                          distance = distance_temp;
                        }
                    }
                }

              if(distance == 1e6)
                {
                  ROS_WARN("all narrowphase distance process cause nan");
                  return false;
                }

            }

          /* closest points in local object frame */
          //ROS_WARN("distance = %f", distance_data.result.min_distance);
          //std::cout << " point of env in local frame: x = " << distance_data.result.nearest_points[0](0) << " y = " << distance_data.result.nearest_points[0](1) << " z = " << distance_data.result.nearest_points[0](2) << std::endl;
          //std::cout << " point on robot in local frame: x = " << distance_data.result.nearest_points[1](0) << " y = " << distance_data.result.nearest_points[1](1) << " z = " << distance_data.result.nearest_points[1](2) << std::endl;

          if(distance_data.result.min_distance <= 0)
            {
              ROS_ERROR("distance: %f is negative", distance_data.result.min_distance);
              return false;
            }

          /* update the min distance */
          if(distance_data.result.min_distance < min_dist_)
            {
              min_dist_ = distance_data.result.min_distance;
              min_dist_link_ = link_info.name;
            }

          /* closest points in world object frame */
          /* this seaching algorithm is not good! */
          /* TODO: create orignail callback function instead of defaultDistanceFunction */
          KDL::Vector p_in_env;
          std::vector< fcl::CollisionObject<double> * > env_objs;
          collision_manager_->getObjects(env_objs);
          for(auto env_obj : env_objs)
            {
              if(env_obj->collisionGeometry().get() == distance_data.result.o1)
                {
                  //ROS_INFO("the object world position is [%f, %f, %f]", env_obj->getTranslation()[0], env_obj->getTranslation()[1], env_obj->getTranslation()[2]);
                  tf::vectorEigenToKDL(env_obj->getTransform() * (distance_data.result.nearest_points[0]), p_in_env);
                  //ROS_INFO("the point in object in world frame is [%f, %f, %f]", p_in_env[0], p_in_env[1], p_in_env[2]);
                }
            }

          KDL::Vector p_in_robot_local_frame;
          tf::vectorEigenToKDL(distance_data.result.nearest_points[1], p_in_robot_local_frame);
          KDL::Vector p_in_robot = f_collision_obj * p_in_robot_local_frame;
          if(debug)
            std::cout << " point of robot in world frame: x = " <<  p_in_robot.x() << " y = " << p_in_robot.y() << " z = " << p_in_robot.z()  << std::endl;

          /* conver normal to root link: from env to robot */
          tf::Vector3 distance_arrow;
          tf::vectorKDLToTF(p_in_robot - p_in_env, distance_arrow);
          // ROS_ERROR("distance_arrow: [%f, %f, %f], normalized: [%f, %f, %f]",
          //           distance_arrow.x(), distance_arrow.y(), distance_arrow.z(),
          //           distance_arrow.normalized().x(), distance_arrow.normalized().y(), distance_arrow.normalized().z());
          tf::Vector3 n_in_root_link =  Base<motion_planner>::planner_->getTargetRootPose().getBasis().inverse() * distance_arrow.normalized();
          Eigen::Vector3d n_in_root_link_eigen;
          tf::vectorTFToEigen(n_in_root_link, n_in_root_link_eigen);

          /* update jacobian and update constraint */
          int index = std::distance(robot_collision_model_.begin(), it);

          Eigen::MatrixXd jacobian;
          if(!getJacobian(jacobian, joint_positions, link_info.name.c_str(), f_link, f_collision_obj_offset * p_in_robot_local_frame, debug)) return false;

          // std::cout << "n_in_root_link_eigen.transpose() * jacobian: \n" << n_in_root_link_eigen.transpose() * jacobian << std::endl;
          // std::cout << "n_in_root_link: \n" << n_in_root_link_eigen.transpose() << std::endl;
          A.block(index, 0, 1, A.cols()) = n_in_root_link_eigen.transpose() * jacobian;

          if (distance_data.result.min_distance < collision_distance_constraint_range_)
            lb(index) = - collision_damping_gain_ *  (distance_data.result.min_distance - collision_distance_forbidden_range_)/ (collision_distance_constraint_range_ - collision_distance_forbidden_range_);
        }

      if(debug)
        {
          std::cout << "constraint (" << Base<motion_planner>::constraint_name_.c_str()  << "): matrix A: \n" << A << std::endl;
          std::cout << "constraint name: " << Base<motion_planner>::constraint_name_ << ", lb: \n" << lb << std::endl;
          std::cout << "constraint name: " << Base<motion_planner>::constraint_name_ << ", ub: \n" << ub.transpose() << std::endl;
        }
    }

    template <class motion_planner>
    bool CollisionAvoidance<motion_planner>::defaultDistanceFunction(fcl::CollisionObject<double>* o1, fcl::CollisionObject<double>* o2, void* cdata_, double& dist)
    {
      auto* cdata = static_cast<DistanceData*>(cdata_);
      const fcl::DistanceRequest<double>& request = cdata->request;
      fcl::DistanceResult<double>& result = cdata->result;

      if(cdata->done) { dist = result.min_distance; return true; }

      distance(o1, o2, request, result);

      dist = result.min_distance;

      if(dist <= 0) return true; // in collision or in touch

      return cdata->done;
    }

    template <class motion_planner>
    bool CollisionAvoidance<motion_planner>::getJacobian(Eigen::MatrixXd& jacobian, KDL::JntArray joint_positions, std::string parent_link_name, KDL::Frame f_parent_link, KDL::Vector contact_offset, bool debug)
    {
      jacobian = Eigen::MatrixXd::Zero(3, Base<motion_planner>::j_ndof_ + 6);

      /* calculate the jacobian */
      KDL::TreeJntToJacSolver jac_solver(Base<motion_planner>::planner_->getRobotModelPtr()->getModelTree());
      KDL::Jacobian jac(Base<motion_planner>::planner_->getRobotModelPtr()->getModelTree().getNrOfJoints());
      if(jac_solver.JntToJac(joint_positions, jac, parent_link_name) == KDL::SolverI::E_NOERROR)
        {
          /* offset for jacobian */
          jac.changeRefPoint(f_parent_link.M * contact_offset);
          if(debug) std::cout << "raw jacobian for " << parent_link_name << " (only joints):\n" << jac.data << std::endl;
          /* joint reassignment  */
          for(size_t i = 0; i < joint_map_.size(); i++)
            jacobian.block(0, 6 + i , 3, 1) = jac.data.block(0, joint_map_.at(i), 3, 1);

          /* full body */
          if(Base<motion_planner>::full_body_)
            {
              /* consider root is attached with a 6Dof free joint */

              /* get end frame position from root */
              KDL::Vector end_frame = f_parent_link * contact_offset;
              if(debug)
                ROS_INFO("collision avoidance end pose from root link: [%f %f %f]", end_frame.x(), end_frame.y(), end_frame.z());

              /* root link */
              jacobian.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
              jacobian.block(0, 3, 3, 1) = (Eigen::Vector3d(1, 0, 0)).cross(Eigen::Vector3d(end_frame.data));
              jacobian.block(0, 4, 3, 1) = (Eigen::Vector3d(0, 1, 0)).cross(Eigen::Vector3d(end_frame.data));
              jacobian.block(0, 5, 3, 1) = (Eigen::Vector3d(0, 0, 1)).cross(Eigen::Vector3d(end_frame.data));
            }

          if(debug) std::cout << "jacobian: \n" << jacobian << std::endl;
          return true;
        }

      ROS_WARN("constraint (%s) can not calculate the jacobian", Base<motion_planner>::constraint_name_.c_str());
      return false;
    }

    template <class motion_planner>
    void CollisionAvoidance<motion_planner>::result()
    {
      std::cout << Base<motion_planner>::constraint_name_ << "min distance: "
                << min_dist_ << ", at " << min_dist_link_ << std::endl;
    }
      };
};

#include <pluginlib/class_list_macros.h>
#include <hydrus/differential_kinematics/planner_core.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::CollisionAvoidance<differential_kinematics::Planner>, differential_kinematics::constraint::Base<differential_kinematics::Planner>);



