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

#ifndef DIFFERENTIA_KINEMATICS_COLLISION_AVOIDANCE_H
#define DIFFERENTIA_KINEMATICS_COLLISION_AVOIDANCE_H

#include <hydrus/differential_kinematics/constraint/base_plugin.h>

/* for env obstacle model */
#include <visualization_msgs/MarkerArray.h>

/* robot model */
#include <urdf/model.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <sensor_msgs/JointState.h>

/* for collision detection */
#include <fcl/fcl.h>
// #include <fcl/collision_object.h>
// #include <fcl/shape/geometric_shapes.h>
// #include <fcl/collision_data.h>
// #include <fcl/distance.h>
// #include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>

/* uitls */
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_kdl.h>
namespace differential_kinematics
{
  namespace constraint
  {
    template <class motion_planner>
    class CollisionAvoidance :public Base<motion_planner>
    {
    public:
      CollisionAvoidance();
      ~CollisionAvoidance() {}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<motion_planner> planner, std::string constraint_name,
                              bool orientation, bool full_body);


      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false) ;
      bool directConstraint(){return false;}

      void result();

      void setEnv(visualization_msgs::MarkerArray env_collision)
      {
        if (env_collision.markers.size() == 0)
          {
            ROS_ERROR("no valid envrioment is provided, env object is zero");
          }

        for(auto obj : env_collision.markers)
          {
            assert(0.99f < obj.pose.orientation.w * obj.pose.orientation.w + obj.pose.orientation.x * obj.pose.orientation.x
                   + obj.pose.orientation.y * obj.pose.orientation.y + obj.pose.orientation.z * obj.pose.orientation.z);

            collision_manager_->registerObject(new fcl::CollisionObject<double>(createGeometryObject(obj), Eigen::Quaternion<double>(obj.pose.orientation.w, obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z).matrix(), Eigen::Vector3d(obj.pose.position.x, obj.pose.position.y, obj.pose.position.z)));
          }

      }

      std::shared_ptr<fcl::CollisionGeometry<double>> createGeometryObject(visualization_msgs::Marker obj)
      {
        if(obj.type == visualization_msgs::Marker::CUBE)
          {
            //ROS_INFO("env obstacle: CUBE");
            return std::shared_ptr<fcl::CollisionGeometry<double> >(new fcl::Box<double>(obj.scale.x, obj.scale.y, obj.scale.z));
          }
        else if(obj.type == visualization_msgs::Marker::SPHERE)
          {
            //ROS_INFO("env obstacle: SPHERE");
            if(obj.scale.x != obj.scale.y || obj.scale.x != obj.scale.z)
              {
                ROS_ERROR("the sphere size is wrong, scale is [%f, %f, %f]", obj.scale.x, obj.scale.y, obj.scale.z);
                return nullptr;
              }

            return std::shared_ptr<fcl::CollisionGeometry<double> >(new fcl::Sphere<double>(obj.scale.x /2 )); // diameter -> radius
          }
        else if(obj.type == visualization_msgs::Marker::CYLINDER)
          {
            //ROS_INFO("env obstacle: CYLINDER");
            if(obj.scale.x != obj.scale.y)
              {
                ROS_ERROR("the cylinder size is wrong, scale is [%f, %f, %f]", obj.scale.x, obj.scale.y, obj.scale.z);
                return nullptr;
              }

            return std::shared_ptr<fcl::CollisionGeometry<double> >(new fcl::Cylinder<double>(obj.scale.x / 2, obj.scale.z));  // diameter -> radius
          }
        else
          {
            ROS_WARN("currently not supported: %d", (int)obj.type);
          }

        return nullptr;
      }

      std::shared_ptr<fcl::CollisionGeometry<double> > createGeometryObject(boost::shared_ptr<urdf::Link> link);

      bool closestPoint();

      struct DistanceData
      {
        DistanceData() { done = false; }

        fcl::DistanceRequest<double>  request;
        fcl::DistanceResult<double>  result;
        bool done;
      };

      static bool defaultDistanceFunction(fcl::CollisionObject<double> * o1, fcl::CollisionObject<double> * o2, void* cdata_, double& dist);

    private:
      boost::shared_ptr<fcl::BroadPhaseCollisionManager<double> > collision_manager_;
      std::vector<fcl::CollisionObject<double>  *> robot_collision_model_;

      double collision_damping_gain_;
      double collision_distance_constraint_range_;
      double collision_distance_forbidden_range_;

      bool getJacobian(Eigen::MatrixXd& jacobian, KDL::JntArray joint_positions, std::string parent_link_name, KDL::Frame f_parent_link, KDL::Vector contact_offset, bool debug = false);


      /* not good varible */
      std::vector<int> joint_map_;

      /* result */
      double min_dist_;
      std::string min_dist_link_;
    };
  };
};

#endif
