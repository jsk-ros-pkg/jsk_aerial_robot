// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials providedpubli
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
#include <ros/ros.h>
#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <hydrus/hydrus_tilted_robot_model.h>
#include <pluginlib/class_loader.h>
#include <nlopt.hpp>
#include "std_msgs/Float64.h"

class OptimizePosePlanner
{
public:

  OptimizePosePlanner(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~OptimizePosePlanner() {}

  const sensor_msgs::JointState getOptJointAngles() const { return opt_joint_angles_; };
  const sensor_msgs::JointState getRefJointAngles() const { return ref_joint_angles_; };
  boost::shared_ptr<aerial_robot_model::RobotModel> getRobotModel() { return robot_model_; };
  boost::shared_ptr<HydrusRobotModel> getHydrusRobotModel() { return hydrus_robot_model_; };


private:

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Timer planner_timer_;

  ros::Publisher max_fctmin_pub_;
  ros::Publisher max_fcrpmin_pub_;
  ros::Publisher opt_joint_angle_pub_;
  ros::Subscriber ref_joint_angle_sub_;

  bool do_plan_;

  boost::shared_ptr<nlopt::opt> nl_solver_;

  sensor_msgs::JointState opt_joint_angles_, ref_joint_angles_;
  std_msgs::Float64 Max_fctmin_msg;
  std_msgs::Float64 Max_fcrpmin_msg;
  std_msgs::Float64 Fc_rp_min_;

  pluginlib::ClassLoader<aerial_robot_model::RobotModel> robot_model_loader_;
  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
  boost::shared_ptr<HydrusTiltedRobotModel> hydrus_robot_model_;

  // optimization parameter
  double fc_t_min_thresh_;
  double fc_rp_min_thre_;
  double delta_angle_range_;
  bool opt_switch_RollPitchDist_;
  double max_fctmin_;
  double max_fcrpmin_;

  void cmdJointAngleCallback(const sensor_msgs::JointStateConstPtr& cmd_msg);
  void plan(const ros::TimerEvent & e);
};



namespace
{
  double maximizeFCTMin(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
  {
    OptimizePosePlanner *planner = reinterpret_cast<OptimizePosePlanner*>(ptr);
    auto robot_model = planner->getRobotModel();

    sensor_msgs::JointState joint_angles = planner->getRefJointAngles();

    assert(joint_angles.position.size() == x.size());

    for (size_t i = 0; i < x.size(); i++)
      {
        joint_angles.position.at(i) += x.at(i);
      }

    robot_model->updateRobotModel(joint_angles);
    return robot_model->getFeasibleControlTMin();
  }

  double maximizeRollPitchDistsMin(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
  {
    OptimizePosePlanner *planner = reinterpret_cast<OptimizePosePlanner*>(ptr);
    auto hydrus_robot_model = planner->getHydrusRobotModel();

    sensor_msgs::JointState joint_angles = planner->getRefJointAngles();

    assert(joint_angles.position.size() == x.size());

    for (size_t i = 0; i < x.size(); i++)
      {
        joint_angles.position.at(i) += x.at(i);
      }

    hydrus_robot_model->updateRobotModel(joint_angles);
    return hydrus_robot_model->getFeasibleControlRollPitchMin();
  }        

};


OptimizePosePlanner::OptimizePosePlanner(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp), do_plan_(false),
  robot_model_loader_("aerial_robot_model", "aerial_robot_model::RobotModel")
{
  // load robot model plugin
  std::string plugin_name;
  if(nhp_.getParam("robot_model", plugin_name))
    {
      std::cout << "plugin" << std::endl;
      try
        {
          robot_model_ = robot_model_loader_.createInstance(plugin_name);
          hydrus_robot_model_ = boost::make_shared<HydrusTiltedRobotModel>();
        }
      catch(pluginlib::PluginlibException& ex)
        {
          ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }
    }
  else
    {
      std::cout << "not plug in" << std::endl;
      ROS_ERROR("can not find plugin rosparameter for robot model, use default class: aerial_robot_model::RobotModel");
      robot_model_ = boost::make_shared<aerial_robot_model::RobotModel>();
      hydrus_robot_model_ = boost::make_shared<HydrusTiltedRobotModel>();
    }

  // rosparam
  nhp_.param("fc_t_min_thresh", fc_t_min_thresh_, 1.0);
  nhp_.param("/hydrus/fc_rp_min_thre", fc_rp_min_thre_, 1.0);
  nhp_.param("delta_angle_range", delta_angle_range_, 0.4);
  nhp_.param("optimize_RollPitchDist", opt_switch_RollPitchDist_, false);

  // get joint states info from robot model
  for(auto name: robot_model_->getJointNames())
    {
      ref_joint_angles_.name.push_back(name);
      ref_joint_angles_.position.push_back(0);
    }

  // pub & sub
  ref_joint_angle_sub_ = nh_.subscribe("ref_joint_angles", 1, &OptimizePosePlanner::cmdJointAngleCallback, this);
  opt_joint_angle_pub_ = nh_.advertise<sensor_msgs::JointState>("opt_joint_angles", 1);
  max_fctmin_pub_ = nh_.advertise<std_msgs::Float64>("opt_max_fctmin", 1);
  max_fcrpmin_pub_ = nh_.advertise<std_msgs::Float64>("opt_max_fcrpmin", 1);

  // timer
  planner_timer_ = nh_.createTimer(ros::Duration(0.1), &OptimizePosePlanner::plan, this);
}

void OptimizePosePlanner::cmdJointAngleCallback(const sensor_msgs::JointStateConstPtr& cmd_msg)
{
  // receive the reference pose from tele-operation
  if (cmd_msg->name.size() != cmd_msg->position.size())
    {
      ROS_ERROR("[NLOpt] the size of name and position in joint angles should be the same");
      return;
    }

  for (int i = 0; i < cmd_msg->name.size(); i++)
    {
      auto itr = std::find(ref_joint_angles_.name.begin(), ref_joint_angles_.name.end(),
                      cmd_msg->name.at(i));
      if (itr == cmd_msg->name.end())
        {
          ROS_WARN_STREAM("[NLOpt] cannot find " << cmd_msg->name.at(i) << ": in robot model");
          continue;
        }

      const int index = std::distance(ref_joint_angles_.name.begin(), itr);

      ref_joint_angles_.position.at(index) = cmd_msg->position.at(i);
    }

  do_plan_ = true;
}

void OptimizePosePlanner::plan(const ros::TimerEvent & e)
{
  // do planner for new optimized pose
  if(do_plan_)
    {
      Fc_rp_min_.data = hydrus_robot_model_->getFeasibleControlRollPitchMin();
      size_t joint_size = ref_joint_angles_.position.size();

      // initialize
      if(nl_solver_ == nullptr)
        {
          if(opt_switch_RollPitchDist_ == false)
          {
            nl_solver_ = boost::make_shared<nlopt::opt>(nlopt::LN_COBYLA, joint_size);

            nl_solver_->set_max_objective(maximizeFCTMin, this);
            //nl_solver_->add_inequality_constraint(xxxFunc, this, 1e-8); // TODO: please add some constraints if necessary
            nl_solver_->set_xtol_rel(1e-4); //1e-4
            nl_solver_->set_maxeval(1000); // 1000 times

            // lb and ub
            std::vector<double> lb(joint_size, -delta_angle_range_);
            std::vector<double> ub(joint_size, delta_angle_range_);
            nl_solver_->set_lower_bounds(lb);
            nl_solver_->set_upper_bounds(ub);
          }

          if(opt_switch_RollPitchDist_ == true)
          {
            // maximaize Roll Pitch Dists
            nl_solver_ = boost::make_shared<nlopt::opt>(nlopt::LN_COBYLA, joint_size);
            nl_solver_->set_max_objective(maximizeRollPitchDistsMin, this);
            //nl_solver_->add_inequality_constraint(xxxFunc, this, 1e-8); // TODO: please add some constraints if necessary
            nl_solver_->set_xtol_rel(1e-4); //1e-4
            nl_solver_->set_maxeval(1000); // 1000 times
            // lb and ub
            std::vector<double> lb(joint_size, -delta_angle_range_);
            std::vector<double> ub(joint_size, delta_angle_range_);
            nl_solver_->set_lower_bounds(lb);
            nl_solver_->set_upper_bounds(ub);
          }

        }

      std::vector<double> delta_angles(joint_size, 0);

      if(opt_switch_RollPitchDist_ == false)
      {
        nlopt::result result = nl_solver_->optimize(delta_angles, max_fctmin_);
        if (result > 0)
          {
            opt_joint_angles_ = ref_joint_angles_; // reset
            for (size_t i = 0; i < joint_size; i++)
              {
                opt_joint_angles_.position.at(i) += delta_angles.at(i); 
              }
          }
        else
          {
            ROS_WARN("[NLOPT] cannot solve the optimization problem, error code is %d", result);
            // please check https://nlopt.readthedocs.io/en/latest/NLopt_Reference/#return-values
          }        
        if (max_fctmin_ < fc_t_min_thresh_)
          {
            ROS_WARN("[NLOPT] max FCTMIN of the optimal pose is still lower than the threshold (%f < %f), so do not change pose", max_fctmin_, fc_t_min_thresh_);
          }
      }

      if(opt_switch_RollPitchDist_ == true)
      {
        nlopt::result result = nl_solver_->optimize(delta_angles, max_fcrpmin_);
        if (result > 0)
          {
            opt_joint_angles_ = ref_joint_angles_; // reset
            for (size_t i = 0; i < joint_size; i++)
              {
                opt_joint_angles_.position.at(i) += delta_angles.at(i); 
              }
          }
        else
          {
            ROS_WARN("[NLOPT] cannot solve the optimization problem, error code is %d", result);
            // please check https://nlopt.readthedocs.io/en/latest/NLopt_Reference/#return-values
          }        
        if (max_fcrpmin_ < fc_rp_min_thre_)
          {
            ROS_WARN("[NLOPT] max FCRPMIN of the optimal pose is still lower than the threshold (%f < %f), so do not change pose", max_fcrpmin_, fc_rp_min_thre_);
          }
      } 
    }

  if(opt_joint_angles_.position.size() == 0) return;

  Max_fctmin_msg.data = max_fctmin_;
  Max_fcrpmin_msg.data = max_fcrpmin_;

  // publish the joint state
  opt_joint_angles_.header.stamp = ros::Time::now();
  opt_joint_angle_pub_.publish(opt_joint_angles_);
  max_fctmin_pub_.publish(Max_fctmin_msg);
  max_fcrpmin_pub_.publish(Max_fcrpmin_msg);

}

int main (int argc, char **argv)
{
  ros::init (argc, argv, "optimize_pose_planner");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  OptimizePosePlanner planner(nh, nhp);
  ros::spin();

  return 0;
}





