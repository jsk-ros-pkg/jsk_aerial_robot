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


#include <aerial_robot_model/servo_bridge.h>

ServoBridge::ServoBridge(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh),nhp_(nhp)
{
  nh_.param("/use_sim_time", simulation_mode_, false);

  if(simulation_mode_)
    {
      if(!ros::service::waitForService(nh_.getNamespace() + std::string("/controller_manager/load_controller")))
        {
          ROS_ERROR("can't find the controller manager: %s", (nh_.getNamespace() + std::string("/controller_manager/load_controller")).c_str());
          return;
        }
    }

  /* get robot URDF model via robot_description*/
  urdf::Model urdf_model;
  if (!urdf_model.initParam("robot_description"))
    ROS_ERROR("Failed to extract urdf model from rosparam");

  /* get additional config for servos from ros parameters */
  XmlRpc::XmlRpcValue all_servos_params;
  nh_.getParam("servo_controller", all_servos_params);

  for(auto servo_group_params: all_servos_params)
    {
      ROS_ASSERT(servo_group_params.second.getType() == XmlRpc::XmlRpcValue::TypeStruct);

      ServoGroupHandler servo_group_handler;
      for(auto servo_params : servo_group_params.second)
        {
          if(servo_params.first.find("controller") != string::npos)
            {
              /* get parameters from urdf file */
              double upper_limit = urdf_model.getJoint(servo_params.second["name"])->limits->upper;
              double lower_limit = urdf_model.getJoint(servo_params.second["name"])->limits->lower;

              /* get parameters from rosparam */
              int angle_sgn = servo_params.second.hasMember("angle_sgn")?
                servo_params.second["angle_sgn"]:servo_group_params.second["angle_sgn"];
              int zero_point_offset = servo_params.second.hasMember("zero_point_offset")?
                servo_params.second["zero_point_offset"]:servo_group_params.second["zero_point_offset"];
              double angle_scale = servo_params.second.hasMember("angle_scale")?
                servo_params.second["angle_scale"]:servo_group_params.second["angle_scale"];

              double torque_scale = servo_group_params.second.hasMember("torque_scale")?
                servo_group_params.second["torque_scale"]:(servo_params.second.hasMember("torque_scale")?servo_params.second["torque_scale"]: XmlRpc::XmlRpcValue(1.0));

              /* for low pass filtering */
              bool filter_flag = servo_group_params.second.hasMember("filter_flag")?
                servo_group_params.second["filter_flag"]:(servo_params.second.hasMember("filter_flag")?servo_params.second["filter_flag"]: XmlRpc::XmlRpcValue(false));
              double sample_freq = servo_group_params.second.hasMember("sample_freq")?
                servo_group_params.second["sample_freq"]:(servo_params.second.hasMember("sample_freq")?servo_params.second["sample_freq"]: XmlRpc::XmlRpcValue(0.0));
              double cutoff_freq = servo_group_params.second.hasMember("cutoff_freq")?
                servo_group_params.second["cutoff_freq"]:(servo_params.second.hasMember("cutoff_freq")?servo_params.second["cutoff_freq"]: XmlRpc::XmlRpcValue(0.0));

              servo_group_handler.push_back(SingleServoHandlePtr(new SingleServoHandle(servo_params.second["name"], servo_params.second["id"], angle_sgn, zero_point_offset, angle_scale, upper_limit, lower_limit, torque_scale, servo_group_params.second.hasMember("state_sub_topic"), filter_flag, sample_freq, cutoff_freq)));

              /* rosparam and load controller for gazebo */
              if(simulation_mode_)
                {
                  if(!servo_group_params.second.hasMember("simulation") &&
                     !servo_params.second.hasMember("simulation"))
                    {
                      ROS_ERROR("please set gazebo servo parameters for %s, using sub namespace 'simulation:'", string(servo_params.second["name"]).c_str());

                      continue;
                    }

                  vector<string> parameter_names = {"type", "pid", "init_value"};
                  for(auto parameter_name : parameter_names)
                    {
                      if(!servo_params.second.hasMember("simulation") ||
                         (servo_params.second.hasMember("simulation") &&
                          !servo_params.second["simulation"].hasMember(parameter_name)))
                        {
                          if(!servo_group_params.second["simulation"].hasMember(parameter_name))
                            {
                              ROS_ERROR("can not find '%s' gazebo paramter for servo %s", parameter_name.c_str(),  string(servo_params.second["name"]).c_str());
                              return;
                            }
                          nh_.setParam(string("servo_controller/") + servo_group_params.first + string("/") + servo_params.first + string("/simulation/") + parameter_name, servo_group_params.second["simulation"][parameter_name]);
                        }
                    }
                  nh_.setParam(string("servo_controller/") + servo_group_params.first + string("/") + servo_params.first + string("/simulation/joint"), servo_params.second["name"]);


                  /* load the controller */
                  ros::ServiceClient controller_loader = nh_.serviceClient<controller_manager_msgs::LoadController>(nh_.getNamespace() + std::string("/controller_manager/load_controller"));
                  controller_manager_msgs::LoadController load_srv;
                  load_srv.request.name = nh_.getNamespace() + string("/servo_controller/") + servo_group_params.first + string("/") + servo_params.first + string("/simulation");

                  if (controller_loader.call(load_srv))
                    ROS_INFO("load gazebo controller for %s", string(servo_params.second["name"]).c_str());
                  else
                    ROS_ERROR("Failed to call service %s", controller_loader.getService().c_str());

                  /* start the controller */
                  ros::ServiceClient controller_starter = nh_.serviceClient<controller_manager_msgs::SwitchController>(nh_.getNamespace() + std::string("/controller_manager/switch_controller"));
                  controller_manager_msgs::SwitchController switch_srv;
                  switch_srv.request.start_controllers.push_back(load_srv.request.name);
                  switch_srv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;

                  if (controller_starter.call(switch_srv))
                    ROS_INFO("start servo controller for %s", string(servo_params.second["name"]).c_str());
                  else
                    ROS_ERROR("Failed to call service %s", controller_starter.getService().c_str());

                  /* init the servo command publisher to the controller */
                  servo_ctrl_sim_pubs_[servo_group_params.first].push_back(nh_.advertise<std_msgs::Float64>(load_srv.request.name + string("/command"), 1));
                  // wait for the publisher initialization
                  while(servo_ctrl_sim_pubs_[servo_group_params.first].back().getNumSubscribers() == 0 && ros::ok())
                    ros::Duration(0.1).sleep();

                  /* set the initial angle */
                  std_msgs::Float64 msg;
                  nh_.getParam(string("servo_controller/") + servo_group_params.first + string("/") + servo_params.first + string("/simulation/init_value"), msg.data);
                  servo_ctrl_sim_pubs_[servo_group_params.first].back().publish(msg);
                }
            }
        }

      servos_handler_.insert(make_pair(servo_group_params.first, servo_group_handler));

      /* ros pub/sub, service */
      /* Get servo states (i.e. joint angles) from real machine, if necessary */
      if(servo_group_params.second.hasMember("state_sub_topic"))
        servo_states_subs_.push_back(nh_.subscribe<spinal::ServoStates>((string)servo_group_params.second["state_sub_topic"], 10, boost::bind(&ServoBridge::servoStatesCallback, this, _1, servo_group_params.first)));

      /* subscribe target servo state from controller */
      servo_ctrl_subs_.push_back(nh_.subscribe<sensor_msgs::JointState>(servo_group_params.first + string("_ctrl"), 10, boost::bind(&ServoBridge::servoCtrlCallback, this, _1, servo_group_params.first)));
      /* publish target servo state to real machine */
      servo_ctrl_pubs_.insert(make_pair(servo_group_params.first, nh_.advertise<spinal::ServoControlCmd>(servo_group_params.second["ctrl_pub_topic"], 1)));

      /* torque on/off */
      if(servo_group_params.second.hasMember("torque_pub_topic"))
        {
          servo_torque_ctrl_srvs_.push_back(nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(servo_group_params.first + string("/torque_enable"), boost::bind(&ServoBridge::servoTorqueCtrlCallback, this, _1, _2, servo_group_params.first)));
          servo_torque_ctrl_pubs_.insert(make_pair(servo_group_params.first, nh_.advertise<spinal::ServoTorqueCmd>((string)servo_group_params.second["torque_pub_topic"], 1)));
        }
    }

  if(!simulation_mode_) servo_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

  /* create simple action server to support followjointtrajectory */
  as_ = std::make_shared<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>>(nh_, std::string("follow_joint_trajectory"), boost::bind(&ServoBridge::followJointTrajectoryCallback, this, _1), false);

  as_->start();
}

void ServoBridge::servoStatesCallback(const spinal::ServoStatesConstPtr& state_msg, const string& servo_group_name)
{
  if(state_msg->servos.size() != servos_handler_[servo_group_name].size())
    {
      ROS_ERROR("[servo bridge, servo state callback]: the joint num from rosparam %d is not equal with ros msgs %d", (int)servos_handler_[servo_group_name].size(), (int)state_msg->servos.size());
      return;
    }

  for(auto it: state_msg->servos)
    {
      auto servo_handler = find_if(servos_handler_[servo_group_name].begin(),
                                   servos_handler_[servo_group_name].end(),
                                   [&](SingleServoHandlePtr s) {return it.index == s->getId();} );

      if(servo_handler == servos_handler_[servo_group_name].end())
        {
          ROS_ERROR("[servo bridge, servo state callback]: no matching joint handler for servo index %d", it.index);
          return;
        }
      (*servo_handler)->setCurrAngleVal((double)it.angle, ValueType::BIT); // angle (position)
      (*servo_handler)->setCurrTorqueVal((double)it.load); // torque (effort)
    }

  sensor_msgs::JointState servo_states_msg;
  servo_states_msg.header.stamp = state_msg->stamp;

  for(auto servo_group : servos_handler_)
    {
      for(auto servo_handler: servo_group.second)
        {
          servo_states_msg.name.push_back(servo_handler->getName());
          servo_states_msg.position.push_back(servo_handler->getCurrAngleVal(ValueType::RADIAN));
          servo_states_msg.effort.push_back(servo_handler->getCurrTorqueVal());
        }
    }
  servo_states_pub_.publish(servo_states_msg);
}

void ServoBridge::servoCtrlCallback(const sensor_msgs::JointStateConstPtr& servo_ctrl_msg, const string& servo_group_name)
{
  spinal::ServoControlCmd target_angle_msg;

  if(servo_ctrl_msg->name.size() > 0)
    {
      for(int i = 0; i < servo_ctrl_msg->name.size(); i++)
        {/* servo name is assigned */

          if(servo_ctrl_msg->position.size() !=  servo_ctrl_msg->name.size())
            {
              ROS_ERROR("[servo bridge, servo control control]: the servo position num and name num are different in ros msgs [%d vs %d]",
                        (int)servo_ctrl_msg->position.size(), (int)servo_ctrl_msg->name.size());
              return;
            }

          // use servo_name to search the servo_handler
          auto servo_handler = find_if(servos_handler_[servo_group_name].begin(), servos_handler_[servo_group_name].end(),
                                       [&](SingleServoHandlePtr s) {return servo_ctrl_msg->name.at(i)  == s->getName();});

          if(servo_handler == servos_handler_[servo_group_name].end())
          {
            ROS_ERROR("[servo bridge, servo control callback]: no matching servo handler for %s", servo_ctrl_msg->name.at(i).c_str());
            return;
          }

          (*servo_handler)->setTargetAngleVal(servo_ctrl_msg->position[i], ValueType::RADIAN);
          target_angle_msg.index.push_back((*servo_handler)->getId());
          target_angle_msg.angles.push_back((*servo_handler)->getTargetAngleVal(ValueType::BIT));

          if(simulation_mode_)
            {
              std_msgs::Float64 msg;
              msg.data = servo_ctrl_msg->position[i];
              servo_ctrl_sim_pubs_[servo_group_name].at(distance(servos_handler_[servo_group_name].begin(), servo_handler)).publish(msg);
            }
        }
    }
  else
    { /* for fast tranmission: no searching process, in the predefine order */

      if(servo_ctrl_msg->position.size() != servos_handler_[servo_group_name].size())
        {
          ROS_ERROR("[servo bridge, servo control control]: the joint num from rosparam %d is not equal with ros msgs %d",
                    (int)servos_handler_[servo_group_name].size(), (int)servo_ctrl_msg->position.size());
          return;
        }

      for(int i = 0; i < servo_ctrl_msg->position.size(); i++)
        {
          /*  use the kinematics order (e.g. joint1 ~ joint N, gimbal_roll -> gimbal_pitch) */
          SingleServoHandlePtr servo_handler = servos_handler_[servo_group_name].at(i);
          servo_handler->setTargetAngleVal(servo_ctrl_msg->position[i], ValueType::RADIAN);
          target_angle_msg.index.push_back(servo_handler->getId());
          target_angle_msg.angles.push_back(servo_handler->getTargetAngleVal(ValueType::BIT));

          if(simulation_mode_)
            {
              std_msgs::Float64 msg;
              msg.data = servo_ctrl_msg->position[i];
              servo_ctrl_sim_pubs_[servo_group_name].at(i).publish(msg);
            }
        }
    }

  servo_ctrl_pubs_[servo_group_name].publish(target_angle_msg);
}

bool ServoBridge::servoTorqueCtrlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res, const std::string& servo_group_name)
{
  spinal::ServoTorqueCmd torque_off_msg;
  for(auto servo_handler: servos_handler_[servo_group_name])
    {
      torque_off_msg.index.push_back(servo_handler->getId());
      torque_off_msg.torque_enable.push_back(req.data);
    }

  servo_torque_ctrl_pubs_[servo_group_name].publish(torque_off_msg);

  return true;
}

void ServoBridge::followJointTrajectoryCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
  int num_points = goal->trajectory.points.size();

  // make sure the joints in the goal are in joints of the controller
  std::vector<SingleServoHandlePtr> servos;
  std::vector<std::string> groups;

  for(const auto& name : goal->trajectory.joint_names)
    {
      for(const auto& servo_group : servos_handler_)
        {
          auto servo_handler = find_if(servo_group.second.begin(), servo_group.second.end(),
                                       [&](SingleServoHandlePtr s) {return name  == s->getName();});

          if(servo_handler != servo_group.second.end())
            {
              servos.push_back(*servo_handler);
              groups.push_back(servo_group.first);
            }
        }
    }

  if(servos.size() != goal->trajectory.joint_names.size())
    {
      std::string msg("Incoming trajectory joints do not match the joints of the controller");
      control_msgs::FollowJointTrajectoryResult res;
      res.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      ROS_ERROR_STREAM(msg);
      as_->setAborted(res, msg);
      return;
    }

  // make sure trajectory is not empty
  if(num_points == 0)
    {
      std::string msg("Incoming trajectory is empty");
      control_msgs::FollowJointTrajectoryResult res;
      res.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      ROS_ERROR_STREAM(msg);
      as_->setAborted(res, msg);
      return;
    }

  // correlate the joints we're commanding to the joints in the message
  std::vector<double> durations (num_points, 0.0);

  // find out the duration of each segment in the trajectory
  durations.at(0) = goal->trajectory.points.at(0).time_from_start.toSec();
  for(int i = 1; i < num_points; i++)
    durations.at(i) = (goal->trajectory.points.at(i).time_from_start - goal->trajectory.points.at(i-1).time_from_start).toSec();

  if (goal->trajectory.points[0].positions.empty())
    {
      std::string msg("First point of trajectory has no positions");
      control_msgs::FollowJointTrajectoryResult res;
      res.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      ROS_ERROR_STREAM(msg);
      as_->setAborted(res, msg);
      return;
    }

  struct Segment
  {
    double start_time; // trajectory segment start time
    double end_time; // trajectory segment end time
    std::vector<double> positions;
    std::vector<double> velocities;

    Segment(int num_joints):
      start_time(0),
      end_time(0),
      positions(num_joints, 0),
      velocities(num_joints, 0)
    {}
  };

  std::vector<Segment> trajectory;
  ros::Time time = ros::Time::now() + ros::Duration(0.01);

  for(int i = 0; i < num_points; i++)
    {
      double seg_start_time = 0;
      double seg_duration = 0;
      Segment seg{(int)goal->trajectory.joint_names.size()};

      if(goal->trajectory.header.stamp == ros::Time(0.0))
        seg.start_time = (time + goal->trajectory.points.at(i).time_from_start).toSec() - durations.at(i);
      else
        seg.start_time = (goal->trajectory.header.stamp + goal->trajectory.points.at(i).time_from_start).toSec() - durations.at(i);

      seg.end_time = seg.start_time + durations.at(i);

      // Checks that the incoming segment has the right number of elements.
      if(goal->trajectory.points[i].velocities.size() > 0 && goal->trajectory.points[i].velocities.size() != goal->trajectory.joint_names.size())
        {
          std::string msg = std::string("Command point ") + std::to_string(i) + std::string(" has wrong amount of velocities");
          control_msgs::FollowJointTrajectoryResult res;
          res.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
          ROS_ERROR_STREAM(msg);
          as_->setAborted(res, msg);
          return;
        }

      if(goal->trajectory.points[i].positions.size() != goal->trajectory.joint_names.size())
        {
          std::string msg = std::string("Command point ") + std::to_string(i) + std::string(" has wrong amount of positions");
          control_msgs::FollowJointTrajectoryResult res;
          res.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
          ROS_ERROR_STREAM(msg);
          as_->setAborted(res, msg);
          return;
        }

      for(int j = 0; j < goal->trajectory.joint_names.size(); j++)
        {
          if(goal->trajectory.points.at(i).velocities.size() > 0)
            seg.velocities.at(j) = goal->trajectory.points.at(i).velocities.at(j);

          seg.positions.at(j) = goal->trajectory.points.at(i).positions.at(j);
        }

      trajectory.push_back(seg);
    }

  ROS_INFO("Trajectory start requested at %.3lf, waiting...", goal->trajectory.header.stamp.toSec());

  ros::Rate rate(1000);
  while(goal->trajectory.header.stamp > time)
    {
      time = ros::Time::now();
      rate.sleep();
    }

  double total_duration = std::accumulate(durations.begin(), durations.end(), 0);
  ROS_INFO("Trajectory start time is %.3lf, end time is %.3lf, total duration is %.3lf", time.toSec(),  time.toSec() + total_duration, total_duration);

  std::map <std::string, spinal::ServoControlCmd> target_angle_group_msg;
  for (const auto& servo_group : servos_handler_)
    target_angle_group_msg.insert(std::make_pair(servo_group.first, spinal::ServoControlCmd()));

  control_msgs::FollowJointTrajectoryFeedback feedback;
  feedback.joint_names = goal->trajectory.joint_names;
  feedback.header.stamp = time;
  feedback.desired.positions.resize(goal->trajectory.joint_names.size());
  feedback.actual.positions.resize(goal->trajectory.joint_names.size());
  feedback.error.positions.resize(goal->trajectory.joint_names.size());

  for(int seg = 0; seg < trajectory.size(); seg++)
    {
      ROS_DEBUG("current segment is %d time left %f cur time %f", seg, durations.at(seg) - (time.toSec() - trajectory.at(seg).start_time), time.toSec());

      if(durations.at(seg) == 0)
        {
          ROS_DEBUG("skipping segment %d with duration of 0 seconds", seg);
          continue;
        }

      for(int j = 0; j < goal->trajectory.joint_names.size(); j++)
        {
          auto servo = servos.at(j);
          auto servo_group_name = groups.at(j);

          servo->setTargetAngleVal(trajectory.at(seg).positions.at(j), ValueType::RADIAN);
          // TODO: currently, velocity is not supported in our system.

          auto& target_angle_msg = target_angle_group_msg.at(servo_group_name);
          target_angle_msg.index.push_back(servo->getId());
          target_angle_msg.angles.push_back(servo->getTargetAngleVal(ValueType::BIT));

          if(simulation_mode_)
            {
              std_msgs::Float64 msg;
              msg.data = trajectory.at(seg).positions.at(j);

              auto servo_group = servos_handler_.at(servo_group_name);
              auto servo_handler = find_if(servo_group.begin(), servo_group.end(),
                                           [&](SingleServoHandlePtr s) {return servo->getName()  == s->getName();});

              servo_ctrl_sim_pubs_.at(servo_group_name).at(distance(servo_group.begin(), servo_handler)).publish(msg);
            }
        }

      for(const auto& servo_group: target_angle_group_msg)
        {
          if(servo_group.second.index.size() > 0)
            servo_ctrl_pubs_[servo_group.first].publish(servo_group.second);
        }

      while(time.toSec() < trajectory.at(seg).end_time)
        {
          // check if new trajectory was received, if so abort current trajectory execution
          // by setting the goal to the current position
          if (as_->isPreemptRequested())
            {
              for(int j = 0; j < goal->trajectory.joint_names.size(); j++)
                {
                  auto servo = servos.at(j);
                  auto servo_group_name = groups.at(j);

                  servo->setTargetAngleVal(servo->getCurrAngleVal(ValueType::RADIAN), ValueType::RADIAN);

                  auto& target_angle_msg = target_angle_group_msg.at(servo_group_name);
                  target_angle_msg.index.push_back(servo->getId());
                  target_angle_msg.angles.push_back(servo->getTargetAngleVal(ValueType::BIT));

                  if(simulation_mode_)
                    {
                      std_msgs::Float64 msg;
                      msg.data = trajectory.at(seg).positions.at(j);

                      auto servo_group = servos_handler_.at(servo_group_name);
                      auto servo_handler = find_if(servo_group.begin(), servo_group.end(),
                                                   [&](SingleServoHandlePtr s) {return servo->getName()  == s->getName();});

                      servo_ctrl_sim_pubs_.at(servo_group_name).at(distance(servo_group.begin(), servo_handler)).publish(msg);
                    }
                }

              for(const auto& servo_group: target_angle_group_msg)
                {
                  if(servo_group.second.index.size() > 0)
                    servo_ctrl_pubs_[servo_group.first].publish(servo_group.second);
                }

              as_->setPreempted();
              if(as_->isNewGoalAvailable())
                ROS_WARN("New trajectory received. Aborting old trajectory.");
              else
                ROS_WARN("Canceled trajectory following action");
              return;
            }

          if((time - feedback.header.stamp).toSec() > 0.1) // 10 Hz
            {
              feedback.header.stamp = time;
              for(int j = 0; j < goal->trajectory.joint_names.size(); j++)
                {
                  const auto servo = servos.at(j);
                  feedback.desired.positions.at(j) = servo->getTargetAngleVal(ValueType::RADIAN);
                  feedback.actual.positions.at(j) = servo->getCurrAngleVal(ValueType::RADIAN);
                  feedback.error.positions.at(j) = feedback.desired.positions.at(j) - feedback.actual.positions.at(j);
                }
              as_->publishFeedback(feedback);
            }

          rate.sleep();
          time = ros::Time::now();
        }

      // Verify trajectory constraints
      for(int j = 0; j < goal->trajectory.joint_names.size(); j++)
        {
          const auto servo = servos.at(j);
          auto tol = find_if(goal->path_tolerance.begin(), goal->path_tolerance.end(),
                             [&](control_msgs::JointTolerance s) {return servo->getName() == s.name;});

          if(tol == goal->path_tolerance.end()) continue;

          double pos_err = fabs(servo->getTargetAngleVal(ValueType::RADIAN) - servo->getCurrAngleVal(ValueType::RADIAN));
          double pos_tol = tol->position;
          if(pos_tol > 0 && pos_err > pos_tol)
            {
              std::string msg = std::string("Unsatisfied position tolerance for ") + servo->getName() + std::string(", trajectory point") \
              + std::to_string(seg) + std::string(", ") + std::to_string(pos_err) + std::string(" is larger than ") \
              + std::to_string(pos_tol);

              ROS_WARN_STREAM(msg);
              control_msgs::FollowJointTrajectoryResult res;
              res.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
              as_->setAborted(res, msg);
              return;
            }
        }
    }

  for(int j = 0; j < goal->trajectory.joint_names.size(); j++)
    {
      const auto servo = servos.at(j);
      ROS_DEBUG("desired pos was %f, actual pos is %f, error is %f", trajectory.back().positions.at(j), servo->getCurrAngleVal(ValueType::RADIAN),
                servo->getCurrAngleVal(ValueType::RADIAN) - servo->getCurrAngleVal(ValueType::RADIAN));
    }

  // Checks that we have ended inside the goal constraints
  for(int j = 0; j < goal->trajectory.joint_names.size(); j++)
    {
      const auto servo = servos.at(j);
      auto tol = find_if(goal->path_tolerance.begin(), goal->path_tolerance.end(),
                         [&](control_msgs::JointTolerance s) {return servo->getName() == s.name;});
      if(tol == goal->goal_tolerance.end()) continue;

      double pos_err = fabs(servo->getTargetAngleVal(ValueType::RADIAN) - servo->getCurrAngleVal(ValueType::RADIAN));
      double pos_tol = tol->position;

      if(pos_tol > 0 && pos_err > pos_tol)
        {
          std::string msg = std::string("Aborting because ") + servo->getName() + std::string(" wound up outside the goal constraints, ") +
            std::to_string(pos_err) + std::string(" is larger than ") + std::to_string(pos_tol);

          ROS_WARN_STREAM(msg);
          control_msgs::FollowJointTrajectoryResult res;
          res.error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
          as_->setAborted(res, msg);
          return;
        }
    }

  std::string msg("Trajectory execution successfully completed");
  ROS_INFO_STREAM(msg);

  control_msgs::FollowJointTrajectoryResult res;
  res.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
  as_->setSucceeded(res, msg);
}


