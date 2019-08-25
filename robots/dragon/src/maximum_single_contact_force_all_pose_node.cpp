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


int main (int argc, char **argv)
{
  ros::init (argc, argv, "grasping_motion");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  double max_root_roll;
  double min_root_roll;
  double max_root_pitch;
  double min_root_pitch;
  double max_root_yaw;
  double min_root_yaw;
  double increment;
  bool verbose;

  nhp.param("max_root_roll", max_root_roll, 0.0);
  nhp.param("min_root_roll", min_root_roll, 0.0);
  nhp.param("max_root_pitch", max_root_pitch, 0.0);
  nhp.param("min_root_pitch", min_root_pitch, 0.0);
  nhp.param("max_root_yaw", max_root_yaw, 0.0);
  nhp.param("min_root_yaw", min_root_yaw, 0.0);
  nhp.param("increment", increment, 0.1);
  nhp.param("verbose", verbose, false);

  boost::shared_ptr<DragonRobotModel> robot_model_ptr(new DragonRobotModel(true));
  MaximumSingleContact* max_single_contact_force_node = new MaximumSingleContact(nh, nhp, robot_model_ptr);

  std::vector<double> max_joint_angle_vec, min_joint_angle_vec;
  if(nhp.hasParam("max_joint_angle_vec"))
    {
      nhp.getParam("max_joint_angle_vec", max_joint_angle_vec);
      if(max_joint_angle_vec.size() != 2 * (robot_model_ptr->getRotorNum() -1))
        {
          ROS_ERROR_STREAM("the vector size for max_joint_ange_vec is wrong: " << max_joint_angle_vec.size());
          return 0;
        }
    }
  else
    {
      max_joint_angle_vec.resize(2 * (robot_model_ptr->getRotorNum() -1), M_PI/2);
    }

  if(nhp.hasParam("min_joint_angle_vec"))
    {
      nhp.getParam("min_joint_angle_vec", min_joint_angle_vec);
      if(min_joint_angle_vec.size() != 2 * (robot_model_ptr->getRotorNum() -1))
        {
          ROS_ERROR_STREAM("the vector size for min_joint_ange_vec is wrong: " << min_joint_angle_vec.size());
          return 0;
        }
    }
  else
    {
      min_joint_angle_vec.resize(2 * (robot_model_ptr->getRotorNum() -1), -M_PI/2);
    }


  sensor_msgs::JointState joint_state;
  for(int i = 1; i < robot_model_ptr->getRotorNum(); i++)
    {
      joint_state.name.push_back(std::string("joint") + std::to_string(i) + std::string("_yaw"));
      joint_state.name.push_back(std::string("joint") + std::to_string(i) + std::string("_pitch"));
      joint_state.position.push_back(min_joint_angle_vec.at(2* (i-1)));
      joint_state.position.push_back(min_joint_angle_vec.at(2* (i-1) + 1));
    }
  sensor_msgs::JointState joint_state_max_up_force, joint_state_max_down_force, joint_state_max_level_force;

  double roll = min_root_roll;
  double pitch = min_root_pitch;
  double yaw = min_root_yaw;
  double roll_max_up_force, pitch_max_up_force, yaw_max_up_force;
  double roll_max_down_force, pitch_max_down_force, yaw_max_down_force;
  double roll_max_level_force, pitch_max_level_force, yaw_max_level_force;

  double up_force, down_force, level_force;
  double max_up_force = 0;
  double max_down_force = 0;
  double max_level_force = 0;

  uint64_t max_cnt = 1;
  max_cnt *= ((max_root_yaw - min_root_yaw) / increment + 1);
  //ROS_WARN("max_cnt: %ld", max_cnt);
  max_cnt *= ((max_root_pitch - min_root_pitch) / increment +1);
  //ROS_WARN("max_cnt: %ld", max_cnt);
  max_cnt *= ((max_root_roll - min_root_roll) / increment + 1);
  //ROS_WARN("max_cnt: %ld", max_cnt);
  for(int i = 0; i < joint_state.position.size(); i ++)
    {
      max_cnt *= ((max_joint_angle_vec.at(i) - min_joint_angle_vec.at(i)) / increment + 1);
      //ROS_WARN("max_cnt: %ld", max_cnt);
    }
  ROS_WARN_STREAM("max count is " << max_cnt);
  uint64_t cnt = 1;
  double t = ros::Time::now().toSec();

  while(ros::ok())
    {
      if(verbose)
        {
          std::cout << "Root att(rpy): [" << roll << ", " << pitch << ", " << yaw << "], " ;
          std::cout << "joint angles: [";

          for(const auto& angle: joint_state.position)
            std::cout << angle << ", ";
          std::cout << "]" << std::endl;
        }
      else
        {
          if(ros::Time::now().toSec() - t > 10.0) //sec
            {
              t = ros::Time::now().toSec();
              //std::cout << "\r" << "Process: " << (double)cnt/max_cnt*100 << "\%" ;
              std::cout << "Process: " << (double)cnt/(double)max_cnt*100 << "\%" << std::endl;
            }
        }

      if(max_single_contact_force_node->getMaximumSingleContactForce("link1", roll, pitch, yaw, joint_state, up_force, down_force, level_force))
        {
          if(up_force > max_up_force )
            {
              max_up_force = up_force;
              roll_max_up_force = roll;
              pitch_max_up_force = pitch;
              yaw_max_up_force = yaw;
              joint_state_max_up_force = joint_state;
            }
          if(down_force > max_down_force )
            {
              max_down_force = down_force;
              roll_max_down_force = roll;
              pitch_max_down_force = pitch;
              yaw_max_down_force = yaw;
              joint_state_max_down_force = joint_state;
            }
          if(level_force > max_level_force )
            {
              max_level_force = level_force;
              roll_max_level_force = roll;
              pitch_max_level_force = pitch;
              yaw_max_level_force = yaw;
              joint_state_max_level_force = joint_state;
            }
        }
#if 0
      // debug
      {
        /* root att increament */
        roll += increment;
        if(roll > max_root_roll) pitch += increment;
        if(pitch > max_root_pitch) yaw += increment;

        /* check the finish condition */
        if(roll > max_root_roll && pitch > max_root_pitch && yaw > max_root_yaw)
          break;

        /* reset the root att and joint angle */
        if(roll > max_root_roll) roll = min_root_roll;
        if(pitch > max_root_pitch) pitch = min_root_pitch;
        continue;
      }
#endif

      /* joint angle increment */
      joint_state.position.at(0) += increment;
      for(int index = 0; index < joint_state.position.size() - 1; index++)
        if(joint_state.position.at(index) > max_joint_angle_vec.at(index)) joint_state.position.at(index+1) += increment;

      bool stop_joint_search = true;
      for(int index = 0; index < joint_state.position.size(); index++)
        if(joint_state.position.at(index) <= max_joint_angle_vec.at(index)) stop_joint_search = false;
      if(stop_joint_search)
        {
          /* root att increament */
          roll += increment;
          if(roll > max_root_roll) pitch += increment;
          if(pitch > max_root_pitch) yaw += increment;
        }

      /* check the finish condition */
      if(roll > max_root_roll && pitch > max_root_pitch && yaw > max_root_yaw && stop_joint_search)
        break;

      /* reset the root att and joint angle */
      if(roll > max_root_roll) roll = min_root_roll;
      if(pitch > max_root_pitch) pitch = min_root_pitch;
      //if(yaw > max_root_yaw) yaw = min_root_yaw; //not neccesary
      for(int index = 0; index < joint_state.position.size(); index++)
        {
          if(joint_state.position.at(index) > max_joint_angle_vec.at(index))
            joint_state.position.at(index) = min_joint_angle_vec.at(index);
        }

      cnt++;
    };
  std::cout << std::endl;

  ROS_INFO("Result:");

  std::cout << "Max up force:" << max_up_force << std::endl;
  std::cout << "Root att(rpy): [" << roll_max_up_force << ", " << pitch_max_up_force << ", " << yaw_max_up_force << "]" << std::endl;
  std::cout << "Joint angles: \n";
  for(int i = 0; i < joint_state_max_up_force.position.size(); i++)
    {
      std::cout << "[" << joint_state_max_up_force.name.at(i) << ": " << joint_state_max_up_force.position.at(i) << "]; ";
    }
  std::cout << std::endl;

  std::cout << "Max down force:" << max_down_force << std::endl;
  std::cout << "Root att(rpy): [" << roll_max_down_force << ", " << pitch_max_down_force << ", " << yaw_max_down_force << "]" << std::endl;
  std::cout << "Joint angles: \n";
  for(int i = 0; i < joint_state_max_down_force.position.size(); i++)
    {
      std::cout << "[" << joint_state_max_down_force.name.at(i) << ": " << joint_state_max_down_force.position.at(i) << "]; ";
    }
  std::cout << std::endl;

  std::cout << "Max level force:" << max_level_force << std::endl;
  std::cout << "Root att(rpy): [" << roll_max_level_force << ", " << pitch_max_level_force << ", " << yaw_max_level_force << "]" << std::endl;
  std::cout << "Joint angles: \n";
  for(int i = 0; i < joint_state_max_level_force.position.size(); i++)
    {
      std::cout << "[" << joint_state_max_level_force.name.at(i) << ": " << joint_state_max_level_force.position.at(i) << "]; ";
    }
  std::cout << std::endl;

  delete max_single_contact_force_node;
  return 0;
}





