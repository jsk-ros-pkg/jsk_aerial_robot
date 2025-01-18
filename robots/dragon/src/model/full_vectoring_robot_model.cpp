// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, JSK Lab
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

#include <dragon/model/full_vectoring_robot_model.h>

using namespace Dragon;

namespace
{
  double minimumControlWrench(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
  {
    FullVectoringRobotModel *model = reinterpret_cast<FullVectoringRobotModel*>(ptr);
    int rotor_num = model->getRotorNum();
    std::vector<Eigen::Matrix3d> link_rot = model->getLinksRotationFromCog<Eigen::Matrix3d>();
    std::vector<Eigen::Vector3d> gimbal_roll_pos = model->getGimbalRollOriginFromCog<Eigen::Vector3d>();
    auto model_for_plan = model->getRobotModelForPlan();
    std::vector<Eigen::Vector3d> rotor_pos = model_for_plan->getRotorsOriginFromCog<Eigen::Vector3d>();

    std::vector<int> roll_locked_gimbal = model->getRollLockedGimbalForPlan();
    for(int i = 0; i < rotor_num; i++)
    {
      /*
         Since the position of the force acting point would change according to the gimbal roll,
there is a diffiretial chain about the roll angle. But we here approximate it to the gimbal roll frame along the link axis, rather than the true force acting point (gimbal pitch frame). The offset is 0.037m, which is a small offset in the optimization problem.
       */
      if(roll_locked_gimbal.at(i) == 1) rotor_pos.at(i) = gimbal_roll_pos.at(i);
    }

    const auto f_min_list = model->calcFeasibleControlFxyDists(roll_locked_gimbal, x, rotor_num, link_rot);
    const auto t_min_list = model->calcFeasibleControlTDists(roll_locked_gimbal, x, rotor_num, rotor_pos, link_rot);

    return model->getMinForceNormalizedWeight() * f_min_list.minCoeff() +  model->getMinTorqueNormalizedWeight() * t_min_list.minCoeff();
  }
}

FullVectoringRobotModel::FullVectoringRobotModel(bool init_with_rosparam, bool verbose, double edf_radius, double edf_max_tilt) :
  HydrusLikeRobotModel(init_with_rosparam, verbose, 0, 0, 10, edf_radius, edf_max_tilt)
{
  if (init_with_rosparam)
    {
      getParamFromRos();
    }

  /* gimbal roll lock */
  const int rotor_num = getRotorNum();
  roll_lock_status_accumulator_.resize(rotor_num, 0);
  roll_lock_angle_smooth_.resize(rotor_num, 0);
  prev_roll_locked_gimbal_.resize(rotor_num, 0);
  roll_locked_gimbal_.resize(rotor_num, 0);
  gimbal_roll_origin_from_cog_.resize(rotor_num);
  setGimbalNominalAngles(std::vector<double>(0)); // for online initialize

  robot_model_for_plan_ = boost::make_shared<aerial_robot_model::transformable::RobotModel>();

  if(debug_verbose_)
    {
      if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
        ros::console::notifyLoggerLevelsChanged();
    }
}

void FullVectoringRobotModel::getParamFromRos()
{
  ros::NodeHandle nh;

  nh.param("debug_verbose", debug_verbose_, false);

  nh.param("gimbal_lock_threshold", gimbal_lock_threshold_, 0.3); // > 10deg
  nh.param("link_att_change_threshold", link_att_change_threshold_, 0.2); // 10deg
  nh.param("lock_status_change_threshold", lock_status_change_threshold_, 10); // 10deg
  nh.param("gimbal_delta_angle", gimbal_delta_angle_, 0.3); // 10deg
  nh.param("robot_model_refine_max_iteration", robot_model_refine_max_iteration_, 1);
  nh.param("robot_model_refine_threshold", robot_model_refine_threshold_, 0.01); // m
  nh.param("gimbal_roll_change_threshold", gimbal_roll_change_threshold_, 0.02); // rad/s
  nh.param("min_force_weight", min_force_weight_, 1.0);
  nh.param("min_torque_weight", min_torque_weight_, 1.0);
}

void FullVectoringRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  KDL::Rotation cog_desire_orientation = getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_plan_->setCogDesireOrientation(cog_desire_orientation); // update the cog orientation

  /* 1. first assume the gimbals are level */
  KDL::TreeFkSolverPos_recursive fk_solver(getTree());
  KDL::Frame f_baselink;
  fk_solver.JntToCart(joint_positions, f_baselink, getBaselinkName());
  const KDL::Rotation cog_rot = f_baselink.M * cog_desire_orientation.Inverse();

  const auto joint_index_map = getJointIndexMap();
  KDL::JntArray gimbal_processed_joint = joint_positions;
  std::vector<double> gimbal_nominal_angles(0);
  std::vector<KDL::Rotation> links_rotation_from_cog(0);
  std::vector<int> roll_locked_gimbal(getRotorNum(), 0);

  for(int i = 0; i < getRotorNum(); ++i)
    {
      std::string s = std::to_string(i + 1);
      KDL::Frame f;
      fk_solver.JntToCart(joint_positions, f, std::string("link") + s);

      links_rotation_from_cog.push_back(cog_rot.Inverse() * f.M);
      double r, p, y;
      links_rotation_from_cog.back().GetRPY(r, p, y);

      gimbal_nominal_angles.push_back(-r);
      gimbal_nominal_angles.push_back(-p);
    }
  setLinksRotationFromCog(links_rotation_from_cog);

  // online initialization
  if(getGimbalNominalAngles().size() == 0)
    {
      for(int i = 0; i < getRotorNum(); ++i)
        {
          std::string s = std::to_string(i + 1);
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_roll"))->second) = gimbal_nominal_angles.at(i * 2);
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = gimbal_nominal_angles.at(i * 2 + 1);
        }
      robot_model_for_plan_->updateRobotModel(gimbal_processed_joint);

      setGimbalNominalAngles(gimbal_nominal_angles);
    }

  /* 2. check the orientation (pitch angle) of link to decide whether to lock gimbal roll */
  // TODO: we should not use the link pitch angle to decide, should use the angle between link and gimbal (thus means the gimbal pitch angle)
  double max_pitch = 0;
  bool roll_lock_status_change = false;
  for(int i = 0; i < getRotorNum(); ++i)
    {
      if(fabs(gimbal_nominal_angles.at(i * 2 + 1)) > max_pitch)
        max_pitch = fabs(gimbal_nominal_angles.at(i * 2 + 1));

      if(fabs(fabs(gimbal_nominal_angles.at(i * 2 + 1)) - M_PI /2) < gimbal_lock_threshold_)
        {
          ROS_DEBUG_STREAM_NAMED("robot_model", "link" << i+1 << " pitch: " << -gimbal_nominal_angles.at(i * 2 + 1) << " exceeds");
          roll_locked_gimbal.at(i) = 1;
        }
      else
        {
          roll_locked_gimbal.at(i) = 0;
        }

      /* case1: the locked rotors have change  */
      if(prev_roll_locked_gimbal_.at(i) != roll_locked_gimbal.at(i))
        {
          ROS_DEBUG_STREAM_NAMED("robot_model", "roll locked status change, gimbal " << i+1 <<" , cnt: " << roll_lock_status_accumulator_.at(i));

          /* robust1: deal with the chattering around the changing bound by adding momentum */
          roll_lock_status_accumulator_.at(i)++;
          if(roll_lock_status_accumulator_.at(i) > lock_status_change_threshold_)
            {
              roll_lock_status_accumulator_.at(i) = 0;
              roll_lock_status_change = true;
              ROS_INFO_STREAM_NAMED("robot_model", "rotor " << i + 1 << ": the gimbal roll lock status changes from " << prev_roll_locked_gimbal_.at(i) << " to " << roll_locked_gimbal.at(i));
              if(roll_locked_gimbal.at(i) == 0)
                {
                  roll_lock_angle_smooth_.at(i) = 1;
                }
            }
          else
            {
              /* force change back the roll lock status */
              roll_locked_gimbal.at(i) = prev_roll_locked_gimbal_.at(i);
            }
        }
      else
        {
          roll_lock_status_accumulator_.at(i) = 0;
        }

    }
  ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0, "robot_model", "max link pitch: " << max_pitch);

  /* 3. calculate the optimized locked gimbal roll angles */
  int gimbal_lock_num = std::accumulate(roll_locked_gimbal.begin(), roll_locked_gimbal.end(), 0);

  if(gimbal_lock_num > 0)
    {
      /* case2: the link attitude change more than a threshold  */
      /* robust: deal with the vibration of the link orientation from joint angles and cog attitude, not update the gimbal roll lock angles so often */
      for(int i = 0; i < getRotorNum(); i++)
        {
          tf::Quaternion delta_q;
          tf::quaternionKDLToTF(prev_links_rotation_from_cog_.at(i).Inverse() * links_rotation_from_cog.at(i), delta_q);
          if(fabs(delta_q.getAngle()) > link_att_change_threshold_)
            {
              ROS_INFO_STREAM_NAMED("robot_model", "link " << i + 1 << ": the orientation is change more than threshold: " << delta_q.getAngle());
              roll_lock_status_change = true;
            }
        }

      if(roll_lock_status_change)
        {
          /* roughly update the CoG and gimbal roll origin based on the level (horizontal) nominal gimbal angles and joints for search the optimiazed locked gimbal roll angles*/
          for(int i = 0; i < getRotorNum(); ++i)
            {
              std::string s = std::to_string(i + 1);
              gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_roll"))->second) = gimbal_nominal_angles.at(i * 2);
              gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = gimbal_nominal_angles.at(i * 2 + 1);
            }
          robot_model_for_plan_->updateRobotModel(gimbal_processed_joint);
          std::map<std::string, KDL::Frame> seg_tf_map = robot_model_for_plan_->getSegmentsTf();
          for(int i = 0; i < getRotorNum(); ++i)
            {
              std::string s = std::to_string(i + 1);
              std::string gimbal_roll = std::string("gimbal") + s + std::string("_roll_module");
              KDL::Frame f  = seg_tf_map.at(gimbal_roll);
              gimbal_roll_origin_from_cog_.at(i) = (robot_model_for_plan_->getCog<KDL::Frame>().Inverse() * f).p;
            }

          setRollLockedGimbalForPlan(roll_locked_gimbal);
          locked_angles_ = calcBestLockGimbalRoll(roll_locked_gimbal, prev_roll_locked_gimbal_, locked_angles_);
          prev_roll_locked_gimbal_ = roll_locked_gimbal;
          prev_links_rotation_from_cog_ = links_rotation_from_cog;
        }
    }
  else
    {
      locked_angles_.resize(0);
      prev_roll_locked_gimbal_ = roll_locked_gimbal;
      prev_links_rotation_from_cog_ = links_rotation_from_cog;
    }

  /* 4: smooth the nominal gimbal angles, to avoid sudden change */
  int gimbal_lock_index = 0;
  std::vector<double>  gimbal_nominal_angles_curr = getGimbalNominalAngles();
  for(int i = 0; i < getRotorNum(); ++i)
    {
      /* only smooth roll angles */
      if(roll_locked_gimbal.at(i) == 0)
        {
          if(roll_lock_angle_smooth_.at(i) == 1)
            {
              ROS_DEBUG_NAMED("robot_model", "smooth the gimbal roll angle %d, for after free the lock", i+1);
              roll_locked_gimbal.at(i) = 1; // force change back to lock status

              if(gimbal_nominal_angles.at(i * 2) - gimbal_nominal_angles_curr.at(i * 2) > gimbal_roll_change_threshold_)
                gimbal_nominal_angles_curr.at(i * 2) += gimbal_roll_change_threshold_;
              else if(gimbal_nominal_angles.at(i * 2) - gimbal_nominal_angles_curr.at(i * 2) < - gimbal_roll_change_threshold_)
                gimbal_nominal_angles_curr.at(i * 2) -= gimbal_roll_change_threshold_;
              else
                {
                  gimbal_nominal_angles_curr.at(i * 2) = gimbal_nominal_angles.at(i * 2);
                  roll_lock_angle_smooth_.at(i) = 0; // stop smoothing
                  if(debug_verbose_) ROS_INFO_STREAM_NAMED("robot_model", "free the gimbal roll after the smoothing for rotor" << i+1);
                }

              if(debug_verbose_) ROS_DEBUG_STREAM_NAMED("robot_model", "smoothing rotor " << i + 1 << ", desired roll angle: " << gimbal_nominal_angles.at(i * 2) << ", curr angle: " << gimbal_nominal_angles_curr.at(i * 2));

            }
          else
            {
              gimbal_nominal_angles_curr.at(i * 2) = gimbal_nominal_angles.at(i * 2);
            }
        }
      else
        {
          assert(roll_lock_angle_smooth_.at(i) == 1);

          if(locked_angles_.at(gimbal_lock_index) - gimbal_nominal_angles_curr.at(i * 2) > gimbal_roll_change_threshold_)
            gimbal_nominal_angles_curr.at(i * 2) += gimbal_roll_change_threshold_;
          else if(locked_angles_.at(gimbal_lock_index) - gimbal_nominal_angles_curr.at(i * 2) < - gimbal_roll_change_threshold_)
            gimbal_nominal_angles_curr.at(i * 2) -= gimbal_roll_change_threshold_;
          else
            gimbal_nominal_angles_curr.at(i * 2) = locked_angles_.at(gimbal_lock_index);

          if(debug_verbose_) ROS_DEBUG_STREAM_NAMED("robot_model", "smoothing rotor " << i + 1 << ", desired roll angle: " << locked_angles_.at(gimbal_lock_index) << ", curr angle: " << gimbal_nominal_angles_curr.at(i * 2));

          gimbal_lock_index++;
        }
      gimbal_nominal_angles_curr.at(i * 2 + 1) = gimbal_nominal_angles.at(i * 2 + 1);
    }
  gimbal_lock_num = std::accumulate(roll_locked_gimbal.begin(), roll_locked_gimbal.end(), 0); // update

  /* 5: (new) refine the rotor origin from cog */
  /* 5.1. init update the CoG and gimbal roll origin based on the level (horizontal) nominal gimbal angles and joints */
  for(int i = 0; i < getRotorNum(); ++i)
    {
      std::string s = std::to_string(i + 1);
      gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_roll"))->second) = gimbal_nominal_angles_curr.at(i * 2);
      gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = gimbal_nominal_angles_curr.at(i * 2 + 1);
    }
  robot_model_for_plan_->updateRobotModel(gimbal_processed_joint);

  /* 5.2. convergence  */
  double t = ros::Time::now().toSec();
  for(int j = 0; j < robot_model_refine_max_iteration_; j++)
    {
      /* 5.2.1. update the wrench allocation matrix  */
      std::vector<Eigen::Vector3d> rotors_origin_from_cog = robot_model_for_plan_->getRotorsOriginFromCog<Eigen::Vector3d>();
      Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 3 * getRotorNum() - gimbal_lock_num);
      Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
      wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
      Eigen::MatrixXd mask(3,2);
      mask << 1, 0, 0, 0, 0, 1;
      int last_col = 0;
      for(int i = 0; i < getRotorNum(); i++)
        {
          wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i));

          if(roll_locked_gimbal.at(i) == 0)
            {
              /* 3DoF */
              full_q_mat.block(0, last_col, 6, 3) = wrench_map * aerial_robot_model::kdlToEigen(links_rotation_from_cog.at(i));
              last_col += 3;
            }
          else
            {
              /* gimbal lock: 2Dof */
              full_q_mat.block(0, last_col, 6, 2) = wrench_map * aerial_robot_model::kdlToEigen(links_rotation_from_cog.at(i)) * aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(gimbal_nominal_angles_curr.at(i * 2), 0, 0)) * mask;
              last_col += 2;
            }
        }

      /* 5.2.2. update the vectoring force for hovering and the gimbal angles */
      Eigen::VectorXd hover_vectoring_f = aerial_robot_model::pseudoinverse(full_q_mat) * getGravity() * robot_model_for_plan_->getMass();
      Eigen::VectorXd static_thrust = Eigen::VectorXd::Zero(getRotorNum());
      if(debug_verbose_) ROS_DEBUG_STREAM("vectoring force for hovering in iteration "<< j+1 << ": " << hover_vectoring_f.transpose());
      last_col = 0;
      for(int i = 0; i < getRotorNum(); i++)
        {
          if(roll_locked_gimbal.at(i) == 0)
            {
              static_thrust(i) = hover_vectoring_f.segment(last_col, 3).norm();
              Eigen::Vector3d f = hover_vectoring_f.segment(last_col, 3);

              gimbal_nominal_angles_curr.at(i * 2) = atan2(-f[1], f[2]);
              gimbal_nominal_angles_curr.at(i * 2 + 1) = atan2(f[0], -f[1] * sin(gimbal_nominal_angles_curr.at(i * 2)) + f[2] * cos(gimbal_nominal_angles_curr.at(i * 2)));
              last_col += 3;
            }
          else
            {
              static_thrust(i) = hover_vectoring_f.segment(last_col, 2).norm();
              Eigen::Vector2d f = hover_vectoring_f.segment(last_col, 2);

              // roll is locked
              gimbal_nominal_angles_curr.at(i * 2 + 1) = atan2(f[0], f[1]);
              last_col += 2;
            }
        }

      /* 5.2.3. check the change of the rotor origin whether converge*/
      std::vector<Eigen::Vector3d> prev_rotors_origin_from_cog = rotors_origin_from_cog;
      for(int i = 0; i < getRotorNum(); ++i)
        {
          std::string s = std::to_string(i + 1);
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_roll"))->second) = gimbal_nominal_angles_curr.at(i * 2);
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = gimbal_nominal_angles_curr.at(i * 2 + 1);
        }
      robot_model_for_plan_->updateRobotModel(gimbal_processed_joint);
      rotors_origin_from_cog = robot_model_for_plan_->getRotorsOriginFromCog<Eigen::Vector3d>();

      double max_diff = 1e-6;
      for(int i = 0; i < getRotorNum(); i++)
        {
          double diff = (rotors_origin_from_cog.at(i) - prev_rotors_origin_from_cog.at(i)).norm();
          if(diff > max_diff) max_diff = diff;
        }
      if(debug_verbose_) ROS_DEBUG_STREAM_NAMED("robot_model", "refine rotor origin: iteration "<< j+1 << ", max_diff: " << max_diff);

      if(max_diff < robot_model_refine_threshold_)
        {
          if(debug_verbose_) ROS_DEBUG_STREAM_NAMED("robot_model", "refine rotor origin: converge in iteration " << j+1 << " max_diff " << max_diff << ", use " << ros::Time::now().toSec() - t << "sec");
          setStaticThrust(static_thrust);

          setVectoringForceWrenchMatrix(full_q_mat);
          hover_vectoring_f_ = hover_vectoring_f;
          break;
        }

      if(j == robot_model_refine_max_iteration_ - 1)
        {
          setStaticThrust(static_thrust);
          hover_vectoring_f_ = hover_vectoring_f;
          setVectoringForceWrenchMatrix(full_q_mat);
          ROS_WARN_STREAM_NAMED("robot_model", "refine rotor origin: can not converge in iteration " << j+1 << " max_diff " << max_diff);
        }
    }

  /* 7. update */
  aerial_robot_model::RobotModel::updateRobotModelImpl(gimbal_processed_joint);

  std::vector<KDL::Vector> f_edfs;
  auto seg_tf_map = getSegmentsTf();
  for(int i = 0; i < getRotorNum(); ++i)
    {
      std::string s = std::to_string(i + 1);
      std::string edf = std::string("edf") + s + std::string("_left");
      KDL::Frame f  = seg_tf_map.at(edf);
      f_edfs.push_back((getCog<KDL::Frame>().Inverse() * f).p);

      edf = std::string("edf") + s + std::string("_right");
      f  = seg_tf_map.at(edf);
      f_edfs.push_back((getCog<KDL::Frame>().Inverse() * f).p);

      std::string gimbal_roll = std::string("gimbal") + s + std::string("_roll_module");
      f  = seg_tf_map.at(gimbal_roll);
      gimbal_roll_origin_from_cog_.at(i) = ((getCog<KDL::Frame>().Inverse() * f).p);
    }
  setEdfsOriginFromCog(f_edfs);

  setGimbalNominalAngles(gimbal_nominal_angles_curr);
  setGimbalProcessedJoint(gimbal_processed_joint);
  setRollLockedGimbal(roll_locked_gimbal);

  return;
  Eigen::Matrix3d inertia_inv = getInertia<Eigen::Matrix3d>().inverse();
  double mass_inv =  1 / getMass();
  Eigen::MatrixXd full_q_mat = getVectoringForceWrenchMatrix();
  full_q_mat.topRows(3) =  mass_inv * full_q_mat.topRows(3) ;
  full_q_mat.bottomRows(3) =  inertia_inv * full_q_mat.bottomRows(3);
  //ROS_INFO_STREAM_THROTTLE(1.0, "full_q_mat : \n" << full_q_mat);
  ROS_INFO_STREAM_THROTTLE(1.0, "full_q_mat_inv : \n" << aerial_robot_model::pseudoinverse(full_q_mat));
  //ROS_INFO_STREAM_THROTTLE(1.0, "mul : \n" << full_q_mat * aerial_robot_model::pseudoinverse(full_q_mat));
  std::stringstream ss;
  for(auto angle: gimbal_nominal_angles_curr) ss << angle << ", ";
  ROS_INFO_STREAM_THROTTLE(1.0, "gimbal nominal angles: \n" << ss.str());
  ROS_INFO_STREAM_THROTTLE(1.0, "hovering force: \n" << hover_vectoring_f_);
}

Eigen::VectorXd FullVectoringRobotModel::calcFeasibleControlFxyDists(const std::vector<int>& roll_locked_gimbal, const std::vector<double>& locked_angles, int rotor_num, const std::vector<Eigen::Matrix3d>& link_rot)
{
  /* only consider F_x and F_y */

  std::vector<Eigen::Vector2d> u(0);

  int gimbal_lock_index = 0;
  for (int i = 0; i < rotor_num; ++i)
    {
      if(roll_locked_gimbal.at(i) == 0)
        {
          // ominidirectional: 2DoF
          u.push_back(Eigen::Vector2d(1, 0)); // from the tilted x force
          u.push_back(Eigen::Vector2d(0, 1)); // from the tilted y force
        }
      else
        {
          // lock gimbal roll: 1DOF
          Eigen::Vector3d gimbal_roll_z_axis = link_rot.at(i) * aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(locked_angles.at(gimbal_lock_index), 0, 0)) * Eigen::Vector3d(0, 0, 1);
          u.push_back(Eigen::Vector2d(gimbal_roll_z_axis(0), gimbal_roll_z_axis(1)).normalized());
          gimbal_lock_index++;
        }
    }

  Eigen::VectorXd f_min(u.size()); // f_min_i; i in [0, u.size()]

  for (int i = 0; i < u.size(); ++i)
    {
      double f_min_ij = 0.0;
      for (int j = 0; j < u.size(); ++j)
        {
          if (i == j) continue;
          f_min_ij += fabs(u.at(i).x()*u.at(j).y() - u.at(j).x()*u.at(i).y()); // we omit the norm of u.at(i), since u is unit vector
        }
      f_min(i) = f_min_ij;
    }

  return f_min;
}

Eigen::VectorXd FullVectoringRobotModel::calcFeasibleControlTDists(const std::vector<int>& roll_locked_gimbal, const std::vector<double>& locked_angles, int rotor_num, const std::vector<Eigen::Vector3d>& rotor_pos, const std::vector<Eigen::Matrix3d>& link_rot)
{
  std::vector<Eigen::Vector3d> v(0);

  int gimbal_lock_index = 0;
  for (int i = 0; i < rotor_num; ++i)
    {
      if(roll_locked_gimbal.at(i) == 0)
        {
          // ominidirectional: 3DoF
          v.push_back(rotor_pos.at(i).cross(Eigen::Vector3d(1, 0, 0))); // from the tilted x force
          v.push_back(rotor_pos.at(i).cross(Eigen::Vector3d(0, 1, 0))); // from the tilted y force
          v.push_back(rotor_pos.at(i).cross(Eigen::Vector3d(0, 0, 1))); // from the z force
        }
      else
        {
          // lock gimbal roll: 2DOF
          Eigen::Matrix3d gimbal_roll_rot =  link_rot.at(i) * aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(locked_angles.at(gimbal_lock_index), 0, 0));
          v.push_back(rotor_pos.at(i).cross(gimbal_roll_rot * Eigen::Vector3d(1, 0, 0))); // from the x force
          v.push_back(rotor_pos.at(i).cross(gimbal_roll_rot * Eigen::Vector3d(0, 0, 1))); // from the z force
          gimbal_lock_index++;
        }
    }

  Eigen::VectorXd t_min(v.size() * (v.size() - 1) / 2); // t_min_ij; i in [0, v.size()], i > j

  int t_min_index = 0;

  for (int i = 0; i < v.size(); ++i)
    {
      for (int j = i + 1; j < v.size(); ++j)
        {
          double t_min_ij = 0.0;
          if(v.at(i).cross(v.at(j)).norm() < 1e-5)
            {
              // assume v_i and v_j has same direction, so this is not plane can generate
              t_min_ij = 1e6;
              ROS_DEBUG_NAMED("robot_model", "the direction of v%d and v%d are too close, so no plane can generate by these two vector", i, j);
            }
          else
            {
              for (int k = 0; k < v.size(); ++k)
                {

                  if (i == k || j == k) continue;
                  double v_triple_product = calcTripleProduct(v.at(i), v.at(j), v.at(k));
                  t_min_ij += fabs(v_triple_product);
                }
            }
          t_min(t_min_index) = t_min_ij;

          t_min_index++;
        }
    }

  return t_min;
}


std::vector<double> FullVectoringRobotModel::calcBestLockGimbalRoll(const std::vector<int>& roll_locked_gimbal, const std::vector<int>& prev_roll_locked_gimbal, const std::vector<double>& prev_opt_locked_angles)
{
  int rotor_num = getRotorNum();
  std::vector<Eigen::Vector3d> rotor_pos = robot_model_for_plan_->getRotorsOriginFromCog<Eigen::Vector3d>();
  std::vector<Eigen::Vector3d> gimbal_roll_pos = getGimbalRollOriginFromCog<Eigen::Vector3d>();
  for(int i = 0; i < rotor_num; i++)
    {
      /*
         Since the position of the force acting point would change according to the gimbal roll,
there is a diffiretial chain about the roll angle. But we here approximate it to the gimbal roll frame along the link axis, rather than the true force acting point (gimbal pitch frame). The offset is 0.037m, which is a small offset in the optimization problem.
       */
      if(roll_locked_gimbal.at(i) == 1) rotor_pos.at(i) = gimbal_roll_pos.at(i);
    }
  const std::vector<Eigen::Matrix3d> link_rot = getLinksRotationFromCog<Eigen::Matrix3d>();

#if 1
  /* TODO: use nonlinear differentiable optimization methods, such as SQP */

  /* nonlinear optimization for vectoring angles planner */
  double start_t = ros::Time::now().toSec();
  int num = std::accumulate(roll_locked_gimbal.begin(), roll_locked_gimbal.end(), 0);
  nlopt::opt nl_solver(nlopt::LN_COBYLA, num);
  nl_solver.set_max_objective(minimumControlWrench, this);
  nl_solver.set_xtol_rel(1e-4); //1e-4
  nl_solver.set_maxeval(1000); // 1000 times
  ROS_DEBUG_NAMED("robot_model", "nlopt init time: %f", ros::Time::now().toSec() - start_t); // TODO: change to DEBUG

  std::vector<double> lb(num, - M_PI / 2 - 0.1);
  std::vector<double> ub(num, M_PI / 2 + 0.1);
  std::vector<double> opt_locked_angles(num);

  int lock_cnt = 0;
  assert(prev_opt_locked_angles.size() == std::accumulate(prev_roll_locked_gimbal.begin(), prev_roll_locked_gimbal.end(), 0));

  for(int i = 0; i < roll_locked_gimbal.size(); i++)
    {
      if(roll_locked_gimbal.at(i) == prev_roll_locked_gimbal.at(i))
        {
          if(roll_locked_gimbal.at(i) == 1)
            {
              /* update the range by using the last optimization result with the assumption that the motion is cotinuous */
              lb.at(lock_cnt) = prev_opt_locked_angles.at(lock_cnt) - gimbal_delta_angle_;
              ub.at(lock_cnt) = prev_opt_locked_angles.at(lock_cnt) + gimbal_delta_angle_;
              opt_locked_angles.at(lock_cnt) = prev_opt_locked_angles.at(lock_cnt);
              lock_cnt++;
            }
        }
      else
        {
          /* reset all range */
          lb.resize(num, - M_PI / 2 - 0.1);
          ub.resize(num, M_PI / 2 + 0.1);
          opt_locked_angles.resize(num, 0);
          break;
        }
    }

  nl_solver.set_lower_bounds(lb);
  nl_solver.set_upper_bounds(ub);

  /* normalized the weight */
  min_force_normalized_weight_ = min_force_weight_ / rotor_num;
  double max_min_torque = calcFeasibleControlTDists(std::vector<int>(rotor_num, 0), std::vector<double>(), rotor_num, rotor_pos, link_rot).minCoeff();
  ROS_DEBUG_STREAM_NAMED("robot_model", "max_min_torque: " << max_min_torque);

  min_torque_normalized_weight_ = min_torque_weight_ / max_min_torque;

  double max_min_control_wrench;
  start_t = ros::Time::now().toSec();
  nlopt::result result = nl_solver.optimize(opt_locked_angles, max_min_control_wrench);

  ROS_DEBUG_STREAM_NAMED("robot_model", "nlopt process time: " << ros::Time::now().toSec() - start_t);  // change to DEBUG
  ROS_DEBUG_STREAM_NAMED("robot_model", "nlopt: opt result: " << max_min_control_wrench);

  std::stringstream ss;
  for(auto angle: opt_locked_angles) ss << angle << ", ";
  ROS_INFO_STREAM_NAMED("robot_model", "nlopt: locked angles: " << ss.str());

  const auto f_min_list = calcFeasibleControlFxyDists(roll_locked_gimbal, opt_locked_angles, rotor_num, link_rot);
  const auto t_min_list = calcFeasibleControlTDists(roll_locked_gimbal, opt_locked_angles, rotor_num, rotor_pos, link_rot);

  ROS_DEBUG_NAMED("robot_model", "opt F min: %f, opt T min: %f", f_min_list.minCoeff(), t_min_list.minCoeff());

  return opt_locked_angles;

#else
  /* simplest way: roll = 0 */
  std::vector<double> locked_angles(0);
  for(auto i: roll_locked_gimbal)
    {
      if(i == 1) locked_angles.push_back(0);
    }

  const auto f_min_list = calcFeasibleControlFxyDists(roll_locked_gimbal, locked_angles, rotor_num, link_rot);
  const auto t_min_list = calcFeasibleControlTDists(roll_locked_gimbal, locked_angles, rotor_num, rotor_pos, link_rot);

  ROS_DEBUG_NAMED("robot_model", "F min: %f, T min: %f", f_min_list.minCoeff(), t_min_list.minCoeff());
  return locked_angles;
#endif
}

bool FullVectoringRobotModel::stabilityCheck(bool verbose)
{
  return aerial_robot_model::RobotModel::stabilityCheck(verbose);
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(Dragon::FullVectoringRobotModel, aerial_robot_model::RobotModel);
