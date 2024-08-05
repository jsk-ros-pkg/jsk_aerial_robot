#include <delta/navigation/delta_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

void RollingNavigator::transformPlanner()
{
  if(ros::Time::now().toSec() > transform_finish_time_)
    {
      ROS_INFO_STREAM("[navigaton] finished transforming planning");
      transforming_flag_ = false;
    }
  ROS_INFO_STREAM_THROTTLE(2.0, "[navigation] planning baselink rotation with joint motion");

  /* contacting state */
  int contacting_link_index = rolling_robot_model_->getContactingLink();
  std::string contacting_link_name = std::string("link") + std::to_string(contacting_link_index + 1);

  /* update current target baselink rotation based on current joint angle */
  KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_plan_->setCogDesireOrientation(cog_desire_orientation);
  KDL::JntArray joint_positions = robot_model_->getJointPositions();

  /* if final run, target baselink rotation is calculated from target joint angle */
  if(!transforming_flag_)
    {
      const auto& joint_index_map = robot_model_->getJointIndexMap();
      for(int i = 0; i < robot_model_->getJointNum() - robot_model_->getRotorNum(); i++)
        {
          joint_positions(joint_index_map.find(std::string("joint") + std::to_string(i + 1))->second) = transform_target_joint_angles_.at(i);
        }
    }

  robot_model_for_plan_->updateRobotModel(joint_positions);
  const auto& seg_tf_map = robot_model_for_plan_->getSegmentsTf();
  KDL::Frame baselink_frame = seg_tf_map.at(robot_model_->getBaselinkName());
  KDL::Frame contacting_link_frame = seg_tf_map.at(contacting_link_name);

  /* calculate desired baselink rotation */
  KDL::Rotation current_target_baselink_rot = transform_initial_cog_R_contact_link_ * (contacting_link_frame.Inverse() * baselink_frame).M;
  double qx, qy, qz, qw;
  current_target_baselink_rot.GetQuaternion(qx, qy, qz, qw);

  /* set current target. if final run, set only in final target */
  if(transforming_flag_) setCurrentTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
  setFinalTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
}

void RollingNavigator::IK()
{
  if(!ik_solving_flag_) return;

  KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_plan_->setCogDesireOrientation(cog_desire_orientation);
  robot_model_for_plan_->updateRobotModel(ik_current_joint_positons_);

  /* calculate joint jacobian */
  const auto& joint_index_map = robot_model_->getJointIndexMap();
  const auto& current_seg_tf_map = robot_model_for_plan_->getSegmentsTf();
  KDL::Vector current_cl_p_ee_in_initial_cog = ik_initial_cog_R_cl_ * ((current_seg_tf_map.at(ik_cl_name_).Inverse() * current_seg_tf_map.at(ik_ee_name_)).p);

  double eps = 1e-8;
  KDL::JntArray joint_positions;
  Eigen::MatrixXd ee_joint_jacobian = Eigen::MatrixXd::Zero(3, robot_model_->getJointNum() - robot_model_->getRotorNum());
  for(int i = 0; i < robot_model_->getJointNum() - robot_model_->getRotorNum(); i++)
    {
      joint_positions = ik_current_joint_positons_;
      joint_positions(joint_index_map.find(std::string("joint") + std::to_string(i + 1))->second) += eps;
      robot_model_for_plan_->updateRobotModel(joint_positions);

      const auto& seg_tf_map = robot_model_for_plan_->getSegmentsTf();

      KDL::Vector cl_p_ee_in_initial_cog = ik_initial_cog_R_cl_ * ((seg_tf_map.at(ik_cl_name_).Inverse() * seg_tf_map.at(ik_ee_name_)).p);
      Eigen::Vector3d cl_dp_ee_in_initial_cog = kdlToEigen(cl_p_ee_in_initial_cog) - kdlToEigen(current_cl_p_ee_in_initial_cog);

      ee_joint_jacobian.block(0, i, 3, 1) = 1.0 / eps * cl_dp_ee_in_initial_cog;
    }

  /* calculate diff and delta joint angle */
  Eigen::Vector3d ee_p_diff_in_initial_cog = ik_target_cl_p_ee_in_initial_cog_ - kdlToEigen(current_cl_p_ee_in_initial_cog);
  Eigen::VectorXd d_joint_angle = 1.0 * ee_joint_jacobian.transpose() * ee_p_diff_in_initial_cog;
  for(int i = 0; i < robot_model_->getJointNum() - robot_model_->getRotorNum(); i++)
    {
      ik_current_joint_positons_(joint_index_map.find(std::string("joint") + std::to_string(i + 1))->second) += d_joint_angle(i);
    }
  // std::cout << "ee p diff: " << ee_p_diff_in_initial_cog.transpose() << std::endl;
  // std::cout << "ee joint jacobian \n" << ee_joint_jacobian.transpose() << std::endl;
  // std::cout << "d joint angle: " << d_joint_angle.transpose() << std::endl;
  // std::cout << std::endl;

  ik_solve_step_++;

  /* convergence check and send to robot */
  if(std::max( fabs(ee_p_diff_in_initial_cog.minCoeff()), fabs(ee_p_diff_in_initial_cog.maxCoeff()) ) < 1e-3)
    {
      ROS_INFO_STREAM("[navigation] finish solving ik in " << ik_solve_step_ << " step");

      bool ok = true;
      std::string rosout_msg = "[navigation] ik result: ";
      sensor_msgs::JointState msg;
      msg.name = {"joint1", "joint2"};
      msg.position = {};
      for(int i = 0; i < robot_model_->getJointNum() - robot_model_->getRotorNum(); i++)
        {
          msg.position.push_back(ik_current_joint_positons_(joint_index_map.find(std::string("joint") + std::to_string(i + 1))->second));

          if(msg.position.back() > 2.0 / 3.0 * M_PI)
            ok = false;

          rosout_msg += std::to_string(msg.position.back()) + std::string(" ");
        }
      if(ok)
        {
          joints_control_pub_.publish(msg);
          ROS_INFO_STREAM(rosout_msg);
        }
      else
        ROS_ERROR_STREAM(rosout_msg);

      ik_solving_flag_ = false;
    }
  else if(ik_solve_step_ > 3.0 / loop_du_)
    {
      ROS_ERROR_STREAM("[navigation] could not solve ik");
      ik_solving_flag_ = false;
    }
}

void RollingNavigator::jointsControlCallback(const sensor_msgs::JointStatePtr & msg)
{
  /* error: size is not same */
  if(msg->name.size() != msg->position.size())
    {
      ROS_ERROR_STREAM("[navigation] size of name " << msg->name.size() << " and position " << msg->position.size() << " is not same");
      return;
    }

  /* normal transformation */
  if(getCurrentGroundNavigationMode() != aerial_robot_navigation::ROLLING_STATE)
    {
      ROS_WARN_STREAM("[navigation] do not plan baselink rotation because current state is " << indexToGroundNavigationModeString(getCurrentGroundNavigationMode()));
      return;
    }

  /* check contacting state */
  int contacting_link_index = rolling_robot_model_->getContactingLink();
  std::string contacting_link_name = std::string("link") + std::to_string(contacting_link_index + 1);
  if(contacting_link_index == 1)
    {
      ROS_WARN_STREAM("[navigation] do not plan baselink rotation because baselink is contacting");
      return;
    }

  /* get target joint angles */
  std::vector<double> current_joint_angles = rolling_robot_model_->getCurrentJointAngles();
  transform_target_joint_angles_.resize(current_joint_angles.size());
  for(int i = 0; i < transform_target_joint_angles_.size(); i++)
    {
      transform_target_joint_angles_.at(i) = current_joint_angles.at(i);
    }
  for(int i = 0; i < msg->name.size(); i++)
    {
      for(int j = 0; j < transform_target_joint_angles_.size(); j++)
        {
          if(msg->name.at(i) == std::string("joint") + std::to_string(j + 1))
            {
              transform_target_joint_angles_.at(j) = msg->position.at(i);
            }
        }
    }

  /* calculate transforming duration */
  double max_diff = 0.0;
  for(int i = 0; i < transform_target_joint_angles_.size(); i++)
    {
      max_diff = std::max(max_diff, fabs(transform_target_joint_angles_.at(i) - current_joint_angles.at(i)));
    }
  transform_finish_time_ = ros::Time::now().toSec() + max_diff / joint_angvel_;

  /* get initial rotation transfrom */
  const auto& seg_tf_map = robot_model_->getSegmentsTf();
  KDL::Frame transform_initial_cog = robot_model_->getCog<KDL::Frame>();
  KDL::Frame transform_initial_contact_link = seg_tf_map.at(contacting_link_name);
  transform_initial_cog_R_contact_link_ = (transform_initial_cog.Inverse() * transform_initial_contact_link).M;

  ROS_INFO_STREAM("[navigation] start baselink rotation planning with joint motion from " << std::setprecision(12) << ros::Time::now().toSec() << " to " << transform_finish_time_);
  transforming_flag_ = true;
}

void RollingNavigator::ikTargetRelEEPosCallback(const geometry_msgs::Vector3Ptr & msg)
{
  ik_solving_flag_ = false; // do not solve ik in other thread before initial process

  /* define contacting link (cl) and enf effector (ee) */
  int contacting_link_index = rolling_robot_model_->getContactingLink();
  ik_cl_name_ = std::string("link") + std::to_string(contacting_link_index + 1);
  if(contacting_link_index == 1)
    {
      ROS_ERROR_STREAM("[navigation] could not define end effector link");
      return;
    }
  if(contacting_link_index == 0)
    {
      ik_ee_name_ = "link3_end";
    }
  else if(contacting_link_index == 1) return;
  else
    {
      ik_ee_name_ = "link1";
    }

  Eigen::Vector3d ik_target_rel_ee_p_in_initial_cog = Eigen::Vector3d(msg->x, 0.0, msg->z); //omit y direction

  ROS_INFO_STREAM("[navigation] ik target: " << ik_target_rel_ee_p_in_initial_cog.transpose());

  /* get robot model information from initial state */
  KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_plan_->setCogDesireOrientation(cog_desire_orientation);
  ik_current_joint_positons_ = robot_model_->getJointPositions();
  robot_model_for_plan_->updateRobotModel(ik_current_joint_positons_);

  /* initial rel pos of ee from cl in cog frame */
  const auto& seg_tf_map = robot_model_for_plan_->getSegmentsTf();
  KDL::Frame ik_initial_cog = robot_model_for_plan_->getCog<KDL::Frame>();
  ik_initial_cog_R_cl_ = (ik_initial_cog.Inverse() * seg_tf_map.at(ik_cl_name_)).M;

  ik_target_cl_p_ee_in_initial_cog_ = ik_target_rel_ee_p_in_initial_cog + kdlToEigen(ik_initial_cog_R_cl_ * ((seg_tf_map.at(ik_cl_name_).Inverse() * seg_tf_map.at(ik_ee_name_)).p));

  ik_solve_step_ = 0;
  ik_solving_flag_ = true;
}
