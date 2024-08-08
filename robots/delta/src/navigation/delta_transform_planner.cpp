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

void RollingNavigator::calcEndEffetorJacobian()
{
  // double eps = 0.1;
  double eps = 1e-6;

  /* update robot model for planning with current state */
  KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_plan_->setCogDesireOrientation(cog_desire_orientation);
  KDL::JntArray init_joint_positions = robot_model_->getJointPositions();
  try {
    robot_model_for_plan_->updateRobotModel(init_joint_positions);
  }
  catch(std::runtime_error e) {
    return;
  }
  const auto& joint_index_map = robot_model_->getJointIndexMap();

  /* contacting information */
  robot_model_for_plan_->calcContactPoint();
  double circle_radius = rolling_robot_model_->getCircleRadius();
  int contacting_link = rolling_robot_model_->getContactingLink();
  double contacting_angle_in_link = rolling_robot_model_->getContactingAngleInLink();
  std::string cl_name, ee_name;
  if(contacting_link == 0) {
    cl_name = "link1";
    ee_name = "link3_end";
  }
  else if(contacting_link == 1) {
    return;
  }
  else if(contacting_link == 2) {
    cl_name = "link3_end";
    ee_name = "link1";
  }

  full_body_ik_jacobian_ = Eigen::MatrixXd::Zero(3, 1 + robot_model_->getJointNum() - robot_model_->getRotorNum()); // dp = J * [dtheta, dq(in R^2)]^T

  /* calculate jacobian of end effector position w.r.t. desired coordinate pitch angle */
  {
    KDL::Rotation R_y_eps = KDL::Rotation::Identity();
    R_y_eps.DoRotY(eps);

    const std::map<std::string, KDL::Frame>& seg_tf_map = robot_model_for_plan_->getSegmentsTf();
    KDL::Frame cog = robot_model_for_plan_->getCog<KDL::Frame>();
    KDL::Frame contact_point = robot_model_for_plan_->getContactPoint<KDL::Frame>();
    KDL::Frame d_contact_point;
    std::vector<KDL::Frame> links_center_frame_from_cog = robot_model_for_plan_->getLinksCenterFrameFromCog();

    /* new contact point position */
    KDL::Vector link_i_center_p_dcp_in_link_i_center;
    link_i_center_p_dcp_in_link_i_center.x(circle_radius * cos(contacting_angle_in_link + eps));
    link_i_center_p_dcp_in_link_i_center.y(circle_radius * sin(contacting_angle_in_link + eps));
    link_i_center_p_dcp_in_link_i_center.z(0.0);
    KDL::Vector cog_p_dcp_in_cog = links_center_frame_from_cog.at(contacting_link) * link_i_center_p_dcp_in_link_i_center;
    d_contact_point.p = cog * cog_p_dcp_in_cog;

    /* new contact point rotation */
    KDL::Rotation dcog_R_base = R_y_eps * cog_desire_orientation;
    KDL::Rotation root_R_base = seg_tf_map.at(robot_model_->getBaselinkName()).M;
    KDL::Rotation root_R_dcog = root_R_base * dcog_R_base.Inverse();
    d_contact_point.M = root_R_dcog;

    /* calculate the difference between contact point (cp) and end effector (ee) in each state */
    Eigen::Vector3d cp_p_ee_in_cp = kdlToEigen((contact_point.Inverse() * seg_tf_map.at(ee_name)).p);
    Eigen::Vector3d dcp_p_ee_in_cp = kdlToEigen((d_contact_point.Inverse() * seg_tf_map.at(ee_name)).p) + Eigen::Vector3d(circle_radius * eps, 0, 0); // dcp_p_ee_in_dcp + rolling offset (dcp is contacted to the ground)
    full_body_ik_jacobian_.block(0, 0, 3, 1) = 1.0 / eps * (dcp_p_ee_in_cp - cp_p_ee_in_cp);
  }

  /* calculate contact point w.r.t contacting link frame */
  const std::map<std::string, KDL::Frame>& current_seg_tf_map = robot_model_for_plan_->getSegmentsTf();
  KDL::Frame root_f_current_cp = robot_model_for_plan_->getContactPoint<KDL::Frame>();
  KDL::Frame current_cl_f_current_cp = current_seg_tf_map.at(cl_name).Inverse() * root_f_current_cp;
  KDL::Frame current_cp_f_current_ee = root_f_current_cp.Inverse() * current_seg_tf_map.at(ee_name);
  Eigen::Vector3d current_cp_p_current_ee = kdlToEigen(current_cp_f_current_ee.p);

  /* calculate jacobian of end effector position w.r.t. joint angles */
  {
    for(int i = 0; i < robot_model_->getJointNum() - robot_model_->getRotorNum(); i++)
      {
        KDL::JntArray joint_positions = init_joint_positions;
        joint_positions(joint_index_map.find(std::string("joint") + std::to_string(i + 1))->second) += eps;
        robot_model_for_plan_->updateRobotModel(joint_positions);

        const std::map<std::string, KDL::Frame>& seg_tf_map = robot_model_for_plan_->getSegmentsTf();

        KDL::Frame root_f_after_ee = seg_tf_map.at(ee_name);
        KDL::Frame current_cp_f_after_ee = (seg_tf_map.at(cl_name) * current_cl_f_current_cp).Inverse() * root_f_after_ee;

        Eigen::Vector3d current_cp_p_after_ee = kdlToEigen(current_cp_f_after_ee.p);
        full_body_ik_jacobian_.block(0, 1 + i, 3, 1) = 1.0 / eps * (current_cp_p_after_ee - current_cp_p_current_ee);
      }
  }
}

void RollingNavigator::fullBodyIKSolve()
{
  std::string cl_name, ee_name;
  if(full_body_ik_initial_contacting_link_ == 0) {
    cl_name = "link1";
    ee_name = "link3_end";
  }
  else if(full_body_ik_initial_contacting_link_ == 1) {
    return;
  }
  else if(full_body_ik_initial_contacting_link_ == 2) {
    cl_name = "link3_end";
    ee_name = "link1";
  }

  /* update robot model for plan from current state */
  {
    KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
    robot_model_for_plan_->setCogDesireOrientation(cog_desire_orientation);
    KDL::JntArray joint_positions = robot_model_->getJointPositions();
    robot_model_for_plan_->updateRobotModel(joint_positions);
    robot_model_for_plan_->calcContactPoint();
  }
  const std::map<std::string, KDL::Frame>& seg_tf_map = robot_model_for_plan_->getSegmentsTf();

  /* calculate initial contact point */
  KDL::Frame root_f_initial_cp =  seg_tf_map.at(cl_name) * full_body_ik_initial_cl_f_cp_;
  geometry_msgs::TransformStamped initial_cp_tf = aerial_robot_model::kdlToMsg(root_f_initial_cp);
  initial_cp_tf.header.stamp = ros::Time::now();
  initial_cp_tf.header.frame_id = tf::resolve(tf_prefix_, std::string("root"));
  initial_cp_tf.child_frame_id = tf::resolve(tf_prefix_, std::string("initial_cp"));
  br_.sendTransform(initial_cp_tf);

  /* calculate current contact point */
  KDL::Frame root_f_current_cp = rolling_robot_model_->getContactPoint<KDL::Frame>();

  /* calculate current rolling angle from initial contact state */
  KDL::Rotation current_cp_R_initial_cp = (root_f_current_cp.Inverse() * root_f_initial_cp).M;
  double whatever1, rolling_pitch, whatever2; current_cp_R_initial_cp.GetRPY(whatever1, rolling_pitch, whatever2);
  double rolling_x_offset = rolling_robot_model_->getCircleRadius() * rolling_pitch;
  // ROS_INFO_STREAM_THROTTLE(1.0, "currcp_R_init_cp rpy: " << whatever1 << " " << rolling_pitch << " " << whatever2);

  /* calculate end effector convergence error considering rolling offset */
  KDL::Frame current_cp_f_ee = root_f_current_cp.Inverse() * seg_tf_map.at(ee_name);
  Eigen::Vector3d initial_cp_p_ee = Eigen::Vector3d(rolling_x_offset, 0, 0) + kdlToEigen(current_cp_f_ee.p); // rolling_offset + current_cp_p_ee_in_current_cp
  Eigen::Vector3d ik_diff = full_body_ik_initial_cp_p_ee_target_ - initial_cp_p_ee;
  // ROS_INFO_STREAM_THROTTLE(1.0, "ik diff: " << ik_diff.transpose());

  /* tf debug */
  {
    KDL::Frame initial_cp_on_ground;
    initial_cp_on_ground.p = KDL::Vector(-rolling_x_offset, 0, 0);
    initial_cp_on_ground.M = KDL::Rotation::Identity();
    geometry_msgs::TransformStamped initial_cp_on_ground_tf = aerial_robot_model::kdlToMsg(initial_cp_on_ground);
    initial_cp_on_ground_tf.header.stamp = ros::Time::now();
    initial_cp_on_ground_tf.header.frame_id = tf::resolve(tf_prefix_, std::string("contact_point"));
    initial_cp_on_ground_tf.child_frame_id = tf::resolve(tf_prefix_, std::string("initial_cp_on_ground"));
    br_.sendTransform(initial_cp_on_ground_tf);

    KDL::Frame full_body_ik_target;
    full_body_ik_target.p = KDL::Vector(full_body_ik_initial_cp_p_ee_target_(0), full_body_ik_initial_cp_p_ee_target_(1), full_body_ik_initial_cp_p_ee_target_(2));
    full_body_ik_target.M = KDL::Rotation::Identity();
    geometry_msgs::TransformStamped full_body_ik_target_tf = aerial_robot_model::kdlToMsg(full_body_ik_target);
    full_body_ik_target_tf.header.stamp = ros::Time::now();
    full_body_ik_target_tf.header.frame_id = tf::resolve(tf_prefix_, std::string("initial_cp_on_ground"));
    full_body_ik_target_tf.child_frame_id = tf::resolve(tf_prefix_, std::string("full_body_ik_target"));
    br_.sendTransform(full_body_ik_target_tf);
  }

  /* calculate ik step */
  Eigen::VectorXd ik_step = aerial_robot_model::pseudoinverse(full_body_ik_jacobian_) * ik_diff;

  /* send current joint angle plus ik result */
  const auto& joint_index_map = robot_model_->getJointIndexMap();
  {
    KDL::JntArray joint_positions = robot_model_->getJointPositions();
    sensor_msgs::JointState msg;
    msg.name = {"joint1", "joint2"};
    msg.position = {};
    bool ok = true;
    for(int i = 0; i < robot_model_->getJointNum() - robot_model_->getRotorNum(); i++)
      {
        msg.position.push_back(joint_positions(joint_index_map.find(std::string("joint") + std::to_string(i + 1))->second) + ik_step(1 + i));
        if(msg.position.back() < 0.0 || 2.0 / 3.0 * M_PI < msg.position.back())
          {
            ok = false;
          }
      }
    if(ok)
      joints_control_pub_.publish(msg);
    else
      {
        std::string rosout_msg = "[navigation] ik result: ";
        for(size_t i = 0; i < msg.position.size(); i++)
          {
            rosout_msg += std::to_string(msg.position.at(i)) + " ";
          }
        ROS_ERROR_STREAM_THROTTLE(3.0, rosout_msg);
      }
  }

  /* calculate final contact point from ik result and bipass to baselinkRotationProcess */
  {
    KDL::Rotation R_y_eps = KDL::Rotation::Identity();
    R_y_eps.DoRotY(ik_step(0));
    KDL::Rotation current_cog_R_base = robot_model_->getCogDesireOrientation<KDL::Rotation>();
    KDL::Rotation final_cog_R_base = R_y_eps * current_cog_R_base;
    setRotationControlLink(cl_name);
    setFinalTargetControlLinkRotation(final_cog_R_base * (seg_tf_map.at(robot_model_->getBaselinkName()).Inverse() * seg_tf_map.at(cl_name)).M);
  }
}
