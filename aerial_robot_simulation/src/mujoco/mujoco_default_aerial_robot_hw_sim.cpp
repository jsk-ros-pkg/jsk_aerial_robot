#include <aerial_robot_simulation/mujoco/mujoco_default_aerial_robot_hw_sim.h>

namespace mujoco_ros::control
{

bool DefaultAerialRobotHWSim::InitSim(const mjModel* m_ptr, mjData* d_ptr, mujoco_ros::MujocoEnv* mujoco_env_ptr,
                                      const std::string& robot_namespace, ros::NodeHandle model_nh,
                                      const urdf::Model* const urdf_model,
                                      std::vector<transmission_interface::TransmissionInfo> transmissions,
                                      bool ignore_actuators)
{
  std::vector<transmission_interface::TransmissionInfo> standard_transmissions;
  std::vector<transmission_interface::TransmissionInfo> rotor_transmissions;

  /* separate rotor transmissions and standard transmissions */
  for (const auto& trans : transmissions)
  {
    bool is_rotor = false;
    for (const auto& hw_iface : trans.actuators_[0].hardware_interfaces_)
    {
      if (hw_iface == "RotorInterface" || hw_iface == "hardware_interface/RotorInterface")
      {
        is_rotor = true;
        break;
      }
    }
    if (is_rotor)
    {
      rotor_transmissions.push_back(trans);
    }
    else
      standard_transmissions.push_back(trans);
  }

  /* Initialize standard joint handlers */
  DefaultRobotHWSim::InitSim(m_ptr, d_ptr, mujoco_env_ptr, robot_namespace, model_nh, urdf_model,
                             standard_transmissions);

  /* Initialize rotor handlers */
  size_t num_rotors = rotor_transmissions.size();
  rotor_names_.resize(num_rotors);
  rotor_cmd_.resize(num_rotors);
  rotor_pos_.resize(num_rotors);
  rotor_vel_.resize(num_rotors);
  rotor_eff_.resize(num_rotors);
  for (size_t i = 0; i < num_rotors; i++)
  {
    const auto& transmission = rotor_transmissions[i];

    std::string joint_name = transmission.joints_[0].name_;

    rotor_names_.at(i) = joint_name;
    rotor_cmd_.at(i) = 0.0;
    rotor_pos_.at(i) = 0.0;
    rotor_vel_.at(i) = 0.0;
    rotor_eff_.at(i) = 0.0;

    /* create handle for each rotor */
    hardware_interface::JointStateHandle state_handle(rotor_names_.at(i), &rotor_pos_.at(i), &rotor_vel_.at(i),
                                                      &rotor_eff_.at(i));
    hardware_interface::JointHandle rotor_handle(state_handle, &rotor_cmd_.at(i));

    rotor_interface_.registerHandle(rotor_handle);
  }

  registerInterface(&rotor_interface_);

  robot_ns_ = model_nh.getNamespace();
  odom_pub_ = model_nh.advertise<nav_msgs::Odometry>("odom", 10);

  ros::NodeHandle simulation_nh = ros::NodeHandle(model_nh, "simulation");
  simulation_nh.param("odom_pub_rate", odom_pub_rate_, 0.01);          // [sec]
  simulation_nh.param("tf_broadcast_rate", tf_broadcast_rate_, 0.01);  // [sec]
  simulation_nh.param("publish_odom", publish_odom_, true);
  simulation_nh.param("publish_tf", publish_tf_, false);
  root_link_name_ = mj_id2name(m_ptr_, mjtObj_::mjOBJ_BODY, 1);  // 0: world
  simulation_nh.param("root_link_name", root_link_name_,
                      root_link_name_);  // overwrite root link name if specified in yaml
  odom_pub_last_time_ = 0.0;
  tf_broadcast_last_time_ = 0.0;

  return true;
}

void DefaultAerialRobotHWSim::ReadSim(ros::Time time, ros::Duration period)
{
  DefaultRobotHWSim::ReadSim(time, period);

  // reset last publish time after simulator reset
  if (ros::Time(time).toSec() < odom_pub_last_time_ || ros::Time(time).toSec() < tf_broadcast_last_time_)
  {
    odom_pub_last_time_ = 0.0;
    tf_broadcast_last_time_ = 0.0;
  }

  if (publish_odom_ && (ros::Time(time).toSec() - odom_pub_last_time_ >= odom_pub_rate_))
  {
    publishOdometry(time);
    odom_pub_last_time_ = ros::Time(time).toSec();
  }

  if (publish_tf_ && (ros::Time(time).toSec() - tf_broadcast_last_time_ >= tf_broadcast_rate_))
  {
    publishTF(time);
    tf_broadcast_last_time_ = ros::Time(time).toSec();
  }
}

void DefaultAerialRobotHWSim::WriteSim(ros::Time time, ros::Duration period)
{
  DefaultRobotHWSim::WriteSim(time, period);

  /* rotor part */
  for (size_t i = 0; i < rotor_names_.size(); i++)
  {
    int rotor_id = mj_name2id(m_ptr_, mjtObj_::mjOBJ_ACTUATOR, rotor_names_[i].c_str());
    if (std::isfinite(rotor_cmd_.at(i)))
      d_ptr_->ctrl[rotor_id] = rotor_cmd_.at(i);
  }
}

void DefaultAerialRobotHWSim::publishOdometry(ros::Time time)
{
  nav_msgs::Odometry odom_msg;
  tf::Vector3 v_global(d_ptr_->qvel[0], d_ptr_->qvel[1], d_ptr_->qvel[2]);
  tf::Quaternion q(d_ptr_->qpos[4], d_ptr_->qpos[5], d_ptr_->qpos[6], d_ptr_->qpos[3]);
  tf::Vector3 v_local = tf::quatRotate(q.inverse(), v_global);

  odom_msg.header.stamp = time;
  odom_msg.header.frame_id = "world";
  odom_msg.child_frame_id = tf::resolve(robot_ns_, mj_id2name(m_ptr_, mjtObj_::mjOBJ_BODY, 1));
  odom_msg.pose.pose.position.x = d_ptr_->qpos[0];
  odom_msg.pose.pose.position.y = d_ptr_->qpos[1];
  odom_msg.pose.pose.position.z = d_ptr_->qpos[2];
  odom_msg.pose.pose.orientation.w = d_ptr_->qpos[3];
  odom_msg.pose.pose.orientation.x = d_ptr_->qpos[4];
  odom_msg.pose.pose.orientation.y = d_ptr_->qpos[5];
  odom_msg.pose.pose.orientation.z = d_ptr_->qpos[6];
  odom_msg.twist.twist.linear.x = v_local.x();
  odom_msg.twist.twist.linear.y = v_local.y();
  odom_msg.twist.twist.linear.z = v_local.z();
  odom_msg.twist.twist.angular.x = d_ptr_->qvel[3];
  odom_msg.twist.twist.angular.y = d_ptr_->qvel[4];
  odom_msg.twist.twist.angular.z = d_ptr_->qvel[5];
  odom_pub_.publish(odom_msg);
}

void DefaultAerialRobotHWSim::publishTF(ros::Time time)
{
  geometry_msgs::TransformStamped root_pose_transform;
  root_pose_transform.header.stamp = time;
  root_pose_transform.header.frame_id = "world";
  root_pose_transform.child_frame_id = tf::resolve(robot_ns_, root_link_name_);
  root_pose_transform.transform.translation.x = d_ptr_->qpos[0];
  root_pose_transform.transform.translation.y = d_ptr_->qpos[1];
  root_pose_transform.transform.translation.z = d_ptr_->qpos[2];
  root_pose_transform.transform.rotation.w = d_ptr_->qpos[3];
  root_pose_transform.transform.rotation.x = d_ptr_->qpos[4];
  root_pose_transform.transform.rotation.y = d_ptr_->qpos[5];
  root_pose_transform.transform.rotation.z = d_ptr_->qpos[6];
  tf_broadcaster_.sendTransform(root_pose_transform);
}

}  // namespace mujoco_ros::control

PLUGINLIB_EXPORT_CLASS(mujoco_ros::control::DefaultAerialRobotHWSim, mujoco_ros::control::RobotHWSim)
