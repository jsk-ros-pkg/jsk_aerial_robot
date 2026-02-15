#include <aerial_robot_simulation/mujoco/mujoco_aerial_robot_hw_sim.h>

namespace mujoco_ros_control
{
bool AerialRobotHWSim::initSim(const mjModel* m_ptr, mjData* d_ptr, mujoco_ros::MujocoEnv* mujoco_env_ptr,
                               const std::string& robot_namespace, ros::NodeHandle model_nh,
                               const urdf::Model* const urdf_model,
                               std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // separate rotor transmissions and standard transmissions
  std::vector<transmission_interface::TransmissionInfo> standard_transmissions;
  std::vector<transmission_interface::TransmissionInfo> rotor_transmissions;

  for (const auto& trans : transmissions)
  {
    bool is_rotor = false;
    for (const auto& hw_iface : trans.actuators_[0].hardware_interfaces_)
    {
      if (hw_iface == "RotorInterface")
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
    {
      standard_transmissions.push_back(trans);
    }
  }

  DefaultRobotHWSim::initSim(m_ptr, d_ptr, mujoco_env_ptr, robot_namespace, model_nh, urdf_model,
                             standard_transmissions);

  rotor_list_.resize(0);

  // get entire mass
  float mass = 0.0;
  for (int i = 0; i < m_ptr_->nbody; i++)
  {
    mass += m_ptr_->body_mass[i];
  }
  ROS_INFO_STREAM("[mujoco] robot mass is " << mass);

  // get rotor names from mujoco model
  int motor_num = 0;
  for (int i = 0; i < m_ptr_->nu; i++)
  {
    std::string actuator_name = mj_id2name(m_ptr_, mjtObj_::mjOBJ_ACTUATOR, i);
    if (actuator_name.find("rotor") != std::string::npos)
    {
      rotor_list_.push_back(actuator_name);
      motor_num++;
    }
  }

  // init joints from rosparam
  XmlRpc::XmlRpcValue all_servos_params;
  model_nh.getParam("servo_controller", all_servos_params);
  std::string init_value_param_name = "init_value";
  for (auto servo_group_params : all_servos_params)
  {
    if (servo_group_params.second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      continue;
    for (auto servo_params : servo_group_params.second)
    {
      if (servo_params.first.find("controller") != string::npos)
      {
        std::string servo_name = static_cast<std::string>(servo_params.second["name"]);
        double init_value = 0.0;

        // check simulation param exists
        if (!servo_group_params.second.hasMember("simulation") && !servo_params.second.hasMember("simulation"))
        {
          ROS_ERROR("please set mujoco servo parameters for %s, using sub namespace 'simulation:'",
                    string(servo_params.second["name"]).c_str());
          continue;
        }

        // search init_value in servo params
        if (!servo_params.second.hasMember("simulation") ||
            (servo_params.second.hasMember("simulation") &&
             !servo_params.second["simulation"].hasMember(init_value_param_name)))
        {
          // search init_value in servo group params
          if (!servo_group_params.second["simulation"].hasMember(init_value_param_name))
          {
            ROS_ERROR("can not find '%s' servo paramter for servo %s", init_value_param_name.c_str(),
                      string(servo_params.second["name"]).c_str());
            return false;
          }
          // use param of servo group
          init_value = static_cast<double>(servo_group_params.second["simulation"][init_value_param_name]);
        }
        else
        {
          // use param of servo
          init_value = static_cast<double>(servo_params.second["simulation"][init_value_param_name]);
        }
        d_ptr_->qpos[m_ptr_->jnt_qposadr[mj_name2id(m_ptr_, mjtObj_::mjOBJ_JOINT, servo_name.c_str())]] = init_value;
        d_ptr_->qvel[m_ptr_->jnt_dofadr[mj_name2id(m_ptr_, mjtObj_::mjOBJ_JOINT, servo_name.c_str())]] = 0.0;
        d_ptr_->qfrc_applied[m_ptr_->jnt_dofadr[mj_name2id(m_ptr_, mjtObj_::mjOBJ_JOINT, servo_name.c_str())]] = 0.0;
      }
    }
  }

  /* Initialize spinal interface */
  spinal_interface_.init(model_nh, rotor_list_.size());
  registerInterface(&spinal_interface_);

  ros::NodeHandle simulation_nh = ros::NodeHandle(model_nh, "simulation");
  simulation_nh.param("ground_truth_pub_rate", ground_truth_pub_rate_, 0.01);                               // [sec]
  simulation_nh.param("ground_truth_pos_noise", ground_truth_pos_noise_, 0.0);                              // m
  simulation_nh.param("ground_truth_vel_noise", ground_truth_vel_noise_, 0.0);                              // m/s
  simulation_nh.param("ground_truth_rot_noise", ground_truth_rot_noise_, 0.0);                              // rad
  simulation_nh.param("ground_truth_angular_noise", ground_truth_angular_noise_, 0.0);                      // rad/s
  simulation_nh.param("ground_truth_rot_drift", ground_truth_rot_drift_, 0.0);                              // rad
  simulation_nh.param("ground_truth_vel_drift", ground_truth_vel_drift_, 0.0);                              // m/s
  simulation_nh.param("ground_truth_angular_drift", ground_truth_angular_drift_, 0.0);                      // rad/s
  simulation_nh.param("ground_truth_rot_drift_frequency", ground_truth_rot_drift_frequency_, 0.0);          // 1/s
  simulation_nh.param("ground_truth_vel_drift_frequency", ground_truth_vel_drift_frequency_, 0.0);          // 1/s
  simulation_nh.param("ground_truth_angular_drift_frequency", ground_truth_angular_drift_frequency_, 0.0);  // 1/s

  simulation_nh.param("mocap_pub_rate", mocap_pub_rate_, 0.01);     // [sec]
  simulation_nh.param("mocap_pos_noise", mocap_pos_noise_, 0.001);  // m
  simulation_nh.param("mocap_rot_noise", mocap_rot_noise_, 0.001);  // rad
  ground_truth_pub_ = model_nh.advertise<nav_msgs::Odometry>("ground_truth", 1);
  mocap_pub_ = model_nh.advertise<geometry_msgs::PoseStamped>("mocap/pose", 1);

  return true;
}

void AerialRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  /* get the pose of the fc site in mujoco, and set it as the ground truth value for spinal interface */
  int fc_id = mj_name2id(m_ptr_, mjtObj_::mjOBJ_SITE, "fc");
  mjtNum* site_xpos = d_ptr_->site_xpos;
  mjtNum* site_xmat = d_ptr_->site_xmat;
  tf::Matrix3x3 fc_rot_mat =
      tf::Matrix3x3(site_xmat[9 * fc_id + 0], site_xmat[9 * fc_id + 1], site_xmat[9 * fc_id + 2],
                    site_xmat[9 * fc_id + 3], site_xmat[9 * fc_id + 4], site_xmat[9 * fc_id + 5],
                    site_xmat[9 * fc_id + 6], site_xmat[9 * fc_id + 7], site_xmat[9 * fc_id + 8]);
  tf::Quaternion fc_quat;
  fc_rot_mat.getRotation(fc_quat);

  /* get the imu sensor data in mujoco, and set it as the input for spinal interface */
  tf::Vector3 acc, gyro, mag;
  for (int i = 0; i < m_ptr_->nsensor; i++)
  {
    if (std::string(mj_id2name(m_ptr_, mjtObj_::mjOBJ_SENSOR, i)) == "acc")
    {
      for (int j = 0; j < m_ptr_->sensor_dim[i]; j++)
      {
        acc[j] = d_ptr_->sensordata[m_ptr_->sensor_adr[i] + j];
      }
    }
    if (std::string(mj_id2name(m_ptr_, mjtObj_::mjOBJ_SENSOR, i)) == "gyro")
    {
      for (int j = 0; j < m_ptr_->sensor_dim[i]; j++)
      {
        gyro[j] = d_ptr_->sensordata[m_ptr_->sensor_adr[i] + j];
      }
    }
    if (std::string(mj_id2name(m_ptr_, mjtObj_::mjOBJ_SENSOR, i)) == "mag")
    {
      for (int j = 0; j < m_ptr_->sensor_dim[i]; j++)
      {
        mag[j] = d_ptr_->sensordata[m_ptr_->sensor_adr[i] + j];
      }
    }
  }

  spinal_interface_.setImuValue(acc.x(), acc.y(), acc.z(), gyro.x(), gyro.y(), gyro.z());
  spinal_interface_.setMagValue(mag.x(), mag.y(), mag.z());

  spinal_interface_.stateEstimate();

  /* publish ground truth value */
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = time;
  odom_msg.pose.pose.position.x = site_xpos[3 * fc_id + 0];
  odom_msg.pose.pose.position.y = site_xpos[3 * fc_id + 1];
  odom_msg.pose.pose.position.z = site_xpos[3 * fc_id + 2];
  odom_msg.pose.pose.orientation.x = fc_quat.x();
  odom_msg.pose.pose.orientation.y = fc_quat.y();
  odom_msg.pose.pose.orientation.z = fc_quat.z();
  odom_msg.pose.pose.orientation.w = fc_quat.w();

  /* set ground truth for controller: use the value with noise */
  spinal_interface_.setGroundTruthStates(fc_quat.x(), fc_quat.y(), fc_quat.z(), fc_quat.w(), gyro.x(), gyro.y(),
                                         gyro.z());

  if ((time - last_mocap_time_).toSec() >= mocap_pub_rate_)
  {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = time;
    pose_msg.pose.position.x = site_xpos[3 * fc_id + 0] + gazebo::gaussianKernel(mocap_pos_noise_);
    pose_msg.pose.position.y = site_xpos[3 * fc_id + 1] + gazebo::gaussianKernel(mocap_pos_noise_);
    pose_msg.pose.position.z = site_xpos[3 * fc_id + 2] + gazebo::gaussianKernel(mocap_pos_noise_);

    tf::Quaternion q_delta;
    q_delta.setRPY(gazebo::gaussianKernel(mocap_rot_noise_), gazebo::gaussianKernel(mocap_rot_noise_),
                   gazebo::gaussianKernel(mocap_rot_noise_));
    tf::Quaternion q_noise = fc_quat * q_delta;
    pose_msg.pose.orientation.x = q_noise.x();
    pose_msg.pose.orientation.y = q_noise.y();
    pose_msg.pose.orientation.z = q_noise.z();
    pose_msg.pose.orientation.w = q_noise.w();

    mocap_pub_.publish(pose_msg);
    last_mocap_time_ = time;
  }

  DefaultRobotHWSim::readSim(time, period);
}

void AerialRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
  // normal joints control by default robot hw sim
  DefaultRobotHWSim::writeSim(time, period);

  // set rotor force got from spinal interface to mujoco actuator
  for (int i = 0; i < spinal_interface_.getMotorNum(); i++)
  {
    int rotor_id = mj_name2id(m_ptr_, mjOBJ_ACTUATOR, rotor_list_.at(i).c_str());
    double rotor_force = spinal_interface_.getForce(i);
    d_ptr_->ctrl[rotor_id] = rotor_force;
  }
}

}  // namespace mujoco_ros_control

PLUGINLIB_EXPORT_CLASS(mujoco_ros_control::AerialRobotHWSim, mujoco_ros::control::RobotHWSim)
