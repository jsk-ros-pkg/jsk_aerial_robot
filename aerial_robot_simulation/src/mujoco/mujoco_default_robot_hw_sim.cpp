#include <aerial_robot_simulation/mujoco/mujoco_default_robot_hw_sim.h>

namespace mujoco_ros_control
{

  bool DefaultRobotHWSim::init(const std::string& robot_namespace,
                              ros::NodeHandle model_nh,
                              mjModel* mujoco_model,
                              mjData* mujoco_data
                              )
  {
    mujoco_model_ = mujoco_model;
    mujoco_data_ = mujoco_data;

    control_input_.resize(mujoco_model_->nu);
    joint_list_.resize(0);
    rotor_list_.resize(0);

    // get entire mass
    float mass = 0.0;
    for(int i = 0; i < mujoco_model_->nbody; i++)
      {
       mass += mujoco_model_->body_mass[i];
      }
    ROS_INFO_STREAM("[mujoco] robot mass is " << mass);


    // get joint names from mujoco model
    for(int i = 0; i < mujoco_model_->njnt; i++)
      {
      if(mujoco_model_->jnt_type[i] > 1)
        {
          joint_list_.push_back(mj_id2name(mujoco_model_, mjtObj_::mjOBJ_JOINT, i));
        }
      }

    // get rotor names from mujoco model
    int motor_num = 0;
    for(int i = 0; i < mujoco_model_->nu; i++)
      {
        std::string actuator_name = mj_id2name(mujoco_model_, mjtObj_::mjOBJ_ACTUATOR, i);
        if(actuator_name.find("rotor") != std::string::npos)
          {
            rotor_list_.push_back(actuator_name);
            motor_num++;
          }
      }

    // init joints from rosparam
    XmlRpc::XmlRpcValue joint_servos_params;
    model_nh.getParam("servo_controller/joints", joint_servos_params);
    for(int i = 0; i < joint_list_.size(); i++)
      {
        std::string controller_name = "controller" + std::to_string(i);
        if(joint_servos_params[controller_name].valid())
          {
            double init_value = static_cast<double>(joint_servos_params[controller_name]["simulation"]["init_value"]);
            std::string servo_name = static_cast<std::string>(joint_servos_params[controller_name]["name"]);
            control_input_.at((mj_name2id(mujoco_model_, mjtObj_::mjOBJ_ACTUATOR, servo_name.c_str()))) = init_value;
          }
      }

    /* Initialize spinal interface */
    spinal_interface_.init(model_nh, rotor_list_.size());
    registerInterface(&spinal_interface_);

    control_mode_ = FORCE_CONTROL_MODE;
    sim_vel_sub_ = model_nh.subscribe("sim_cmd_vel", 1, &DefaultRobotHWSim::cmdVelCallback, this);
    sim_pos_sub_ = model_nh.subscribe("sim_cmd_pos", 1, &DefaultRobotHWSim::cmdPosCallback, this);

    ros::NodeHandle simulation_nh = ros::NodeHandle(model_nh, "simulation");
    simulation_nh.param("ground_truth_pub_rate", ground_truth_pub_rate_, 0.01); // [sec]
    simulation_nh.param("ground_truth_pos_noise", ground_truth_pos_noise_, 0.0); // m
    simulation_nh.param("ground_truth_vel_noise", ground_truth_vel_noise_, 0.0); // m/s
    simulation_nh.param("ground_truth_rot_noise", ground_truth_rot_noise_, 0.0); // rad
    simulation_nh.param("ground_truth_angular_noise", ground_truth_angular_noise_, 0.0); // rad/s
    simulation_nh.param("ground_truth_rot_drift", ground_truth_rot_drift_, 0.0); // rad
    simulation_nh.param("ground_truth_vel_drift", ground_truth_vel_drift_, 0.0); // m/s
    simulation_nh.param("ground_truth_angular_drift", ground_truth_angular_drift_, 0.0); // rad/s
    simulation_nh.param("ground_truth_rot_drift_frequency", ground_truth_rot_drift_frequency_, 0.0); // 1/s
    simulation_nh.param("ground_truth_vel_drift_frequency", ground_truth_vel_drift_frequency_, 0.0); // 1/s
    simulation_nh.param("ground_truth_angular_drift_frequency", ground_truth_angular_drift_frequency_, 0.0); // 1/s

    simulation_nh.param("mocap_pub_rate", mocap_pub_rate_, 0.01); // [sec]
    simulation_nh.param("mocap_pos_noise", mocap_pos_noise_, 0.001); // m
    simulation_nh.param("mocap_rot_noise", mocap_rot_noise_, 0.001); // rad
    ground_truth_pub_ = model_nh.advertise<nav_msgs::Odometry>("ground_truth", 1);
    mocap_pub_ = model_nh.advertise<geometry_msgs::PoseStamped>("mocap/pose", 1);

    joint_state_pub_ = model_nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    control_input_sub_ = model_nh.subscribe("mujoco/ctrl_input", 1, &DefaultRobotHWSim::controlInputCallback, this);

    return true;
  }

  void DefaultRobotHWSim::read(const ros::Time& time, const ros::Duration& period)
  {
    int fc_id = mj_name2id(mujoco_model_, mjtObj_::mjOBJ_SITE, "fc");
    mjtNum* site_xpos = mujoco_data_->site_xpos;
    mjtNum* site_xmat = mujoco_data_->site_xmat;
    tf::Matrix3x3 fc_rot_mat = tf::Matrix3x3(site_xmat[9 * fc_id + 0], site_xmat[9 * fc_id + 1], site_xmat[9 * fc_id + 2],
                                             site_xmat[9 * fc_id + 3], site_xmat[9 * fc_id + 4], site_xmat[9 * fc_id + 5],
                                             site_xmat[9 * fc_id + 6], site_xmat[9 * fc_id + 7], site_xmat[9 * fc_id + 8]);
    tf::Quaternion fc_quat;
    fc_rot_mat.getRotation(fc_quat);

    spinal::Imu imu_msg;
    for(int i = 0; i < mujoco_model_->nsensor; i++)
      {
        if(std::string(mj_id2name(mujoco_model_, mjtObj_::mjOBJ_SENSOR, i)) == "acc")
          {
            for(int j = 0; j < mujoco_model_->sensor_dim[i]; j++)
              {
                imu_msg.acc_data[j] = mujoco_data_->sensordata[mujoco_model_->sensor_adr[i] + j];
              }
          }
        if(std::string(mj_id2name(mujoco_model_, mjtObj_::mjOBJ_SENSOR, i)) == "gyro")
          {
            for(int j = 0; j < mujoco_model_->sensor_dim[i]; j++)
              {
                imu_msg.gyro_data[j] = mujoco_data_->sensordata[mujoco_model_->sensor_adr[i] + j];
              }
          }
        if(std::string(mj_id2name(mujoco_model_, mjtObj_::mjOBJ_SENSOR, i)) == "mag")
          {
            for(int j = 0; j < mujoco_model_->sensor_dim[i]; j++)
              {
                imu_msg.mag_data[j] = mujoco_data_->sensordata[mujoco_model_->sensor_adr[i] + j];
              }
          }
      }

    spinal_interface_.setImuValue(imu_msg.acc_data[0], imu_msg.acc_data[1], imu_msg.acc_data[2], imu_msg.gyro_data[0], imu_msg.gyro_data[1], imu_msg.gyro_data[2]);
    spinal_interface_.setMagValue(imu_msg.mag_data[0], imu_msg.mag_data[1], imu_msg.mag_data[2]);

    spinal_interface_.stateEstimate();

    /* publish ground truth value */
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = time;
    odom_msg.pose.pose.position.x =site_xpos[3 * fc_id + 0];
    odom_msg.pose.pose.position.y =site_xpos[3 * fc_id + 1];
    odom_msg.pose.pose.position.z =site_xpos[3 * fc_id + 2];
    odom_msg.pose.pose.orientation.x = fc_quat.x();
    odom_msg.pose.pose.orientation.y = fc_quat.y();
    odom_msg.pose.pose.orientation.z = fc_quat.z();
    odom_msg.pose.pose.orientation.w = fc_quat.w();

    spinal_interface_.setTrueBaselinkOrientation(fc_quat.x(),
                                                 fc_quat.y(),
                                                 fc_quat.z(),
                                                 fc_quat.w());

    if((time - last_mocap_time_).toSec() >= mocap_pub_rate_)
      {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = time;
        pose_msg.pose.position.x = site_xpos[3 * fc_id + 0] + gazebo::gaussianKernel(mocap_pos_noise_);
        pose_msg.pose.position.y = site_xpos[3 * fc_id + 1] + gazebo::gaussianKernel(mocap_pos_noise_);
        pose_msg.pose.position.z = site_xpos[3 * fc_id + 2] + gazebo::gaussianKernel(mocap_pos_noise_);


        tf::Quaternion q_delta;
        q_delta.setRPY(gazebo::gaussianKernel(mocap_rot_noise_),
                       gazebo::gaussianKernel(mocap_rot_noise_),
                       gazebo::gaussianKernel(mocap_rot_noise_));
        tf::Quaternion q_noise = fc_quat * q_delta;
        pose_msg.pose.orientation.x = q_noise.x();
        pose_msg.pose.orientation.y = q_noise.y();
        pose_msg.pose.orientation.z = q_noise.z();
        pose_msg.pose.orientation.w = q_noise.w();

        mocap_pub_.publish(pose_msg);
        last_mocap_time_ = time;
      }

    if((time - last_joint_state_time_).toSec() >= joint_state_pub_rate_)
      {
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.stamp = time;
        joint_state_msg.name = joint_list_;

        mjtNum* qpos = mujoco_data_->qpos;
        int* jnt_qposadr = mujoco_model_->jnt_qposadr;
        mjtNum* qvel = mujoco_data_->qvel;
        int* jnt_dofadr = mujoco_model_->jnt_dofadr;
        for(int i = 0; i < mujoco_model_->njnt; i++)
          {
            if(mujoco_model_->jnt_type[i] > 1)
              {
                joint_state_msg.position.push_back(qpos[jnt_qposadr[i]]);
                joint_state_msg.velocity.push_back(qvel[jnt_dofadr[i]]);
              }
          }
        joint_state_pub_.publish(joint_state_msg);
        last_joint_state_time_ = time;
      }

  }

  void DefaultRobotHWSim::write(const ros::Time& time, const ros::Duration& period)
  {
      for(int i = 0; i < spinal_interface_.getMotorNum(); i++)
      {
        int rotor_id = mj_name2id(mujoco_model_, mjOBJ_ACTUATOR, rotor_list_.at(i).c_str());
        double rotor_force = spinal_interface_.getForce(i);
        control_input_.at(rotor_id) = rotor_force;
      }

      for(int i = 0; i < control_input_.size(); i++)
      {
        mujoco_data_->ctrl[i] = control_input_.at(i);
      }
  }

  void DefaultRobotHWSim::controlInputCallback(const sensor_msgs::JointState & msg)
  {
    if(msg.name.size() != msg.position.size())
      {
        ROS_INFO("mujoco: size of actuator names and size of input is not same.");
      }
    for(int i = 0; i < msg.name.size(); i++)
      {
        int actuator_id = mj_name2id(mujoco_model_, mjtObj_::mjOBJ_ACTUATOR, msg.name.at(i).c_str());
        if(actuator_id == -1)
          {
            ROS_INFO_STREAM("mujoco: joint name " <<  msg.name.at(i) << " does not exist");
          }
        else
          {
            control_input_.at(actuator_id) = msg.position.at(i);
          }
      }
  }

}

PLUGINLIB_EXPORT_CLASS(mujoco_ros_control::DefaultRobotHWSim, mujoco_ros_control::RobotHWSim)
