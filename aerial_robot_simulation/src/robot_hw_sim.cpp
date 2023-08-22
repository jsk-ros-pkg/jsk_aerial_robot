#include <aerial_robot_simulation/robot_hw_sim.h>

namespace mujoco_ros_control
{

  bool MujocoRobotHWSim::init(const std::string& robot_namespace,
                              ros::NodeHandle model_nh,
                              mjModel* mujoco_model,
                              mjData* mujoco_data
                              )
  {
    mujoco_model_ = mujoco_model;
    mujoco_data_ = mujoco_data;

    int motor_num = 0;
    joint_list_.resize(0);
    rotor_list_.resize(0);
    for(int i = 0; i < mujoco_model_->nu; i++)
      {
        std::string actuator_name = mj_id2name(mujoco_model_, mjtObj_::mjOBJ_ACTUATOR, i);
        if(actuator_name.find("rotor") != std::string::npos)
          {
            rotor_list_.push_back(actuator_name);
            motor_num++;
          }
        else
          {
            joint_list_.push_back(actuator_name);
          }
      }
    std::cout << "motor num: " <<motor_num << std::endl;

    /* Initialize spinal interface */
    spinal_interface_.init(model_nh, rotor_list_.size());
    registerInterface(&spinal_interface_);

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


    std::cout << "mujoco robot hw sim init" << std::endl;
    return true;
  }

  void MujocoRobotHWSim::read(const ros::Time& time, const ros::Duration& period)
  {

    int fc_id = mj_name2id(mujoco_model_, mjtObj_::mjOBJ_SITE, "fc");
    mjtNum* site_xpos = mujoco_data_->site_xpos;
    mjtNum* site_xmat = mujoco_data_->site_xmat;
    tf::Matrix3x3 fc_rot_mat = tf::Matrix3x3(site_xmat[9 * fc_id + 0], site_xmat[9 * fc_id + 1], site_xmat[9 * fc_id + 2],
                                             site_xmat[9 * fc_id + 3], site_xmat[9 * fc_id + 4], site_xmat[9 * fc_id + 5],
                                             site_xmat[9 * fc_id + 6], site_xmat[9 * fc_id + 7], site_xmat[9 * fc_id + 8]);
    tfScalar r = 0, p = 0, y = 0;
    fc_rot_mat.getRPY(r, p, y);
    tf::Quaternion fc_quat = tf::Quaternion(r, p, y);

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

    if((time - last_mocap_time_).toSec() >= mocap_pub_rate_)
      {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = time;
        pose_msg.pose.position.x = site_xpos[3 * fc_id + 0];
        pose_msg.pose.position.y = site_xpos[3 * fc_id + 1];
        pose_msg.pose.position.z = site_xpos[3 * fc_id + 2];
        pose_msg.pose.orientation.x = fc_quat.x();
        pose_msg.pose.orientation.y = fc_quat.y();
        pose_msg.pose.orientation.z = fc_quat.z();
        pose_msg.pose.orientation.w = fc_quat.w();

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

  void MujocoRobotHWSim::write(const ros::Time& time, const ros::Duration& period)
  {
    // std::cout << "write func" << std::endl;

    for(int i = 0; i < spinal_interface_.getMotorNum(); i++)
      {
        int rotor_id = mj_name2id(mujoco_model_, mjOBJ_ACTUATOR, rotor_list_.at(i).c_str());
        double rotor_force = spinal_interface_.getForce(i);
        mujoco_data_->ctrl[rotor_id] = rotor_force;
      }
  }

}

PLUGINLIB_EXPORT_CLASS(mujoco_ros_control::MujocoRobotHWSim, mujoco_ros_control::MujocoRobotHWSimPlugin)
