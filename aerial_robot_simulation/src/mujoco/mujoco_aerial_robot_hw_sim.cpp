#include <aerial_robot_simulation/mujoco/mujoco_aerial_robot_hw_sim.h>

namespace mujoco_ros_control
{

  bool AerialRobotHWSim::init(const std::string& robot_namespace,
                              ros::NodeHandle model_nh,
                              mjModel* mujoco_model,
                              mjData* mujoco_data
                              )
  {
    DefaultRobotHWSim::init(robot_namespace, model_nh, mujoco_model, mujoco_data);

    rotor_list_.resize(0);

    // get entire mass
    float mass = 0.0;
    for(int i = 0; i < mujoco_model_->nbody; i++)
      {
       mass += mujoco_model_->body_mass[i];
      }
    ROS_INFO_STREAM("[mujoco] robot mass is " << mass);


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
    XmlRpc::XmlRpcValue all_servos_params;
    model_nh.getParam("servo_controller", all_servos_params);
    std::string init_value_param_name = "init_value";
    for(auto servo_group_params: all_servos_params)
      {
        if (servo_group_params.second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
          continue;
        for(auto servo_params : servo_group_params.second)
          {
            if(servo_params.first.find("controller") != string::npos)
              {
                std::string servo_name = static_cast<std::string>(servo_params.second["name"]);
                double init_value = 0.0;

                // check simulation param exists
                if(!servo_group_params.second.hasMember("simulation") &&
                   !servo_params.second.hasMember("simulation"))
                  {
                    ROS_ERROR("please set mujoco servo parameters for %s, using sub namespace 'simulation:'", string(servo_params.second["name"]).c_str());
                    continue;
                  }

                // search init_value in servo params
                if(!servo_params.second.hasMember("simulation") ||
                   (servo_params.second.hasMember("simulation") && !servo_params.second["simulation"].hasMember(init_value_param_name)))
                  {
                    // search init_value in servo group params
                    if(!servo_group_params.second["simulation"].hasMember(init_value_param_name))
                      {
                        ROS_ERROR("can not find '%s' gazebo paramter for servo %s", init_value_param_name.c_str(),  string(servo_params.second["name"]).c_str());
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
                control_input_.at((mj_name2id(mujoco_model_, mjtObj_::mjOBJ_ACTUATOR, servo_name.c_str()))) = init_value;
              }
          }
      }

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

    return true;
  }

  void AerialRobotHWSim::read(const ros::Time& time, const ros::Duration& period)
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

    DefaultRobotHWSim::read(time, period);
  }

  void AerialRobotHWSim::write(const ros::Time& time, const ros::Duration& period)
  {
    for(int i = 0; i < spinal_interface_.getMotorNum(); i++)
      {
        int rotor_id = mj_name2id(mujoco_model_, mjOBJ_ACTUATOR, rotor_list_.at(i).c_str());
        double rotor_force = spinal_interface_.getForce(i);
        control_input_.at(rotor_id) = rotor_force;
      }

      DefaultRobotHWSim::write(time, period);
  }

}

PLUGINLIB_EXPORT_CLASS(mujoco_ros_control::AerialRobotHWSim, mujoco_ros_control::RobotHWSim)
