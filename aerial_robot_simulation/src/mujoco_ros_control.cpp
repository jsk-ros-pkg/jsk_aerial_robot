#include <aerial_robot_simulation/mujoco_ros_control.h>

MujocoRosControl::MujocoRosControl(ros::NodeHandle nh):
  nh_(nh),
  clock_spinner_(1, &clock_loop_queue_),
  publish_spinner_(1, &publish_loop_queue_),
  callback_spinner_(4)
{
  ros::NodeHandle mujoco_model_nh("~");
  std::string xml_path;
  mujoco_model_nh.getParam("model", xml_path);
  if(!mujoco_model_nh.getParam("model", xml_path))
    {
      ROS_INFO("Could not get xml path from rosparam\n");
      return;
    }
  std::cout << xml_path << std::endl;

  const char* xml_path_char = xml_path.c_str();

  char error[1000];
  mujoco_model_ = mj_loadXML(xml_path_char, NULL, error, 1000);
  if(!mujoco_model_)
    {
      ROS_INFO("Could not load mujoco model with error %s.\n", error);
      return;
    }

  mujoco_data_ = mj_makeData(mujoco_model_);
  if(!mujoco_data_)
    {
      ROS_INFO("Could not create mujoco data from model\n");
      return;
    }

  control_input_.resize(mujoco_model_->nu);

  std::cout << "joint list: ";
  for(int i = 0; i < mujoco_model_->njnt; i++)
    {
      if(mujoco_model_->jnt_type[i] > 1)
        {
          joint_names_.push_back(mj_id2name(mujoco_model_, mjtObj_::mjOBJ_JOINT, i));
          std::cout << mj_id2name(mujoco_model_, mjtObj_::mjOBJ_JOINT, i) << " ";
        }
    }
  std::cout << std::endl;

  nh_.getParam("rotor_list", rotor_names_);
  std::cout << "rotor list: ";
  for(int i = 0; i < rotor_names_.size(); i++)
    {
      std::cout << rotor_names_.at(i) << " ";
    }
  std::cout << std::endl;

  // init joints from rosparam
  XmlRpc::XmlRpcValue joint_servos_params;
  nh_.getParam("servo_controller/joints", joint_servos_params);
  for(int i = 0; i < joint_names_.size(); i++)
    {
      std::string controller_name = "controller" + std::to_string(i);
      if(joint_servos_params[controller_name].valid())
        {
          double init_value = static_cast<double>(joint_servos_params[controller_name]["simulation"]["init_value"]);
          std::string servo_name = static_cast<std::string>(joint_servos_params[controller_name]["name"]);
          control_input_.at((mj_name2id(mujoco_model_, mjtObj_::mjOBJ_ACTUATOR, servo_name.c_str()))) = init_value;
        }
    }

  MujocoVisualizationUtils &mujoco_visualization_utils = MujocoVisualizationUtils::getInstance();

  // init GLFW, create window, make OpenGL context current, request v-sync
  glfwInit();
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // make context current
  glfwMakeContextCurrent(window);

  // initialize mujoco visualization functions
  mujoco_visualization_utils.init(mujoco_model_, mujoco_data_, window);

  // create scene and context
  std::cout << "glfw init" << std::endl;

  control_input_sub_ = nh_.subscribe("mujoco/ctrl_input", 1, &MujocoRosControl::controlInputCallback, this);
  four_axis_command_sub_ = nh_.subscribe("four_axes/command", 1, &MujocoRosControl::fourAxisCommandCallback, this);
  mocap_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mocap/pose", 1);
  imu_pub_ = nh_.advertise<spinal::Imu>("imu", 1);
  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 10);

  zero_time_ = ros::Time::now();
  ros::TimerOptions clock_ops(ros::Duration(1.0 / 1000.0),
                        boost::bind(&MujocoRosControl::clockCallback, this, _1),
                        &clock_loop_queue_);
  clock_timer_ = nh_.createTimer(clock_ops);
  clock_spinner_.start();

  ros::TimerOptions publish_ops(ros::Duration(1.0 / 200.0),
                        boost::bind(&MujocoRosControl::publishCallback, this, _1),
                        &publish_loop_queue_);
  publish_timer_ = nh_.createTimer(publish_ops);
  publish_spinner_.start();

  callback_spinner_.start();

  // run main loop, target real-time simulation and 60 fps rendering
  while ( ros::ok() && !glfwWindowShouldClose(window) )
    {
      // advance interactive simulation for 1/60 sec
      // Assuming MuJoCo can simulate faster than real-time, which it usually can,
      // this loop will finish on time for the next frame to be rendered at 60 fps.
      mjtNum sim_start = mujoco_data_->time;

      while ( mujoco_data_->time - sim_start < 1.0/60.0 && ros::ok() )
        {
          for(int i = 0; i < mujoco_model_->nu; i++)
            {
              mujoco_data_->ctrl[i] = control_input_.at(i);
            }
          mj_step(mujoco_model_, mujoco_data_);
        }
      mujoco_visualization_utils.update(window);
    }

  mujoco_visualization_utils.terminate();
  std::cout << "terminate" << std::endl;
}

MujocoRosControl::~MujocoRosControl()
{
  mj_deleteData(mujoco_data_);
  mj_deleteModel(mujoco_model_);

  clock_timer_.stop();
  clock_spinner_.stop();
}

void MujocoRosControl::fourAxisCommandCallback(const spinal::FourAxisCommand & msg)
{
  if(rotor_names_.size() != msg.base_thrust.size())
    {
      ROS_INFO_STREAM("mujoco: size of rotor list " << rotor_names_.size() << " and input size " << msg.base_thrust.size() << " is not same");
      return;
    }
  for(int i = 0; i < msg.base_thrust.size(); i++)
    {
      int actuator_id = mj_name2id(mujoco_model_, mjtObj_::mjOBJ_ACTUATOR, rotor_names_.at(i).c_str());
      control_input_.at(actuator_id) = msg.base_thrust.at(i);
    }
}

void MujocoRosControl::controlInputCallback(const aerial_robot_msgs::ControlInput & msg)
{
  if(msg.name.size() != msg.input.size())
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
          control_input_.at(actuator_id) = msg.input.at(i);
        }
    }
}

double MujocoRosControl::getCurrentTime()
{
  return (ros::Time::now() - zero_time_).toSec();
}


void MujocoRosControl::clockCallback(const ros::TimerEvent & e)
{
  rosgraph_msgs::Clock ros_time;
  ros_time.clock.fromSec(getCurrentTime());
  clock_pub_.publish(ros_time);
}

void MujocoRosControl::publishCallback(const ros::TimerEvent & e)
{
  publish_cnt_ = (publish_cnt_ + 1) % 200;
  int fc_id = mj_name2id(mujoco_model_, mjtObj_::mjOBJ_SITE, "fc");
  mjtNum* site_xpos = mujoco_data_->site_xpos;
  mjtNum* site_xmat = mujoco_data_->site_xmat;
  tf::Matrix3x3 fc_rot_mat = tf::Matrix3x3(site_xmat[9 * fc_id + 0], site_xmat[9 * fc_id + 1], site_xmat[9 * fc_id + 2],
                                           site_xmat[9 * fc_id + 3], site_xmat[9 * fc_id + 4], site_xmat[9 * fc_id + 5],
                                           site_xmat[9 * fc_id + 6], site_xmat[9 * fc_id + 7], site_xmat[9 * fc_id + 8]);
  tfScalar r = 0, p = 0, y = 0;
  fc_rot_mat.getRPY(r, p, y);
  tf::Quaternion fc_quat = tf::Quaternion(r, p, y);

  // if(getCurrentTime() - imu_pub_last_time_ > imu_pub_rate_)
  if(publish_cnt_ % 1 == 0)
    {
      spinal::Imu imu_msg;
      imu_msg.stamp = (ros::Time) getCurrentTime();

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
      imu_msg.angles[0] = r;
      imu_msg.angles[1] = p;
      imu_msg.angles[2] = y;
      imu_pub_.publish(imu_msg);

      imu_pub_last_time_ = getCurrentTime();
    }

  // if(getCurrentTime() - mocap_pub_last_time_ > mocap_pub_rate_)
  if(publish_cnt_ % 2 == 0)
    {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.stamp = (ros::Time) getCurrentTime();
      pose_msg.pose.position.x = site_xpos[3 * fc_id + 0];
      pose_msg.pose.position.y = site_xpos[3 * fc_id + 1];
      pose_msg.pose.position.z = site_xpos[3 * fc_id + 2];
      pose_msg.pose.orientation.x = fc_quat.x();
      pose_msg.pose.orientation.y = fc_quat.y();
      pose_msg.pose.orientation.z = fc_quat.z();
      pose_msg.pose.orientation.w = fc_quat.w();
      mocap_pub_.publish(pose_msg);

      mocap_pub_last_time_ = getCurrentTime();
    }


  // if(getCurrentTime() - joint_state_pub_last_time_ > joint_state_pub_rate_)
  if(publish_cnt_ % 4 == 0)
    {
      sensor_msgs::JointState joint_state_msg;
      joint_state_msg.header.stamp = (ros::Time) getCurrentTime();
      joint_state_msg.name = joint_names_;

      mjtNum* qpos = mujoco_data_->qpos;
      int* jnt_qposadr = mujoco_model_->jnt_qposadr;
      for(int i = 0; i < mujoco_model_->njnt; i++)
        {
          if(mujoco_model_->jnt_type[i] > 1)
            {
              joint_state_msg.position.push_back(qpos[jnt_qposadr[i]]);
            }
        }
      joint_state_pub_.publish(joint_state_msg);

      joint_state_pub_last_time_ = getCurrentTime();
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mujoco_ros_control");
  ros::NodeHandle nh;

  MujocoRosControl mujoco_ros_control(nh);
}
