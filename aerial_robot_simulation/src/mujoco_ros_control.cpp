#include <aerial_robot_simulation/mujoco_ros_control.h>

namespace mujoco_ros_control
{
  MujocoRosControl::MujocoRosControl(ros::NodeHandle &nh):
    nh_(nh)
  {
  }

  MujocoRosControl::~MujocoRosControl()
  {
    mj_deleteData(mujoco_data_);
    mj_deleteModel(mujoco_model_);
  }

  bool MujocoRosControl::init()
  {
    std::string xml_path;
    nh_.getParam("mujoco_model_path", xml_path);
    if(!nh_.getParam("mujoco_model_path", xml_path))
      {
        ROS_INFO("Could not get xml path from rosparam\n");
        return false;
      }

    const char* xml_path_char = xml_path.c_str();

    char error[1000];
    mujoco_model_ = mj_loadXML(xml_path_char, NULL, error, 1000);
    if(!mujoco_model_)
      {
        ROS_INFO("Could not load mujoco model with error %s.\n", error);
        return false;
      }

    mujoco_model_->opt.timestep = 1.0 / clock_pub_freq_;

    mujoco_data_ = mj_makeData(mujoco_model_);
    if(!mujoco_data_)
      {
        ROS_INFO("Could not create mujoco data from model\n");
        return false;
      }
    else
      {
        ROS_INFO_STREAM("Created mujoco model from " << xml_path);
      }

    /* hardware interface  */
    robot_hw_sim_loader_.reset(new pluginlib::ClassLoader<mujoco_ros_control::MujocoRobotHWSimPlugin>("aerial_robot_simulation", "mujoco_ros_control::MujocoRobotHWSimPlugin"));
    try
      {
        robot_hw_sim_ = robot_hw_sim_loader_->createInstance("mujoco_ros_control/MujocoRobotHWSim");
      }
    catch(pluginlib::PluginlibException& ex)
      {
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
      }
    std::string robot_ns = nh_.getNamespace().substr(1, nh_.getNamespace().size () - 1);
    robot_hw_sim_->init(robot_ns, nh_, mujoco_model_, mujoco_data_);

    /* attitude controller */
    controller_manager_.reset(new controller_manager::ControllerManager(robot_hw_sim_.get(), nh_));

    clock_pub_ =  nh_.advertise<rosgraph_msgs::Clock>("/clock", 10);

    return true;
  }


  void MujocoRosControl::publishSimTime()
  {
    ros::Time sim_time = (ros::Time) mujoco_data_->time;
    if((sim_time - last_clock_pub_time_).toSec() < 1.0 /(double) clock_pub_freq_ )
      {
        return;
      }
    ros::Time current_time = (ros::Time) mujoco_data_->time;
    rosgraph_msgs::Clock ros_time;
    ros_time.clock.fromSec(current_time.toSec());
    last_clock_pub_time_ = sim_time;
    clock_pub_.publish(ros_time);
  }

  void MujocoRosControl::update()
  {
    publishSimTime();

    ros::Time sim_time = (ros::Time) mujoco_data_->time;
    ros::Time sim_time_ros(sim_time.sec, sim_time.nsec);

    ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

    mj_step1(mujoco_model_, mujoco_data_);

    robot_hw_sim_->read(sim_time_ros, sim_period);

    controller_manager_->update(sim_time_ros, sim_period);

    robot_hw_sim_->write(sim_time_ros, sim_period);

    mj_step2(mujoco_model_, mujoco_data_);

    last_update_sim_time_ros_ = sim_time_ros;
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mujoco_ros_control");
  ros::NodeHandle nh;
  mujoco_ros_control::MujocoRosControl mujoco_ros_control(nh);

  // viewer
  MujocoVisualizationUtils &mujoco_visualization_utils = MujocoVisualizationUtils::getInstance();

  mujoco_ros_control.init();

  // init GLFW, create window, make OpenGL context current, request v-sync
  glfwInit();
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // make context current
  glfwMakeContextCurrent(window);

  // initialize mujoco visualization functions
  mujoco_visualization_utils.init(mujoco_ros_control.mujoco_model_, mujoco_ros_control.mujoco_data_, window);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok() && !glfwWindowShouldClose(window))
    {
      mjtNum sim_start = mujoco_ros_control.mujoco_data_->time;
      while(mujoco_ros_control.mujoco_data_->time - sim_start < 1.0 / 60.0 && ros::ok())
        {
          mujoco_ros_control.update();
        }
      mujoco_visualization_utils.update(window);
    }
  mujoco_visualization_utils.terminate();

  return 0;
}

