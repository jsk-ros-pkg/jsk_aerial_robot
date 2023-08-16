#include <aerial_robot_simulation/mujoco_robot_hw_sim.h>

MujocoRobotHWSim::MujocoRobotHWSim(ros::NodeHandle nh):
  nh_(nh),
  clock_spinner_(1, &clock_loop_queue_),
  read_spinner_(1, &read_loop_queue_)
{
  std::string xml_path;
  nh_.getParam("model", xml_path);
  if(!nh_.getParam("model", xml_path))
    {
      ROS_INFO("Could not get xml path from rosparam\n");
      return;
    }

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
  else
    {
      ROS_INFO_STREAM("Created mujoco model from " << xml_path);
    }

  // viewer
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

  // spinal Interface
  spinal_interface_ = boost::make_shared<MujocoSpinalInterface>();
  spinal_interface_->init(nh_);
  simulation_attitude_controller_.init(spinal_interface_, nh_);

  // ros callback timers
  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 10);

  zero_time_ = ros::Time::now();
  std::cout << zero_time_ << std::endl;

  ros::TimerOptions clock_ops(ros::Duration(1.0 / 1000.0),
                              boost::bind(&MujocoRobotHWSim::clockCallback, this, _1),
                              &clock_loop_queue_);
  clock_timer_ = nh_.createTimer(clock_ops);
  clock_spinner_.start();

  ros::TimerOptions read_ops(ros::Duration(1.0 / 100.0),
                                boost::bind(&MujocoRobotHWSim::readSim, this, _1),
                                &read_loop_queue_);
  read_timer_ = nh_.createTimer(read_ops);
  read_spinner_.start();

  while(ros::ok() && !glfwWindowShouldClose(window))
    {
      mjtNum sim_start = mujoco_data_->time;
      while(mujoco_data_->time - sim_start < 1.0 / 60.0 && ros::ok())
        {
        }
      mujoco_visualization_utils.update(window);
    }
}

MujocoRobotHWSim::~MujocoRobotHWSim()
{
  clock_timer_.stop();
  clock_spinner_.stop();
  read_timer_.stop();
  read_spinner_.stop();
}

void MujocoRobotHWSim::clockCallback(const ros::TimerEvent & e)
{
  rosgraph_msgs::Clock ros_time;
  ros_time.clock.fromSec((ros::Time::now() - zero_time_).toSec());
  clock_pub_.publish(ros_time);
}

void MujocoRobotHWSim::readSim(const ros::TimerEvent & e)
{
  spinal_interface_->stateEstimate();

  MujocoRobotHWSim::writeSim();

}


void MujocoRobotHWSim::writeSim()
{
  simulation_attitude_controller_.update();
  mj_step(mujoco_model_, mujoco_data_);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mujoco_robot_hw_sim");
  ros::NodeHandle nh;

  MujocoRobotHWSim mujoco_robot_hw_sim(nh);
}
