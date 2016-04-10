#include <boost/shared_ptr.hpp>
#include <pluginlib/class_loader.h>
#include <kalman_filter/kf_base_plugin.h>

int main(int argc, char** argv)
{
  ros::init (argc, argv, "kf_plugin_test");

  ros::NodeHandle nh;

  pluginlib::ClassLoader<kf_base_plugin::KalmanFilter> kf_loader("kalman_filter", "kf_base_plugin::KalmanFilter");

  try
     {
       boost::shared_ptr<kf_base_plugin::KalmanFilter> kf_pos_vel_acc  = kf_loader.createInstance("kalman_filter/kf_pose_vel_acc");
       kf_pos_vel_acc->initialize(nh, std::string("test"), 0);

       ROS_INFO("Result OK");
     }
   catch(pluginlib::PluginlibException& ex)
     {
       ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
     }
   return 0;
}
