#include <boost/shared_ptr.hpp>
#include <pluginlib/class_loader.h>
#include <kalman_filter/kf_base.h>

int main(int argc, char** argv)
{
  ros::init (argc, argv, "kf_plugin_test");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  pluginlib::ClassLoader<kf_base::KalmanFilterP<2,1,1> > kf_loader("kalman_filter", "kf_base::KalmanFilterP");

  try
     {
       boost::shared_ptr<kf_base::KalmanFilterP<2,1,1> > triangle = kf_loader.createInstance("kalman_filter/kf_pose_vel_acc_p");
       triangle->initialize(nh, nh_private);


       ROS_INFO("Result OK");
     }
   catch(pluginlib::PluginlibException& ex)
     {
       ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
     }
   return 0;
}
