#include <boost/shared_ptr.hpp>
#include <pluginlib/class_loader.h>
#include <aerial_robot_base/sensor/sensor_base_plugin.h>

int main(int argc, char** argv)
{
  ros::init (argc, argv, "sensor_plugin_test");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  BasicEstimator* estimator = new BasicEstimator(nh, nhp);

  pluginlib::ClassLoader<sensor_base_plugin::SensorBase> sensor_loader("aerial_robot_base", "sensor_base_plugin::SensorBase");

  try
     {
       boost::shared_ptr<sensor_base_plugin::SensorBase> imu  = sensor_loader.createInstance("sensor_plugin/imu");
       imu->initialize(nh, nhp, estimator);

       ROS_INFO("Result OK");
     }
   catch(pluginlib::PluginlibException& ex)
     {
       ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
     }

  delete estimator;

   return 0;
}
