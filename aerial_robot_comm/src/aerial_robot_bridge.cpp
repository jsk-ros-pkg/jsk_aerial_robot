#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <aerial_robot_comm/aerial_robot_plugins/aerial_robot_base_plugin.h>

class AerialRobotBridge
{
public:

  AerialRobotBridge(ros::NodeHandle nh, ros::NodeHandle nhp)
  {
    plugin_loader_ =  boost::shared_ptr< pluginlib::ClassLoader<aerial_robot_plugin::Base> >( new pluginlib::ClassLoader<aerial_robot_plugin::Base>("aerial_robot_comm", "aerial_robot_plugin::Base"));


    for (auto &pl_name : plugin_loader_->getDeclaredClasses())
      {
        try {
          auto plugin = plugin_loader_->createInstance(pl_name);
          plugin->initialize(nh, nhp);
          loaded_plugins_.push_back(plugin);

          ROS_INFO_STREAM("Aerial Robot Bridge Plugin " << pl_name << " loaded and initialized");

        } catch (pluginlib::PluginlibException &ex) {
          ROS_ERROR_STREAM("Plugin " << pl_name << " load exception: " << ex.what());
        }

      }
  }

  ~AerialRobotBridge() {}


private:
  boost::shared_ptr< pluginlib::ClassLoader<aerial_robot_plugin::Base> > plugin_loader_;
  std::vector< boost::shared_ptr<aerial_robot_plugin::Base> > loaded_plugins_;

};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "aerial_robot_bridge");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  AerialRobotBridge*  aerial_robot_bridge_node = new AerialRobotBridge(nh, nh_private);

  ros::spin ();
  delete aerial_robot_bridge_node;

  return 0;
}

