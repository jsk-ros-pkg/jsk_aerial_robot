#include <aerial_robot_model/transformable_aerial_robot_model_ros.h>

namespace aerial_robot_model {
  RobotModelRos::RobotModelRos(ros::NodeHandle nh, ros::NodeHandle nhp):
    nh_(nh),
    nhp_(nhp),
    robot_model_loader_("aerial_robot_model", "aerial_robot_model::RobotModel")
  {
    // subscriber
    joint_state_sub_ = nh_.subscribe("joint_states", 1, &RobotModelRos::jointStateCallback, this);
    // service server
    add_extra_module_service_ = nh_.advertiseService("add_extra_module", &RobotModelRos::addExtraModuleCallback, this);

    // rosparam
    nhp_.param("tf_prefix", tf_prefix_, std::string(""));

    // load robot model plugin
    std::string plugin_name;
    if(nh_.getParam("robot_model_plugin_name", plugin_name))
      {
        try
          {
            robot_model_ = robot_model_loader_.createInstance(plugin_name);
          }
        catch(pluginlib::PluginlibException& ex)
          {
            ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
          }
      }
    else
      {
        ROS_ERROR("can not find plugin rosparameter for robot model, use default class: aerial_robot_model::RobotModel");
        robot_model_ = boost::make_shared<aerial_robot_model::RobotModel>();
      }
 }

  void RobotModelRos::jointStateCallback(const sensor_msgs::JointStateConstPtr& state)
  {
    joint_state_ = *state;
    robot_model_->updateRobotModel(*state);

    ROS_INFO_ONCE("initialized robot model, the mass is %f", robot_model_->getMass());

    geometry_msgs::TransformStamped tf = robot_model_->getCog<geometry_msgs::TransformStamped>();
    tf.header = state->header;
    tf.header.frame_id = tf::resolve(tf_prefix_, robot_model_->getRootFrameName());
    tf.child_frame_id = tf::resolve(tf_prefix_, std::string("cog"));
    br_.sendTransform(tf);
  }

  bool RobotModelRos::addExtraModuleCallback(aerial_robot_model::AddExtraModule::Request &req, aerial_robot_model::AddExtraModule::Response &res)
  {
    switch(req.action)
      {
      case aerial_robot_model::AddExtraModule::Request::ADD:
        {
          geometry_msgs::TransformStamped ts;
          ts.transform = req.transform;
          KDL::Frame f = tf2::transformToKDL(ts);
          KDL::RigidBodyInertia rigid_body_inertia(req.inertia.m, KDL::Vector(req.inertia.com.x, req.inertia.com.y, req.inertia.com.z),
                                                   KDL::RotationalInertia(req.inertia.ixx, req.inertia.iyy,
                                                                          req.inertia.izz, req.inertia.ixy,
                                                                          req.inertia.ixz, req.inertia.iyz));
          res.status = robot_model_->addExtraModule(req.module_name, req.parent_link_name, f, rigid_body_inertia);
          return res.status;
          break;
        }
      case aerial_robot_model::AddExtraModule::Request::REMOVE:
        {
          res.status = robot_model_->removeExtraModule(req.module_name);
          return res.status;
          break;
        }
      default:
        {
          ROS_WARN("[extra module]: wrong action %d", req.action);
          return false;
          break;
        }
      }
    ROS_ERROR("[extra module]: should not reach here ");
    return false;
  }
} //namespace aerial_robot_model
