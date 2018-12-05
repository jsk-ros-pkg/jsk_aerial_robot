#include <aerial_robot_model/transformable_aerial_robot_model_ros.h>
namespace aerial_robot_model {
  RobotModelRos::RobotModelRos(ros::NodeHandle nh, ros::NodeHandle nhp, std::unique_ptr<aerial_robot_model::RobotModel> robot_model, bool enable_cog2baselink_tf_pub):
    nh_(nh),
    nhp_(nhp),
    robot_model_(std::move(robot_model)),
    kinematics_updated_(false),
    enable_cog2baselink_tf_pub_(enable_cog2baselink_tf_pub)
  {
    //publisher
    if (enable_cog2baselink_tf_pub_)
      cog2baselink_tf_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("cog2baselink", 1);
    //subscriber
    actuator_state_sub_ = nh_.subscribe("joint_states", 1, &RobotModelRos::actuatorStateCallback, this);
    desire_coordinate_sub_ = nh_.subscribe("desire_coordinate", 1, &RobotModelRos::desireCoordinateCallback, this);
    //service server
    add_extra_module_service_ = nhp_.advertiseService("add_extra_module", &RobotModelRos::addExtraModuleCallback, this);
  }
  void RobotModelRos::actuatorStateCallback(const sensor_msgs::JointStateConstPtr& state)
  {
    if(getRobotModel().getActuatorJointMap().empty()) getRobotModel().setActuatorJointMap(*state);
    if(getRobotModel().getVerbose()) ROS_ERROR("start kinematics");
    getRobotModel().updateRobotModel(*state);
    if(getRobotModel().getVerbose()) ROS_ERROR("finish kinematics");

    if (enable_cog2baselink_tf_pub_)
      {
        geometry_msgs::TransformStamped tf = getRobotModel().getCog<geometry_msgs::TransformStamped>();
        tf.header = state->header;
        tf.header.frame_id = getRobotModel().getRootFrameName();
        tf.child_frame_id = "cog";
        br_.sendTransform(tf);
        geometry_msgs::TransformStamped transform_msg = getRobotModel().getCog2Baselink<geometry_msgs::TransformStamped>();
        transform_msg.header = state->header;
        transform_msg.header.frame_id = std::string("cog");
        transform_msg.child_frame_id = getRobotModel().getBaselinkName();
        cog2baselink_tf_pub_.publish(transform_msg);
      }

    if(!kinematics_updated_)
      {
        ROS_ERROR("the total mass is %f", getRobotModel().getMass());
        kinematics_updated_ = true;
      }
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
          res.status = getRobotModel().addExtraModule(req.module_name, req.parent_link_name, f, rigid_body_inertia);
          return res.status;
          break;
        }
      case aerial_robot_model::AddExtraModule::Request::REMOVE:
        {
          res.status = getRobotModel().removeExtraModule(req.module_name);
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
  void RobotModelRos::desireCoordinateCallback(const spinal::DesireCoordConstPtr& msg)
  {
    getRobotModel().setCogDesireOrientation(msg->roll, msg->pitch, msg->yaw);
  }
} //namespace aerial_robot_model
