#include <aerial_robot_model/joint_state_publisher.h>


JointStatePublisher::JointStatePublisher(ros::NodeHandle nh, ros::NodeHandle nh_private) : jointStateNodeHandle(nh), jointStateNodeHandlePrivate(nh_private)
{
  joint_pub = jointStateNodeHandle.advertise<sensor_msgs::JointState>("joint_states", 1);

  pubLoopRate = 20;

  // robot state
  //double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

  startTime = ros::Time::now();

  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "head_cylinder";
  delta_t_ = 0;

  pubTimer =
    jointStateNodeHandlePrivate.createTimer(ros::Duration(1.0 / pubLoopRate),
                                           &JointStatePublisher::pubFunction,
                                           this);

}


JointStatePublisher::~JointStatePublisher()
{
}

void JointStatePublisher::pubFunction(const ros::TimerEvent & e)
{
  delta_t_ = ros::Time::now().toSec() - startTime.toSec();
  //update joint_state 
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(3);
  joint_state.position.resize(3);
  joint_state.name[0] ="link3_servo_rod_link3_servo_joint";
  joint_state.position[0] = hydraMotion(M_PI);
  joint_state.name[1] ="link2_servo_rod_link2_servo_joint";
  joint_state.position[1] = hydraMotion(M_PI/2);
  joint_state.name[2] ="link1_servo_rod_link1_servo_joint";
  joint_state.position[2] = hydraMotion(0);

  //send the joint state and transform
  joint_pub.publish(joint_state);


  // update transform
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.transform.translation.x = cos(delta_t_)*2;
  odom_trans.transform.translation.y = sin(delta_t_)*2;
  odom_trans.transform.translation.z = .7;
  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(M_PI/2 + delta_t_);
  broadcaster.sendTransform(odom_trans);

}
double JointStatePublisher::hydraMotion(double phase)
{
  return sin(delta_t_ + phase) ;
}

