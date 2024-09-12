#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

class TfPublisher
{
public:
  TfPublisher(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp)
  {
    server_.reset( new interactive_markers::InteractiveMarkerServer("tf_publisher","",false) );

    ros::Duration(0.1).sleep();

    std::string ns = ros::this_node::getNamespace();
    nhp_.param("target_frame", target_frame_, std::string("root"));
    target_frame_ = tf::resolve(ns, target_frame_);
    nhp_.param("reference_frame", reference_frame_, std::string("fixed_frame"));
    reference_frame_ = tf::resolve(ns, reference_frame_);
    nhp_.param("tf_loop_rate", tf_loop_rate_, 60.0);

    intMarkerInit();

    target_pose_.setIdentity();
    tf_timer_ = nh_.createTimer(ros::Duration(1.0 / tf_loop_rate_), &TfPublisher::tfPublish, this);

    server_->applyChanges();

  }

  ~TfPublisher()
  {
    server_.reset();
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Timer  tf_timer_;
  double tf_loop_rate_;
  std::string target_frame_;
  std::string reference_frame_;
  tf::TransformBroadcaster br_;
  tf::Transform target_pose_;;
  boost::mutex tf_mutex_;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  void tfProcessFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    if( feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE )
      {

        tf::Quaternion rotation(feedback->pose.orientation.x, feedback->pose.orientation.y,feedback->pose.orientation.z, feedback->pose.orientation.w);
        tf::Vector3 origin(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);

        setTransform(origin, rotation);

      }

    server_->applyChanges();
  }

  void tfPublish(const ros::TimerEvent & e)
  {//end pose
    tf::Transform t;
    ros::Time time = ros::Time::now();
    t = getTransform();
    br_.sendTransform(tf::StampedTransform(t, time, reference_frame_, target_frame_));
  }

  void setTransform(tf::Vector3 origin, tf::Quaternion rotation)
  {
    boost::lock_guard<boost::mutex> lock(tf_mutex_);
    target_pose_.setOrigin(origin);
    target_pose_.setRotation(rotation);
  }

  tf::Transform getTransform()
  {
    boost::lock_guard<boost::mutex> lock(tf_mutex_);
    return target_pose_;
  }


  void intMarkerInit()
  {
    visualization_msgs::InteractiveMarker int_marker;
    visualization_msgs::InteractiveMarkerControl rotate_control;

    //end pose
    int_marker.controls.resize(0);
    int_marker.header.frame_id = reference_frame_;
    int_marker.name = target_frame_  + std::string("_control");
    rotate_control.orientation.w = 1;
    rotate_control.orientation.x = 1;
    rotate_control.orientation.y = 0;
    rotate_control.orientation.z = 0;
    rotate_control.name = "rotate_x";
    rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(rotate_control);
    rotate_control.name = "move_x";
    rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(rotate_control);

    rotate_control.orientation.w = 1;
    rotate_control.orientation.x = 0;
    rotate_control.orientation.y = 1;
    rotate_control.orientation.z = 0;
    rotate_control.name = "rotate_z";
    rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(rotate_control);
    rotate_control.name = "move_z";
    rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(rotate_control);
    rotate_control.orientation.w = 1;
    rotate_control.orientation.x = 0;
    rotate_control.orientation.y = 0;
    rotate_control.orientation.z = 1;
    rotate_control.name = "rotate_y";
    rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(rotate_control);
    rotate_control.name = "move_y";
    rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

    int_marker.controls.push_back(rotate_control);

    server_->insert(int_marker,  boost::bind(&TfPublisher::tfProcessFeedback, this, _1));
  }

};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "dragon_joint_state_publisher");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  TfPublisher *tf_publisher = new TfPublisher(n, np);
  ros::spin();
  delete tf_publisher;

  return 0;
}

