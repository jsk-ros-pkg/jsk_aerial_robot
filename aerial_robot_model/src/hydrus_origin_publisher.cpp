#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <boost/thread/mutex.hpp>


class HeadStatePublisher
{
public:
  HeadStatePublisher(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh), nhp_(nhp)
  {
    nhp_.param("base", base_name_, std::string("world"));
    nhp_.param("tf_prefix", tf_prefix_, std::string(""));
    nhp_.param("tf_loop_rate", tf_loop_rate_, 60.0);

    server_.reset( new interactive_markers::InteractiveMarkerServer("dragon2_joints","",false) );
    ros::Duration(0.1).sleep();

    intMarkerInit();


    end_pose_.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    tf_timer_ = nh_.createTimer(ros::Duration(1.0 / tf_loop_rate_), &HeadStatePublisher::tfPublish, this);

    server_->applyChanges();
  }

  ~HeadStatePublisher(){}

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  boost::mutex mutex_;

  std::string tf_prefix_;
  std::string base_name_;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  ros::Timer  tf_timer_;
  double tf_loop_rate_;

  tf::TransformBroadcaster br_;
  tf::Transform end_pose_;
  boost::mutex tf_mutex_;

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
    br_.sendTransform(tf::StampedTransform(t, time, base_name_,
                                           tf_prefix_ + std::string("/link1")
                                            ));

  }

  void setTransform(tf::Vector3 origin, tf::Quaternion rotation)
  {
    boost::lock_guard<boost::mutex> lock(tf_mutex_);
    end_pose_.setOrigin(origin);
    end_pose_.setRotation(rotation);
  }

  tf::Transform getTransform()
  {
    boost::lock_guard<boost::mutex> lock(tf_mutex_);
    return end_pose_;
  }

  void intMarkerInit()
  {
    visualization_msgs::InteractiveMarker int_marker;
    visualization_msgs::InteractiveMarkerControl rotate_control;

    //end pose
    int_marker.controls.resize(0);
    int_marker.header.frame_id = base_name_;
    int_marker.name = std::string("head") +  std::string("_control");
    int_marker.description = int_marker.name;
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

    server_->insert(int_marker,  boost::bind(&HeadStatePublisher::tfProcessFeedback, this, _1));

  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dragon_joint_state_publisher");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  HeadStatePublisher *hydrus_state = new HeadStatePublisher(n, np);
  ros::spin();
  delete hydrus_state;

  return 0;
}

