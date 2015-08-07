#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <boost/thread/mutex.hpp>

class LinkModule
{
public:
  LinkModule(ros::NodeHandle nh, ros::NodeHandle nhp, const boost::shared_ptr<interactive_markers::InteractiveMarkerServer> &server): nh_(nh), nhp_(nhp)
  {
    nhp_.param("tf_prefix", tf_prefix_, std::string(""));
    server_ = server;
  }
  ~LinkModule(){}

  virtual void getJointsValue(const boost::shared_ptr<sensor_msgs::JointState> &joint_state)
  {
    boost::lock_guard<boost::mutex> lock(mutex_);

    for(std::vector<std::string>::iterator it = joints_name_.begin() ; it != joints_name_.end() ; ++it)
      {
        joint_state->name.push_back(*it);
      }
    for(std::vector<float>::iterator it = joints_value_.begin() ; it != joints_value_.end() ; ++it)
      {
        joint_state->position.push_back(*it);
      }

    return;
  }

protected:

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  std::vector<std::string> joints_name_;
  std::vector<float> joints_value_;
  
  std::stringstream link_no_str_;
  boost::mutex mutex_;

  std::string tf_prefix_;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  virtual void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

    if( feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE )
      {
        ROS_INFO_STREAM( s.str() << ": pose changed"
                         << "\nposition = "
                         << feedback->pose.position.x
                         << ", " << feedback->pose.position.y
                         << ", " << feedback->pose.position.z
                         << "\norientation = "
                         << feedback->pose.orientation.w
                         << ", " << feedback->pose.orientation.x
                         << ", " << feedback->pose.orientation.y
                         << ", " << feedback->pose.orientation.z
                         << "\nframe: " << feedback->header.frame_id
                         << " time: " << feedback->header.stamp.sec << "sec, "
                         << feedback->header.stamp.nsec << " nsec" );


        tf::Quaternion q(feedback->pose.orientation.x, feedback->pose.orientation.y,feedback->pose.orientation.z, feedback->pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;

        if(feedback->control_name == "rotate_x")
          {
            setJointValue(roll,feedback->marker_name);
          }
        if(feedback->control_name == "rotate_y")
          {
            setJointValue(pitch,feedback->marker_name);
          }
        if(feedback->control_name == "rotate_z")
          {
            setJointValue(yaw, feedback->marker_name);
          }


      }

    server_->applyChanges();
  }

  virtual void setJointValue(float value, std::string joint_name)
  {
    boost::lock_guard<boost::mutex> lock(mutex_);



    for(std::vector<std::string>::iterator it = joints_name_.begin() ; it != joints_name_.end() ; ++it)
      {
        if(*it == joint_name)
          {
            size_t index = std::distance( joints_name_.begin(), it );
            joints_value_[index] = value;

          }
      }
  }


};


class DragonLink: public LinkModule
{
public:
  DragonLink(ros::NodeHandle nh, ros::NodeHandle nhp, int link_no, const boost::shared_ptr<interactive_markers::InteractiveMarkerServer> &server):
    LinkModule(nh, nhp, server)
  {
    
    link_no_str_ << link_no;
    joints_name_.resize(0);

    joints_name_.push_back(std::string("link") + link_no_str_.str() + std::string("_front_yaw_joint"));
    joints_name_.push_back(std::string("link") + link_no_str_.str() + std::string("_tiltrotor_pitch_joint"));
    joints_name_.push_back(std::string("link") + link_no_str_.str() + std::string("_tiltrotor_roll_joint"));
    joints_name_.push_back(std::string("link") + link_no_str_.str() + std::string("_propeller_joint"));
    joints_name_.push_back(std::string("link") + link_no_str_.str() + std::string("_rear_pitch_joint"));
 
    std::vector<float> tmp(joints_name_.size(), 0.0);
    joints_value_ = tmp;

    intMarkerInit();

  }

  ~DragonLink(){}


private:


  void intMarkerInit()
  {
    visualization_msgs::InteractiveMarker int_marker;
    visualization_msgs::InteractiveMarkerControl rotate_control;

    //front yaw joint
    int_marker.header.frame_id = tf_prefix_ + std::string("/link") + link_no_str_.str() + "_front_joint_rod";
    //int_marker.name = tf_prefix_ + std::string("/") + joints_name_[0];
    int_marker.name = joints_name_[0];
    int_marker.description = int_marker.name;
    int_marker.scale = 0.1;
    rotate_control.orientation.w = 1;
    rotate_control.orientation.x = 0;
    rotate_control.orientation.y = 1;
    rotate_control.orientation.z = 0;
    rotate_control.name = "rotate_z";
    rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.resize(0);
    int_marker.controls.push_back(rotate_control);
    server_->insert(int_marker,  boost::bind(&DragonLink::processFeedback, this, _1));

    //tiltrotor pitch joint
    int_marker.header.frame_id = tf_prefix_ + std::string("/link") + link_no_str_.str() + "_abdomen2";
    int_marker.name = joints_name_[1];
    int_marker.description = int_marker.name;
    int_marker.scale = 0.1;
    rotate_control.orientation.w = 1;
    rotate_control.orientation.x = 0;
    rotate_control.orientation.y = 1;
    rotate_control.orientation.z = 0;
    rotate_control.name = "rotate_z";
    rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.resize(0);
    int_marker.controls.push_back(rotate_control);
    server_->insert(int_marker,  boost::bind(&DragonLink::processFeedback, this, _1));


    //tiltrotor roll joint
    int_marker.header.frame_id = tf_prefix_ + std::string("/link") + link_no_str_.str() + "_abdomen3";
    int_marker.name = joints_name_[2];
    int_marker.description = int_marker.name;
    int_marker.scale = 0.1;
    rotate_control.orientation.w = 1;
    rotate_control.orientation.x = 1;
    rotate_control.orientation.y = 0;
    rotate_control.orientation.z = 0;
    rotate_control.name = "rotate_x";
    rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.resize(0);
    int_marker.controls.push_back(rotate_control);
    server_->insert(int_marker,  boost::bind(&DragonLink::processFeedback, this, _1));

    //propeller joint
    int_marker.header.frame_id = tf_prefix_ + std::string("/link") + link_no_str_.str() + "_abdomen";
    int_marker.name = joints_name_[3];
    int_marker.description = int_marker.name;
    int_marker.scale = 0.1;
    rotate_control.orientation.w = 1;
    rotate_control.orientation.x = 0;
    rotate_control.orientation.y = 0;
    rotate_control.orientation.z = 1;
    rotate_control.name = "rotate_y";
    rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.resize(0);
    int_marker.controls.push_back(rotate_control);
    server_->insert(int_marker,  boost::bind(&DragonLink::processFeedback, this, _1));

    //rear pitch joint
    int_marker.header.frame_id = tf_prefix_ + std::string("/link") + link_no_str_.str() + "_abdomen3";
    int_marker.name = joints_name_[4];
    int_marker.description = int_marker.name;
    int_marker.scale = 0.1;
    int_marker.pose.position.x = -0.448;
    rotate_control.orientation.w = 1;
    rotate_control.orientation.x = 0;
    rotate_control.orientation.y = 0; 
    rotate_control.orientation.z = 1;
    rotate_control.name = "rotate_y"; 
    rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.resize(0);
    int_marker.controls.push_back(rotate_control);
    server_->insert(int_marker,  boost::bind(&DragonLink::processFeedback, this, _1));

  }


};

class DragonHead: public LinkModule
{
public:
  DragonHead(ros::NodeHandle nh, ros::NodeHandle nhp, int link_no, bool end_pose_flag, const boost::shared_ptr<interactive_markers::InteractiveMarkerServer> &server):
    LinkModule(nh, nhp, server)
  {
    link_no_str_ << link_no;
    joints_name_.resize(0);

    joints_name_.push_back(std::string("link") + link_no_str_.str() + std::string("_left_gripper_joint"));
    joints_name_.push_back(std::string("link") + link_no_str_.str() + std::string("_right_gripper_joint"));
    joints_name_.push_back(std::string("link") + link_no_str_.str() + std::string("_roll_joint"));

 
    std::vector<float> tmp(joints_name_.size(), 0.0);
    joints_value_ = tmp;

    end_pose_flag_ = end_pose_flag;

    intMarkerInit();

    nhp_.param("tf_loop_rate", tf_loop_rate_, 60.0);

   end_pose_.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    tf_timer_ = nh_.createTimer(ros::Duration(1.0 / tf_loop_rate_), &DragonHead::tfPublish, this);

  }

  ~DragonHead(){}

private:
  ros::Timer  tf_timer_;
  double tf_loop_rate_;

  bool end_pose_flag_;

  tf::TransformBroadcaster br_;
  tf::Transform end_pose_;
  boost::mutex tf_mutex_;

  void setJointValue(float value, std::string joint_name)
  {
    ROS_WARN("value :%f", value);
    boost::lock_guard<boost::mutex> lock(mutex_);

    std::string joint_name_tmp(joint_name, 6, 7);

    for(std::vector<float>::iterator it = joints_value_.begin() ; it != joints_value_.end() ; ++it)
      {
        size_t index = std::distance( joints_value_.begin(), it );
        if(strstr(joints_name_[index].c_str(),joint_name_tmp.c_str()))
           *it = value;
      }
  }

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

    if(atoi(link_no_str_.str().c_str()) == 0)
      {
        t.setOrigin(tf::Vector3(-0.075, 0.0, 0.0));
        t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        br_.sendTransform(tf::StampedTransform(t, time, 
                                               tf_prefix_ + std::string("/link") + link_no_str_.str() + std::string("_end_pose"), 
                                               tf_prefix_ + std::string("/link") + link_no_str_.str() + std::string("_rear_junction") ));


        t = getTransform();
        br_.sendTransform(tf::StampedTransform(t, time, 
                                               std::string("/world") + link_no_str_.str(),
                                               tf_prefix_ + std::string("/link") + link_no_str_.str() + std::string("_end_pose")
                                               ));

      }
    else if(end_pose_flag_)
      {

    t.setOrigin(tf::Vector3(0.075, 0.0, 0.0));
    t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    br_.sendTransform(tf::StampedTransform(t, time, 
                                           tf_prefix_ + std::string("/link") + link_no_str_.str() + std::string("_rear_junction"), 
                                           tf_prefix_ + std::string("/link") + link_no_str_.str() + std::string("_end_pose") ));


    t = getTransform();
    br_.sendTransform(tf::StampedTransform(t.inverse(), time, 
                                           tf_prefix_ + std::string("/link") + link_no_str_.str() + std::string("_end_pose"),
                                           std::string("/world") + link_no_str_.str()
                                            ));

      }

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

    // gripper joints
    int_marker.header.frame_id = tf_prefix_ + std::string("/link") + link_no_str_.str() + std::string("_head_base");
    int_marker.name = std::string("link") + link_no_str_.str() + std::string("_gripper");
    int_marker.description = std::string("gripper");
    int_marker.scale = 0.1;
    rotate_control.orientation.w = 1;
    rotate_control.orientation.x = 0;
    rotate_control.orientation.y = 0;
    rotate_control.orientation.z = 1;
    rotate_control.name = "rotate_y";
    rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.resize(0);
    int_marker.controls.push_back(rotate_control);
    server_->insert(int_marker,  boost::bind(&DragonHead::processFeedback, this, _1));

    //roll joint
    int_marker.header.frame_id = tf_prefix_ + std::string("/link") + link_no_str_.str() + std::string("_rear_junction");
    int_marker.name = std::string("link") + link_no_str_.str() +  std::string("_roll");
    int_marker.description = std::string("roll");
    rotate_control.orientation.w = 1;
    rotate_control.orientation.x = 1;
    rotate_control.orientation.y = 0;
    rotate_control.orientation.z = 0;
    rotate_control.name = "rotate_x";
    rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.resize(0);
    int_marker.controls.push_back(rotate_control);
    server_->insert(int_marker,  boost::bind(&DragonHead::processFeedback, this, _1));

    //end pose
    if(end_pose_flag_)
      {
        int_marker.controls.resize(0);
        int_marker.header.frame_id = std::string("/world") + link_no_str_.str();
        //int_marker.name = tf_prefix_ + std::string("/end_pose") + link_no_str_.str() +  std::string("_control");
        int_marker.name = std::string("end_pose") + link_no_str_.str() +  std::string("_control");
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

        server_->insert(int_marker,  boost::bind(&DragonHead::tfProcessFeedback, this, _1));
      }
  }

};



class JointStatePublisher
{
public:
  JointStatePublisher(ros::NodeHandle nh, ros::NodeHandle nh_private) : nh_(nh), nhp_(nh_private)
  {
    initRosParam();

    
    server_.reset( new interactive_markers::InteractiveMarkerServer("dragon2_joints","",false) );

    ros::Duration(0.1).sleep();

    if(head_flag_)
      {
        boost::shared_ptr<DragonHead> dragon_head(new DragonHead(nh_, nhp_, 0, true, server_));
        dragon_links_.push_back(dragon_head);
      }

    for(int i = 0; i < link_num_; i++)
      {
        boost::shared_ptr<DragonLink> dragon_link(new DragonLink(nh_, nhp_, i+1,  server_));
        dragon_links_.push_back(dragon_link);
      }

    if(head_flag_)
      {
        boost::shared_ptr<DragonHead> dragon_head(new DragonHead(nh_, nhp_, link_num_ + 1, two_end_pose_flag_, server_));
        dragon_links_.push_back(dragon_head);
      }

    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

    joint_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / joint_pub_loop_rate_), &JointStatePublisher::jointsPublish, this);


    server_->applyChanges();

  }

  ~JointStatePublisher()
  {
    server_.reset();
  }


private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  ros::Publisher joint_pub_;


  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  ros::Timer  joint_pub_timer_;


  int link_no_;

  double joint_pub_loop_rate_;

  int link_num_;
  bool head_flag_;

  bool two_end_pose_flag_;
  
  std::vector< boost::shared_ptr<LinkModule> > dragon_links_;

  void initRosParam()
  {
    nhp_.param("joint_pub_loop_rate", joint_pub_loop_rate_, 10.0);
    nhp_.param("link_num", link_num_, 6);
    nhp_.param("head_flag", head_flag_, true);

    nhp_.param("two_end_pose_flag", two_end_pose_flag_, false);

  }

  void jointsPublish(const ros::TimerEvent & e)
  {

    boost::shared_ptr<sensor_msgs::JointState> joint_state_ptr(new sensor_msgs::JointState());

    joint_state_ptr->header.stamp = ros::Time::now();

    for(std::vector< boost::shared_ptr<LinkModule> >::iterator it = dragon_links_.begin() ; it != dragon_links_.end() ; ++it)
      {
        (*it)->getJointsValue(joint_state_ptr);
      }


    joint_pub_.publish(*joint_state_ptr);
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "dragon_joint_state_publisher");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  JointStatePublisher *dragon_joints_state = new JointStatePublisher(n, np);
  ros::spin();
  delete dragon_joints_state;
  
  return 0;
}

