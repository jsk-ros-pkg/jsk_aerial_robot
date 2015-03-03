#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>

class TrajMotion
{

public:
  TrajMotion(ros::NodeHandle nh, ros::NodeHandle nh_private)
 :nh_(nh), nh_private_(nh_private)
  {
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    path_sub_= nh_.subscribe("/simple_path", 1, &TrajMotion::pathCallback, this, ros::TransportHints().tcpNoDelay());

    nh_private_.param("loop_rate",loop_rate_ , 20);
    nh_private_.param("link_num", link_num_ , 6);
    //nh_private_.param("head_link_name", head_link_name_ , std::string("link0"));

    links_length_.resize(link_num_);
    internal_nodes_.resize(link_num_);
    for(int i = 0; i < link_num_; i++)
      {
        std::stringstream ss;
        ss << i;
        std::string joint_str = std::string("joint")  + ss.str() + std::string("_name");  // "10"
        std::stringstream ss2;
        ss2 << i+1;
        std::string link_length_str = std::string("link") + ss2.str() + std::string("_length");
        nh_private_.param(joint_str, internal_nodes_[i] , std::string("link0"));
        nh_private_.param(link_length_str, links_length_[i], 0.5);

        ROS_INFO("%s: %f", internal_nodes_[i].c_str(), links_length_[i]);

        //internal_nodes_[i] = std::string("link") + str + std::string("_servo_rod") + std::string("_link") + str + std::string("_servo_joint");

      }

    start_time_ = ros::Time::now();
    delta_t_ = 0;
    start_flag_ = false;
    motion_cnt_  = 0;
    
    head_trans_.header.frame_id = "map";
    head_trans_.child_frame_id = internal_nodes_[0];

    pub_timer_ = nh_private_.createTimer(ros::Duration(1.0 / (float)loop_rate_),
                                        &TrajMotion::jointPubFunc, this);

  }
  ~TrajMotion(){}

  const static double degree = M_PI/180;

  void pathCallback(const nav_msgs::PathConstPtr& msg)
  {
    start_flag_ = true;

    trajectory_.poses.resize(0);

    for(int i = 0; i < (int)msg->poses.size(); i++)
      {
        trajectory_.poses.push_back(msg->poses[i]);
      }
    
    //TODO: complementation about start point and end point
  }


  void jointPubFunc(const ros::TimerEvent & e)
  {
    if(start_flag_)
      {
        int start_point = motion_cnt_;
        int prev_point = start_point;
        sensor_msgs::JointState joint_state;
        ROS_WARN("start_point is %d", start_point);
        for(int i = 0; i < link_num_; i ++)
          {
            
            int next_point = internalNode(links_length_[i], start_point, trajectory_);
            if(i == 0) //define head position and orientation
              {
                float angle = atan2(trajectory_.poses[start_point].pose.position.y - trajectory_.poses[next_point].pose.position.y, trajectory_.poses[start_point].pose.position.x - trajectory_.poses[next_point].pose.position.x);

                head_trans_.header.stamp = ros::Time::now();
                head_trans_.transform.translation.x = trajectory_.poses[start_point].pose.position.x;
                head_trans_.transform.translation.y = trajectory_.poses[start_point].pose.position.y;
                head_trans_.transform.translation.z = .7;
                head_trans_.transform.rotation = tf::createQuaternionMsgFromYaw(angle);
                broadcaster_.sendTransform(head_trans_);

                joint_state.header.stamp = head_trans_.header.stamp;
                joint_state.name.resize(link_num_ -1);
                joint_state.position.resize(link_num_ -1);

              }
            else
              {
                float angle = angle2vector(prev_point, start_point, next_point, trajectory_);
                joint_state.name[i - 1 ] = internal_nodes_[i];
                joint_state.position[i - 1] = angle;
                ROS_INFO("angle : %f", angle);
              }
            
            prev_point = start_point;
            start_point = next_point;
          }
        //send the joint state and transform
        joint_pub_.publish(joint_state);
        motion_cnt_++;
        if(motion_cnt_ == (int)trajectory_.poses.size()) motion_cnt_ = 0;
      }
  }


  int internalNode(float link_length, int start_point, nav_msgs::Path traj)
  {
    int next_point = start_point;
    while(1)
      {
        next_point--;
        if(next_point < 0) next_point = traj.poses.size() -1;
        float distance = 
          distanceBetweenTwoPoints(traj.poses[next_point].pose.position.x,
                                   traj.poses[start_point].pose.position.x,
                                   traj.poses[next_point].pose.position.y,
                                   traj.poses[start_point].pose.position.y);
        if(distance >= link_length) break;

        if (next_point == start_point )
          {
            ROS_ERROR("can not find point");
            return -1;
          }
      }

    ROS_WARN("next_point is %d", next_point);
    return next_point;
  }

  
  float angle2vector(int prev_point, int start_point, int next_point, nav_msgs::Path traj)
  {
    float angle0 = atan2(trajectory_.poses[prev_point].pose.position.y - trajectory_.poses[start_point].pose.position.y, trajectory_.poses[prev_point].pose.position.x - trajectory_.poses[start_point].pose.position.x);

    float angle1 = atan2(trajectory_.poses[start_point].pose.position.y - trajectory_.poses[next_point].pose.position.y, trajectory_.poses[start_point].pose.position.x - trajectory_.poses[next_point].pose.position.x);

    //ROS_INFO("angle1: %f, angle0: %f",angle1, angle0);


    return (angle1 - angle0);
  }

  float distanceBetweenTwoPoints(float x1, float x2, float y1, float y2)
  {
    return sqrt((x1 - x2) * (x1 -x2) + (y1 - y2) * (y1 - y2)); 
  }


private:

  ros::NodeHandle nh_, nh_private_;
  ros::Timer  pub_timer_;
  ros::Publisher joint_pub_;
  ros::Subscriber path_sub_;
  nav_msgs::Path trajectory_;
  tf::TransformBroadcaster broadcaster_;
  tf::TransformListener listener_;
  geometry_msgs::TransformStamped head_trans_;

  int loop_rate_;
  int link_num_;
  std::string head_link_name_;
  std::vector<std::string> internal_nodes_;
  std::vector<double> links_length_;

  int motion_cnt_ ;

  ros::Time start_time_;
  float delta_t_;
  bool start_flag_;

};


int main (int argc, char **argv)
{
  ros::init (argc, argv, "trajectory_moion");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  TrajMotion* trajMotionNode = new TrajMotion(nh, nh_private);
  ros::spin ();
  delete trajMotionNode;
  return 0;
}

