#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <Eigen/Eigen>
#include "aerial_robot_control/minco_trajectory/minco.hpp"
#include "aerial_robot_control/minco_trajectory/trajectory.hpp"

class Root3DTrajDemo
{
public:
  Root3DTrajDemo() : nh_("~"), trajectory_started_(false), trajectory_generated_(false)
  {
    nh_.param("control_frequency", control_frequency_, 40.0);
    nh_.param("segment_time", segment_time_, 10.0);
    root_target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/dragon/root/target_pose", 10);
    root_pose_sub_ = nh_.subscribe("/dragon/root/pose", 10, &Root3DTrajDemo::rootPoseCallback, this);
    double timer_period = 1.0 / control_frequency_;
    publish_timer_ = nh_.createTimer(ros::Duration(timer_period), &Root3DTrajDemo::publishTimerCallback, this, false, false);
    loadKeyPoints();
    ROS_INFO("[Root3DTrajDemo] Node initialized");
    ROS_INFO("[Root3DTrajDemo] Control frequency: %.1f Hz", control_frequency_);
    ROS_INFO("[Root3DTrajDemo] Segment time: %.1f seconds", segment_time_);
    ROS_INFO("[Root3DTrajDemo] Number of key points: %zu", key_points_.size());
  }

  void rootPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    current_root_pose_ = *msg;
    has_root_pose_ = true;
  }

  void loadKeyPoints()
  {
    XmlRpc::XmlRpcValue key_points_param;
    if (nh_.getParam("key_points", key_points_param))
    {
      if (key_points_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("[Root3DTrajDemo] key_points parameter is not an array!");
        ros::shutdown();
        return;
      }
      for (int i = 0; i < key_points_param.size(); i++)
      {
        if (key_points_param[i].getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_ERROR("[Root3DTrajDemo] key_points[%d] is not an array!", i);
          continue;
        }
        std::vector<double> point;
        for (int j = 0; j < key_points_param[i].size(); j++)
        {
          if (key_points_param[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
            point.push_back(static_cast<double>(key_points_param[i][j]));
          else if (key_points_param[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt)
            point.push_back(static_cast<int>(key_points_param[i][j]));
        }
        if (point.size() == 3)
          key_points_.push_back(point);
        else
          ROS_WARN("[Root3DTrajDemo] key_points[%d] has wrong size: %zu (expected 3)", i, point.size());
      }
      if (key_points_.empty())
      {
        ROS_ERROR("[Root3DTrajDemo] No valid key points loaded!");
        ros::shutdown();
        return;
      }
      ROS_INFO("[Root3DTrajDemo] Loaded %zu key points from parameter server", key_points_.size());
    }
    else
    {
      ROS_ERROR("[Root3DTrajDemo] No key_points parameter found! Please provide a configuration file.");
      ROS_ERROR("[Root3DTrajDemo] Use: roslaunch dragon root_3D_traj_demo.launch config_file:=<path_to_config>");
      ros::shutdown();
    }
  }

  bool generateTrajectory()
  {
    if (!has_root_pose_)
    {
      ROS_WARN("[Root3DTrajDemo] Waiting for root pose data...");
      return false;
    }
    Eigen::Vector3d current_pos(
      current_root_pose_.pose.position.x,
      current_root_pose_.pose.position.y,
      current_root_pose_.pose.position.z
    );
    ROS_INFO("[Root3DTrajDemo] Current position: [%.2f, %.2f, %.2f]",
             current_pos.x(), current_pos.y(), current_pos.z());
    int pieceNum = key_points_.size();
    minco::MINCO_S3NU minco;
    Eigen::Matrix3d headState;
    headState.col(0) = current_pos;
    headState.col(1) = Eigen::Vector3d(0.0, 0.0, 0.0);
    headState.col(2) = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Matrix3d tailState;
    tailState.col(0) = Eigen::Vector3d(key_points_.back()[0], key_points_.back()[1], key_points_.back()[2]);
    tailState.col(1) = Eigen::Vector3d(0.0, 0.0, 0.0);
    tailState.col(2) = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Matrix3Xd waypoints(3, pieceNum - 1);
    for (int i = 0; i < pieceNum - 1; i++)
      waypoints.col(i) = Eigen::Vector3d(key_points_[i][0], key_points_[i][1], key_points_[i][2]);
    Eigen::VectorXd timeAllocation(pieceNum);
    for (int i = 0; i < pieceNum; i++)
      timeAllocation(i) = segment_time_;
    minco.setConditions(headState, tailState, pieceNum);
    minco.setParameters(waypoints, timeAllocation);
    minco.getTrajectory(trajectory_);
    trajectory_generated_ = true;
    trajectory_start_time_ = ros::Time::now();
    ROS_INFO("[Root3DTrajDemo] Trajectory generated successfully!");
    ROS_INFO("[Root3DTrajDemo] Total duration: %.2f seconds", trajectory_.getTotalDuration());
    ROS_INFO("[Root3DTrajDemo] Number of pieces: %d", trajectory_.getPieceNum());
    precomputeCommands();
    return true;
  }

  void precomputeCommands()
  {
    precomputed_commands_.clear();
    double total_duration = trajectory_.getTotalDuration();
    double dt = 1.0 / control_frequency_;
    int num_commands = static_cast<int>(total_duration / dt) + 1;
    ROS_INFO("[Root3DTrajDemo] Precomputing %d control commands...", num_commands);
    
    double prev_yaw = 0.0;  // Initialize previous yaw
    double prev_roll = 0.0;
    double prev_pitch = 0.0;
    
    for (int idx = 0; idx < num_commands; idx++)
    {
      double elapsed_time = idx * dt;
      Eigen::Vector3d pos = trajectory_.getPos(elapsed_time);
      Eigen::Vector3d vel = trajectory_.getVel(elapsed_time);
      
      double roll, pitch, yaw;
      
      if (vel.norm() < 1e-6)  // If velocity is too small, use previous attitude
      {
        roll = prev_roll;
        pitch = prev_pitch;
        yaw = prev_yaw;
      }
      else
      {
        Eigen::Vector3d tangent = vel.normalized();
        // Root frame x-axis is opposite to the forward direction
        Eigen::Vector3d x_axis = -tangent;
        // Choose z-axis as world coordinate system z-axis
        Eigen::Vector3d z_axis(0, 0, 1);
        // Calculate y-axis
        Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
        // Re-orthogonalize z-axis
        z_axis = x_axis.cross(y_axis).normalized();
        Eigen::Matrix3d rot;
        rot.col(0) = x_axis;
        rot.col(1) = y_axis;
        rot.col(2) = z_axis;
        tf::Matrix3x3 tf_rot(
          rot(0,0), rot(0,1), rot(0,2),
          rot(1,0), rot(1,1), rot(1,2),
          rot(2,0), rot(2,1), rot(2,2)
        );
        tf_rot.getRPY(roll, pitch, yaw);
        
        // Update previous attitude
        prev_roll = roll;
        prev_pitch = pitch;
        prev_yaw = yaw;
      }
      
      geometry_msgs::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.pose.position.x = pos.x();
      msg.pose.position.y = pos.y();
      msg.pose.position.z = pos.z();
      tf::Quaternion q;
      q.setRPY(roll, pitch, yaw);
      msg.pose.orientation.x = q.x();
      msg.pose.orientation.y = q.y();
      msg.pose.orientation.z = q.z();
      msg.pose.orientation.w = q.w();
      precomputed_commands_.push_back(msg);
    }
    current_command_index_ = 0;
    ROS_INFO("[Root3DTrajDemo] Precomputation completed! Total commands: %zu", precomputed_commands_.size());
  }

  void publishTimerCallback(const ros::TimerEvent& event)
  {
    publishTrajectoryCommand();
  }

  void publishTrajectoryCommand()
  {
    if (!trajectory_generated_)
      return;
    if (current_command_index_ >= precomputed_commands_.size())
    {
      ROS_INFO("[Root3DTrajDemo] Trajectory execution completed!");
      trajectory_started_ = false;
      trajectory_generated_ = false;
      publish_timer_.stop();
      return;
    }
    geometry_msgs::PoseStamped msg = precomputed_commands_[current_command_index_];
    msg.header.stamp = ros::Time::now();
    root_target_pose_pub_.publish(msg);
    current_command_index_++;
  }

  void run()
  {
    ros::Rate rate(10.0);
    while (ros::ok() && !has_root_pose_)
    {
      ros::spinOnce();
      rate.sleep();
    }
    if (!generateTrajectory())
    {
      ROS_ERROR("[Root3DTrajDemo] Failed to generate trajectory!");
      return;
    }
    trajectory_started_ = true;
    ROS_INFO("[Root3DTrajDemo] Starting trajectory execution...");
    publish_timer_.start();
    ros::spin();
    ROS_INFO("[Root3DTrajDemo] Demo completed.");
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher root_target_pose_pub_;
  ros::Subscriber root_pose_sub_;
  ros::Timer publish_timer_;
  double control_frequency_;
  double segment_time_;
  geometry_msgs::PoseStamped current_root_pose_;
  bool has_root_pose_ = false;
  std::vector<std::vector<double>> key_points_;
  std::vector<geometry_msgs::PoseStamped> precomputed_commands_;
  Trajectory<5> trajectory_;
  bool trajectory_started_;
  bool trajectory_generated_;
  ros::Time trajectory_start_time_;
  int current_command_index_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "root_3D_traj_demo");
  ROS_INFO("========================================");
  ROS_INFO("  Root Head Traj Demo for Dragon");
  ROS_INFO("========================================");
  Root3DTrajDemo demo;
  demo.run();
  return 0;
}
