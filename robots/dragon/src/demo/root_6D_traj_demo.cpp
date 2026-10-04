#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <Eigen/Eigen>
#include "aerial_robot_control/minco_trajectory/minco.hpp"
#include "aerial_robot_control/minco_trajectory/trajectory.hpp"

class Root6DTrajDemo
{
public:
  Root6DTrajDemo() : nh_("~"), trajectory_started_(false), trajectory_generated_(false)
  {
    // Initialize ROS parameters
    nh_.param("control_frequency", control_frequency_, 40.0);
    nh_.param("segment_time", segment_time_, 10.0);
    
    // Initialize publisher and subscriber
    root_target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/dragon/root/target_pose", 10);
    root_pose_sub_ = nh_.subscribe("/dragon/root/pose", 10, &Root6DTrajDemo::rootPoseCallback, this);
    
    // Create timer for trajectory publishing
    double timer_period = 1.0 / control_frequency_;
    publish_timer_ = nh_.createTimer(ros::Duration(timer_period), &Root6DTrajDemo::publishTimerCallback, this, false, false);

    // Load key poses from parameter server
    loadKeyPoses();
    
    ROS_INFO("[Root6DTrajDemo] Node initialized");
    ROS_INFO("[Root6DTrajDemo] Control frequency: %.1f Hz", control_frequency_);
    ROS_INFO("[Root6DTrajDemo] Segment time: %.1f seconds", segment_time_);
    ROS_INFO("[Root6DTrajDemo] Number of key poses: %zu", key_poses_.size());
  }
  
  void rootPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    current_root_pose_ = *msg;
    has_root_pose_ = true;
  }
  
  void loadKeyPoses()
  {
    // Try to load key poses from parameter server
    XmlRpc::XmlRpcValue key_poses_param;
    if (nh_.getParam("key_poses", key_poses_param))
    {
      if (key_poses_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("[Root6DTrajDemo] key_poses parameter is not an array!");
        ros::shutdown();
        return;
      }
      
      for (int i = 0; i < key_poses_param.size(); i++)
      {
        if (key_poses_param[i].getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_ERROR("[Root6DTrajDemo] key_poses[%d] is not an array!", i);
          continue;
        }
        
        std::vector<double> pose;
        for (int j = 0; j < key_poses_param[i].size(); j++)
        {
          if (key_poses_param[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
          {
            pose.push_back(static_cast<double>(key_poses_param[i][j]));
          }
          else if (key_poses_param[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt)
          {
            pose.push_back(static_cast<int>(key_poses_param[i][j]));
          }
        }
        
        if (pose.size() == 6)  // x, y, z, roll, pitch, yaw
        {
          key_poses_.push_back(pose);
        }
        else
        {
          ROS_WARN("[Root6DTrajDemo] key_poses[%d] has wrong size: %zu (expected 6)", i, pose.size());
        }
      }
      
      if (key_poses_.empty())
      {
        ROS_ERROR("[Root6DTrajDemo] No valid key poses loaded!");
        ros::shutdown();
        return;
      }
      
      ROS_INFO("[Root6DTrajDemo] Loaded %zu key poses from parameter server", key_poses_.size());
    }
    else
    {
      ROS_ERROR("[Root6DTrajDemo] No key_poses parameter found! Please provide a configuration file.");
      ROS_ERROR("[Root6DTrajDemo] Use: roslaunch dragon root_6D_traj_demo.launch config_file:=<path_to_config>");
      ros::shutdown();
    }
  }
  
  bool generateTrajectory()
  {
    if (!has_root_pose_)
    {
      ROS_WARN("[Root6DTrajDemo] Waiting for root pose data...");
      return false;
    }
    
    // Extract current position from odometry
    Eigen::Vector3d current_pos(
      current_root_pose_.pose.position.x,
      current_root_pose_.pose.position.y,
      current_root_pose_.pose.position.z
    );
    
    ROS_INFO("[Root6DTrajDemo] Current position: [%.2f, %.2f, %.2f]",
             current_pos.x(), current_pos.y(), current_pos.z());
    
    // Number of trajectory pieces
    int pieceNum = key_poses_.size();
    
    // Create MINCO instance for minimum jerk trajectory
    minco::MINCO_S3NU minco;
    
    // Set initial state (position, velocity, acceleration)
    Eigen::Matrix3d headState;
    headState.col(0) = current_pos;
    headState.col(1) = Eigen::Vector3d(0.0, 0.0, 0.0);  // Initial velocity
    headState.col(2) = Eigen::Vector3d(0.0, 0.0, 0.0);  // Initial acceleration
    
    // Set final state (last key pose)
    Eigen::Matrix3d tailState;
    tailState.col(0) = Eigen::Vector3d(key_poses_.back()[0], key_poses_.back()[1], key_poses_.back()[2]);
    tailState.col(1) = Eigen::Vector3d(0.0, 0.0, 0.0);  // Final velocity
    tailState.col(2) = Eigen::Vector3d(0.0, 0.0, 0.0);  // Final acceleration
    
    // Define intermediate waypoints (all key poses except the last one)
    Eigen::Matrix3Xd waypoints(3, pieceNum - 1);
    for (int i = 0; i < pieceNum - 1; i++)
    {
      waypoints.col(i) = Eigen::Vector3d(key_poses_[i][0], key_poses_[i][1], key_poses_[i][2]);
    }
    
    // Define time allocation for each piece
    Eigen::VectorXd timeAllocation(pieceNum);
    for (int i = 0; i < pieceNum; i++)
    {
      timeAllocation(i) = segment_time_;
    }
    
    // Generate trajectory
    minco.setConditions(headState, tailState, pieceNum);
    minco.setParameters(waypoints, timeAllocation);
    minco.getTrajectory(trajectory_);
    
    // Store full poses for trajectory execution
    full_key_poses_.clear();
    
    // Add current pose as first waypoint
    std::vector<double> current_pose(6);
    current_pose[0] = current_root_pose_.pose.position.x;
    current_pose[1] = current_root_pose_.pose.position.y;
    current_pose[2] = current_root_pose_.pose.position.z;
    
    // Convert quaternion to RPY
    tf::Quaternion q;
    tf::quaternionMsgToTF(current_root_pose_.pose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_pose[3] = roll;
    current_pose[4] = pitch;
    current_pose[5] = yaw;
    
    full_key_poses_.push_back(current_pose);
    
    // Add all key poses
    for (const auto& kp : key_poses_)
    {
      full_key_poses_.push_back(kp);
    }
    
    trajectory_generated_ = true;
    trajectory_start_time_ = ros::Time::now();
    
    ROS_INFO("[Root6DTrajDemo] Trajectory generated successfully!");
    ROS_INFO("[Root6DTrajDemo] Total duration: %.2f seconds", trajectory_.getTotalDuration());
    ROS_INFO("[Root6DTrajDemo] Number of pieces: %d", trajectory_.getPieceNum());
    
    // Precompute all control commands
    precomputeCommands();
    
    return true;
  }
  
  void precomputeCommands()
  {
    precomputed_commands_.clear();
    
    double total_duration = trajectory_.getTotalDuration();
    double dt = 1.0 / control_frequency_;
    int num_commands = static_cast<int>(total_duration / dt) + 1;
    
    ROS_INFO("[Root6DTrajDemo] Precomputing %d control commands...", num_commands);
    
    for (int idx = 0; idx < num_commands; idx++)
    {
      double elapsed_time = idx * dt;
      
      // Get position from trajectory at current time
      Eigen::Vector3d pos = trajectory_.getPos(elapsed_time);
      
      // Interpolate orientation
      int piece_idx = findPieceIndex(elapsed_time);
      double piece_time = getPieceTime(elapsed_time, piece_idx);
      double alpha = piece_time / segment_time_;  // Interpolation factor [0, 1]
      
      // Interpolate orientation (roll, pitch, yaw)
      std::vector<double> interpolated_pose = interpolatePose(piece_idx, alpha);
      
      // Update position from MINCO trajectory (more accurate)
      interpolated_pose[0] = pos.x();
      interpolated_pose[1] = pos.y();
      interpolated_pose[2] = pos.z();
      
      // Create PoseStamped message
      geometry_msgs::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.pose.position.x = interpolated_pose[0];
      msg.pose.position.y = interpolated_pose[1];
      msg.pose.position.z = interpolated_pose[2];
      
      // Convert roll, pitch, yaw to quaternion
      tf::Quaternion q;
      q.setRPY(interpolated_pose[3], interpolated_pose[4], interpolated_pose[5]);
      msg.pose.orientation.x = q.x();
      msg.pose.orientation.y = q.y();
      msg.pose.orientation.z = q.z();
      msg.pose.orientation.w = q.w();
      
      precomputed_commands_.push_back(msg);
    }
    
    current_command_index_ = 0;
    ROS_INFO("[Root6DTrajDemo] Precomputation completed! Total commands: %zu", precomputed_commands_.size());
  }
  
  void publishTimerCallback(const ros::TimerEvent& event)
  {
    publishTrajectoryCommand();
  }
  
  void publishTrajectoryCommand()
  {
    if (!trajectory_generated_)
    {
      return;
    }
    
    // Check if trajectory is complete
    if (current_command_index_ >= precomputed_commands_.size())
    {
      ROS_INFO("[Root6DTrajDemo] Trajectory execution completed!");
      trajectory_started_ = false;
      trajectory_generated_ = false;
      publish_timer_.stop();
      return;
    }
    
    // Get precomputed command and publish
    geometry_msgs::PoseStamped msg = precomputed_commands_[current_command_index_];
    msg.header.stamp = ros::Time::now();
    
    root_target_pose_pub_.publish(msg);
    
    current_command_index_++;
  }
  
  void run()
  {
    ros::Rate rate(10.0);  // Check rate for initial data
        
    // Wait for initial data
    while (ros::ok() && !has_root_pose_)
    {
      ros::spinOnce();
      rate.sleep();
    }
        
    // Generate trajectory once at the beginning
    if (!generateTrajectory())
    {
      ROS_ERROR("[Root6DTrajDemo] Failed to generate trajectory!");
      return;
    }
    
    trajectory_started_ = true;
    
    ROS_INFO("[Root6DTrajDemo] Starting trajectory execution...");
    
    // Start the timer for trajectory publishing
    publish_timer_.start();
    
    // Keep spinning
    ros::spin();
    
    ROS_INFO("[Root6DTrajDemo] Demo completed.");
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
  
  std::vector<std::vector<double>> key_poses_;  // [x, y, z, roll, pitch, yaw]
  std::vector<std::vector<double>> full_key_poses_;  // Includes current pose
  std::vector<geometry_msgs::PoseStamped> precomputed_commands_;  // Precomputed control commands
  
  Trajectory<5> trajectory_;
  bool trajectory_started_;
  bool trajectory_generated_;
  ros::Time trajectory_start_time_;
  int current_command_index_;
  
  int findPieceIndex(double elapsed_time)
  {
    double accumulated_time = 0.0;
    for (int i = 0; i < trajectory_.getPieceNum(); i++)
    {
      accumulated_time += segment_time_;
      if (elapsed_time < accumulated_time)
      {
        return i;
      }
    }
    return trajectory_.getPieceNum() - 1;
  }
  
  double getPieceTime(double elapsed_time, int piece_idx)
  {
    double piece_start_time = piece_idx * segment_time_;
    return elapsed_time - piece_start_time;
  }
  
  std::vector<double> interpolatePose(int piece_idx, double alpha)
  {
    // Clamp alpha to [0, 1]
    alpha = std::max(0.0, std::min(1.0, alpha));
    
    // Get start and end poses for this piece
    const std::vector<double>& start_pose = full_key_poses_[piece_idx];
    const std::vector<double>& end_pose = full_key_poses_[piece_idx + 1];
    
    std::vector<double> interpolated(6);
    
    // Linear interpolation for position and orientation
    for (int i = 0; i < 6; i++)
    {
      interpolated[i] = start_pose[i] + alpha * (end_pose[i] - start_pose[i]);
    }
    
    return interpolated;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "root_6D_traj_demo");
  
  ROS_INFO("========================================");
  ROS_INFO("  Simple Pose Demo for Dragon");
  ROS_INFO("========================================");
  
  Root6DTrajDemo demo;
  demo.run();
  
  return 0;
}
