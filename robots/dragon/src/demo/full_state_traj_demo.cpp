#include <ros/ros.h>
#include <aerial_robot_msgs/FullStateTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <Eigen/Eigen>
#include "aerial_robot_control/minco_trajectory/minco.hpp"
#include "aerial_robot_control/minco_trajectory/trajectory.hpp"

class FullStateTraj
{
public:
  FullStateTraj() : nh_("~"), trajectory_started_(false), trajectory_generated_(false)
  {
    // Initialize ROS parameters
    nh_.param("control_frequency", control_frequency_, 40.0);
    nh_.param("segment_time", segment_time_, 10.0);
    nh_.param("joint_num", joint_num_, 6);
    
    // Initialize publishers and subscribers
    full_state_target_pub_ = nh_.advertise<aerial_robot_msgs::FullStateTarget>("/dragon/full_state_target", 10);
    root_target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/dragon/root/target_pose", 10);
    root_pose_sub_ = nh_.subscribe("/dragon/root/pose", 10, &FullStateTraj::rootPoseCallback, this);
    joint_state_sub_ = nh_.subscribe("/dragon/joint_states", 10, &FullStateTraj::jointStateCallback, this);
    
    // Create timer for trajectory publishing
    double timer_period = 1.0 / control_frequency_;
    publish_timer_ = nh_.createTimer(ros::Duration(timer_period), &FullStateTraj::publishTimerCallback, this, false, false);

    // Load key states from parameter server
    loadKeyStates();
    
    ROS_INFO("[FullStateTraj] Node initialized");
    ROS_INFO("[FullStateTraj] Control frequency: %.1f Hz", control_frequency_);
    ROS_INFO("[FullStateTraj] Segment time: %.1f seconds", segment_time_);
    ROS_INFO("[FullStateTraj] Number of key states: %zu", key_states_.size());
  }
  
  void rootPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    current_root_pose_ = *msg;
    has_root_pose_ = true;
  }
  
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    current_joint_state_ = *msg;
    
    // Initialize link_joint_indices_ on first callback
    if (link_joint_indices_.empty() && !msg->name.empty())
    {
      for (size_t i = 0; i < msg->name.size(); i++)
      {
        const std::string& joint_name = msg->name[i];
        // Link joints are named like "joint1_pitch", "joint1_yaw", etc.
        if (joint_name.find("joint") != std::string::npos && 
            (joint_name.find("pitch") != std::string::npos || 
             joint_name.find("yaw") != std::string::npos))
        {
          link_joint_indices_.push_back(i);
        }
      }
      ROS_INFO("[FullStateTraj] Detected %zu link joints", link_joint_indices_.size());
    }
    
    has_joint_state_ = true;
  }
  
  void loadKeyStates()
  {
    // Try to load key states from parameter server
    XmlRpc::XmlRpcValue key_states_param;
    if (nh_.getParam("key_states", key_states_param))
    {
      if (key_states_param.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("[FullStateTraj] key_states parameter is not an array!");
        ros::shutdown();
        return;
      }
      
      for (int i = 0; i < key_states_param.size(); i++)
      {
        if (key_states_param[i].getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_ERROR("[FullStateTraj] key_states[%d] is not an array!", i);
          continue;
        }
        
        std::vector<double> state;
        for (int j = 0; j < key_states_param[i].size(); j++)
        {
          if (key_states_param[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
          {
            state.push_back(static_cast<double>(key_states_param[i][j]));
          }
          else if (key_states_param[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt)
          {
            state.push_back(static_cast<int>(key_states_param[i][j]));
          }
        }
        
        if (state.size() == 12)
        {
          key_states_.push_back(state);
        }
        else
        {
          ROS_WARN("[FullStateTraj] key_states[%d] has wrong size: %zu (expected 12)", i, state.size());
        }
      }
      
      if (key_states_.empty())
      {
        ROS_ERROR("[FullStateTraj] No valid key states loaded!");
        ros::shutdown();
        return;
      }
      
      ROS_INFO("[FullStateTraj] Loaded %zu key states from parameter server", key_states_.size());
    }
    else
    {
      ROS_ERROR("[FullStateTraj] No key_states parameter found! Please provide a configuration file.");
      ROS_ERROR("[FullStateTraj] Use: roslaunch dragon full_state_traj_demo.launch config_file:=<path_to_config>");
      ros::shutdown();
    }
  }
  
  bool generateTrajectory()
  {
    if (!has_root_pose_ || !has_joint_state_)
    {
      ROS_WARN("[FullStateTraj] Waiting for root pose and joint state data...");
      return false;
    }
    
    // Extract current state from odometry and joint states
    std::vector<double> current_state = getCurrentState();
    
    ROS_INFO("[FullStateTraj] Current state: pos=[%.2f, %.2f, %.2f], rpy=[%.2f, %.2f, %.2f]",
             current_state[0], current_state[1], current_state[2],
             current_state[3], current_state[4], current_state[5]);
    
    // Number of trajectory pieces (current state + all key states)
    int pieceNum = key_states_.size();
    
    // Create MINCO instance for minimum jerk trajectory
    minco::MINCO_S3NU minco;
    
    // Set initial state (position, velocity, acceleration)
    Eigen::Matrix3d headState;
    headState.col(0) = Eigen::Vector3d(current_state[0], current_state[1], current_state[2]);
    headState.col(1) = Eigen::Vector3d(0.0, 0.0, 0.0);  // Initial velocity
    headState.col(2) = Eigen::Vector3d(0.0, 0.0, 0.0);  // Initial acceleration
    
    // Set final state (last key state)
    Eigen::Matrix3d tailState;
    tailState.col(0) = Eigen::Vector3d(key_states_.back()[0], key_states_.back()[1], key_states_.back()[2]);
    tailState.col(1) = Eigen::Vector3d(0.0, 0.0, 0.0);  // Final velocity
    tailState.col(2) = Eigen::Vector3d(0.0, 0.0, 0.0);  // Final acceleration
    
    // Define intermediate waypoints (all key states except the last one)
    Eigen::Matrix3Xd waypoints(3, pieceNum - 1);
    for (int i = 0; i < pieceNum - 1; i++)
    {
      waypoints.col(i) = Eigen::Vector3d(key_states_[i][0], key_states_[i][1], key_states_[i][2]);
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
    
    // Store full states for trajectory execution
    full_key_states_.clear();
    full_key_states_.push_back(current_state);  // Add current state as first waypoint
    for (const auto& ks : key_states_)
    {
      full_key_states_.push_back(ks);
    }
    
    trajectory_generated_ = true;
    trajectory_start_time_ = ros::Time::now();
    
    ROS_INFO("[FullStateTraj] Trajectory generated successfully!");
    ROS_INFO("[FullStateTraj] Total duration: %.2f seconds", trajectory_.getTotalDuration());
    ROS_INFO("[FullStateTraj] Number of pieces: %d", trajectory_.getPieceNum());
    
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
    
    ROS_INFO("[FullStateTraj] Precomputing %d control commands...", num_commands);
    
    for (int idx = 0; idx < num_commands; idx++)
    {
      double elapsed_time = idx * dt;
      
      // Get position and velocity from trajectory at current time
      Eigen::Vector3d pos = trajectory_.getPos(elapsed_time);
      Eigen::Vector3d vel = trajectory_.getVel(elapsed_time);
      
      // Interpolate orientation and joint states
      int piece_idx = findPieceIndex(elapsed_time);
      double piece_time = getPieceTime(elapsed_time, piece_idx);
      double alpha = piece_time / segment_time_;  // Interpolation factor [0, 1]
      
      // Interpolate full state
      std::vector<double> interpolated_state = interpolateState(piece_idx, alpha);
      
      // Update position from MINCO trajectory (more accurate)
      interpolated_state[0] = pos.x();
      interpolated_state[1] = pos.y();
      interpolated_state[2] = pos.z();
      
      // Create FullStateTarget message
      aerial_robot_msgs::FullStateTarget msg;
      msg.header.frame_id = "world";
      
      // Set root state
      msg.root_state.child_frame_id = "root";
      msg.root_state.pose.pose.position.x = interpolated_state[0];
      msg.root_state.pose.pose.position.y = interpolated_state[1];
      msg.root_state.pose.pose.position.z = interpolated_state[2];
      
      // Convert roll, pitch, yaw to quaternion
      tf::Quaternion q;
      q.setRPY(interpolated_state[3], interpolated_state[4], interpolated_state[5]);
      msg.root_state.pose.pose.orientation.x = q.x();
      msg.root_state.pose.pose.orientation.y = q.y();
      msg.root_state.pose.pose.orientation.z = q.z();
      msg.root_state.pose.pose.orientation.w = q.w();
      
      // Set linear velocity from trajectory
      msg.root_state.twist.twist.linear.x = vel.x();
      msg.root_state.twist.twist.linear.y = vel.y();
      msg.root_state.twist.twist.linear.z = vel.z();
      
      msg.root_state.twist.twist.angular.x = 0.0;
      msg.root_state.twist.twist.angular.y = 0.0;
      msg.root_state.twist.twist.angular.z = 0.0;
      
      // Set joint state
      msg.joint_state.position.clear();
      for (int i = 6; i < 6 + joint_num_; i++)
      {
        msg.joint_state.position.push_back(interpolated_state[i]);
      }
      
      precomputed_commands_.push_back(msg);
    }
    
    current_command_index_ = 0;
    ROS_INFO("[FullStateTraj] Precomputation completed! Total commands: %zu", precomputed_commands_.size());
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
      ROS_INFO("[FullStateTraj] Trajectory execution completed!");
      trajectory_started_ = false;
      trajectory_generated_ = false;
      publish_timer_.stop();
      return;
    }
    
    // Get precomputed command and publish
    aerial_robot_msgs::FullStateTarget msg = precomputed_commands_[current_command_index_];
    msg.header.stamp = ros::Time::now();
    msg.root_state.header.stamp = msg.header.stamp;
    msg.joint_state.header.stamp = msg.header.stamp;
    
    full_state_target_pub_.publish(msg);
    
    // Publish root target pose
    geometry_msgs::PoseStamped root_pose_msg;
    root_pose_msg.header.stamp = msg.header.stamp;
    root_pose_msg.header.frame_id = "world";
    root_pose_msg.pose = msg.root_state.pose.pose;
    root_target_pose_pub_.publish(root_pose_msg);
    
    current_command_index_++;
  }
  
  void run()
  {
    ros::Rate rate(10.0);  // Check rate for initial data
        
    // Wait for initial data
    while (ros::ok() && (!has_root_pose_ || !has_joint_state_))
    {
      ros::spinOnce();
      rate.sleep();
    }
        
    // Generate trajectory once at the beginning
    if (!generateTrajectory())
    {
      ROS_ERROR("[FullStateTraj] Failed to generate trajectory!");
      return;
    }
    
    trajectory_started_ = true;
    
    ROS_INFO("[FullStateTraj] Starting trajectory execution...");
    
    // Start the timer for trajectory publishing
    publish_timer_.start();
    
    // Keep spinning
    ros::spin();
    
    ROS_INFO("[FullStateTraj] Demo completed.");
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher full_state_target_pub_;
  ros::Publisher root_target_pose_pub_;
  ros::Subscriber root_pose_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Timer publish_timer_;
  
  double control_frequency_;
  double segment_time_;
  int joint_num_;
  std::vector<int> link_joint_indices_;
  
  geometry_msgs::PoseStamped current_root_pose_;
  sensor_msgs::JointState current_joint_state_;
  bool has_root_pose_ = false;
  bool has_joint_state_ = false;
  
  std::vector<std::vector<double>> key_states_;
  std::vector<std::vector<double>> full_key_states_;  // Includes current state
  std::vector<aerial_robot_msgs::FullStateTarget> precomputed_commands_;  // Precomputed control commands
  
  Trajectory<5> trajectory_;
  bool trajectory_started_;
  bool trajectory_generated_;
  ros::Time trajectory_start_time_;
  int current_command_index_;
  
  std::vector<double> getCurrentState()
  {
    std::vector<double> state(12, 0.0);
    
    // Position
    state[0] = current_root_pose_.pose.position.x;
    state[1] = current_root_pose_.pose.position.y;
    state[2] = current_root_pose_.pose.position.z;
    
    // Orientation (convert quaternion to RPY)
    tf::Quaternion q;
    tf::quaternionMsgToTF(current_root_pose_.pose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    state[3] = roll;
    state[4] = pitch;
    state[5] = yaw;
    
    // Joint states (extract link joint positions using indices)
    for (int i = 0; i < std::min(joint_num_, (int)link_joint_indices_.size()); i++)
    {
      int joint_idx = link_joint_indices_[i];
      if (joint_idx < (int)current_joint_state_.position.size())
      {
        state[6 + i] = current_joint_state_.position[joint_idx];
      }
    }
    
    return state;
  }
  
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
  
  std::vector<double> interpolateState(int piece_idx, double alpha)
  {
    // Clamp alpha to [0, 1]
    alpha = std::max(0.0, std::min(1.0, alpha));
    
    // Get start and end states for this piece
    const std::vector<double>& start_state = full_key_states_[piece_idx];
    const std::vector<double>& end_state = full_key_states_[piece_idx + 1];
    
    std::vector<double> interpolated(12);
    
    // Linear interpolation for all states
    for (int i = 0; i < 12; i++)
    {
      interpolated[i] = start_state[i] + alpha * (end_state[i] - start_state[i]);
    }
    
    return interpolated;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "full_state_traj_demo");
  
  ROS_INFO("========================================");
  ROS_INFO("  Full State Trajectory Demo for Dragon");
  ROS_INFO("========================================");
  
  FullStateTraj demo;
  demo.run();
  
  return 0;
}
