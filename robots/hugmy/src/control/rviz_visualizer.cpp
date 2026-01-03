#include <hugmy/control/haptics_visualizer.h>

HapticsVisualizer::HapticsVisualizer(ros::NodeHandle& nh)
    : HapticsController(nh)
{
    marker_array_pub_ = nh.advertise <visualization_msgs::MarkerArray>("/debug/target_vector", 1);
    norm_pub_ = nh.advertise<std_msgs::Float32>("/debug/target_vector_norm", 1);
    thrust_0_pub_ = nh.advertise<std_msgs::Float32>("/debug/thrust_1", 1);
    thrust_1_pub_ = nh.advertise<std_msgs::Float32>("/debug/thrust_2", 1);
    thrust_2_pub_ = nh.advertise<std_msgs::Float32>("/debug/thrust_3", 1);
    thrust_3_pub_ = nh.advertise<std_msgs::Float32>("/debug/thrust_4", 1);
}

void HapticsVisualizer::updateRviz() {
  visualization_msgs::MarkerArray marker_array;

  geometry_msgs::Point current_pos;
  current_pos.x = pose_.position.x;
  current_pos.y = pose_.position.y;
  current_pos.z = 0.8;
  
  geometry_msgs::Point target_pos;
  target_pos.x = target_x_;
  target_pos.y = target_y_;
  target_pos.z = 0.8;

  // target_vec
  Eigen::Vector2d target_vec = Eigen::Vector2d(target_x_, target_y_) - Eigen::Vector2d(current_pos.x, current_pos.y);
  // ROS_ERROR("Target vector: (%.2f, %.2f)", target_vec.x(), target_vec.y());
  publishVectorArrow(target_vec, current_pos, "target_vector", 0, 1.0, 0.0, 0.0);

  // last_vec
  Eigen::Vector2d last_vec = Eigen::Vector2d(target_x_, target_y_) - Eigen::Vector2d(last_pos_.x, last_pos_.y);
  geometry_msgs::Point last_pos;
  last_pos.x = last_pos_.x;
  last_pos.y = last_pos_.y;
  last_pos.z = 0.8;
  publishVectorArrow(last_vec, last_pos, "last_vector", 1, 0.0, 1.0, 0.0);

  visualization_msgs::Marker current_point;
  current_point.header.frame_id = "world";
  current_point.header.stamp = ros::Time::now();
  current_point.ns = "current_position";
  current_point.id = 2;
  current_point.type = visualization_msgs::Marker::SPHERE;
  current_point.action = visualization_msgs::Marker::ADD;
  current_point.pose.position = current_pos;
  current_point.pose.orientation.w = 1.0;
  current_point.scale.x = 0.1;
  current_point.scale.y = 0.1;
  current_point.scale.z = 0.1;
  current_point.color.r = 0.0;
  current_point.color.g = 0.0;
  current_point.color.b = 1.0;
  current_point.color.a = 1.0;
  marker_array.markers.push_back(current_point);
  

  
  visualization_msgs::Marker target_point;
  target_point.header.frame_id = "world";
  target_point.header.stamp = ros::Time::now();
  target_point.ns = "target_position";
  target_point.id = 3;
  target_point.type = visualization_msgs::Marker::SPHERE;
  target_point.action = visualization_msgs::Marker::ADD;
  target_point.pose.position = target_pos;
  target_point.pose.orientation.w = 1.0;
  target_point.scale.x = 0.12;
  target_point.scale.y = 0.12;
  target_point.scale.z = 0.12;
  target_point.color.r = 1.0;
  target_point.color.g = 0.0;
  target_point.color.b = 0.0;
  target_point.color.a = 1.0;
  marker_array.markers.push_back(target_point);
  
  //waypoints
  if (!waypoints_.empty()) {
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      visualization_msgs::Marker wp_marker;
      wp_marker.header.frame_id = "world";
      wp_marker.header.stamp = ros::Time::now();
      wp_marker.ns = "waypoints";
      wp_marker.id = 100 + static_cast<int>(i);
      
      wp_marker.type = visualization_msgs::Marker::SPHERE;
      wp_marker.action = visualization_msgs::Marker::ADD;

      wp_marker.pose.position.x = waypoints_[i].x();
      wp_marker.pose.position.y = waypoints_[i].y();
      wp_marker.pose.position.z = 0.8;
      wp_marker.pose.orientation.w = 1.0;

      wp_marker.scale.x = 0.8;
      wp_marker.scale.y = 0.8;
      wp_marker.scale.z = 0.8;
      
      if (static_cast<int>(i) == current_wp_idx_) {
	wp_marker.color.r = 1.0;
	wp_marker.color.g = 1.0;
	wp_marker.color.b = 0.0;
	wp_marker.color.a = 0.6;
      } else {
	wp_marker.color.r = 0.0;
	wp_marker.color.g = 1.0;
	wp_marker.color.b = 1.0;
	wp_marker.color.a = 0.6;
      }
      marker_array.markers.push_back(wp_marker);
    }
    
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "world";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "waypoint_path";
    path_marker.id = 200;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    
    path_marker.scale.x = 0.03;
    path_marker.color.r = 0.8;
    path_marker.color.g = 0.8;
    path_marker.color.b = 0.8;
    path_marker.color.a = 1.0;
    
    geometry_msgs::Point p;
    
    p.x = current_pos.x;
    p.y = current_pos.y;
    p.z = 0.8;
    path_marker.points.push_back(p);
    
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      p.x = waypoints_[i].x();
      p.y = waypoints_[i].y();
      p.z = 0.8;
      path_marker.points.push_back(p);
    }
    
    marker_array.markers.push_back(path_marker);
  }
  
  marker_array.markers.insert(marker_array.markers.end(),
			      arrow_array_.markers.begin(), arrow_array_.markers.end());
  marker_array_pub_.publish(marker_array);
  arrow_array_.markers.clear();
  
  
  std_msgs::Float32 norm_msg;
  norm_msg.data = target_vec.norm();
  norm_pub_.publish(norm_msg);
  
  std_msgs::Float32 thrust_0_msg, thrust_1_msg, thrust_2_msg, thrust_3_msg;
  thrust_0_msg.data = motor_pwms_[0];
  thrust_1_msg.data = motor_pwms_[1];
  thrust_2_msg.data = motor_pwms_[2];
  thrust_3_msg.data = motor_pwms_[3];
  thrust_0_pub_.publish(thrust_0_msg);
  thrust_1_pub_.publish(thrust_1_msg);
  thrust_2_pub_.publish(thrust_2_msg);
  thrust_3_pub_.publish(thrust_3_msg);

}

void HapticsVisualizer::publishVectorArrow(const Eigen::Vector2d& vec, const geometry_msgs::Point& start_point,
                                                const std::string& ns, int id, float r, float g, float b) {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    arrow.header.stamp = ros::Time::now();
    arrow.ns = ns;
    arrow.id = id;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    arrow.scale.x = 0.05;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1.0;  // 不透明度

    geometry_msgs::Point end_point;
    // 矢印の終点はベクトルの終点
    end_point.x = start_point.x + vec.x();
    end_point.y = start_point.y + vec.y();
    end_point.z = start_point.z;  // 高さは同じにする

    arrow.points.push_back(start_point);
    arrow.points.push_back(end_point);
    arrow_array_.markers.push_back(arrow);
}
