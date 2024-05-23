#include "tree_tracking.h"
#include "circle_detection.h"

TreeTracking::TreeTracking(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp),
   tree_db_(nh, nhp)
{
  nhp_.param("urg_yaw_offset", urg_yaw_offset_, 0.0);
  nhp_.param("laser_scan_topic_name", laser_scan_topic_name_, string("scan"));
  nhp_.param("uav_odom_topic_name", odom_topic_name_, string("odom"));
  nhp_.param("tree_radius_max", tree_radius_max_, 0.3);
  nhp_.param("tree_radius_min", tree_radius_min_, 0.08);
  nhp_.param("tree_scan_angle_thre", tree_scan_angle_thre_, 0.1);
  nhp_.param("tree_circle_regulation_thre", tree_circle_regulation_thre_, 0.01);
  nhp_.param("tree_global_location_topic_name", tree_global_location_topic_name_, string("tree_global_location"));
  nhp_.param("visualization_marker_topic_name", visualization_marker_topic_name_, string("visualization_marker"));

  sub_laser_scan_ = nh_.subscribe(laser_scan_topic_name_, 1, &TreeTracking::laserScanCallback, this);
  sub_odom_ = nh_.subscribe(odom_topic_name_, 1, &TreeTracking::uavOdomCallback, this);
  pub_visualization_marker_ = nh_.advertise<visualization_msgs::MarkerArray>(visualization_marker_topic_name_, 1);
}

// void TreeTracking::visionDetectionCallback(const geometry_msgs::Vector3StampedConstPtr& vision_detection_msg)
// {
//   ROS_WARN("tree tracking: start tracking");
//   /* stop the vision detection */
//   std_msgs::Bool stop_msg;
//   stop_msg.data = false;
//   pub_stop_vision_detection_.publish(stop_msg);

//   tf::Matrix3x3 rotation;
//   rotation.setRPY(0, 0, vision_detection_msg->vector.y + uav_yaw_ + urg_yaw_offset_);
//   tf::Vector3 target_tree_global_location = uav_odom_ + rotation * tf::Vector3(vision_detection_msg->vector.z, 0, 0);
//   initial_target_tree_direction_vec_ = rotation * tf::Vector3(vision_detection_msg->vector.z, 0, 0);
//   initial_target_tree_direction_vec_ /= initial_target_tree_direction_vec_.length(); //normalize

//   /* start laesr-only subscribe */
//   sub_laser_scan_ = nh_.subscribe(laser_scan_topic_name_, 1, &TreeTracking::laserScanCallback, this);

//   /* add target tree to the tree data base */
//   TreeHandlePtr new_tree = TreeHandlePtr(new TreeHandle(nh_, nhp_, target_tree_global_location));
//   tree_db_.add(new_tree);
//   tree_db_.setCenterTree(new_tree);
//   target_trees_.resize(0);
//   target_trees_.push_back(new_tree);

//   /* set the search center as the first target tree(with color marker) pos */
//   search_center_ = target_tree_global_location;

//   sub_vision_detection_.shutdown(); //stop
// }

void TreeTracking::uavOdomCallback(const nav_msgs::OdometryConstPtr& uav_msg)
{
  tf::Quaternion uav_q(uav_msg->pose.pose.orientation.x,
                       uav_msg->pose.pose.orientation.y,
                       uav_msg->pose.pose.orientation.z,
                       uav_msg->pose.pose.orientation.w);
  tf::Matrix3x3  uav_orientation_(uav_q);
  tfScalar r,p,y;
  uav_orientation_.getRPY(r, p, y);
  uav_odom_.setX(uav_msg->pose.pose.position.x);
  uav_odom_.setY(uav_msg->pose.pose.position.y);

  /* we only consider in the 2D space */
  //uav_odom_.setZ(uav_msg->pose.pose.position.z);
  uav_odom_.setZ(0);
  uav_roll_ = r; uav_pitch_ = p; uav_yaw_ = y;
}

void TreeTracking::laserScanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
   ROS_INFO("receive new laser scan");

  /* extract the cluster */
  vector<int> cluster_index;
  for (size_t i = 0; i < scan_msg->ranges.size(); i++)
      if(scan_msg->ranges[i] > 0) cluster_index.push_back(i);

  /* find the tree most close to the previous target tree */
  bool target_update = false;
  // int prev_vote = target_trees_.back()->getVote();
  for ( vector<int>::iterator it = cluster_index.begin(); it != cluster_index.end(); ++it)
    {
      // /* we do not update trees pos if there is big tilt */
      // if(fabs(uav_pitch_) > uav_tilt_thre_)
      //   {
      //     ROS_WARN("Too much tilt: %f", uav_pitch_);
      //     break;
      //   }

      tf::Vector3 tree_global_location;

      /* calculate the distance */
      tf::Matrix3x3 rotation;
      rotation.setRPY(0, 0, *it * scan_msg->angle_increment + scan_msg->angle_min + uav_yaw_ + urg_yaw_offset_);
      tree_global_location = uav_odom_ + rotation * tf::Vector3(scan_msg->ranges[*it], 0, 0);

      /* add tree to the database */
      // if(verbose_) cout << "Scan input tree No." << distance(cluster_index.begin(), it) << ": start update" << endl;

      /* calc radius with circle fitting */
          vector<tf::Vector3> points;
	  int scan_point_num = 0;
          for (int i = *it; !isnan(scan_msg->ranges[i]) && i < scan_msg->ranges.size(); i++)
            {
              double r = fabs(scan_msg->ranges[i]);
              double theta = scan_msg->angle_min + (scan_msg->angle_increment) * i;
              tf::Vector3 point(r * cos(theta), r * sin(theta), 0);
              points.push_back(point);
	      scan_point_num++;
            }
          for (int i = (*it) - 1; !isnan(scan_msg->ranges[i]) && i > 0; i--)
            {
              double r = fabs(scan_msg->ranges[i]);
              double theta = scan_msg->angle_min + (scan_msg->angle_increment) * i;
              tf::Vector3 point(r * cos(theta), r * sin(theta), 0);
              points.push_back(point);
	      scan_point_num++;
            }
	  /* calc position and radius */
	  tf::Vector3 tree_center_pos; double tree_radius, regulation;
	  CircleDetection::circleFitting(points, tree_center_pos, tree_radius, regulation);
	  /* angle filter */
	  double scan_angle_real = scan_point_num * scan_msg->angle_increment;
	  double scan_angle_virtual = M_PI - 2 * acos(tree_radius / tree_center_pos.length());
	  /* tree position filter */
	  rotation.setRPY(0, 0, uav_yaw_ + urg_yaw_offset_);
	  tf::Vector3 tree_center_global_location = uav_odom_ + rotation * tf::Vector3(tree_center_pos.x(), tree_center_pos.y(), 0);
	  // tf::Vector3 initial_target_tree_pos = target_trees_.at(0)->getPos();
	  // double projected_length_from_initial_target = initial_target_tree_direction_vec_.dot(tree_center_global_location - initial_target_tree_pos); 
	  

	  if (tree_radius > tree_radius_min_ && tree_radius < tree_radius_max_ && fabs((scan_angle_real - scan_angle_virtual) / scan_angle_real) < tree_scan_angle_thre_ && regulation < tree_circle_regulation_thre_)
            {
               target_update += tree_db_.updateSingleTree(tree_center_global_location, tree_radius, false);
            }
          else
            {
              // if(verbose_)
                ROS_INFO("radius: %f, min: %f, max: %f", tree_radius, tree_radius_min_, tree_radius_max_);
            }
      // else
      //   {
      //     target_update += tree_db_.updateSingleTree(tree_global_location, 0.2, only_target_);
      //   }
    }

  /* update the whole database(sorting) */
  tree_db_.update();

  /* send command for policy system */
  tree_db_.visualization(scan_msg->header);
}
