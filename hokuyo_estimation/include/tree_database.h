#ifndef TREE_DATABASE_H_
#define TREE_DATABASE_H_

/* ros */
#include <ros/ros.h>

/* ros msg/srv */
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

/* uitls */
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <cstdlib>

using namespace std;

class TreeHandle
{
public:
  TreeHandle(): pos_(0,0,0), vote_(0), radius_(0) {}
  TreeHandle(ros::NodeHandle nh, ros::NodeHandle nhp, tf::Vector3 pos);
  ~TreeHandle(){}

  boost::shared_ptr<TreeHandle> getHandle() { return boost::shared_ptr<TreeHandle>(this); }
  void updatePos(const tf::Vector3& pos, bool lpf = true);
  void setRadius(double radius, bool lpf = true);
  const double getRadius(){ return radius_; }
  const tf::Vector3 getPos() { return pos_; }
  inline void setVote(int vote) {vote_ = vote; }
  inline int getVote() { return vote_; }

private:
  ros::NodeHandle nh_, nhp_;

  double filter_rate_;

  tf::Vector3 pos_;
  double radius_;
  int vote_;
};

typedef boost::shared_ptr<TreeHandle>  TreeHandlePtr;

bool operator>(const TreeHandlePtr& left, const TreeHandlePtr& right);

class TreeDataBase
{
public:
  TreeDataBase(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~TreeDataBase(){}

  void add(const TreeHandlePtr new_tree);
  bool updateSingleTree(const tf::Vector3& tree_pos, const double& tree_radius, const bool only_target = false);

  void update();
  void setCenterTree(TreeHandlePtr center_tree) { center_tree_ = center_tree; }
  void visualization(std_msgs::Header header);
  void eraseTreeDB();
  void save();
  bool load(string file_name);

  int getIndex(TreeHandlePtr target_tree);

  int validTreeNum()
  {
    return (trees_.size()>valid_num_)?valid_num_:trees_.size();
  }

  inline void getTrees(vector<TreeHandlePtr>& trees) { trees = trees_; }
private:
  ros::NodeHandle nh_, nhp_;

  /* ros param */
  double tree_margin_radius_; /* the margin area to check the candidate position for a tree  */
  int valid_num_; /* the number of valid tree to  */
  bool verbose_;
  bool visualization_;
  double tree_cut_rate_;
  string visualization_marker_topic_name_;

  ros::Publisher pub_visualization_marker_;

  /* the set for trees */
  vector<TreeHandlePtr> trees_;
  TreeHandlePtr center_tree_;
};
#endif
