#include "tree_database.h"
 
TreeHandle::TreeHandle(ros::NodeHandle nh, ros::NodeHandle nhp, tf::Vector3 pos):nh_(nh), nhp_(nhp), pos_(pos), vote_(1), radius_(-1)
{
  nhp_.param("filter_rate", filter_rate_, 0.8);
}

void TreeHandle::updatePos(const tf::Vector3& pos, bool lpf)
{
  if(lpf) pos_ = filter_rate_ * pos_  + (1 - filter_rate_) * pos;
  else pos_ = pos;
  vote_++;
}

void TreeHandle::setRadius(double radius, bool lpf)
{
  if (radius_ < 0) {
    radius_ = radius; //init
  } else {
    if(lpf) radius_ = filter_rate_ * radius_  + (1 - filter_rate_) * radius;
    else radius_ = radius;
  }
}

bool operator>(const TreeHandlePtr& left, const TreeHandlePtr& right)
{
  return left->getVote() > right->getVote() ;
}

TreeDataBase::TreeDataBase(ros::NodeHandle nh, ros::NodeHandle nhp):nh_(nh), nhp_(nhp)
{
  trees_.resize(0);
  nhp_.param("tree_margin_radius", tree_margin_radius_, 1.0); // 1.0[m]
  nhp_.param("valid_num", valid_num_, 7);
  nhp_.param("verbose", verbose_, false);
  nhp_.param("visualization_marker_topic_name", visualization_marker_topic_name_, string("/visualization_marker"));
  nhp_.param("tree_cut_rate", tree_cut_rate_, 0.1);
  pub_visualization_marker_ = nh_.advertise<visualization_msgs::MarkerArray>(visualization_marker_topic_name_, 1);
}

void TreeDataBase::add(const TreeHandlePtr new_tree)
{
  trees_.push_back(new_tree);
  ROS_INFO("add new tree No.%d: [%f, %f]", (int)trees_.size(), new_tree->getPos().x(), new_tree->getPos().y());
}

bool TreeDataBase::updateSingleTree(const tf::Vector3& tree_pos, const double& tree_radius, const bool only_target)
{
  bool new_tree = true;
  float min_dist = 1e6;
  TreeHandlePtr target_tree;
  int tree_index = 0;
  for(vector<TreeHandlePtr>::iterator it = trees_.begin(); it != trees_.end(); ++it)
    {
      float dist = (tree_pos - (*it)->getPos()).length();
      size_t index = distance(trees_.begin(), it);

      /* we assume that the distance of any two trees is more than min_ditance_ */
      if(dist < tree_margin_radius_)
	{
          if(!new_tree)
            {
              if(verbose_) ROS_WARN("there are two trees which are to close to each other");
            }
	  new_tree = false;
	}

      /* update */
      if(min_dist > dist)
	{
	  min_dist = dist;
	  target_tree = *it;
	  tree_index = index;
	  if(verbose_) cout << "Database tree No." << tree_index << ": udpate min_dist: " << min_dist << endl;
	}
    }

  /* add new tree if necessary */
  if(new_tree)
    {
      if(!only_target)
        {
          TreeHandlePtr new_tree = TreeHandlePtr(new TreeHandle(nh_, nhp_, tree_pos));
          new_tree->setRadius(tree_radius);
          add(new_tree);
          return true;
        }
    }
  else
    {
      /* update the global pos of the tree */
      target_tree->updatePos(tree_pos,false);
      target_tree->setRadius(tree_radius);
      if(verbose_) cout << "Database tree No." << tree_index << ": update, small diff:" << min_dist << endl;
      return true;
    }

  return false;
}

void TreeDataBase::update()
{
  /* sort */
  std::sort(trees_.begin(),trees_.end(),std::greater<TreeHandlePtr>());

  if(verbose_)
    {
      cout << "update(sort): ";
      for (vector<TreeHandlePtr>::iterator it = trees_.begin(); it != trees_.end(); it++)
	cout << (*it)->getVote() << ", ";
      cout << endl;
    }
}

void TreeDataBase::visualization(std_msgs::Header header)
{
  visualization_msgs::MarkerArray msg;

  for (vector<TreeHandlePtr>::iterator it = trees_.begin(); it != trees_.end(); it++) {
    size_t index = distance(trees_.begin(), it);

    /* only show top level trees */
    if(index == valid_num_) break;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "tree_diameter";
    marker.id = index;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (*it)->getPos().x();
    marker.pose.position.y = (*it)->getPos().y();
    marker.pose.position.z = 3.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.5;
    marker.color.g = 1.0;
    marker.color.a = 1.0;
    ostringstream sout;
    sout << fixed << setprecision(3) << (((*it)->getRadius()) * 2);
    marker.text = sout.str();
    marker.lifetime = ros::Duration();
    msg.markers.push_back(marker);

    marker.ns = "tree";
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.z = 2.0; //tree height
    marker.scale.x = marker.scale.y = (*it)->getRadius() * 2;
    marker.pose.position.z = marker.scale.z / 2;
    marker.color.r = 0.95;
    marker.color.g = 0.59;
    if(*it == center_tree_) marker.color.g = 0.1; //special color for center tree
    marker.color.b = 0;
    marker.color.a = 0.5;
    msg.markers.push_back(marker);
  }
  pub_visualization_marker_.publish(msg);
}


void TreeDataBase::save()
{
  //boost::posix_time::ptime t = ros::Time::now().toBoost();
  boost::posix_time::ptime t = boost::posix_time::second_clock::local_time();
  boost::gregorian::date d = t.date();
  std::string date = boost::gregorian::to_iso_extended_string(d);
  std::ostringstream h_os; h_os << t.time_of_day().hours();
  std::ostringstream m_os; m_os << t.time_of_day().minutes();

  std::ofstream ofs;
  ofs.open(std::getenv("HOME") + string("/.ros/") +
           date + string("-") + h_os.str() + string("-") +
           m_os.str() + string("-") + string("trees.yaml"));

  int center_index = getIndex(center_tree_);
  ofs << "tree_num: " << (int)trees_.size()  << " " <<  "center_tree: " << center_index << std::endl;

  for (vector<TreeHandlePtr>::iterator it = trees_.begin(); it != trees_.end(); it++)
    ofs << (*it)->getPos().x() << " " << (*it)->getPos().y() << " " << (*it)->getRadius() << " " << (*it)->getVote()  << std::endl;

}

bool TreeDataBase::load(string file_name)
{
  std::ifstream ifs(file_name.c_str());

  if(ifs.fail())
    {
      ROS_ERROR("File do not exist");
      return false;
    }

  std::string str;
  std::stringstream ss_header;
  std::string header1, header2;
  int tree_num;
  int center_tree_index;

  std::getline(ifs, str);
  ss_header.str(str);
  ss_header >> header1 >> tree_num >> header2 >> center_tree_index;
  ROS_INFO("%s: %d, %s: %d", header1.c_str(), tree_num, header2.c_str(), center_tree_index);
  
  int all_vote = 0;

  /* get the tree data */
  for(int i = 0; i < tree_num; i++)
    {
      std::stringstream ss;
      std::getline(ifs, str);
      ss.str(str);
      float x = 0, y = 0, radius = 0;
      int vote = 0;
      ss >> x >> y >> radius >> vote;
      ROS_INFO("tree_pos: [%f, %f]; radius: %f; vote: %d", x, y, radius, vote);
      TreeHandlePtr new_tree = TreeHandlePtr(new TreeHandle(nh_, nhp_, tf::Vector3(x, y, 0)));
      new_tree->setRadius(radius);
      new_tree->setVote(vote);
      add(new_tree);
      all_vote += vote;
      
      if(i == center_tree_index) center_tree_ = new_tree;
    }

  /* filter trees */
  update(); //sort
  int cut_vote = 0;
  for (int i = tree_num - 1; i > 0; i--) {
    cut_vote += trees_.at(i)->getVote();
    trees_.pop_back();
    if (cut_vote > all_vote * tree_cut_rate_) break;
  } 
  valid_num_ = tree_num;
  return true;
}

int TreeDataBase::getIndex(TreeHandlePtr target_tree)
{
  int i = 0;
  for (auto &tree : trees_)
    {
      if(tree == target_tree) return i;
      i++;
    }
  return -1;
}



