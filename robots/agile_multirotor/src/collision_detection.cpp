#include <collision_detection.h>

int getIndex(std::vector<std::string> v, std::string value)
{
    for(int i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}

void CollisionDetection::CollisionCallback(const gazebo_msgs::ModelStates::ConstPtr& model_states)
{
    // ROS_INFO("callback function is called");
    int tree_model_index = getIndex(model_states->name, "sub_target_tree1::cylinder1::link");
    // ROS_INFO("sub_target_tree1 index: [%d]", tree_model_index);
    geometry_msgs::Pose tree_model_pose = model_states->pose[tree_model_index];
    ROS_INFO("tree x: [%lf]", tree_model_pose.position.x);
}

CollisionDetection::CollisionDetection(){
    sub = nh.subscribe("/gazebo/link_states", 1, &CollisionDetection::CollisionCallback, this);
}
int main (int argc, char **argv)
{
  ros::init(argc, argv, "collision_detection"); //The third argument to init() is the name of the node
  CollisionDetection collision;
    while(ros::ok()){
    // ROS_INFO("in while loop");
    ros::spinOnce();
    }

  return 0;
}