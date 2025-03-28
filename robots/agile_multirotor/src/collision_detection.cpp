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

double distance_2d(geometry_msgs::Pose a, geometry_msgs::Pose b){
    return std::sqrt(std::pow(a.position.x-b.position.x,2)+std::pow(a.position.y-b.position.y,2));
}

void CollisionDetection::CollisionCallback(const gazebo_msgs::ModelStates::ConstPtr& model_states)
{
    // ROS_INFO("callback function is called");
    int tree_model_index = getIndex(model_states->name, "sub_target_tree1::cylinder1::link");
    // ROS_INFO("sub_target_tree1 index: [%d]", tree_model_index);
    geometry_msgs::Pose tree_pose = model_states->pose[tree_model_index];

    // ROS_INFO("tree x: [%lf]", tree_model_pose.position.x);
    
    int drone_model_index = getIndex(model_states->name, "multirotor::root");
    // ROS_INFO("sub_target_tree1 index: [%d]", tree_model_index);
    geometry_msgs::Pose drone_pose = model_states->pose[drone_model_index];

    bool is_collision = distance_2d(tree_pose,drone_pose)<radius_drone+radius_tree1;

    ROS_INFO("is_collision: [%s]", is_collision ? "true" : "false");
    std_msgs::Bool msg;
    msg.data = is_collision;
    pub.publish(msg);
}

CollisionDetection::CollisionDetection(){
    pub = nh.advertise<std_msgs::Bool>("/is_collision", 1);
    sub = nh.subscribe("/gazebo/link_states", 1, &CollisionDetection::CollisionCallback, this);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "collision_detection"); //The third argument to init() is the name of the node
  CollisionDetection collision;
    while(ros::ok()){
    // ROS_INFO("in while loop");
    ros::spinOnce();
    }

  return 0;
}