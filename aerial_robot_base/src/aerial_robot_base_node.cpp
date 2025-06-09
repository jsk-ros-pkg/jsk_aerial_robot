#include "aerial_robot_base/aerial_robot_base.h"

// Initialization of the AerialRobotBase node

int main (int argc, char **argv)
{
    ros::init(argc, argv, "aerial_robot_base_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    AerialRobotBase* aerialRobotBaseNode = new AerialRobotBase(nh, nh_private);
    ros::waitForShutdown();

    delete aerialRobotBaseNode;
    return 0;
}