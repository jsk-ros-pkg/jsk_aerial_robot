#include <ros/ros.h>
#include <tf/transform_listener.h>



int main(int argc, char** argv){
  ros::init(argc, argv, "tf_listener_test");

  ros::NodeHandle node;
  tf::TransformListener tf_;

  ros::Rate rate(40.0);
  while (node.ok()){
    tf::StampedTransform transform;
    
    ros::Duration dur (0.02);
    if (tf_.waitForTransform("/head_cylinder","link4_abdomen", ros::Time(0),dur))
      {
        double time0 = ros::Time::now().toSec();
        tf_.lookupTransform("/head_cylinder", "/link1_abdomen", ros::Time(0), transform);
        ROS_INFO("time1: %f", ros::Time::now().toSec() - time0);
        time0 = ros::Time::now().toSec();
        tf_.lookupTransform("/head_cylinder", "/link2_abdomen", ros::Time(0), transform); 
        ROS_INFO("time2: %f", ros::Time::now().toSec() - time0);
        time0 = ros::Time::now().toSec();
        tf_.lookupTransform("/head_cylinder", "/link3_abdomen", ros::Time(0), transform);
        ROS_INFO("time3: %f", ros::Time::now().toSec() - time0);
        time0 = ros::Time::now().toSec();
        tf_.lookupTransform("/head_cylinder", "/link4_abdomen", ros::Time(0), transform);
        ROS_INFO("time4: %f", ros::Time::now().toSec() - time0);
      }

    rate.sleep();
  }
  return 0;
};
