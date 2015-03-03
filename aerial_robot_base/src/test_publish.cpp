#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "publish");

  ros::NodeHandle nh_;

  ros::Publisher  multiArray_pub = nh_.advertise < std_msgs::Int16MultiArray  > ("ctrl_pub", 1);
  ros::Rate rate(50.0);

  while (nh_.ok()){

    std_msgs::Int16MultiArray data_msg;
    data_msg.data.clear();
    data_msg.data.push_back(1);
    data_msg.data.push_back(2);
    data_msg.data.push_back(3);
    data_msg.data.push_back(4);
    // data_msg.data.push_back(1);
    // data_msg.data.push_back(2);
    // data_msg.data.push_back(3);
    // data_msg.data.push_back(4);
    multiArray_pub.publish(data_msg);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;

};
