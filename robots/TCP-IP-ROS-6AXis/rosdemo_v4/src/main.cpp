#include "rosDemoCRV4.h"

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");    // 中文乱码
    ros::init(argc, argv, "rosdemo_v4");
    ros::NodeHandle nh("~");
    RosDemoCRV4 serviceHandler(&nh);
    std::vector<double> pointA;
    std::vector<double> pointB;
    if (!nh.getParam("/rosdemo_v4/pointA", pointA)) {
        ROS_ERROR("Failed to get parameter 'pointA'");
    }
    if (!nh.getParam("/rosdemo_v4/pointB", pointB)) {
        ROS_ERROR("Failed to get parameter 'pointB'");
    }
    // 创建一个新线程，并将obj和data作为引用传递给匿名函数
    std::thread threadMove([&serviceHandler, &pointA, &pointB]() {
        int currentCommandID = 2147483647;    // 初始值  int-max
        while (true) {
            serviceHandler.movePoint(pointA, currentCommandID);
            serviceHandler.finishPoint(currentCommandID);
            serviceHandler.movePoint(pointB, currentCommandID);
            serviceHandler.finishPoint(currentCommandID);
        }
    });
    threadMove.detach();
    ros::spin();
    return 0;
}
