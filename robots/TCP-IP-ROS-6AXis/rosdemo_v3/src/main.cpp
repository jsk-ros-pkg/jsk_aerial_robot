#include "rosDemoCRV3.h"

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");    // 中文乱码
    ros::init(argc, argv, "rosdemo_v3");
    ros::NodeHandle nh("~");
    RosDemoCRV3 serviceHandler(&nh);
    std::vector<double> pointA;
    std::vector<double> pointB;
    if (!nh.getParam("/rosdemo_v3/pointA", pointA)) {
        ROS_ERROR("Failed to get parameter 'pointA'");
    }
    if (!nh.getParam("/rosdemo_v3/pointB", pointB)) {
        ROS_ERROR("Failed to get parameter 'pointB'");
    }
    // 创建一个新线程，并将obj和data作为引用传递给匿名函数
    std::thread threadMove([&serviceHandler, &pointA, &pointB]() {
        while (true) {
            serviceHandler.movePoint(pointA);
            serviceHandler.finishPoint(pointA);
            serviceHandler.movePoint(pointB);
            serviceHandler.finishPoint(pointB);
        }
    });
    threadMove.detach();
    ros::spin();
    return 0;
}
