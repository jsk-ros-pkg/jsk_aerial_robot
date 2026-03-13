
#pragma once
#include "ros/ros.h"
#include <thread>
#include <nlohmann/json.hpp>
#include <std_msgs/String.h>
#include <memory>
#include <mutex>
#include <fstream>
#include "rosdemo_v3/EnableRobot.h"
#include "rosdemo_v3/DisableRobot.h"
#include "rosdemo_v3/ClearError.h"
#include "rosdemo_v3/GetErrorID.h"
#include "rosdemo_v3/MovL.h"
#include "rosdemo_v3/JointMovJ.h"
#include "rosdemo_v3/Continues.h"
#include "rosdemo_v3/Sync.h"

class RosDemoCRV3
{
public:
    RosDemoCRV3(ros::NodeHandle* nh);
    void movePoint(std::vector<double>& pointA);
    void finishPoint(std::vector<double>& pointA);

private:
    ros::ServiceClient m_enableRobot;
    ros::ServiceClient m_disableRobot;
    ros::ServiceClient m_clearError;
    ros::ServiceClient m_getErrorID;
    ros::ServiceClient m_jointmovJ;
    ros::ServiceClient m_continue;
    ros::ServiceClient m_sync;
    ros::Subscriber subFeedInfo;
    std::mutex m_mutex;
    std::thread threadClearRobotError;
    std::thread threadParseRobotError;
    struct FeedInfo
    {
        int EnableStatus;
        int ErrorStatus;
        int RunQueuedCmd;
        double QactualVec[6];
    };
    FeedInfo feedbackData;
    bool stateFinish{ false };
    nlohmann::json m_JsonDataController;
    nlohmann::json m_JsonDataServo;

private:
    void clearRobotError();
    void getFeedBackInfo(const std_msgs::String::ConstPtr& msg);

    template <typename T>
    bool SendService(ros::ServiceClient serviceClient, T& arg);
    void parseRobotAlarm();
    // Add more service servers if needed
};

// 模板函数 发送service服务
template <typename T>
bool RosDemoCRV3::SendService(ros::ServiceClient serviceClient, T& arg)
{
    // 请求服务
    if (serviceClient.call(arg)) {
        return true;
    }
    return false;
}