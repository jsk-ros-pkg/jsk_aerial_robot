#include "rosDemoCRV3.h"
static const char* kAlarmServoJsonFile = "alarmFile/alarm_servo.json";
static const char* kAlarmControllerJsonFile = "alarmFile/alarm_controller.json";
// Include other necessary headers

RosDemoCRV3::RosDemoCRV3(ros::NodeHandle* nh)
{
    m_enableRobot = nh->serviceClient<rosdemo_v3::EnableRobot>("/dobot_bringup/srv/EnableRobot");
    m_disableRobot = nh->serviceClient<rosdemo_v3::DisableRobot>("/dobot_bringup/srv/DisableRobot");
    m_clearError = nh->serviceClient<rosdemo_v3::ClearError>("/dobot_bringup/srv/ClearError");
    m_getErrorID = nh->serviceClient<rosdemo_v3::GetErrorID>("/dobot_bringup/srv/GetErrorID");
    m_jointmovJ = nh->serviceClient<rosdemo_v3::JointMovJ>("/dobot_bringup/srv/JointMovJ");
    m_continue = nh->serviceClient<rosdemo_v3::Continues>("/dobot_bringup/srv/Continue");
    m_sync = nh->serviceClient<rosdemo_v3::Sync>("/dobot_bringup/srv/Sync");

    subFeedInfo = nh->subscribe("/dobot_bringup/msg/FeedInfo", 10, &RosDemoCRV3::getFeedBackInfo, this);
    // Add more services if needed

    threadParseRobotError = std::thread(&RosDemoCRV3::parseRobotAlarm, this);
    threadParseRobotError.detach();

    threadClearRobotError = std::thread(&RosDemoCRV3::clearRobotError, this);
    threadClearRobotError.detach();
}

void RosDemoCRV3::getFeedBackInfo(const std_msgs::String::ConstPtr& msg)
{
    std::string feedIndo = msg->data;
    // 从 JSON 字符串中提取数组
    if (feedIndo.empty()) {
        return;
    }

    nlohmann::json parsedJson = nlohmann::json::parse(feedIndo);
    std::unique_lock<std::mutex> lockInfo(m_mutex);
    if (parsedJson.count("EnableStatus") && parsedJson["EnableStatus"].is_number()) {
        feedbackData.EnableStatus = parsedJson["EnableStatus"];
    }
    if (parsedJson.count("ErrorStatus") && parsedJson["ErrorStatus"].is_number()) {
        feedbackData.ErrorStatus = parsedJson["ErrorStatus"];
    }
    if (parsedJson.count("RunQueuedCmd") && parsedJson["RunQueuedCmd"].is_number()) {
        feedbackData.RunQueuedCmd = parsedJson["RunQueuedCmd"];
    }
    if (parsedJson.count("QactualVec") && parsedJson["QactualVec"].is_array()) {
        for (int i = 0; i < 6; i++) {
            feedbackData.QactualVec[i] = parsedJson["QactualVec"][i];
        }
    }
}

void RosDemoCRV3::clearRobotError()
{
    while (true) {
        {
            std::unique_lock<std::mutex> lockInfo(m_mutex);
            if (feedbackData.ErrorStatus && feedbackData.EnableStatus)

            {
                rosdemo_v3::GetErrorID srvGetError;

                // 请求服务
                if (SendService(m_getErrorID, srvGetError)) {
                    for (int i = 0; i < srvGetError.response.errorID.size(); i++) {
                        bool alarmState{ false };
                        for (const auto& alarmControllerJson : m_JsonDataController) {
                            if (static_cast<int>(srvGetError.response.errorID[i]) ==
                                static_cast<int>(alarmControllerJson["id"])) {
                                ROS_ERROR("Control ErrorID : %d,  %s, %s", srvGetError.response.errorID[i],
                                          static_cast<std::string>(alarmControllerJson["zh_CN"]["description"]).c_str(),
                                          static_cast<std::string>(alarmControllerJson["en"]["description"]).c_str());
                                alarmState = true;
                                break;
                            }
                        }

                        if (alarmState) {
                            continue;
                        }

                        for (const auto& alarmServoJson : m_JsonDataServo) {
                            if (static_cast<int>(srvGetError.response.errorID[i]) ==
                                static_cast<int>(alarmServoJson["id"])) {
                                ROS_ERROR("Servo ErrorID : %d,  %s, %s", srvGetError.response.errorID[i],
                                          static_cast<std::string>(alarmServoJson["zh_CN"]["description"]).c_str(),
                                          static_cast<std::string>(alarmServoJson["en"]["description"]).c_str());
                                break;
                            }
                        }
                    }
                    rosdemo_v3::ClearError srvClearError;

                    // 此为触发清错
                    if (0) {
                        if (!SendService(m_clearError, srvClearError)) {
                            ROS_ERROR("clearError  service  fail");
                            continue;
                        }
                        rosdemo_v3::Continues srvContinue;

                        // 请求服务
                        if (!SendService(m_continue, srvContinue)) {
                            ROS_ERROR("Continue service  fail");
                            continue;
                        }
                    }

                    // 请求服务

                } else {
                    ROS_ERROR("geterrorid service  fail");
                }
            } else {
                if (feedbackData.EnableStatus && (!feedbackData.RunQueuedCmd)) {
                    rosdemo_v3::Continues srvContinue;
                    if (!SendService(m_continue, srvContinue)) {
                        ROS_ERROR("clearError  service  fail");
                    }
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
}

void RosDemoCRV3::movePoint(std::vector<double>& point)
{
    if (point.size() < 6U) {
        ROS_ERROR("JointMovJ params is  ERROR");
    } else {
        rosdemo_v3::JointMovJ srvJointMovJ;
        srvJointMovJ.request.j1 = point[0];
        srvJointMovJ.request.j2 = point[1];
        srvJointMovJ.request.j3 = point[2];
        srvJointMovJ.request.j4 = point[3];
        srvJointMovJ.request.j5 = point[4];
        srvJointMovJ.request.j6 = point[5];
        if (!SendService(m_jointmovJ, srvJointMovJ)) {
            ROS_ERROR("JointMovJ service fail");
        }
    }
}

void RosDemoCRV3::finishPoint(std::vector<double>& point)
{
    if (point.size() < 6U) {
        ROS_ERROR("JointMovJ params size is ERROR");
    }

    auto compareSize = [=](double value1, double value2) { return std::abs(value1 - value2) < 0.001; };
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::unique_lock<std::mutex> lockInfo(m_mutex);
        for (int i = 0; i < 6; i++) {
            stateFinish = compareSize(feedbackData.QactualVec[i], point[i]);
            if (!stateFinish) {
                break;
            }
        }
        if (stateFinish) {
            break;
        }
    }
}

void RosDemoCRV3::parseRobotAlarm()
{
    // 获取当前cpp文件所在的目录路径
    std::string currentDirectory = __FILE__;
    size_t lastSlash = currentDirectory.find_last_of("/");
    std::string directory = currentDirectory.substr(0, lastSlash + 1);

    // 拼接目标json文件的路径
    std::string jsonServoFilePath = directory + kAlarmServoJsonFile;
    std::string jsonControllerFilePath = directory + kAlarmControllerJsonFile;

    auto readJsonFile = [](const std::string& filePath, nlohmann::json& jsonData) {
        std::ifstream jsonFile(filePath);
        if (jsonFile.is_open()) {
            jsonFile >> jsonData;

            // 在这里使用 jsonData 对象访问json数据
            jsonFile.close();    // 关闭文件流，释放资源
        } else {
            ROS_ERROR("Failed to open json file : %s", filePath.c_str());
        }
    };

    readJsonFile(jsonServoFilePath, m_JsonDataServo);
    readJsonFile(jsonControllerFilePath, m_JsonDataController);
}