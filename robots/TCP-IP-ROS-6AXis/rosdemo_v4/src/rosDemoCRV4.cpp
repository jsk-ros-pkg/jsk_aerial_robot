#include "rosDemoCRV4.h"
static const char* kAlarmServoJsonFile = "alarmFile/alarm_servo.json";
static const char* kAlarmControllerJsonFile = "alarmFile/alarm_controller.json";
// Include other necessary headers

RosDemoCRV4::RosDemoCRV4(ros::NodeHandle* nh)
{
    m_enableRobot = nh->serviceClient<rosdemo_v4::EnableRobot>("/dobot_v4_bringup/srv/EnableRobot");
    m_disableRobot = nh->serviceClient<rosdemo_v4::DisableRobot>("/dobot_v4_bringup/srv/DisableRobot");
    m_clearError = nh->serviceClient<rosdemo_v4::ClearError>("/dobot_v4_bringup/srv/ClearError");
    m_getErrorID = nh->serviceClient<rosdemo_v4::GetErrorID>("/dobot_v4_bringup/srv/GetErrorID");
    m_movj = nh->serviceClient<rosdemo_v4::MovJ>("/dobot_v4_bringup/srv/MovJ");
    memset(&feedbackData, 0, sizeof(feedbackData));

    subFeedInfo = nh->subscribe("/dobot_v4_bringup/msg/FeedInfo", 10, &RosDemoCRV4::getFeedBackInfo, this);
    // Add more services if needed

    threadParseRobotError = std::thread(&RosDemoCRV4::parseRobotAlarm, this);
    threadParseRobotError.detach();

    threadClearRobotError = std::thread(&RosDemoCRV4::warmRobotError, this);
    threadClearRobotError.detach();
}

void RosDemoCRV4::getFeedBackInfo(const std_msgs::String::ConstPtr& msg)
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
    if (parsedJson.count("robot_mode") && parsedJson["robot_mode"].is_number()) {
        feedbackData.RobotMode = parsedJson["robot_mode"];
    }
    if (parsedJson.count("currentCommandId") && parsedJson["currentCommandId"].is_number()) {
        feedbackData.CurrentCommandID = parsedJson["currentCommandId"];
    }
}

void RosDemoCRV4::warmRobotError()
{
    while (true) {
        {
            std::unique_lock<std::mutex> lockInfo(m_mutex);
            if (feedbackData.ErrorStatus) {
                rosdemo_v4::GetErrorID srvGetError;
                static std::vector<int> errorId;
                // 请求服务
                if (SendService(m_getErrorID, srvGetError)) {
                    std::vector<int> errorIdNew;
                    for (int i = 0; i < srvGetError.response.error_id.size(); i++) {
                        errorIdNew.push_back(srvGetError.response.error_id[i]);
                    }
                    std::sort(errorIdNew.begin(), errorIdNew.end());
                    std::sort(errorId.begin(), errorId.end());
                    if (errorIdNew == errorId) {
                        lockInfo.unlock();
                        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                        continue;
                    } else {
                        errorId = errorIdNew;
                    }

                    for (int i = 0; i < errorId.size(); i++) {
                        bool alarmState{ false };
                        for (const auto& alarmControllerJson : m_JsonDataController) {
                            if (static_cast<int>(srvGetError.response.error_id[i]) ==
                                static_cast<int>(alarmControllerJson["id"])) {
                                ROS_ERROR("Control ErrorID : %d,  %s, %s", srvGetError.response.error_id[i],
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
                            if (static_cast<int>(srvGetError.response.error_id[i]) ==
                                static_cast<int>(alarmServoJson["id"])) {
                                ROS_ERROR("Servo ErrorID : %d,  %s, %s", srvGetError.response.error_id[i],
                                          static_cast<std::string>(alarmServoJson["zh_CN"]["description"]).c_str(),
                                          static_cast<std::string>(alarmServoJson["en"]["description"]).c_str());
                                break;
                            }
                        }
                    }

                } else {
                    ROS_ERROR("geterrorid service  fail");
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
}

void RosDemoCRV4::movePoint(std::vector<double>& point, int& currentCommandID)
{
    if (point.size() < 6U) {
        ROS_ERROR("MovJ params is  ERROR");
        return;
    }

    rosdemo_v4::MovJ srvMovJ;
    srvMovJ.request.mode = 1;
    srvMovJ.request.a = point[0];
    srvMovJ.request.b = point[1];
    srvMovJ.request.c = point[2];
    srvMovJ.request.d = point[3];
    srvMovJ.request.e = point[4];
    srvMovJ.request.f = point[5];
    if (!SendService(m_movj, srvMovJ)) {
        ROS_ERROR("MovJ service fail");
    } else {
        currentCommandID = int(srvMovJ.response.res);
        ROS_INFO("CurrentCommandID  %d", currentCommandID);
    }
}

void RosDemoCRV4::finishPoint(int currentCommandID)
{
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::unique_lock<std::mutex> lockInfo(m_mutex);
        if (feedbackData.CurrentCommandID > currentCommandID) {
            ROS_INFO("FINISH %d , %d", feedbackData.CurrentCommandID, currentCommandID);
            break;
        }
        if ((feedbackData.CurrentCommandID == currentCommandID) && (feedbackData.RobotMode == 5)) {
            ROS_INFO("finsih %d", currentCommandID);
            break;
        }
        lockInfo.unlock();
        sleep(0.01);
    }
}

void RosDemoCRV4::parseRobotAlarm()
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
