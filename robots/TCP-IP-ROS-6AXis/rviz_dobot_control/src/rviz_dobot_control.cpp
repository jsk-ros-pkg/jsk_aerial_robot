/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/09
 *
 * <h2><center>&copy; COPYRIGHT 2021 YUE JIANG TECHNOLOGY</center></h2>
 *
 ***********************************************************************************************************************
 */

#include "rviz_dobot_control.h"
#include "ui_control_menu.h"
#include <nlohmann/json.hpp>
#include <fstream>

namespace rviz_dobot_control
{
DobotControl::DobotControl(QWidget* parent)
    : rviz::Panel(parent)
    , is_enable_(false)
    , is_connected_(false)
    , ui(new Ui::ControlMenu)
    , enable_robot_topic_("/dobot_bringup/srv/EnableRobot")
    , disable_robot_topic_("/dobot_bringup/srv/DisableRobot")
    , robot_status_topic_("/dobot_bringup/msg/RobotStatus")
    , robotVersionType("v3")
{
    // 读取机器版本v3/v4
    try {
        std::string packagePath = ros::package::getPath("rviz_dobot_control");
        std::string filePath = packagePath + "/robot_type.json";

        std::ifstream file(filePath);
        nlohmann::json configJson;
        // 检查文件是否成功打开
        if (file.is_open()) {
            file >> configJson;    // 从文件中读取JSON数据
            file.close();          // 关闭文件

            // 读取JSON中的数据
            std::string paramLower = configJson["robotType"];
            std::transform(paramLower.begin(), paramLower.end(), paramLower.begin(), ::tolower);

            if (paramLower.compare("v3") == 0) {
                ROS_INFO("RobotVersionType  %s", paramLower.c_str());
            } else if (paramLower.compare("v4") == 0) {
                enable_robot_topic_ = "/dobot_v4_bringup/srv/EnableRobot";
                disable_robot_topic_ = "/dobot_v4_bringup/srv/DisableRobot";
                robot_status_topic_ = "/dobot_v4_bringup/msg/RobotStatus";
                robotVersionType = "v4";
                ROS_INFO("RobotVersionType  %s", paramLower.c_str());
            } else {
                ROS_ERROR("RobotVersionType is error %s", paramLower.c_str());
            }
        } else {
            ROS_ERROR("open file is error ");
        }

    } catch (const std::exception& e) {
        ROS_ERROR("RobotVersionType yaml file error: %s", e.what());
    }

    ui->setupUi(this);

    ui->is_robot_enable->setText("Disable");
    ui->is_robot_connected->setText("Disconnect");
    ui->enable_robot_topic->setText(enable_robot_topic_);
    ui->disable_robot_topic->setText(disable_robot_topic_);
    ui->robot_status_topic->setText(robot_status_topic_);

    if (robotVersionType == "v3") {
        robot_status_sub_ =
            nh_.subscribe(robot_status_topic_.toStdString(), 100, &DobotControl::listenRobotStatusV3, this);
    }

    if (robotVersionType == "v4") {
        robot_status_sub_ =
            nh_.subscribe(robot_status_topic_.toStdString(), 100, &DobotControl::listenRobotStatusV4, this);
    }

    if (robotVersionType == "v3") {
        enable_robot_client_ =
            nh_.serviceClient<dobot_bringup::EnableRobot>(ui->enable_robot_topic->text().toStdString(), 100);
        disable_robot_client_ =
            nh_.serviceClient<dobot_bringup::DisableRobot>(ui->disable_robot_topic->text().toStdString(), 100);
    }
    if (robotVersionType == "v4") {
        enable_robot_client_ =
            nh_.serviceClient<dobot_v4_bringup::EnableRobot>(ui->enable_robot_topic->text().toStdString(), 100);
        disable_robot_client_ =
            nh_.serviceClient<dobot_v4_bringup::DisableRobot>(ui->disable_robot_topic->text().toStdString(), 100);
    }

    QObject::connect(ui->enable_robot_btn, &QPushButton::clicked, this, &DobotControl::enableRobot);
    QObject::connect(ui->disable_robot_btn, &QPushButton::clicked, this, &DobotControl::disableRobot);
    QObject::connect(ui->enable_robot_topic, &QLineEdit::editingFinished, this,
                     &DobotControl::enableRobotTopicEditFinished);
    QObject::connect(ui->disable_robot_topic, &QLineEdit::editingFinished, this,
                     &DobotControl::disableRobotTopicEditFinished);
    QObject::connect(ui->robot_status_topic, &QLineEdit::editingFinished, this,
                     &DobotControl::robotStatusTopicEditFinished);
}

void DobotControl::load(const Config& config)
{
    QString str;
    if (config.mapGetString(ENABLE_ROBOT_TOPIC_KEY, &str)) {
        enable_robot_topic_ = str;
    }

    if (config.mapGetString(DISABLE_ROBOT_TOPIC_KEY, &str)) {
        disable_robot_topic_ = str;
    }

    if (config.mapGetString(ROBOT_STATUS_TOPIC_KEY, &str)) {
        robot_status_topic_ = str;
    }
}

void DobotControl::listenRobotStatusV3(const dobot_bringup::RobotStatusConstPtr status)
{
    setRobotStatus(status->is_enable, status->is_connected);
}

void DobotControl::listenRobotStatusV4(const dobot_v4_bringup::RobotStatusConstPtr status)
{
    setRobotStatus(status->is_enable, status->is_connected);
}

void DobotControl::save(Config config) const
{
    config.mapSetValue(ENABLE_ROBOT_TOPIC_KEY, ui->enable_robot_topic->text());
    config.mapSetValue(DISABLE_ROBOT_TOPIC_KEY, ui->disable_robot_topic->text());
    config.mapSetValue(ROBOT_STATUS_TOPIC_KEY, ui->robot_status_topic->text());
}

void DobotControl::enableRobot()
{
    if (robotVersionType == "v3") {
        dobot_bringup::EnableRobot srv;
        if (enable_robot_client_.call(srv)) {
            ROS_INFO("enableRobot %d", srv.response.res);
        } else {
            ROS_ERROR("enableRobot failed");
        }
    }

    if (robotVersionType == "v4") {
        dobot_v4_bringup::EnableRobot srv;
        if (enable_robot_client_.call(srv)) {
            ROS_INFO("enableRobot %d", srv.response.res);
        } else {
            ROS_ERROR("enableRobot failed");
        }
    }
}

void DobotControl::disableRobot()
{
    if (robotVersionType == "v3") {
        dobot_bringup::DisableRobot srv;
        if (disable_robot_client_.call(srv)) {
            ROS_INFO("disableRobot %d", srv.response.res);
        } else {
            ROS_ERROR("disableRobot failed");
        }
    }
    if (robotVersionType == "v4") {
        dobot_v4_bringup::DisableRobot srv;
        if (disable_robot_client_.call(srv)) {
            ROS_INFO("disableRobot %d", srv.response.res);
        } else {
            ROS_ERROR("disableRobot failed");
        }
    }
}

void DobotControl::enableRobotTopicEditFinished()
{
    if (enable_robot_topic_ != ui->enable_robot_topic->text()) {
        ROS_INFO("enableRobotTopicEditFinished");
    }
}

void DobotControl::disableRobotTopicEditFinished()
{
    if (disable_robot_topic_ != ui->enable_robot_topic->text()) {
        ROS_INFO("disableRobotTopicEditFinished");
    }
}

void DobotControl::robotStatusTopicEditFinished()
{
    if (robot_status_topic_ != ui->enable_robot_topic->text()) {
        ROS_INFO("robotStatusTopicEditFinished");
    }
}

void DobotControl::setRobotStatus(bool is_enable, bool is_connected)
{
    if (is_enable_ != is_enable) {
        is_enable_ = is_enable;
        ui->is_robot_enable->setText(is_enable_ ? "Enabled" : "Disable");
    }

    if (is_connected_ != is_connected) {
        is_connected_ = is_connected;
        ui->is_robot_connected->setText(is_connected_ ? "Connected" : "Disconnect");
    }
}
}    // namespace rviz_dobot_control

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_dobot_control::DobotControl, rviz::Panel)
// END_TUTORIAL