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

#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>
#include <ros/package.h>

#include <dobot_bringup/EnableRobot.h>
#include <dobot_bringup/DisableRobot.h>
#include <dobot_bringup/RobotStatus.h>

#include <dobot_v4_bringup/EnableRobot.h>
#include <dobot_v4_bringup/DisableRobot.h>
#include <dobot_v4_bringup/RobotStatus.h>

using namespace rviz;

QT_BEGIN_NAMESPACE
namespace Ui
{
class ControlMenu;
}
QT_END_NAMESPACE

namespace rviz_dobot_control
{
class DobotControl : public rviz::Panel
{
    Q_OBJECT

public:
    static constexpr const char* ENABLE_ROBOT_TOPIC_KEY = "EnableRobotTopicKey";
    static constexpr const char* DISABLE_ROBOT_TOPIC_KEY = "DisableRobotTopicKey";
    static constexpr const char* ROBOT_STATUS_TOPIC_KEY = "RobotStatusTopicKey";

private:
    bool is_enable_;
    bool is_connected_;
    Ui::ControlMenu* ui;
    ros::NodeHandle nh_;
    QString enable_robot_topic_;
    QString disable_robot_topic_;
    QString robot_status_topic_;
    ros::Subscriber robot_status_sub_;
    ros::ServiceClient enable_robot_client_;
    ros::ServiceClient disable_robot_client_;
    std::string robotVersionType;

public Q_SLOTS:
    void enableRobot();
    void disableRobot();
    void enableRobotTopicEditFinished();
    void disableRobotTopicEditFinished();
    void robotStatusTopicEditFinished();

public:
    DobotControl(QWidget* parent = nullptr);

    /** @brief Override to load configuration data.  This version loads the name of the panel. */
    virtual void load(const Config& config) override;

    /** @brief Override to save configuration data.  This version saves the name and class ID of the panel. */
    virtual void save(Config config) const override;

    /**
     * listenRobotStatus
     * @param status robot status
     */
    void listenRobotStatusV3(const dobot_bringup::RobotStatusConstPtr status);
    void listenRobotStatusV4(const dobot_v4_bringup::RobotStatusConstPtr status);

private:
    void setRobotStatus(bool is_enable, bool is_connected);
};
}    // namespace rviz_dobot_control