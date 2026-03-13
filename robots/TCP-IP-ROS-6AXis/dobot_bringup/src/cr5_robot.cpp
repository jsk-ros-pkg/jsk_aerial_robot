/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/07
 *
 * <h2><center>&copy; COPYRIGHT 2021 DOBOT CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <ros/ros.h>
#include <ros/param.h>
#include <dobot_bringup/cr5_robot.h>
#include <nlohmann/json.hpp>
#include <std_msgs/String.h>
#include <regex>

CR5Robot::CR5Robot(ros::NodeHandle& nh, std::string name)
    : ActionServer<FollowJointTrajectoryAction>(nh, std::move(name), false)
    , goal_{}
    , control_nh_(nh)
    , trajectory_duration_(1.0)
    , last_robot_mode_(0)
{
    index_ = 0;
    memset(goal_, 0, sizeof(goal_));
    kServoJParam = std::make_shared<ServoJParam>();
}

CR5Robot::~CR5Robot()
{
    backend_task_.stop();
    ROS_INFO("~CR5Robot");
}

void CR5Robot::init()
{
    std::string ip = control_nh_.param<std::string>("robot_ip_address", "192.168.5.1");

    trajectory_duration_ = control_nh_.param("trajectory_duration", 0.3);
    ROS_INFO("trajectory_duration : %0.2f", trajectory_duration_);

    commander_ = std::make_shared<CR5Commander>(ip);
    commander_->init();

    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/EnableRobot", &CR5Robot::enableRobot, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/DisableRobot", &CR5Robot::disableRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/ClearError", &CR5Robot::clearError, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/ResetRobot", &CR5Robot::resetRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/SpeedFactor", &CR5Robot::speedFactor, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/GetErrorID", &CR5Robot::getErrorID, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/User", &CR5Robot::user, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/Tool", &CR5Robot::tool, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/RobotMode", &CR5Robot::robotMode, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/PayLoad", &CR5Robot::payload, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/DO", &CR5Robot::DO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/DOExecute", &CR5Robot::DOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/ToolDO", &CR5Robot::toolDO, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/ToolDOExecute", &CR5Robot::toolDOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/AO", &CR5Robot::AO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/AOExecute", &CR5Robot::AOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/AccJ", &CR5Robot::accJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/AccL", &CR5Robot::accL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/SpeedJ", &CR5Robot::speedJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/SpeedL", &CR5Robot::speedL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/Arch", &CR5Robot::arch, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/CP", &CR5Robot::cp, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/LimZ", &CR5Robot::limZ, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/SetArmOrientation", &CR5Robot::setArmOrientation, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/SetPayload", &CR5Robot::setPayload, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/PositiveSolution", &CR5Robot::positiveSolution, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/InverseSolution", &CR5Robot::inverseSolution, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/PowerOn", &CR5Robot::powerOn, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/RunScript", &CR5Robot::runScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/StopScript", &CR5Robot::stopScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/PauseScript", &CR5Robot::pauseScript, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/ContinueScript", &CR5Robot::continueScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/SetSafeSkin", &CR5Robot::setSafeSkin, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/SetObstacleAvoid", &CR5Robot::setObstacleAvoid, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/GetTraceStartPose", &CR5Robot::getTraceStartPose, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/SetCollisionLevel", &CR5Robot::setCollisionLevel, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/GetAngle", &CR5Robot::getAngle, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/GetPose", &CR5Robot::getPose, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/GetPathStartPose", &CR5Robot::getPathStartPose, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/HandleTrajPoints", &CR5Robot::handleTrajPoints, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/GetSixForceData", &CR5Robot::getSixForceData, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/SetCollideDrag", &CR5Robot::setCollideDrag, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/SetTerminalKeys", &CR5Robot::setTerminalKeys, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/SetTerminal485", &CR5Robot::setTerminal485, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/GetTerminal485", &CR5Robot::getTerminal485, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/TCPSpeed", &CR5Robot::tCPSpeed, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/TCPSpeedEnd", &CR5Robot::tCPSpeedEnd, this));

    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/EmergencyStop", &CR5Robot::emergencyStop, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/ModbusCreate", &CR5Robot::modbusCreate, this));

    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/MovJ", &CR5Robot::movJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/MovL", &CR5Robot::movL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/JointMovJ", &CR5Robot::jointMovJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/Jump", &CR5Robot::jump, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/RelMovJ", &CR5Robot::relMovJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/RelMovL", &CR5Robot::relMovL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/MovLIO", &CR5Robot::movLIO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/MovJIO", &CR5Robot::movJIO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/Arc", &CR5Robot::arc, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/Circle", &CR5Robot::circle3, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/RelMovJTool", &CR5Robot::relMovJTool, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/RelMovLTool", &CR5Robot::relMovLTool, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/RelMovJUser", &CR5Robot::relMovJUser, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/RelMovLUser", &CR5Robot::relMovLUser, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/RelJointMovJ", &CR5Robot::relJointMovJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/ServoJ", &CR5Robot::servoJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/ServoP", &CR5Robot::servoP, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/Sync", &CR5Robot::sync, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/StartTrace", &CR5Robot::startTrace, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/StartPath", &CR5Robot::startPath, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/StartFCTrace", &CR5Robot::startFCTrace, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/MoveJog", &CR5Robot::moveJog, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/StopmoveJog", &CR5Robot::stopmoveJog, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/Wait", &CR5Robot::wait, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/Continue", &CR5Robot::Continue, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/Pause", &CR5Robot::pause, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/ModbusClose", &CR5Robot::modbusClose, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/GetInBits", &CR5Robot::getInBits, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/GetInRegs", &CR5Robot::getInRegs, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/GetHoldRegs", &CR5Robot::getHoldRegs, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/SetHoldRegs", &CR5Robot::setHoldRegs, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/GetCoils", &CR5Robot::getCoils, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/SetCoils", &CR5Robot::setCoils, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/ToolDI", &CR5Robot::toolDI, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/DI", &CR5Robot::DI, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/ToolAI", &CR5Robot::toolAI, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/AI", &CR5Robot::AI, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/DIGroup", &CR5Robot::DIGroup, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/DOGroup", &CR5Robot::DOGroup, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/BrakeControl", &CR5Robot::brakeControl, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/StartDrag", &CR5Robot::startDrag, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/StopDrag", &CR5Robot::stopDrag, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/LoadSwitch", &CR5Robot::loadSwitch, this));

    server_tbl_.push_back(
        control_nh_.advertiseService("/dobot_bringup/srv/TcpDashboard", &CR5Robot::tcpDashboard, this));
    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/TcpRealData", &CR5Robot::tcpRealData, this));

    server_tbl_.push_back(control_nh_.advertiseService("/dobot_bringup/srv/ServoJParam", &CR5Robot::servoJParam, this));
    registerGoalCallback(boost::bind(&CR5Robot::goalHandle, this, _1));
    registerCancelCallback(boost::bind(&CR5Robot::cancelHandle, this, _1));

    backend_task_ = control_nh_.createTimer(ros::Duration(1.5), &CR5Robot::backendTask, this);

    pubFeedInfo = control_nh_.advertise<std_msgs::String>("/dobot_bringup/msg/FeedInfo", 1000);
    threadPubFeedBackInfo = std::thread(&CR5Robot::pubFeedBackInfo, this);
    threadPubFeedBackInfo.detach();
    start();
}

void CR5Robot::pubFeedBackInfo()
{
    RealTimeData* realTimeData = nullptr;
    // 设置发布频率为10Hz
    ros::Rate rate(100);
    while (ros::ok()) {
        realTimeData = (const_cast<RealTimeData*>(commander_->getRealData()));
        nlohmann::json root;
        root["EnableStatus"] = realTimeData->EnableStatus;
        root["ErrorStatus"] = realTimeData->ErrorStatus;
        root["RunQueuedCmd"] = realTimeData->isRunQueuedCmd;
        std::vector<double> qActualVec;
        // memcpy(toolvectoractual.data(), realTimeData.tool_vector_actual, sizeof(realTimeData->tool_vector_actual));
        for (int i = 0; i < 6; i++) {
            qActualVec.push_back(realTimeData->q_actual[i]);
        }
        root["QactualVec"] = qActualVec;
        std::string qActualVecStr = root.dump();

        std_msgs::String msgFeedInfo;
        msgFeedInfo.data = qActualVecStr;
        pubFeedInfo.publish(msgFeedInfo);
        rate.sleep();
    }
}

void CR5Robot::feedbackHandle(const ros::TimerEvent& tm,
                              ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    control_msgs::FollowJointTrajectoryFeedback feedback;

    double current_joints[6];
    getJointState(current_joints);

    for (uint32_t i = 0; i < 6; i++) {
        feedback.joint_names.push_back(std::string("joint") + std::to_string(i + 1));
        feedback.actual.positions.push_back(current_joints[i]);
        feedback.desired.positions.push_back(goal_[i]);
    }

    handle.publishFeedback(feedback);
}

void CR5Robot::moveHandle(const ros::TimerEvent& tm,
                          ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    control_msgs::FollowJointTrajectoryGoalConstPtr trajectory = handle.getGoal();

    if (index_ < trajectory->trajectory.points.size()) {
        auto point = trajectory->trajectory.points[index_].positions;
        double tmp[6];
        for (uint32_t i = 0; i < 6; i++) {
            tmp[i] = point[i] * 180.0 / 3.1415926;
        }

        dobot_bringup::ServoJ srv;
        srv.request.offset1 = tmp[0];
        srv.request.offset2 = tmp[1];
        srv.request.offset3 = tmp[2];
        srv.request.offset4 = tmp[3];
        srv.request.offset5 = tmp[4];
        srv.request.offset6 = tmp[5];
        if (kServoJParam && kServoJParam->trajectory_duration != 0.0) {
            srv.request.t.push_back(kServoJParam->t);
            srv.request.lookahead_time.push_back(kServoJParam->lookahead_time);
            srv.request.gain.push_back(kServoJParam->gain);
            movj_timer_ = control_nh_.createTimer(ros::Duration(kServoJParam->trajectory_duration),
                                                  boost::bind(&CR5Robot::moveHandle, this, _1, handle));
        }
        servoJ(srv.request, srv.response);
        index_++;
    } else {
#define OFFSET_VAL 0.01
        double current_joints[6];
        getJointState(current_joints);
        if ((current_joints[0] >= goal_[0] - OFFSET_VAL) && (current_joints[0] <= goal_[0] + OFFSET_VAL) &&
            (current_joints[1] >= goal_[1] - OFFSET_VAL) && (current_joints[1] <= goal_[1] + OFFSET_VAL) &&
            (current_joints[2] >= goal_[2] - OFFSET_VAL) && (current_joints[2] <= goal_[2] + OFFSET_VAL) &&
            (current_joints[3] >= goal_[3] - OFFSET_VAL) && (current_joints[3] <= goal_[3] + OFFSET_VAL) &&
            (current_joints[4] >= goal_[4] - OFFSET_VAL) && (current_joints[4] <= goal_[4] + OFFSET_VAL) &&
            (current_joints[5] >= goal_[5] - OFFSET_VAL) && (current_joints[5] <= goal_[5] + OFFSET_VAL)) {
            timer_.stop();
            movj_timer_.stop();
            handle.setSucceeded();
        }
    }
}

void CR5Robot::goalHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    index_ = 0;
    for (uint32_t i = 0; i < 6; i++) {
        goal_[i] = handle.getGoal()->trajectory.points[handle.getGoal()->trajectory.points.size() - 1].positions[i];
    }
    timer_ = control_nh_.createTimer(ros::Duration(1.0), boost::bind(&CR5Robot::feedbackHandle, this, _1, handle));
    movj_timer_ = control_nh_.createTimer(ros::Duration(trajectory_duration_),
                                          boost::bind(&CR5Robot::moveHandle, this, _1, handle));
    timer_.start();
    movj_timer_.start();
    handle.setAccepted();
}

void CR5Robot::cancelHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    timer_.stop();
    movj_timer_.stop();
    handle.setSucceeded();
}

void CR5Robot::getJointState(double* point)
{
    commander_->getCurrentJointStatus(point);
}

bool CR5Robot::isEnable() const
{
    return commander_->isEnable();
}

bool CR5Robot::isConnected() const
{
    return commander_->isConnected();
}

void CR5Robot::getToolVectorActual(double* val)
{
    commander_->getToolVectorActual(val);
}

/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  dashboard
 *----------------------------------------------------------------------------------------------------------------------
 */

bool CR5Robot::enableRobot(dobot_bringup::EnableRobot::Request& request, dobot_bringup::EnableRobot::Response& response)
{
    try {
        char cmd[100];
        if (request.args.size() == 1) {
            sprintf(cmd, "EnableRobot(%f)", request.args[0]);
        } else if (request.args.size() == 4) {
            sprintf(cmd, "EnableRobot(%f,%f,%f,%f)", request.args[0], request.args[1], request.args[2],
                    request.args[3]);
        } else {
            sprintf(cmd, "EnableRobot()");
        }

        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const std::exception& err) {
        response.res = -1;
        return false;
    }
}

bool CR5Robot::disableRobot(dobot_bringup::DisableRobot::Request& request,
                            dobot_bringup::DisableRobot::Response& response)
{
    try {
        const char* cmd = "DisableRobot()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const std::exception& err) {
        response.res = -1;
        return false;
    }
}

bool CR5Robot::clearError(dobot_bringup::ClearError::Request& request, dobot_bringup::ClearError::Response& response)
{
    try {
        const char* cmd = "ClearError()";
        commander_->dashboardDoCmd(cmd, response.res);
        response.res = 0;
        return true;
    } catch (const std::exception& err) {
        response.res = -1;
        return false;
    }
}

bool CR5Robot::resetRobot(dobot_bringup::ResetRobot::Request& request, dobot_bringup::ResetRobot::Response& response)
{
    try {
        const char* cmd = "ResetRobot()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::speedFactor(dobot_bringup::SpeedFactor::Request& request, dobot_bringup::SpeedFactor::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SpeedFactor(%d)", request.ratio);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::user(dobot_bringup::User::Request& request, dobot_bringup::User::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "User(%d)", request.index);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::tool(dobot_bringup::Tool::Request& request, dobot_bringup::Tool::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "Tool(%d)", request.index);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::robotMode(dobot_bringup::RobotMode::Request& request, dobot_bringup::RobotMode::Response& response)
{
    try {
        const char* cmd = "RobotMode()";

        std::vector<std::string> result;
        commander_->dashboardDoCmd(cmd, response.res, result);
        if (result.empty()) {
            ROS_ERROR("%s", "robotMode : Empty string");
            response.mode = -1;
            response.res = -1;
            return true;
        }
        response.mode = str2Int(result[0].c_str());
        response.res = 0;
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return true;
    } catch (const std::exception& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return true;
    }
}

bool CR5Robot::payload(dobot_bringup::PayLoad::Request& request, dobot_bringup::PayLoad::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "PayLoad(%0.3f,%0.3f)", request.weight, request.inertia);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::DI(dobot_bringup::DI::Request& request, dobot_bringup::DI::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "DI(%d)", request.index);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::DO(dobot_bringup::DO::Request& request, dobot_bringup::DO::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "DO(%d,%d)", request.index, request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::DOExecute(dobot_bringup::DOExecute::Request& request, dobot_bringup::DOExecute::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "DO(%d,%d)", request.index, request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::toolDO(dobot_bringup::ToolDO::Request& request, dobot_bringup::ToolDO::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "ToolDO(%d,%d)", request.index, request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::toolDOExecute(dobot_bringup::ToolDOExecute::Request& request,
                             dobot_bringup::ToolDOExecute::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "ToolDOExecute(%d,%d)", request.index, request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::AO(dobot_bringup::AO::Request& request, dobot_bringup::AO::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "AO(%d,%d)", request.index, request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::AOExecute(dobot_bringup::AOExecute::Request& request, dobot_bringup::AOExecute::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "AO(%d,%0.3f)", request.index, static_cast<float>(request.value));
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::accJ(dobot_bringup::AccJ::Request& request, dobot_bringup::AccJ::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "AccJ(%d)", request.r);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::accL(dobot_bringup::AccL::Request& request, dobot_bringup::AccL::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "AccL(%d)", request.r);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::speedJ(dobot_bringup::SpeedJ::Request& request, dobot_bringup::SpeedJ::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SpeedJ(%d)", request.r);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::speedL(dobot_bringup::SpeedL::Request& request, dobot_bringup::SpeedL::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SpeedL(%d)", request.r);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::arch(dobot_bringup::Arch::Request& request, dobot_bringup::Arch::Response& response)
{
    try {
        char cmd[100];
        if (!request.cpValue.empty() && !request.cpValue[0].empty()) {
            sprintf(cmd, "Arch(%d,%s)", request.index, request.cpValue[0].c_str());

        } else {
            sprintf(cmd, "Arch(%d)", request.index);
        }
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::cp(dobot_bringup::CP::Request& request, dobot_bringup::CP::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "CP(%d)", request.r);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::limZ(dobot_bringup::LimZ::Request& request, dobot_bringup::LimZ::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "LimZ(%d)", request.value);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::setArmOrientation(dobot_bringup::SetArmOrientation::Request& request,
                                 dobot_bringup::SetArmOrientation::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SetArmOrientation(%d,%d,%d,%d)", request.LorR, request.UorD, request.ForN, request.Config6);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::setPayload(dobot_bringup::SetPayload::Request& request, dobot_bringup::SetPayload::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SetPayload(%f)", request.load);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::positiveSolution(dobot_bringup::PositiveSolution::Request& request,
                                dobot_bringup::PositiveSolution::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "PositiveSolution(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d)", request.offset1, request.offset2,
                request.offset3, request.offset4, request.offset5, request.offset6, request.user, request.tool);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::inverseSolution(dobot_bringup::InverseSolution::Request& request,
                               dobot_bringup::InverseSolution::Response& response)
{
    try {
        char cmd[100];
        if (request.JointNear == "") {
            sprintf(cmd, "InverseSolution(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d)", request.offset1, request.offset2,
                    request.offset3, request.offset4, request.offset5, request.offset6, request.user, request.tool);
        } else {
            sprintf(cmd, "InverseSolution(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d,%d,%s)", request.offset1,
                    request.offset2, request.offset3, request.offset4, request.offset5, request.offset6, request.user,
                    request.tool, request.isJointNear, request.JointNear.c_str());
        }

        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::powerOn(dobot_bringup::PowerOn::Request& request, dobot_bringup::PowerOn::Response& response)
{
    try {
        const char* cmd = "PowerOn()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::runScript(dobot_bringup::RunScript::Request& request, dobot_bringup::RunScript::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "RunScript(%s)", request.projectName.c_str());
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::stopScript(dobot_bringup::StopScript::Request& request, dobot_bringup::StopScript::Response& response)
{
    try {
        const char* cmd = "StopScript()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::pauseScript(dobot_bringup::PauseScript::Request& request, dobot_bringup::PauseScript::Response& response)
{
    try {
        const char* cmd = "PauseScript()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::modbusCreate(dobot_bringup::ModbusCreate::Request& request,
                            dobot_bringup::ModbusCreate::Response& response)
{
    try {
        char cmd[300];
        std::vector<std::string> result;
        if (request.is_rtu.empty()) {
            snprintf(cmd, sizeof(cmd), "ModbusCreate(%s,%d,%d)", request.ip.c_str(), request.port, request.slave_id);
        } else {
            snprintf(cmd, sizeof(cmd), "ModbusCreate(%s,%d,%d,%d)", request.ip.c_str(), request.port, request.slave_id,
                     request.is_rtu[0]);
        }
        commander_->dashboardDoCmd(cmd, response.res, result);
        if (result.size() != 1) {
            ROS_ERROR("%s", "Haven't recv any result");
            response.res = -1;
            response.index = -1;
            return true;
        }

        response.index = str2Int(result[0].c_str());
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        response.index = -1;
        return true;
    } catch (const std::exception& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        response.index = -1;
        return true;
    }
}

bool CR5Robot::modbusClose(dobot_bringup::ModbusClose::Request& request, dobot_bringup::ModbusClose::Response& response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "ModbusClose(%d)", request.index);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::getInBits(dobot_bringup::GetInBits::Request& request, dobot_bringup::GetInBits::Response& response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "GetInBits(%d,%d,%d)", request.index, request.addr, request.count);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::getInRegs(dobot_bringup::GetInRegs::Request& request, dobot_bringup::GetInRegs::Response& response)
{
    try {
        char cmd[100];
        if (!request.valType.empty() && !request.valType[0].empty()) {
            snprintf(cmd, sizeof(cmd), "GetInRegs(%d,%d,%d,%s)", request.index, request.addr, request.count,
                     request.valType[0].c_str());
        } else {
            snprintf(cmd, sizeof(cmd), "GetInRegs(%d,%d,%d)", request.index, request.addr, request.count);
        }
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::getHoldRegs(dobot_bringup::GetHoldRegs::Request& request, dobot_bringup::GetHoldRegs::Response& response)
{
    try {
        char cmd[100];
        std::vector<std::string> result;
        if (!request.valtype.empty() && !request.valtype[0].empty()) {
            snprintf(cmd, sizeof(cmd), "GetHoldRegs(%d,%d,%d,%s)", request.index, request.addr, request.count,
                     request.valtype[0].c_str());
        } else {
            snprintf(cmd, sizeof(cmd), "GetHoldRegs(%d,%d,%d)", request.index, request.addr, request.count);
        }

        commander_->dashboardDoCmd(cmd, response.res, result);
        if (result.empty()) {
            ROS_ERROR("%s", "Haven't recv any result");
            response.res = -1;
            response.index = -1;
            return true;
        }
        response.index = str2Int(result[0].c_str());
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        response.index = -1;
        return false;
    }
}

bool CR5Robot::setHoldRegs(dobot_bringup::SetHoldRegs::Request& request, dobot_bringup::SetHoldRegs::Response& response)
{
    try {
        char cmd[200];
        std::vector<std::string> result;
        if (!request.valtype.empty() && !request.valtype[0].empty()) {
            snprintf(cmd, sizeof(cmd), "SetHoldRegs(%d,%d,%d,%s,%s)", request.index, request.addr, request.count,
                     request.valTab.c_str(), request.valtype[0].c_str());
        } else {
            snprintf(cmd, sizeof(cmd), "SetHoldRegs(%d,%d,%d,%s)", request.index, request.addr, request.count,
                     request.valTab.c_str());
        }

        commander_->dashboardDoCmd(cmd, response.res, result);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return true;
    } catch (const std::exception& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return true;
    }
}

bool CR5Robot::getCoils(dobot_bringup::GetCoils::Request& request, dobot_bringup::GetCoils::Response& response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "GetCoils(%d,%d,%d)", request.index, request.addr, request.count);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::setCoils(dobot_bringup::SetCoils::Request& request, dobot_bringup::SetCoils::Response& response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "SetCoils(%d,%d,%d,%s)", request.index, request.addr, request.count,
                 request.valTab.c_str());
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::getPathStartPose(dobot_bringup::GetPathStartPose::Request& request,
                                dobot_bringup::GetPathStartPose::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "GetPathStartPose(%s)", request.traceName.c_str());
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::getTraceStartPose(dobot_bringup::GetTraceStartPose::Request& request,
                                 dobot_bringup::GetTraceStartPose::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "GetTraceStartPose(%s)", request.traceName.c_str());
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::handleTrajPoints(dobot_bringup::HandleTrajPoints::Request& request,
                                dobot_bringup::HandleTrajPoints::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "HandleTrajPoints(%s)", request.traceName.c_str());
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::getSixForceData(dobot_bringup::GetSixForceData::Request& request,
                               dobot_bringup::GetSixForceData::Response& response)
{
    try {
        const char* cmd = "GetSixForceData()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::setCollideDrag(dobot_bringup::SetCollideDrag::Request& request,
                              dobot_bringup::SetCollideDrag::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SetCollideDrag(%d)", request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::setTerminalKeys(dobot_bringup::SetTerminalKeys::Request& request,
                               dobot_bringup::SetTerminalKeys::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SetTerminalKeys(%d)", request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::setTerminal485(dobot_bringup::SetTerminal485::Request& request,
                              dobot_bringup::SetTerminal485::Response& response)
{
    try {
        char cmd[200];
        sprintf(cmd, "SetTerminal485(%d,%d,%s,%d)", request.baudRate, request.dataLen, request.parityBit.c_str(),
                request.stopBit);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::getTerminal485(dobot_bringup::GetTerminal485::Request& request,
                              dobot_bringup::GetTerminal485::Response& response)
{
    try {
        const char* cmd = "GetTerminal485()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::tCPSpeed(dobot_bringup::TCPSpeed::Request& request, dobot_bringup::TCPSpeed::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "TCPSpeed(%d)", request.vt);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::tCPSpeedEnd(dobot_bringup::TCPSpeedEnd::Request& request, dobot_bringup::TCPSpeedEnd::Response& response)
{
    try {
        const char* cmd = "TCPSpeedEnd()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  real time
 *----------------------------------------------------------------------------------------------------------------------
 */

bool CR5Robot::servoJParam(dobot_bringup::ServoJParam::Request& request, dobot_bringup::ServoJParam::Response& response)
{
    try {
        if (kServoJParam) {
            kServoJParam->trajectory_duration = request.trajectory_duration;
            kServoJParam->t = request.t;
            kServoJParam->lookahead_time = request.lookahead_time;
            kServoJParam->gain = request.gain;
            ROS_INFO("Set ServoJParam- trajectory_duratio: %f, t:%f, lookahead_time:%f, gain:%f ",
                     kServoJParam->trajectory_duration, kServoJParam->t, kServoJParam->lookahead_time,
                     kServoJParam->gain);
            response.res = 0;
        }
    } catch (const std::exception& err) {
        ROS_ERROR("Caught exception: %s", err.what());
        response.res = -1;
    }
    return true;
}

bool CR5Robot::tcpRealData(dobot_bringup::TcpRealData::Request& request, dobot_bringup::TcpRealData::Response& response)
{
    if (!commander_->isConnected()) {
        return false;
    }
    uint32_t index = request.index;
    uint32_t size = request.size;
    constexpr uint32_t limit_size = sizeof(RealTimeData);
    if (index >= limit_size || index + size >= limit_size) {
        return false;
    }
    const char* data = (const char*)commander_->getRealData();
    response.real_data.insert(response.real_data.begin(), data + index, data + index + size);
    return true;
}

bool CR5Robot::toolDI(dobot_bringup::ToolDI::Request& request, dobot_bringup::ToolDI::Response& response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "ToolDI(%d)", request.index);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::AI(dobot_bringup::AI::Request& request, dobot_bringup::AI::Response& response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "AI(%d)", request.index);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::toolAI(dobot_bringup::ToolAI::Request& request, dobot_bringup::ToolAI::Response& response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "ToolAI(%d)", request.index);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::DIGroup(dobot_bringup::DIGroup::Request& request, dobot_bringup::DIGroup::Response& response)
{
    try {
        char cmd[1000];
        std::string str{ "DIGroup(" };
        for (int i = 0; i < request.args.size(); i++) {
            if (i == request.args.size() - 1) {
                str = str + std::to_string(request.args[i]);
                break;
            }
            str = str + std::to_string(request.args[i]) + ",";
        }
        str = str + ")";
        strcpy(cmd, str.c_str());
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::DOGroup(dobot_bringup::DOGroup::Request& request, dobot_bringup::DOGroup::Response& response)
{
    try {
        char cmd[1000];
        std::string str{ "DOGroup(" };
        for (int i = 0; i < request.args.size(); i++) {
            if (i == request.args.size() - 1) {
                str = str + std::to_string(request.args[i]);
                break;
            }
            str = str + std::to_string(request.args[i]) + ",";
        }
        str = str + ")";
        strcpy(cmd, str.c_str());
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::brakeControl(dobot_bringup::BrakeControl::Request& request,
                            dobot_bringup::BrakeControl::Response& response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "BrakeControl(%d,%d)", request.axisID, request.value);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::startDrag(dobot_bringup::StartDrag::Request& request, dobot_bringup::StartDrag::Response& response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "StartDrag()");
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::stopDrag(dobot_bringup::StopDrag::Request& request, dobot_bringup::StopDrag::Response& response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "StopDrag()");
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::loadSwitch(dobot_bringup::LoadSwitch::Request& request, dobot_bringup::LoadSwitch::Response& response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "LoadSwitch(%d)", request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::continueScript(dobot_bringup::ContinueScript::Request& request,
                              dobot_bringup::ContinueScript::Response& response)
{
    try {
        const char* cmd = "ContinueScript()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::setSafeSkin(dobot_bringup::SetSafeSkin::Request& request, dobot_bringup::SetSafeSkin::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SetSafeSkin(%d)", request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::setObstacleAvoid(dobot_bringup::SetObstacleAvoid::Request& request,
                                dobot_bringup::SetObstacleAvoid::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SetObstacleAvoid(%d)", request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::setCollisionLevel(dobot_bringup::SetCollisionLevel::Request& request,
                                 dobot_bringup::SetCollisionLevel::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SetCollisionLevel(%d)", request.level);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::getAngle(dobot_bringup::GetAngle::Request& request, dobot_bringup::GetAngle::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "GetAngle()");
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::getPose(dobot_bringup::GetPose::Request& request, dobot_bringup::GetPose::Response& response)
{
    try {
        char cmd[100];

        if (request.user.empty()) {
            sprintf(cmd, "GetPose()");
        } else {
            sprintf(cmd, "GetPose(%d,%d)", request.user[0], request.tool[0]);
        }

        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::emergencyStop(dobot_bringup::EmergencyStop::Request& request,
                             dobot_bringup::EmergencyStop::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "EmergencyStop()");
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::sync(dobot_bringup::Sync::Request& request, dobot_bringup::Sync::Response& response)
{
    try {
        char result[50];
        const char* cmd = "Sync()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::getErrorID(dobot_bringup::GetErrorID::Request& request, dobot_bringup::GetErrorID::Response& response)
{
    try {
        char cmd[100];
        std::vector<std::string> result;
        sprintf(cmd, "GetErrorID()");
        commander_->dashboardDoCmd(cmd, response.res, result);
        std::string resultStr{ "" };
        for (int i = 0; i < result.size(); i++) {
            resultStr = resultStr.append(result[i]);
        }
        result.clear();
        result = regexRecv(resultStr);
        for (int i = 0; i < result.size(); i++) {
            ROS_ERROR("ErrorID: %s", result[i].c_str());
            response.errorID.push_back(std::stoi(result[i]));
        }

        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::tcpDashboard(dobot_bringup::TcpDashboard::Request& request,
                            dobot_bringup::TcpDashboard::Response& response)
{
    try {
        const char* cmd = request.command.c_str();
        std::vector<std::string> result;
        int res;
        commander_->dashboardDoCmd(cmd, res, result);
        for (const auto& i : result) {
            if (response.result.size() != 0) {
                response.result += ",";
            }
            response.result += i;
        }
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        return false;
    }
}

/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  real time
 *----------------------------------------------------------------------------------------------------------------------
 */

bool CR5Robot::movJ(dobot_bringup::MovJ::Request& request, dobot_bringup::MovJ::Response& response)
{
    try {
        char cmd[200];
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++) {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        sprintf(cmd, "MovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request.x, request.y, request.z, request.a, request.b,
                request.c);
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::movL(dobot_bringup::MovL::Request& request, dobot_bringup::MovL::Response& response)
{
    try {
        char cmd[200];
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++) {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        sprintf(cmd, "MovL(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request.x, request.y, request.z, request.a, request.b,
                request.c);
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::servoJ(dobot_bringup::ServoJ::Request& request, dobot_bringup::ServoJ::Response& response)
{
    try {
        char cmd[100];
        if (request.t.empty()) {
            sprintf(cmd, "ServoJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,t=0.4)", request.offset1, request.offset2,
                    request.offset3, request.offset4, request.offset5, request.offset6);
        } else {
            sprintf(cmd, "ServoJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request.offset1,
                    request.offset2, request.offset3, request.offset4, request.offset5, request.offset6, request.t[0],
                    request.lookahead_time[0], request.gain[0]);
        }
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::jump(dobot_bringup::Jump::Request& request, dobot_bringup::Jump::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "Jump(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request.offset1, request.offset2, request.offset3,
                request.offset4, request.offset5, request.offset6);
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::arc(dobot_bringup::Arc::Request& request, dobot_bringup::Arc::Response& response)
{
    try {
        char cmd[200];
        sprintf(cmd, "Arc(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request.x1,
                request.y1, request.z1, request.a1, request.b1, request.c1, request.x2, request.y2, request.z2,
                request.a2, request.b2, request.c2);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++) {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::circle3(dobot_bringup::Circle::Request& request, dobot_bringup::Circle::Response& response)
{
    try {
        char cmd[200];
        sprintf(cmd, "Circle3(%d, %0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f",
                request.count, request.x1, request.y1, request.z1, request.a1, request.b1, request.c1, request.x2,
                request.y2, request.z2, request.a2, request.b2, request.c2);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++) {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::relMovJTool(dobot_bringup::RelMovJTool::Request& request, dobot_bringup::RelMovJTool::Response& response)
{
    try {
        char cmd[200];
        sprintf(cmd, "RelMovJTool(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d", request.x, request.y, request.z, request.rx,
                request.ry, request.rz, request.tool);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++) {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::relMovLTool(dobot_bringup::RelMovLTool::Request& request, dobot_bringup::RelMovLTool::Response& response)
{
    try {
        char cmd[200];
        sprintf(cmd, "RelMovLTool(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d", request.x, request.y, request.z, request.rx,
                request.ry, request.rz, request.tool);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++) {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::relMovJUser(dobot_bringup::RelMovJUser::Request& request, dobot_bringup::RelMovJUser::Response& response)
{
    try {
        char cmd[200];
        sprintf(cmd, "RelMovJUser(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d", request.x, request.y, request.z, request.rx,
                request.ry, request.rz, request.user);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++) {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::relMovLUser(dobot_bringup::RelMovLUser::Request& request, dobot_bringup::RelMovLUser::Response& response)
{
    try {
        char cmd[200];
        sprintf(cmd, "RelMovLUser(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d", request.x, request.y, request.z, request.rx,
                request.ry, request.rz, request.user);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++) {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::relJointMovJ(dobot_bringup::RelJointMovJ::Request& request,
                            dobot_bringup::RelJointMovJ::Response& response)
{
    try {
        char cmd[200];
        sprintf(cmd, "RelJointMovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request.offset1, request.offset2,
                request.offset3, request.offset4, request.offset5, request.offset6);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++) {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::servoP(dobot_bringup::ServoP::Request& request, dobot_bringup::ServoP::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "ServoP(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request.x, request.y, request.z, request.a,
                request.b, request.c);
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::relMovJ(dobot_bringup::RelMovJ::Request& request, dobot_bringup::RelMovJ::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "RelMovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request.offset1, request.offset2, request.offset3,
                request.offset4, request.offset5, request.offset6);
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::relMovL(dobot_bringup::RelMovL::Request& request, dobot_bringup::RelMovL::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "RelMovL(%0.3f,%0.3f,%0.3f)", request.x, request.y, request.z);
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::jointMovJ(dobot_bringup::JointMovJ::Request& request, dobot_bringup::JointMovJ::Response& response)
{
    try {
        char cmd[200];
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++) {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        sprintf(cmd, "JointMovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request.j1, request.j2, request.j3, request.j4,
                request.j5, request.j6);
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::movLIO(dobot_bringup::MovLIO::Request& request, dobot_bringup::MovLIO::Response& response)
{
    try {
        char cmd[200];
        sprintf(cmd, "MovLIO(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request.x, request.y, request.z, request.rx,
                request.ry, request.rz);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++) {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::movJIO(dobot_bringup::MovJIO::Request& request, dobot_bringup::MovJIO::Response& response)
{
    try {
        char cmd[200];
        sprintf(cmd, "MovJIO(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request.x, request.y, request.z, request.rx,
                request.ry, request.rz);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++) {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::startTrace(dobot_bringup::StartTrace::Request& request, dobot_bringup::StartTrace::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "StartTrace(%s)", request.trace_name.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::startPath(dobot_bringup::StartPath::Request& request, dobot_bringup::StartPath::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "StartPath(%s,%d,%d)", request.trace_name.c_str(), request.const_val, request.cart);
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::startFCTrace(dobot_bringup::StartFCTrace::Request& request,
                            dobot_bringup::StartFCTrace::Response& response)
{
    try {
        char cmd[100];
        sprintf(cmd, "StartFCTrace(%s)", request.trace_name.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::moveJog(dobot_bringup::MoveJog::Request& request, dobot_bringup::MoveJog::Response& response)
{
    try {
        char cmd[100];
        std::string str = "moveJog(" + std::string(request.axisID);
        for (int i = 0; i < request.paramValue.size(); i++) {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcpy(cmd, str.c_str());
        sprintf(cmd, "MoveJog(%s)", request.axisID.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::stopmoveJog(dobot_bringup::StopmoveJog::Request& request, dobot_bringup::StopmoveJog::Response& response)
{
    try {
        char cmd[100] = "moveJog()";
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::wait(dobot_bringup::Wait::Request& request, dobot_bringup::Wait::Response& response)
{
    try {
        char cmd[100] = "Wait()";
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::Continue(dobot_bringup::Continues::Request& request, dobot_bringup::Continues::Response& response)
{
    try {
        char cmd[100] = "Continue()";
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool CR5Robot::pause(dobot_bringup::pause::Request& request, dobot_bringup::pause::Response& response)
{
    try {
        char cmd[100] = "pause()";
        commander_->motionDoCmd(cmd, response.res);
        return true;
    } catch (const TcpClientException& err) {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

int CR5Robot::str2Int(const char* val)
{
    char* err;
    int mode = (int)strtol(val, &err, 10);
    if (*err != 0)
        throw std::logic_error(std::string("Invalid value : ") + val);
    return mode;
}

void CR5Robot::backendTask(const ros::TimerEvent& e)
{
    uint16_t robot_mode = commander_->getRobotMode();
    if (robot_mode == 9 && last_robot_mode_ != 9) {
        dobot_bringup::GetErrorID::Request req = {};
        dobot_bringup::GetErrorID::Response res = {};
        bool ok = getErrorID(req, res);
        if (ok) {
            for (auto errorid : res.errorID) {
                ROS_ERROR("Robot alarm, error id %d", errorid);
            }

        } else {
            ROS_ERROR("Robot alarm");
        }
    }
    last_robot_mode_ = robot_mode;
}

std::vector<std::string> CR5Robot::regexRecv(std::string getRecvInfo)
{
    std::regex pattern("-?\\d+");
    std::smatch matches;
    std::string::const_iterator searchStart(getRecvInfo.cbegin());
    std::vector<std::string> vecErrorId;
    while (std::regex_search(searchStart, getRecvInfo.cend(), matches, pattern)) {
        for (auto& match : matches) {
            vecErrorId.push_back(match.str());
        }
        searchStart = matches.suffix().first;
    }
    return vecErrorId;
};
