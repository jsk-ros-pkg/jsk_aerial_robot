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

#include <string>
#include <memory>
#include <ros/ros.h>
#include <dobot_bringup/commander.h>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <dobot_bringup/EnableRobot.h>
#include <dobot_bringup/DisableRobot.h>
#include <dobot_bringup/ClearError.h>
#include <dobot_bringup/ResetRobot.h>
#include <dobot_bringup/SpeedFactor.h>
#include <dobot_bringup/User.h>
#include <dobot_bringup/Tool.h>
#include <dobot_bringup/RobotMode.h>
#include <dobot_bringup/PayLoad.h>
#include <dobot_bringup/DO.h>
#include <dobot_bringup/DOExecute.h>
#include <dobot_bringup/ToolDO.h>
#include <dobot_bringup/ToolDOExecute.h>
#include <dobot_bringup/AO.h>
#include <dobot_bringup/AOExecute.h>
#include <dobot_bringup/AccJ.h>
#include <dobot_bringup/AccL.h>
#include <dobot_bringup/SpeedJ.h>
#include <dobot_bringup/SpeedL.h>
#include <dobot_bringup/Arch.h>
#include <dobot_bringup/CP.h>
#include <dobot_bringup/LimZ.h>
#include <dobot_bringup/SetArmOrientation.h>
#include <dobot_bringup/PowerOn.h>
#include <dobot_bringup/RunScript.h>
#include <dobot_bringup/StopScript.h>
#include <dobot_bringup/PauseScript.h>
#include <dobot_bringup/ContinueScript.h>
#include <dobot_bringup/GetHoldRegs.h>
#include <dobot_bringup/SetHoldRegs.h>
#include <dobot_bringup/SetSafeSkin.h>
#include <dobot_bringup/SetObstacleAvoid.h>

#include <dobot_bringup/SetCollisionLevel.h>
#include <dobot_bringup/EmergencyStop.h>
#include <dobot_bringup/GetTraceStartPose.h>
#include <dobot_bringup/GetPathStartPose.h>
#include <dobot_bringup/HandleTrajPoints.h>
#include <dobot_bringup/GetSixForceData.h>
#include <dobot_bringup/SetCollideDrag.h>
#include <dobot_bringup/SetTerminalKeys.h>
#include <dobot_bringup/SetTerminal485.h>
#include <dobot_bringup/GetTerminal485.h>
#include <dobot_bringup/TCPSpeed.h>
#include <dobot_bringup/TCPSpeedEnd.h>
#include <dobot_bringup/MovJ.h>
#include <dobot_bringup/MovL.h>
#include <dobot_bringup/Jump.h>
#include <dobot_bringup/Arc.h>
#include <dobot_bringup/Sync.h>
#include <dobot_bringup/Circle.h>
#include <dobot_bringup/ServoJ.h>
#include <dobot_bringup/ServoP.h>
#include <dobot_bringup/ServoJParam.h>
#include <dobot_bringup/StartTrace.h>
#include <dobot_bringup/StartPath.h>
#include <dobot_bringup/StartFCTrace.h>
#include <dobot_bringup/MoveJog.h>
#include <dobot_bringup/RelMovJ.h>
#include <dobot_bringup/RelMovL.h>
#include <dobot_bringup/JointMovJ.h>
#include <dobot_bringup/RobotStatus.h>
#include <dobot_bringup/ModbusCreate.h>
#include <dobot_bringup/GetErrorID.h>
#include <dobot_bringup/SetPayload.h>
#include <dobot_bringup/PositiveSolution.h>
#include <dobot_bringup/InverseSolution.h>
#include <dobot_bringup/ModbusClose.h>
#include <dobot_bringup/GetInBits.h>
#include <dobot_bringup/GetInRegs.h>
#include <dobot_bringup/GetCoils.h>
#include <dobot_bringup/SetCoils.h>
#include <dobot_bringup/DI.h>
#include <dobot_bringup/ToolDI.h>
#include <dobot_bringup/AI.h>
#include <dobot_bringup/ToolAI.h>
#include <dobot_bringup/DIGroup.h>
#include <dobot_bringup/DOGroup.h>
#include <dobot_bringup/BrakeControl.h>
#include <dobot_bringup/StartDrag.h>
#include <dobot_bringup/StopDrag.h>
#include <dobot_bringup/LoadSwitch.h>
#include <dobot_bringup/GetAngle.h>
#include <dobot_bringup/GetPose.h>
#include <dobot_bringup/MovLIO.h>
#include <dobot_bringup/MovJIO.h>
#include <dobot_bringup/RelMovJTool.h>
#include <dobot_bringup/RelMovLTool.h>
#include <dobot_bringup/RelMovJUser.h>
#include <dobot_bringup/RelMovLUser.h>
#include <dobot_bringup/StopmoveJog.h>
#include <dobot_bringup/Wait.h>
#include <dobot_bringup/Continues.h>
#include <dobot_bringup/pause.h>
#include <dobot_bringup/RelJointMovJ.h>
#include <dobot_bringup/TcpRealData.h>
#include <dobot_bringup/TcpDashboard.h>

using namespace actionlib;
using namespace control_msgs;

struct ServoJParam
{
    double trajectory_duration = 0.0;
    double t = 0.0;
    double lookahead_time = 0.0;
    double gain = 0.0;
};

/**
 * CR5Robot
 */
class CR5Robot : protected ActionServer<FollowJointTrajectoryAction>
{
private:
    double goal_[6];
    uint32_t index_;
    ros::Timer timer_;
    ros::Timer movj_timer_;
    ros::Timer backend_task_;
    double trajectory_duration_;
    ros::NodeHandle control_nh_;
    std::shared_ptr<CR5Commander> commander_;
    std::vector<ros::ServiceServer> server_tbl_;
    uint16_t last_robot_mode_;
    std::thread threadPubFeedBackInfo;
    ros::Publisher pubFeedInfo;
    std::shared_ptr<ServoJParam> kServoJParam;

public:
    /**
     * Ctor
     * @param nh node handle
     * @param name topic
     */
    CR5Robot(ros::NodeHandle& nh, std::string name);

    /**
     * CR5Robot
     */
    ~CR5Robot() override;

    /**
     * init
     */
    void init();

    /**
     * getJointState
     * @param point
     */
    void getJointState(double* point);

    /**
     * getToolVectorActual
     * @param val value
     */
    void getToolVectorActual(double* val);

    /**
     * isEnable
     * @return ture enable, otherwise false
     */
    bool isEnable() const;

    /**
     * isConnected
     * @return ture connected, otherwise false
     */
    bool isConnected() const;

protected:
    bool enableRobot(dobot_bringup::EnableRobot::Request& request, dobot_bringup::EnableRobot::Response& response);
    bool disableRobot(dobot_bringup::DisableRobot::Request& request, dobot_bringup::DisableRobot::Response& response);
    bool clearError(dobot_bringup::ClearError::Request& request, dobot_bringup::ClearError::Response& response);
    bool resetRobot(dobot_bringup::ResetRobot::Request& request, dobot_bringup::ResetRobot::Response& response);
    bool speedFactor(dobot_bringup::SpeedFactor::Request& request, dobot_bringup::SpeedFactor::Response& response);
    bool getErrorID(dobot_bringup::GetErrorID::Request& request, dobot_bringup::GetErrorID::Response& response);
    bool user(dobot_bringup::User::Request& request, dobot_bringup::User::Response& response);
    bool tool(dobot_bringup::Tool::Request& request, dobot_bringup::Tool::Response& response);
    bool robotMode(dobot_bringup::RobotMode::Request& request, dobot_bringup::RobotMode::Response& response);
    bool payload(dobot_bringup::PayLoad::Request& request, dobot_bringup::PayLoad::Response& response);
    bool DO(dobot_bringup::DO::Request& request, dobot_bringup::DO::Response& response);
    bool DOExecute(dobot_bringup::DOExecute::Request& request, dobot_bringup::DOExecute::Response& response);
    bool toolDO(dobot_bringup::ToolDO::Request& request, dobot_bringup::ToolDO::Response& response);
    bool toolDOExecute(dobot_bringup::ToolDOExecute::Request& request,
                       dobot_bringup::ToolDOExecute::Response& response);
    bool AO(dobot_bringup::AO::Request& request, dobot_bringup::AO::Response& response);
    bool AOExecute(dobot_bringup::AOExecute::Request& request, dobot_bringup::AOExecute::Response& response);
    bool accJ(dobot_bringup::AccJ::Request& request, dobot_bringup::AccJ::Response& response);
    bool accL(dobot_bringup::AccL::Request& request, dobot_bringup::AccL::Response& response);
    bool speedJ(dobot_bringup::SpeedJ::Request& request, dobot_bringup::SpeedJ::Response& response);
    bool speedL(dobot_bringup::SpeedL::Request& request, dobot_bringup::SpeedL::Response& response);
    bool arch(dobot_bringup::Arch::Request& request, dobot_bringup::Arch::Response& response);
    bool cp(dobot_bringup::CP::Request& request, dobot_bringup::CP::Response& response);
    bool limZ(dobot_bringup::LimZ::Request& request, dobot_bringup::LimZ::Response& response);
    bool setArmOrientation(dobot_bringup::SetArmOrientation::Request& request,
                           dobot_bringup::SetArmOrientation::Response& response);
    bool setPayload(dobot_bringup::SetPayload::Request& request, dobot_bringup::SetPayload::Response& response);
    bool positiveSolution(dobot_bringup::PositiveSolution::Request& request,
                          dobot_bringup::PositiveSolution::Response& response);
    bool inverseSolution(dobot_bringup::InverseSolution::Request& request,
                         dobot_bringup::InverseSolution::Response& response);
    bool powerOn(dobot_bringup::PowerOn::Request& request, dobot_bringup::PowerOn::Response& response);
    bool runScript(dobot_bringup::RunScript::Request& request, dobot_bringup::RunScript::Response& response);
    bool stopScript(dobot_bringup::StopScript::Request& request, dobot_bringup::StopScript::Response& response);
    bool pauseScript(dobot_bringup::PauseScript::Request& request, dobot_bringup::PauseScript::Response& response);
    bool continueScript(dobot_bringup::ContinueScript::Request& request,
                        dobot_bringup::ContinueScript::Response& response);
    bool getHoldRegs(dobot_bringup::GetHoldRegs::Request& request, dobot_bringup::GetHoldRegs::Response& response);
    bool setHoldRegs(dobot_bringup::SetHoldRegs::Request& request, dobot_bringup::SetHoldRegs::Response& response);
    bool getInBits(dobot_bringup::GetInBits::Request& request, dobot_bringup::GetInBits::Response& response);
    bool getInRegs(dobot_bringup::GetInRegs::Request& request, dobot_bringup::GetInRegs::Response& response);
    bool getCoils(dobot_bringup::GetCoils::Request& request, dobot_bringup::GetCoils::Response& response);
    bool setCoils(dobot_bringup::SetCoils::Request& request, dobot_bringup::SetCoils::Response& response);
    bool DI(dobot_bringup::DI::Request& request, dobot_bringup::DI::Response& response);
    bool toolDI(dobot_bringup::ToolDI::Request& request, dobot_bringup::ToolDI::Response& response);
    bool AI(dobot_bringup::AI::Request& request, dobot_bringup::AI::Response& response);
    bool toolAI(dobot_bringup::ToolAI::Request& request, dobot_bringup::ToolAI::Response& response);
    bool DIGroup(dobot_bringup::DIGroup::Request& request, dobot_bringup::DIGroup::Response& response);
    bool DOGroup(dobot_bringup::DOGroup::Request& request, dobot_bringup::DOGroup::Response& response);
    bool brakeControl(dobot_bringup::BrakeControl::Request& request, dobot_bringup::BrakeControl::Response& response);
    bool startDrag(dobot_bringup::StartDrag::Request& request, dobot_bringup::StartDrag::Response& response);
    bool stopDrag(dobot_bringup::StopDrag::Request& request, dobot_bringup::StopDrag::Response& response);
    bool loadSwitch(dobot_bringup::LoadSwitch::Request& request, dobot_bringup::LoadSwitch::Response& response);

    bool setSafeSkin(dobot_bringup::SetSafeSkin::Request& request, dobot_bringup::SetSafeSkin::Response& response);
    bool setObstacleAvoid(dobot_bringup::SetObstacleAvoid::Request& request,
                          dobot_bringup::SetObstacleAvoid::Response& response);
    bool setCollisionLevel(dobot_bringup::SetCollisionLevel::Request& request,
                           dobot_bringup::SetCollisionLevel::Response& response);
    bool emergencyStop(dobot_bringup::EmergencyStop::Request& request,
                       dobot_bringup::EmergencyStop::Response& response);

    bool getTraceStartPose(dobot_bringup::GetTraceStartPose::Request& request,
                           dobot_bringup::GetTraceStartPose::Response& response);
    bool getPathStartPose(dobot_bringup::GetPathStartPose::Request& request,
                          dobot_bringup::GetPathStartPose::Response& response);
    bool handleTrajPoints(dobot_bringup::HandleTrajPoints::Request& request,
                          dobot_bringup::HandleTrajPoints::Response& response);
    bool getSixForceData(dobot_bringup::GetSixForceData::Request& request,
                         dobot_bringup::GetSixForceData::Response& response);
    bool setCollideDrag(dobot_bringup::SetCollideDrag::Request& request,
                        dobot_bringup::SetCollideDrag::Response& response);
    bool setTerminalKeys(dobot_bringup::SetTerminalKeys::Request& request,
                         dobot_bringup::SetTerminalKeys::Response& response);
    bool setTerminal485(dobot_bringup::SetTerminal485::Request& request,
                        dobot_bringup::SetTerminal485::Response& response);
    bool getTerminal485(dobot_bringup::GetTerminal485::Request& request,
                        dobot_bringup::GetTerminal485::Response& response);
    bool tCPSpeed(dobot_bringup::TCPSpeed::Request& request, dobot_bringup::TCPSpeed::Response& response);
    bool tCPSpeedEnd(dobot_bringup::TCPSpeedEnd::Request& request, dobot_bringup::TCPSpeedEnd::Response& response);

    bool getAngle(dobot_bringup::GetAngle::Request& request, dobot_bringup::GetAngle::Response& response);
    bool getPose(dobot_bringup::GetPose::Request& request, dobot_bringup::GetPose::Response& response);
    bool modbusCreate(dobot_bringup::ModbusCreate::Request& request, dobot_bringup::ModbusCreate::Response& response);
    bool modbusClose(dobot_bringup::ModbusClose::Request& request, dobot_bringup::ModbusClose::Response& response);
    bool movJ(dobot_bringup::MovJ::Request& request, dobot_bringup::MovJ::Response& response);
    bool movL(dobot_bringup::MovL::Request& request, dobot_bringup::MovL::Response& response);
    bool jointMovJ(dobot_bringup::JointMovJ::Request& request, dobot_bringup::JointMovJ::Response& response);
    bool jump(dobot_bringup::Jump::Request& request, dobot_bringup::Jump::Response& response);
    bool relMovJ(dobot_bringup::RelMovJ::Request& request, dobot_bringup::RelMovJ::Response& response);
    bool relMovL(dobot_bringup::RelMovL::Request& request, dobot_bringup::RelMovL::Response& response);
    bool movLIO(dobot_bringup::MovLIO::Request& request, dobot_bringup::MovLIO::Response& response);
    bool movJIO(dobot_bringup::MovJIO::Request& request, dobot_bringup::MovJIO::Response& response);
    bool arc(dobot_bringup::Arc::Request& request, dobot_bringup::Arc::Response& response);
    bool circle3(dobot_bringup::Circle::Request& request, dobot_bringup::Circle::Response& response);
    bool relMovJTool(dobot_bringup::RelMovJTool::Request& request, dobot_bringup::RelMovJTool::Response& response);
    bool relMovLTool(dobot_bringup::RelMovLTool::Request& request, dobot_bringup::RelMovLTool::Response& response);
    bool relMovJUser(dobot_bringup::RelMovJUser::Request& request, dobot_bringup::RelMovJUser::Response& response);
    bool relMovLUser(dobot_bringup::RelMovLUser::Request& request, dobot_bringup::RelMovLUser::Response& response);
    bool relJointMovJ(dobot_bringup::RelJointMovJ::Request& request, dobot_bringup::RelJointMovJ::Response& response);
    bool servoJ(dobot_bringup::ServoJ::Request& request, dobot_bringup::ServoJ::Response& response);
    bool servoP(dobot_bringup::ServoP::Request& request, dobot_bringup::ServoP::Response& response);
    bool sync(dobot_bringup::Sync::Request& request, dobot_bringup::Sync::Response& response);
    bool startTrace(dobot_bringup::StartTrace::Request& request, dobot_bringup::StartTrace::Response& response);
    bool startPath(dobot_bringup::StartPath::Request& request, dobot_bringup::StartPath::Response& response);
    bool startFCTrace(dobot_bringup::StartFCTrace::Request& request, dobot_bringup::StartFCTrace::Response& response);
    bool moveJog(dobot_bringup::MoveJog::Request& request, dobot_bringup::MoveJog::Response& response);
    bool stopmoveJog(dobot_bringup::StopmoveJog::Request& request, dobot_bringup::StopmoveJog::Response& response);
    // bool SyncAll(dobot_bringup::SyncAll::Request& request, dobot_bringup::SyncAll::Response& response);
    bool wait(dobot_bringup::Wait::Request& request, dobot_bringup::Wait::Response& response);
    bool Continue(dobot_bringup::Continues::Request& request, dobot_bringup::Continues::Response& response);
    bool pause(dobot_bringup::pause::Request& request, dobot_bringup::pause::Response& response);

    bool tcpRealData(dobot_bringup::TcpRealData::Request& request, dobot_bringup::TcpRealData::Response& response);
    bool tcpDashboard(dobot_bringup::TcpDashboard::Request& request, dobot_bringup::TcpDashboard::Response& response);
    bool servoJParam(dobot_bringup::ServoJParam::Request& request, dobot_bringup::ServoJParam::Response& response);

private:
    static int str2Int(const char* val);

    void feedbackHandle(const ros::TimerEvent& tm,
                        actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void moveHandle(const ros::TimerEvent& tm, actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void goalHandle(actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void cancelHandle(actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);

    void backendTask(const ros::TimerEvent& e);
    void pubFeedBackInfo();
    std::vector<std::string> regexRecv(std::string getRecvInfo);
};
