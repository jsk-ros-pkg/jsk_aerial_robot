

#include <ros/ros.h>
#include <ros/param.h>
#include <dobot_v4_bringup/cr5_v4_robot.h>
#include <dobot_v4_bringup/parseTool.h>
#include <string>
#include <sstream>

namespace parseTool
{

std::string parserenableRobotRequest2String(dobot_v4_bringup::EnableRobot::Request& request)
{
    std::stringstream ss;
    ss << "EnableRobot()";
    return ss.str();
}
std::string parserdisableRobotRequest2String(dobot_v4_bringup::DisableRobot::Request& request)
{
    std::stringstream ss;
    ss << "DisableRobot()";
    return ss.str();
}
std::string parserclearErrorRequest2String(dobot_v4_bringup::ClearError::Request& request)
{
    std::stringstream ss;
    ss << "ClearError()";
    return ss.str();
}
std::string parserspeedFactorRequest2String(dobot_v4_bringup::SpeedFactor::Request& request)
{
    std::stringstream ss;
    ss << "SpeedFactor(" << request.ratio << ")";
    return ss.str();
}
std::string parseruserRequest2String(dobot_v4_bringup::User::Request& request)
{
    std::stringstream ss;
    ss << "User(" << request.index << ")";
    return ss.str();
}
std::string parsertoolRequest2String(dobot_v4_bringup::Tool::Request& request)
{
    std::stringstream ss;
    ss << "Tool(" << request.index << ")";
    return ss.str();
}
std::string parserrobotModeRequest2String(dobot_v4_bringup::RobotMode::Request& request)
{
    std::stringstream ss;
    ss << "RobotMode()";
    return ss.str();
}
std::string parsersetPayloadRequest2String(dobot_v4_bringup::SetPayload::Request& request)
{
    std::stringstream ss;    // SetPayload(load,x,y,z)
    ss << "SetPayload(" << request.load << "," << request.x << "," << request.y << "," << request.z << ")";
    return ss.str();
}
std::string parserDORequest2String(dobot_v4_bringup::DO::Request& request)
{
    std::stringstream ss;    // DO(index,status,time)

    ss << "DO(" << request.index << "," << request.status << "," << request.time << ")";

    return ss.str();
}
std::string parserDOInstantRequest2String(dobot_v4_bringup::DOInstant::Request& request)
{
    std::stringstream ss;

    ss << "DOInstant(" << request.index << "," << request.status << ")";

    return ss.str();
}
std::string parsertoolDORequest2String(dobot_v4_bringup::ToolDO::Request& request)
{
    std::stringstream ss;

    ss << "ToolDO(" << request.index << "," << request.status << ")";

    return ss.str();
}
std::string parsertoolDOInstantRequest2String(dobot_v4_bringup::ToolDOInstant::Request& request)
{
    std::stringstream ss;

    ss << "ToolDOInstant(" << request.index << "," << request.status << ")";

    return ss.str();
}
std::string parserAORequest2String(dobot_v4_bringup::AO::Request& request)
{
    std::stringstream ss;
    ss << "AO(" << request.index << "," << request.value << ")";
    return ss.str();
}
std::string parserAOInstantRequest2String(dobot_v4_bringup::AOInstant::Request& request)
{
    std::stringstream ss;    // AOInstant(index,value)

    ss << "AOInstant(" << request.index << "," << request.value << ")";

    return ss.str();
}
std::string parseraccJRequest2String(dobot_v4_bringup::AccJ::Request& request)
{
    std::stringstream ss;    // AccJ(R)
    ss << "AccJ(" << request.r << ")";
    return ss.str();
}
std::string parseraccLRequest2String(dobot_v4_bringup::AccL::Request& request)
{
    std::stringstream ss;
    ss << "AccL(" << request.r << ")";
    return ss.str();
}
std::string parservelJRequest2String(dobot_v4_bringup::VelJ::Request& request)
{
    std::stringstream ss;
    ss << "VelJ(" << request.r << ")";
    return ss.str();
}
std::string parservelLRequest2String(dobot_v4_bringup::VelL::Request& request)
{
    std::stringstream ss;
    ss << "VelL(" << request.r << ")";
    return ss.str();
}
std::string parsercpRequest2String(dobot_v4_bringup::CP::Request& request)
{
    std::stringstream ss;
    ss << "CP(" << request.r << ")";
    return ss.str();
}
std::string parserpowerOnRequest2String(dobot_v4_bringup::PowerOn::Request& request)
{
    std::stringstream ss;    // PowerOn()
    ss << "PowerOn()";
    return ss.str();
}
std::string parserrunScriptRequest2String(dobot_v4_bringup::RunScript::Request& request)
{
    std::stringstream ss;    // RunScript(projectName)
    ss << "RunScript(" << request.projectName << ")";
    return ss.str();
}
std::string parserstopRequest2String(dobot_v4_bringup::Stop::Request& request)
{
    std::stringstream ss;    // Stop()
    ss << "Stop()";
    return ss.str();
}
std::string parserpauseRequest2String(dobot_v4_bringup::Pause::Request& request)
{
    std::stringstream ss;    // Pause()
    ss << "Pause()";
    return ss.str();
}
std::string parserContinueRequest2String(dobot_v4_bringup::Continue::Request& request)
{
    std::stringstream ss;    // Continue()
    ss << "Continue()";
    return ss.str();
}
std::string parserEnableSafeSkinRequest2String(dobot_v4_bringup::EnableSafeSkin::Request& request)
{
    std::stringstream ss;
    ss << "EnableSafeSkin(" << request.status << ")";
    return ss.str();
}
std::string parserSetSafeSkinRequest2String(dobot_v4_bringup::SetSafeSkin::Request& request)
{
    std::stringstream ss;
    ss << "SetSafeSkin(";
    ss << request.part << "," << request.status << ")";
    return ss.str();
}
std::string parserGetStartPoseRequest2String(dobot_v4_bringup::GetStartPose::Request& request)
{
    std::stringstream ss;
    ss << "GetStartPose(" << request.traceName << ")";
    return ss.str();
}
std::string parserStartPathRequest2String(dobot_v4_bringup::StartPath::Request& request)
{
    std::string stringOrder = "StartPath(";
    stringOrder = stringOrder + request.traceName;
    if (!(request.paramValue.empty() || request.paramValue[0].empty())) {
        for (int i = 0; i < request.paramValue.size(); i++) {
            stringOrder = stringOrder + "," + std::string(request.paramValue[i]);
        }
    }

    stringOrder = stringOrder + ")";
    ROS_INFO("order:%s", stringOrder.c_str());
    return stringOrder;
}
std::string parserPositiveKinRequest2String(dobot_v4_bringup::PositiveKin::Request& request)
{
    std::stringstream ss;
    ss << "PositiveKin(";
    ss << request.J1 << "," << request.J2 << "," << request.J3 << "," << request.J4 << "," << request.J5 << ","
       << request.J6 << ",";
    if (request.user != "")
        ss << ",user=" << request.user;
    if (request.tool != "")
        ss << ", tool=" << request.tool;
    ss << ")";
    return ss.str();
}
std::string parserInverseKinRequest2String(dobot_v4_bringup::InverseKin::Request& request)
{
    // InverseKin(X,Y,Z,Rx,Ry,Rz,User,Tool,useJointNear,JointNear)
    std::stringstream ss;
    ss << "RunScript(" << request.X << "," << request.Y << "," << request.Z << "," << request.Rx << "," << request.Ry
       << "," << request.Rz << ", user=" << request.user << ",tool=" << request.tool
       << ",useJointNear=" << request.useJointNear << ",jointNear=" << request.jointNear << ")";
    return ss.str();
}
std::string parserGetAngleRequest2String(dobot_v4_bringup::GetAngle::Request& request)
{
    std::stringstream ss;
    ss << "GetAngle()";
    return ss.str();
}
std::string parserGetPoseRequest2String(dobot_v4_bringup::GetPose::Request& request)
{
    std::stringstream ss;
    ss << "GetPose()";
    return ss.str();
}
std::string parserEmergencyStopRequest2String(dobot_v4_bringup::EmergencyStop::Request& request)
{
    std::stringstream ss;
    ss << "EmergencyStop(" << request.value << ")";
    return ss.str();
}
std::string parsersetCollisionLevelRequest2String(dobot_v4_bringup::SetCollisionLevel::Request& request)
{
    std::stringstream ss;
    ss << "SetCollisionLevel(" << request.level << ")";
    return ss.str();
}
std::string parserModbusRTUCreateRequest2String(dobot_v4_bringup::ModbusRTUCreate::Request& request)
{
    std::stringstream ss;
    ss << "ModbusRTUCreate(" << request.slave_id << "," << request.baud << "," << request.parity << ","
       << request.data_bit << "," << request.stop_bit << ")";

    return ss.str();
}
std::string parserModbusCreateRequest2String(dobot_v4_bringup::ModbusCreate::Request& request)
{
    std::stringstream ss;    // ModbusCreate(ip,port,slave_id,isRTU)
    ss << "ModbusCreate(" << request.ip << "," << request.port << "," << request.slave_id << "," << request.isRTU
       << ")";

    return ss.str();
}
std::string parserModbusCloseRequest2String(dobot_v4_bringup::ModbusClose::Request& request)
{
    std::stringstream ss;
    ss << "ModbusClose(" << request.index << ")";
    return ss.str();
}
std::string parserGetInBitsRequest2String(dobot_v4_bringup::GetInBits::Request& request)
{
    std::stringstream ss;
    ss << "GetInBits(" << request.index << ", " << request.addr << ", " << request.count << ")";

    return ss.str();
}
std::string parserGetInRegsRequest2String(dobot_v4_bringup::GetInRegs::Request& request)
{
    std::stringstream ss;
    ss << "GetInRegs(request." << request.index << ", request." << request.addr << ", request." << request.count
       << ", request." << request.valType << ")";

    return ss.str();
}
std::string parserGetCoilsRequest2String(dobot_v4_bringup::GetCoils::Request& request)
{
    std::stringstream ss;
    ss << "GetCoils(request." << request.index << ", request." << request.addr << ", request." << request.count << ")";

    return ss.str();
}
std::string parserSetCoilsRequest2String(dobot_v4_bringup::SetCoils::Request& request)
{
    std::stringstream ss;
    ss << "SetCoils(request." << request.index << ", request." << request.addr << ", request." << request.count
       << ", request." << request.valTab << ")";

    return ss.str();
}
std::string parserGetHoldRegsRequest2String(dobot_v4_bringup::GetHoldRegs::Request& request)
{
    std::stringstream ss;
    ss << "GetHoldRegs(request." << request.index << ", request." << request.addr << ", request." << request.count
       << ", request." << request.valType << ")";

    return ss.str();
}
std::string parserSetHoldRegsRequest2String(dobot_v4_bringup::SetHoldRegs::Request& request)
{
    std::stringstream ss;
    ss << "SetHoldRegs(request." << request.index << ", request." << request.addr << ", request." << request.count
       << ", request." << request.valTab << ", request." << request.valType << ")";

    return ss.str();
}
std::string parserGetErrorIDRequest2String(dobot_v4_bringup::GetErrorID::Request& request)
{
    std::stringstream ss;
    ss << "GetErrorID()";
    return ss.str();
}
std::string parserDIRequest2String(dobot_v4_bringup::DI::Request& request)
{
    std::stringstream ss;
    ss << "DI(" << request.index << ")";
    return ss.str();
}
std::string parserAIRequest2String(dobot_v4_bringup::AI::Request& request)
{
    std::stringstream ss;
    ss << "AI(" << request.index << ")";
    return ss.str();
}
std::string parserDIGroupRequest2String(dobot_v4_bringup::DIGroup::Request& request)
{
    std::stringstream ss;
    ss << "DIGroup(";

    for (int i = 0; i < request.args.size(); ++i) {
        if (i == 0) {
            ss << request.args[i];
        } else
            ss << "," << request.args[i];
    }
    ss << ")";

    return ss.str();
}
std::string parserdoGroupRequest2String(dobot_v4_bringup::DOGroup::Request& request)
{
    std::stringstream ss;
    ss << "DOGroup(";
    for (int i = 0; i < request.args.size(); ++i) {
        if (i == 0) {
            ss << request.args[i];
        } else
            ss << "," << request.args[i];
    }
    ss << ")";

    return ss.str();
}
std::string parserbrakeControlRequest2String(dobot_v4_bringup::BrakeControl::Request& request)
{
    std::stringstream ss;
    ss << "BrakeControl(" << request.axisID << "," << request.value << ")";

    return ss.str();
}
std::string parserstartDragRequest2String(dobot_v4_bringup::StartDrag::Request& request)
{
    std::stringstream ss;
    ss << "StartDrag()";
    return ss.str();
}
std::string parserStopDragRequest2String(dobot_v4_bringup::StopDrag::Request& request)
{
    std::stringstream ss;
    ss << "StopDrag()";
    return ss.str();
}
std::string parserDragSensivityRequest2String(dobot_v4_bringup::DragSensivity::Request& request)
{
    std::stringstream ss;
    ss << "DragSensivity(" << request.index << "," << request.value << ")";
    return ss.str();
}
std::string parserGetDORequest2String(dobot_v4_bringup::GetDO::Request& request)
{
    std::stringstream ss;
    ss << "GetDO(" << request.index << ")";
    return ss.str();
}
std::string parserGetAORequest2String(dobot_v4_bringup::GetAO::Request& request)
{
    std::stringstream ss;
    ss << " GetAO(" << request.index << ")";
    return ss.str();
}
std::string parserGetDOGroupRequest2String(dobot_v4_bringup::GetDOGroup::Request& request)
{
    std::stringstream ss;
    ss << "GetDOGroup(";
    for (int i = 0; i < request.index_group.size(); ++i) {
        if (i == 0)
            ss << request.index_group[i];
        else
            ss << "," << request.index_group[i];
    }
    ss << ")";
    return ss.str();
}
std::string parserSetTool485Request2String(dobot_v4_bringup::SetTool485::Request& request)
{
    std::stringstream ss;
    ss << "SetTool485(" << request.baudrate << ")";
    return ss.str();
}
std::string parserSetSafeWallEnableRequest2String(dobot_v4_bringup::SetSafeWallEnable::Request& request)
{
    std::stringstream ss;
    ss << "SetSafeWallEnable(" << request.index << "," << request.value << ")";
    return ss.str();
}
std::string parserSetToolPowerRequest2String(dobot_v4_bringup::SetToolPower::Request& request)
{
    std::stringstream ss;
    ss << "SetToolPower(" << request.status << ")";
    return ss.str();
}
std::string parserSetToolModeRequest2String(dobot_v4_bringup::SetToolMode::Request& request)
{
    std::stringstream ss;
    ss << "SetToolMode(" << request.mode << "," << request.type << ")";
    return ss.str();
}
std::string parserSetBackDistanceRequest2String(dobot_v4_bringup::SetBackDistance::Request& request)
{
    std::stringstream ss;
    ss << "SetBackDistance(" << request.distance << ")";
    return ss.str();
}
std::string parserSetPostCollisionModeRequest2String(dobot_v4_bringup::SetPostCollisionMode::Request& request)
{
    std::stringstream ss;
    ss << "SetPostCollisionMode(" << request.mode << ")";
    return ss.str();
}
std::string parserSetUserRequest2String(dobot_v4_bringup::SetUser::Request& request)
{
    std::stringstream ss;
    ss << "SetUser(" << request.index << "," << request.value << ")";
    return ss.str();
}
std::string parserSetToolRequest2String(dobot_v4_bringup::SetTool::Request& request)
{
    std::stringstream ss;
    ss << "SetTool(" << request.index << "," << request.value << ")";
    return ss.str();
}
std::string parserCalcUserRequest2String(dobot_v4_bringup::CalcUser::Request& request)
{
    std::stringstream ss;
    ss << "CalcUser(" << request.index << "," << request.matrix << "," << request.offset << ")";
    return ss.str();
}
std::string parserCalcToolRequest2String(dobot_v4_bringup::CalcTool::Request& request)
{
    std::stringstream ss;
    ss << "CalcTool(" << request.index << "," << request.matrix << "," << request.offset << ")";
    return ss.str();
}
std::string parserGetInputboolRequest2String(dobot_v4_bringup::GetInputBool::Request& request)
{
    std::stringstream ss;
    ss << "GetInputBool(" << request.address << ")";
    return ss.str();
}
std::string parserGetInputIntRequest2String(dobot_v4_bringup::GetInputInt::Request& request)
{
    std::stringstream ss;
    ss << "GetInputInt(" << request.address << ")";
    return ss.str();
}
std::string parserGetInputFloatRequest2String(dobot_v4_bringup::GetInputFloat::Request& request)
{
    std::stringstream ss;
    ss << "GetInputFloat(" << request.address << ")";
    return ss.str();
}
std::string parserGetOutputboolRequest2String(dobot_v4_bringup::GetOutputBool::Request& request)
{
    std::stringstream ss;
    ss << "GetOutputBool(" << request.address << ")";
    return ss.str();
}
std::string parserGetOutputIntRequest2String(dobot_v4_bringup::GetOutputInt::Request& request)
{
    std::stringstream ss;
    ss << "GetOutputInt(" << request.address << ")";
    return ss.str();
}
std::string parserGetOutputFloatRequest2String(dobot_v4_bringup::GetOutputFloat::Request& request)
{
    std::stringstream ss;
    ss << "GetOutputFloat(" << request.address << ")";
    return ss.str();
}
std::string parserSetOutputboolRequest2String(dobot_v4_bringup::SetOutputBool::Request& request)
{
    std::stringstream ss;
    ss << "SetOutputBool(" << request.address << "," << request.value << ")";
    return ss.str();
}
std::string parserSetOutputIntRequest2String(dobot_v4_bringup::SetOutputInt::Request& request)
{
    std::stringstream ss;
    ss << "SetOutputInt(" << request.address << "," << request.value << ")";

    return ss.str();
}
std::string parserSetOutputFloatRequest2String(dobot_v4_bringup::SetOutputFloat::Request& request)
{
    std::stringstream ss;
    ss << "SetOutputFloat(" << request.address << "," << request.value << ")";
    return ss.str();
}
std::string parsermovJRequest2String(dobot_v4_bringup::MovJ::Request& request)
{
    std::string stringOrder = "MovJ(";
    char cmdCoordinate[100];
    sprintf(cmdCoordinate, "{%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f}", request.a, request.b, request.c, request.d,
            request.e, request.f);

    if (request.mode) {
        stringOrder = stringOrder + "joint=" + cmdCoordinate;
    } else {
        stringOrder = stringOrder + "pose=" + cmdCoordinate;
    }
    if (!(request.paramValue.empty() || request.paramValue[0].empty())) {
        for (int i = 0; i < request.paramValue.size(); i++) {
            stringOrder = stringOrder + "," + std::string(request.paramValue[i]);
        }
    }

    stringOrder = stringOrder + ")";
    ROS_INFO("order:%s", stringOrder.c_str());
    return stringOrder;
}
std::string parsermovLRequest2String(dobot_v4_bringup::MovL::Request& request)
{
    std::string stringOrder = "MovL(";
    char cmdCoordinate[100];
    sprintf(cmdCoordinate, "{%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f}", request.a, request.b, request.c, request.d,
            request.e, request.f);

    if (request.mode) {
        stringOrder = stringOrder + "joint=" + cmdCoordinate;
    } else {
        stringOrder = stringOrder + "pose=" + cmdCoordinate;
    }
    if (!(request.paramValue.empty() || request.paramValue[0].empty())) {
        for (int i = 0; i < request.paramValue.size(); i++) {
            stringOrder = stringOrder + "," + std::string(request.paramValue[i]);
        }
    }

    stringOrder = stringOrder + ")";
    ROS_INFO("order:%s", stringOrder.c_str());
    return stringOrder;
}
std::string parserMovLIORequest2String(dobot_v4_bringup::MovLIO::Request& request)
{
    std::string stringOrder = "MovLIO(";
    char cmdCoordinate[100];
    sprintf(cmdCoordinate, "{%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f}", request.a, request.b, request.c, request.d,
            request.e, request.f);

    if (request.mode) {
        stringOrder = stringOrder + "joint=" + cmdCoordinate;
    } else {
        stringOrder = stringOrder + "pose=" + cmdCoordinate;
    }

    if (!(request.MDIS.empty() || request.MDIS[0].empty())) {
        for (int i = 0; i < request.MDIS.size(); i++) {
            stringOrder = stringOrder + "," + std::string(request.MDIS[i]);
        }
    }

    if (!(request.paramValue.empty() || request.paramValue[0].empty())) {
        for (int i = 0; i < request.paramValue.size(); i++) {
            stringOrder = stringOrder + "," + std::string(request.paramValue[i]);
        }
    }

    stringOrder = stringOrder + ")";
    ROS_INFO("order:%s", stringOrder.c_str());
    return stringOrder;
}

std::string parserMovJIORequest2String(dobot_v4_bringup::MovJIO::Request& request)
{
    std::string stringOrder = "MovJIO(";
    char cmdCoordinate[100];
    sprintf(cmdCoordinate, "{%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f}", request.a, request.b, request.c, request.d,
            request.e, request.f);

    if (request.mode) {
        stringOrder = stringOrder + "joint=" + cmdCoordinate;
    } else {
        stringOrder = stringOrder + "pose=" + cmdCoordinate;
    }

    if (!(request.MDIS.empty() || request.MDIS[0].empty())) {
        for (int i = 0; i < request.MDIS.size(); i++) {
            stringOrder = stringOrder + "," + std::string(request.MDIS[i]);
        }
    }

    if (!(request.paramValue.empty() || request.paramValue[0].empty())) {
        for (int i = 0; i < request.paramValue.size(); i++) {
            stringOrder = stringOrder + "," + std::string(request.paramValue[i]);
        }
    }

    stringOrder = stringOrder + ")";
    ROS_INFO("order:%s", stringOrder.c_str());
    return stringOrder;
}

std::string parserArcRequest2String(dobot_v4_bringup::Arc::Request& request)
{
    std::string stringOrder = "Arc(";
    char cmdCoordinate[100];
    char cmdCoordinate2[100];
    sprintf(cmdCoordinate, "{%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f}", request.a, request.b, request.c, request.d,
            request.e, request.f);
    sprintf(cmdCoordinate2, "{%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f}", request.a2, request.b2, request.c2, request.d2,
            request.e2, request.f2);
    if (request.mode) {
        stringOrder = stringOrder + "joint=" + cmdCoordinate + "," + "joint=" + cmdCoordinate2;
    } else {
        stringOrder = stringOrder + "pose=" + cmdCoordinate + "," + "pose=" + cmdCoordinate2;
    }

    if (!(request.paramValue.empty() || request.paramValue[0].empty())) {
        for (int i = 0; i < request.paramValue.size(); i++) {
            stringOrder = stringOrder + "," + std::string(request.paramValue[i]);
        }
    }

    stringOrder = stringOrder + ")";
    ROS_INFO("order:%s", stringOrder.c_str());
    return stringOrder;
}

std::string parsermoveJogRequest2String(dobot_v4_bringup::MoveJog::Request& request)
{
    std::string stringOrder = "MoveJog(";
    char cmdCoordinate[100];
    sprintf(cmdCoordinate, "%s", request.axisID.c_str());
    stringOrder = stringOrder + cmdCoordinate;
    if (!(request.paramValue.empty() || request.paramValue[0].empty())) {
        for (int i = 0; i < request.paramValue.size(); i++) {
            stringOrder = stringOrder + "," + std::string(request.paramValue[i]);
        }
    }

    stringOrder = stringOrder + ")";
    ROS_INFO("order:%s", stringOrder.c_str());
    return stringOrder;
}

std::string parserStopmoveJogRequest2String(dobot_v4_bringup::StopMoveJog::Request& request)
{
    std::string stringOrder = "MoveJog()";
    ROS_INFO("order:%s", stringOrder.c_str());
    return stringOrder;
}

std::string parserRelMovJToolRequest2String(dobot_v4_bringup::RelMovJTool::Request& request)
{
    std::string stringOrder = "RelMovJTool(";
    char cmdCoordinate[100];
    sprintf(cmdCoordinate, "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request.a, request.b, request.c, request.d, request.e,
            request.f);
    stringOrder = stringOrder + cmdCoordinate;
    if (!(request.paramValue.empty() || request.paramValue[0].empty())) {
        for (int i = 0; i < request.paramValue.size(); i++) {
            stringOrder = stringOrder + "," + std::string(request.paramValue[i]);
        }
    }

    stringOrder = stringOrder + ")";
    ROS_INFO("order:%s", stringOrder.c_str());
    return stringOrder;
}
std::string parserRelMovLToolRequest2String(dobot_v4_bringup::RelMovLTool::Request& request)
{
    std::string stringOrder = "RelMovLTool(";
    char cmdCoordinate[100];
    sprintf(cmdCoordinate, "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request.a, request.b, request.c, request.d, request.e,
            request.f);
    stringOrder = stringOrder + cmdCoordinate;
    if (!(request.paramValue.empty() || request.paramValue[0].empty())) {
        for (int i = 0; i < request.paramValue.size(); i++) {
            stringOrder = stringOrder + "," + std::string(request.paramValue[i]);
        }
    }

    stringOrder = stringOrder + ")";
    ROS_INFO("order:%s", stringOrder.c_str());
    return stringOrder;
}
std::string parserRelMovJUserRequest2String(dobot_v4_bringup::RelMovJUser::Request& request)
{
    std::string stringOrder = "RelMovJUser(";
    char cmdCoordinate[100];
    sprintf(cmdCoordinate, "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request.a, request.b, request.c, request.d, request.e,
            request.f);
    stringOrder = stringOrder + cmdCoordinate;
    if (!(request.paramValue.empty() || request.paramValue[0].empty())) {
        for (int i = 0; i < request.paramValue.size(); i++) {
            stringOrder = stringOrder + "," + std::string(request.paramValue[i]);
        }
    }

    stringOrder = stringOrder + ")";
    ROS_INFO("order:%s", stringOrder.c_str());
    return stringOrder;
}

std::string parserRelMovLUserRequest2String(dobot_v4_bringup::RelMovLUser::Request& request)
{
    std::string stringOrder = "RelMovLUser(";
    char cmdCoordinate[100];
    sprintf(cmdCoordinate, "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request.a, request.b, request.c, request.d, request.e,
            request.f);
    stringOrder = stringOrder + cmdCoordinate;
    if (!(request.paramValue.empty() || request.paramValue[0].empty())) {
        for (int i = 0; i < request.paramValue.size(); i++) {
            stringOrder = stringOrder + "," + std::string(request.paramValue[i]);
        }
    }

    stringOrder = stringOrder + ")";
    ROS_INFO("order:%s", stringOrder.c_str());
    return stringOrder;
}

std::string parserrelJointMovJRequest2String(dobot_v4_bringup::RelJointMovJ::Request& request)
{
    std::string stringOrder = "RelJointMovJ(";
    char cmdCoordinate[100];
    sprintf(cmdCoordinate, "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request.a, request.b, request.c, request.d, request.e,
            request.f);
    stringOrder = stringOrder + cmdCoordinate;
    if (!(request.paramValue.empty() || request.paramValue[0].empty())) {
        for (int i = 0; i < request.paramValue.size(); i++) {
            stringOrder = stringOrder + "," + std::string(request.paramValue[i]);
        }
    }

    stringOrder = stringOrder + ")";
    ROS_INFO("order:%s", stringOrder.c_str());
    return stringOrder;
}
std::string parserGetCurrentCommandIdRequest2String(dobot_v4_bringup::GetCurrentCommandId::Request& request)
{
    std::stringstream ss;
    ss << "GetCurrentCommandID()";
    return ss.str();
}
std::string parserServoJRequest2String(dobot_v4_bringup::ServoJ::Request& request)
{
    std::string stringOrder = "ServoJ(";
    char cmdCoordinate[100];
    sprintf(cmdCoordinate, "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request.a, request.b, request.c, request.d, request.e,
            request.f);
    stringOrder = stringOrder + cmdCoordinate;
    if (!(request.paramValue.empty() || request.paramValue[0].empty())) {
        for (int i = 0; i < request.paramValue.size(); i++) {
            stringOrder = stringOrder + "," + std::string(request.paramValue[i]);
        }
    }

    stringOrder = stringOrder + ")";
    ROS_INFO("order:%s", stringOrder.c_str());
    return stringOrder;
}

std::string parserServoPRequest2String(dobot_v4_bringup::ServoP::Request& request)
{
    std::string stringOrder = "ServoP(";
    char cmdCoordinate[100];
    sprintf(cmdCoordinate, "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request.a, request.b, request.c, request.d, request.e,
            request.f);
    stringOrder = stringOrder + cmdCoordinate;
    if (!(request.paramValue.empty() || request.paramValue[0].empty())) {
        for (int i = 0; i < request.paramValue.size(); i++) {
            stringOrder = stringOrder + "," + std::string(request.paramValue[i]);
        }
    }

    stringOrder = stringOrder + ")";
    ROS_INFO("order:%s", stringOrder.c_str());
    return stringOrder;
}

std::string parserCircleRequest2String(dobot_v4_bringup::Circle::Request& request)
{
    std::stringstream ss;
    std::string stringOrder = "Circle(";
    char cmdCoordinate[100];
    char cmdCoordinate2[100];
    sprintf(cmdCoordinate, "{%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f}", request.a, request.b, request.c, request.d,
            request.e, request.f);
    sprintf(cmdCoordinate2, "{%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f}", request.a2, request.b2, request.c2, request.d2,
            request.e2, request.f2);
    if (request.mode) {
        stringOrder = stringOrder + "joint=" + cmdCoordinate + "," + "joint=" + cmdCoordinate2;
    } else {
        stringOrder = stringOrder + "pose=" + cmdCoordinate + "," + "pose=" + cmdCoordinate2;
    }

    stringOrder = stringOrder + "," + std::to_string(request.count);

    if (!(request.paramValue.empty() || request.paramValue[0].empty())) {
        for (int i = 0; i < request.paramValue.size(); i++) {
            stringOrder = stringOrder + "," + std::string(request.paramValue[i]);
        }
    }

    stringOrder = stringOrder + ")";
    ROS_INFO("order:%s", stringOrder.c_str());
    return stringOrder;
}

std::string parserToolAIRequest2String(dobot_v4_bringup::ToolAI::Request& request)
{
    std::stringstream ss;
    ss << "ToolAI(" << request.index << ")";
    return ss.str();
}
std::string parserToolDIRequest2String(dobot_v4_bringup::ToolDI::Request& request)
{
    std::stringstream ss;
    ss << "ToolDI(" << request.index << ")";
    return ss.str();
}

}    // namespace parseTool