#include <string>
#include <dobot_v4_bringup/EnableRobot.h>
#include <dobot_v4_bringup/DisableRobot.h>
#include <dobot_v4_bringup/ClearError.h>
#include <dobot_v4_bringup/SpeedFactor.h>
#include <dobot_v4_bringup/User.h>
#include <dobot_v4_bringup/Tool.h>
#include <dobot_v4_bringup/RobotMode.h>
#include <dobot_v4_bringup/SetPayload.h>
#include <dobot_v4_bringup/DO.h>
#include <dobot_v4_bringup/DOInstant.h>
#include <dobot_v4_bringup/ToolDO.h>
#include <dobot_v4_bringup/ToolDOInstant.h>
#include <dobot_v4_bringup/AO.h>
#include <dobot_v4_bringup/AOInstant.h>
#include <dobot_v4_bringup/AccJ.h>
#include <dobot_v4_bringup/AccL.h>
#include <dobot_v4_bringup/VelJ.h>
#include <dobot_v4_bringup/VelL.h>
#include <dobot_v4_bringup/CP.h>
#include <dobot_v4_bringup/PowerOn.h>
#include <dobot_v4_bringup/RunScript.h>
#include <dobot_v4_bringup/Stop.h>
#include <dobot_v4_bringup/Pause.h>
#include <dobot_v4_bringup/Continue.h>
#include <dobot_v4_bringup/SetCollisionLevel.h>
#include <dobot_v4_bringup/EnableSafeSkin.h>
#include <dobot_v4_bringup/SetSafeSkin.h>
#include <dobot_v4_bringup/StartPath.h>
#include <dobot_v4_bringup/GetStartPose.h>
#include <dobot_v4_bringup/PositiveKin.h>
#include <dobot_v4_bringup/GetPose.h>
#include <dobot_v4_bringup/InverseKin.h>
#include <dobot_v4_bringup/GetAngle.h>
#include <dobot_v4_bringup/EmergencyStop.h>
#include <dobot_v4_bringup/ModbusRTUCreate.h>
#include <dobot_v4_bringup/ModbusCreate.h>
#include <dobot_v4_bringup/ModbusClose.h>
#include <dobot_v4_bringup/GetInBits.h>
#include <dobot_v4_bringup/GetInRegs.h>
#include <dobot_v4_bringup/GetCoils.h>
#include <dobot_v4_bringup/SetCoils.h>
#include <dobot_v4_bringup/GetHoldRegs.h>
#include <dobot_v4_bringup/SetHoldRegs.h>
#include <dobot_v4_bringup/GetErrorID.h>
#include <dobot_v4_bringup/DI.h>
#include <dobot_v4_bringup/ToolDI.h>
#include <dobot_v4_bringup/AI.h>
#include <dobot_v4_bringup/ToolAI.h>
#include <dobot_v4_bringup/DIGroup.h>
#include <dobot_v4_bringup/DOGroup.h>
#include <dobot_v4_bringup/BrakeControl.h>
#include <dobot_v4_bringup/StartDrag.h>
#include <dobot_v4_bringup/StopDrag.h>
#include <dobot_v4_bringup/DragSensivity.h>
#include <dobot_v4_bringup/GetDO.h>
#include <dobot_v4_bringup/GetAO.h>
#include <dobot_v4_bringup/GetDOGroup.h>
#include <dobot_v4_bringup/SetTool485.h>
#include <dobot_v4_bringup/SetSafeWallEnable.h>
#include <dobot_v4_bringup/SetToolPower.h>
#include <dobot_v4_bringup/SetToolMode.h>
#include <dobot_v4_bringup/SetBackDistance.h>
#include <dobot_v4_bringup/SetPostCollisionMode.h>
#include <dobot_v4_bringup/SetUser.h>
#include <dobot_v4_bringup/SetTool.h>
#include <dobot_v4_bringup/CalcUser.h>
#include <dobot_v4_bringup/CalcTool.h>
#include <dobot_v4_bringup/GetInputBool.h>
#include <dobot_v4_bringup/GetInputInt.h>
#include <dobot_v4_bringup/GetInputFloat.h>
#include <dobot_v4_bringup/GetOutputBool.h>
#include <dobot_v4_bringup/GetOutputInt.h>
#include <dobot_v4_bringup/GetOutputFloat.h>
#include <dobot_v4_bringup/SetOutputBool.h>
#include <dobot_v4_bringup/SetOutputInt.h>
#include <dobot_v4_bringup/SetOutputFloat.h>
#include <dobot_v4_bringup/MovLIO.h>
#include <dobot_v4_bringup/MovJIO.h>
#include <dobot_v4_bringup/Arc.h>
#include <dobot_v4_bringup/Circle.h>
#include <dobot_v4_bringup/MoveJog.h>
#include <dobot_v4_bringup/StopMoveJog.h>
#include <dobot_v4_bringup/RelMovJTool.h>
#include <dobot_v4_bringup/RelMovLTool.h>
#include <dobot_v4_bringup/RelMovJUser.h>
#include <dobot_v4_bringup/RelMovLUser.h>
#include <dobot_v4_bringup/MovJ.h>
#include <dobot_v4_bringup/MovL.h>
#include <dobot_v4_bringup/RelJointMovJ.h>
#include <dobot_v4_bringup/RobotStatus.h>
#include <dobot_v4_bringup/GetCurrentCommandId.h>
#include <dobot_v4_bringup/ServoJ.h>
#include <dobot_v4_bringup/ServoP.h>


namespace parseTool{

std::string parserenableRobotRequest2String(dobot_v4_bringup::EnableRobot::Request &request);
std::string parserdisableRobotRequest2String(dobot_v4_bringup::DisableRobot::Request &request);
std::string parserclearErrorRequest2String(dobot_v4_bringup::ClearError::Request &request);
std::string parserspeedFactorRequest2String(dobot_v4_bringup::SpeedFactor::Request &request);
std::string parseruserRequest2String(dobot_v4_bringup::User::Request &request);
std::string parsertoolRequest2String(dobot_v4_bringup::Tool::Request &request);
std::string parserrobotModeRequest2String(dobot_v4_bringup::RobotMode::Request &request);
std::string parsersetPayloadRequest2String(dobot_v4_bringup::SetPayload::Request &request);
std::string parserDORequest2String(dobot_v4_bringup::DO::Request &request);
std::string parserDOInstantRequest2String(dobot_v4_bringup::DOInstant::Request &request);
std::string parsertoolDORequest2String(dobot_v4_bringup::ToolDO::Request &request);
std::string parsertoolDOInstantRequest2String(dobot_v4_bringup::ToolDOInstant::Request &request);
std::string parserAORequest2String(dobot_v4_bringup::AO::Request &request);
std::string parserAOInstantRequest2String(dobot_v4_bringup::AOInstant::Request &request);
std::string parseraccJRequest2String(dobot_v4_bringup::AccJ::Request &request);
std::string parseraccLRequest2String(dobot_v4_bringup::AccL::Request &request);
std::string parservelJRequest2String(dobot_v4_bringup::VelJ::Request &request);
std::string parservelLRequest2String(dobot_v4_bringup::VelL::Request &request);
std::string parsercpRequest2String(dobot_v4_bringup::CP::Request &request);
std::string parserpowerOnRequest2String(dobot_v4_bringup::PowerOn::Request &request);
std::string parserrunScriptRequest2String(dobot_v4_bringup::RunScript::Request &request);
std::string parserstopRequest2String(dobot_v4_bringup::Stop::Request &request);
std::string parserpauseRequest2String(dobot_v4_bringup::Pause::Request &request);
std::string parserContinueRequest2String(dobot_v4_bringup::Continue::Request &request);
std::string parserEnableSafeSkinRequest2String(dobot_v4_bringup::EnableSafeSkin::Request &request);
std::string parserSetSafeSkinRequest2String(dobot_v4_bringup::SetSafeSkin::Request &request);
std::string parserGetStartPoseRequest2String(dobot_v4_bringup::GetStartPose::Request &request);
std::string parserStartPathRequest2String(dobot_v4_bringup::StartPath::Request &request);
std::string parserPositiveKinRequest2String(dobot_v4_bringup::PositiveKin::Request &request);
std::string parserInverseKinRequest2String(dobot_v4_bringup::InverseKin::Request &request);
std::string parserGetAngleRequest2String(dobot_v4_bringup::GetAngle::Request &request);
std::string parserGetPoseRequest2String(dobot_v4_bringup::GetPose::Request &request);
std::string parserEmergencyStopRequest2String(dobot_v4_bringup::EmergencyStop::Request &request);
std::string parsersetCollisionLevelRequest2String(dobot_v4_bringup::SetCollisionLevel::Request &request);
std::string parserModbusRTUCreateRequest2String(dobot_v4_bringup::ModbusRTUCreate::Request &request);
std::string parserModbusCreateRequest2String(dobot_v4_bringup::ModbusCreate::Request &reques);
std::string parserModbusCloseRequest2String(dobot_v4_bringup::ModbusClose::Request &request);
std::string parserGetInBitsRequest2String(dobot_v4_bringup::GetInBits::Request &request);
std::string parserGetInRegsRequest2String(dobot_v4_bringup::GetInRegs::Request &request);
std::string parserGetCoilsRequest2String(dobot_v4_bringup::GetCoils::Request &request);
std::string parserSetCoilsRequest2String(dobot_v4_bringup::SetCoils::Request &request);
std::string parserGetHoldRegsRequest2String(dobot_v4_bringup::GetHoldRegs::Request &request);
std::string parserSetHoldRegsRequest2String(dobot_v4_bringup::SetHoldRegs::Request &request);
std::string parserGetErrorIDRequest2String(dobot_v4_bringup::GetErrorID::Request &request);
std::string parserDIRequest2String(dobot_v4_bringup::DI::Request &request);
std::string parserAIRequest2String(dobot_v4_bringup::AI::Request &request);
std::string parserDIGroupRequest2String(dobot_v4_bringup::DIGroup::Request &request);
std::string parserdoGroupRequest2String(dobot_v4_bringup::DOGroup::Request &request);
std::string parserbrakeControlRequest2String(dobot_v4_bringup::BrakeControl::Request &request);
std::string parserstartDragRequest2String(dobot_v4_bringup::StartDrag::Request &request);
std::string parserStopDragRequest2String(dobot_v4_bringup::StopDrag::Request &request);
std::string parserDragSensivityRequest2String(dobot_v4_bringup::DragSensivity::Request &request);
std::string parserGetDORequest2String(dobot_v4_bringup::GetDO::Request &request);
std::string parserGetAORequest2String(dobot_v4_bringup::GetAO::Request &request);
std::string parserGetDOGroupRequest2String(dobot_v4_bringup::GetDOGroup::Request &request);
std::string parserSetTool485Request2String(dobot_v4_bringup::SetTool485::Request &request);
std::string parserSetSafeWallEnableRequest2String(dobot_v4_bringup::SetSafeWallEnable::Request &request);
std::string parserSetToolPowerRequest2String(dobot_v4_bringup::SetToolPower::Request &request);
std::string parserSetToolModeRequest2String(dobot_v4_bringup::SetToolMode::Request &request);
std::string parserSetBackDistanceRequest2String(dobot_v4_bringup::SetBackDistance::Request &request);
std::string parserSetPostCollisionModeRequest2String(dobot_v4_bringup::SetPostCollisionMode::Request &request);
std::string parserSetUserRequest2String(dobot_v4_bringup::SetUser::Request &request);
std::string parserSetToolRequest2String(dobot_v4_bringup::SetTool::Request &request);
std::string parserCalcUserRequest2String(dobot_v4_bringup::CalcUser::Request &request);
std::string parserCalcToolRequest2String(dobot_v4_bringup::CalcTool::Request &request);
std::string parserGetInputboolRequest2String(dobot_v4_bringup::GetInputBool::Request &request);
std::string parserGetInputIntRequest2String(dobot_v4_bringup::GetInputInt::Request &request);
std::string parserGetInputFloatRequest2String(dobot_v4_bringup::GetInputFloat::Request &request);
std::string parserGetOutputboolRequest2String(dobot_v4_bringup::GetOutputBool::Request &request);
std::string parserGetOutputIntRequest2String(dobot_v4_bringup::GetOutputInt::Request &request);
std::string parserGetOutputFloatRequest2String(dobot_v4_bringup::GetOutputFloat::Request &request);
std::string parserSetOutputboolRequest2String(dobot_v4_bringup::SetOutputBool::Request &request);
std::string parserSetOutputIntRequest2String(dobot_v4_bringup::SetOutputInt::Request &request);
std::string parserSetOutputFloatRequest2String(dobot_v4_bringup::SetOutputFloat::Request &request);
std::string parsermovJRequest2String(dobot_v4_bringup::MovJ::Request &request);
std::string parsermovLRequest2String(dobot_v4_bringup::MovL::Request &request);
std::string parserMovLIORequest2String(dobot_v4_bringup::MovLIO::Request &request);
std::string parserMovJIORequest2String(dobot_v4_bringup::MovJIO::Request &request);
std::string parserArcRequest2String(dobot_v4_bringup::Arc::Request &request);
std::string parsermoveJogRequest2String(dobot_v4_bringup::MoveJog::Request &request);
std::string parserStopmoveJogRequest2String(dobot_v4_bringup::StopMoveJog::Request& request);
std::string parserRelMovJToolRequest2String(dobot_v4_bringup::RelMovJTool::Request &request);
std::string parserRelMovLToolRequest2String(dobot_v4_bringup::RelMovLTool::Request &request);
std::string parserRelMovJUserRequest2String(dobot_v4_bringup::RelMovJUser::Request &request);
std::string parserRelMovLUserRequest2String(dobot_v4_bringup::RelMovLUser::Request &request);
std::string parserrelJointMovJRequest2String(dobot_v4_bringup::RelJointMovJ::Request &request);
std::string parserGetCurrentCommandIdRequest2String(dobot_v4_bringup::GetCurrentCommandId::Request &request);
std::string parserServoJRequest2String(dobot_v4_bringup::ServoJ::Request &request);
std::string parserServoPRequest2String(dobot_v4_bringup::ServoP::Request &request);
std::string parserCircleRequest2String(dobot_v4_bringup::Circle::Request &request);
std::string parserToolAIRequest2String(dobot_v4_bringup::ToolAI::Request &request);
std::string parserToolDIRequest2String(dobot_v4_bringup::ToolDI::Request &request);

}