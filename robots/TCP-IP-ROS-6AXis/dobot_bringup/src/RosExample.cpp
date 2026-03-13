#include <ros/ros.h>
#include <ros/param.h>
#include <bringup/robot.h>

/******************************
#if PARAMS  条件编译 指令是否有参数
    0  指令不含参数
    1   指令含参数

包括以下指令的例子：
    EnableRobot
    DisableRobot
    DO
    AccJ
    SetArmOrientation
    RunScript
    GetTraceStartPose
    PositiveSolution
    InverseSolution
    GetPose
    ModbusCreate
    GetHoldRegs
    DOGroup
    SetCollideDrag
    SetTerminal485
    MovL
    MovLIO
    MoveJog
    StartTrace
    RelMovJUser
    Circle3
    Arch
    servoJ
******************************/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RosExample");
    ros::NodeHandle nh;

    /******************************
     *******************************
     ******************************
     * 指令：EnableRobot
     * 功能：使能机器人
     ******************************/
    ros::ServiceClient client = nh.serviceClient<dobot_bringup::EnableRobot>("enableRobot");    // 创建服务
    dobot_bringup::EnableRobot srv;    // 消息类型enableRobot.srv

#if PARAMS
    // 无参数
#else

    // 1个参数
    double load = 0.1;
    srv.request.args.push_back(load);    // 添加服务参数信息

    // 4个参数
    double centerX, centerY, centerZ;
    srv.request.args.push_back(load);    // 添加服务参数信息
    srv.request.args.push_back(centerX);
    srv.request.args.push_back(centerY);
    srv.request.args.push_back(centerZ);
#endif

    client.call(srv);    // 发布服务
    ROS_INFO("Result: %d", srv.response.res);

    /******************************
     * 指令：DisableRobot
     * 功能：下使能机器人
     ******************************/

    ros::ServiceClient client = nh.serviceClient<dobot_bringup::DisableRobot>("DisableRobot");    // 创建服务
    dobot_bringup::DisableRobot srv;    // 消息类型disableRobot.srv
    client.call(srv);                   // 发布服务
    ROS_INFO("Result: %d", srv.response.res);

    /******************************
     * 指令： DO
     * 功能：设置数字输出端口状态（队列指令）
     ******************************/

    ros::ServiceClient client = nh.serviceClient<dobot_bringup::DO>("DO");
    dobot_bringup::DO srv;
    int index, status;
    srv.request.index = index;
    srv.request.status = status;
    client.call(srv);    // 发布服务
    ROS_INFO("Result: %d", srv.response.res);

    /******************************
     *******************************
     *******************************
     * 指令： AccJ
     * 功能：设置关节加速度比例。该指令仅对MovJ、MovJIO、MovJR、 JointMovJ指令有效
     ******************************/

    ros::ServiceClient client = nh.serviceClient<dobot_bringup::AccJ>("AccJ");
    dobot_bringup::AccJ srv;
    int R1;
    srv.request.r = R1;
    client.call(srv);

    ROS_INFO("Result: %d", srv.response.res);

    /******************************
     ******************************
     ******************************
     * 指令： SetArmOrientation
     * 功能：设置手系指令。
     ******************************/

    ros::ServiceClient client = nh.serviceClient<dobot_bringup::SetArmOrientation>("SetArmOrientation");
    dobot_bringup::SetArmOrientation srv;
    int LorR, UorD, ForN, Config;
    srv.request.LorR = LorR;    // srv.request.LorR  为SetArmOrientation.srv文件里参数
    srv.request.UorD = UorD;
    srv.request.ForN = ForN;
    srv.request.Config = Config;
    client.call(srv);
    ROS_INFO("Result: %d", srv.response.res);

    /******************************
     * 指令： RunScript
     * 功能：运行lua脚本。
     ******************************/
    ros::ServiceClient client = nh.serviceClient<dobot_bringup::RunScript>("RunScript");
    dobot_bringup::RunScript srv;
    std::string name;
    srv.request.projectName = name;    // srv.request.projectName  为RunScript.srv文件里参数
    client.call(srv);
    ROS_INFO("Result: %d", srv.response.res);

    /******************************
     * 指令： GetTraceStartPose
     * 功能：获取轨迹拟合中首个点位。
     ******************************/
    ros::ServiceClient client = nh.serviceClient<dobot_bringup::GetTraceStartPose>("GetTraceStartPose");
    dobot_bringup::GetTraceStartPose srv;
    std::string traceName;
    srv.request.traceName = traceName;    // srv.request.projectName  GetTraceStartPose.srv文件里参数
    client.call(srv);
    ROS_INFO("Result: %d", srv.response.res);

    /******************************
     * 指令： PositiveSolution
     * 功能：正解。（给定机器人各关节的角度，计算出机器人末端的空间位置）
     ******************************/
    ros::ServiceClient client = nh.serviceClient<dobot_bringup::PositiveSolution>("PositiveSolution");
    dobot_bringup::PositiveSolution srv;
    double J1, J2, J3, J4;
    int User, Tool;
    srv.request.offset1 = J1;    // srv.request.offset1  PositiveSolution.srv文件里参数
    srv.request.offset2 = J2;
    srv.request.offset3 = J3;
    srv.request.offset4 = J4;
    srv.request.user = User;
    srv.request.tool = Tool;
    client.call(srv);
    ROS_INFO("Result: %d", srv.response.res);

    /******************************
     * 指令： InverseSolution
     * 功能：逆解。（已知机器人末端的位置和姿态，计算机器人各关节的角度值）
     ******************************/
    ros::ServiceClient client = nh.serviceClient<dobot_bringup::InverseSolution>("InverseSolution");
    dobot_bringup::InverseSolution srv;
#ifdef
    double J1, J2, J3, J4;
    int User, Tool;
    srv.request.offset1 = J1;    // srv.request.offset1  InverseSolution.srv文件里参数
    srv.request.offset2 = J2;
    srv.request.offset3 = J3;
    srv.request.offset4 = J4;
    srv.request.user = User;
    srv.request.tool = Tool;
#else
    double J1, J2, J3, J4;
    int User, Tool;
    int isJointNear;
    std::string JointNear;
    srv.request.offset1 = J1;    // srv.request.offset1  InverseSolution.srv文件里参数
    srv.request.offset2 = J2;
    srv.request.offset3 = J3;
    srv.request.offset4 = J4;
    srv.request.user = User;
    srv.request.tool = Tool;
    srv.request.isJointNear = isJointNear;
    srv.request.JointNear = JointNear;
#endif
    client.call(srv);
    ROS_INFO("Result: %d", srv.response.res);

    /******************************
     * 指令： GetPose
     * 功能：获取笛卡尔坐标系下机械臂的实时位姿
     ******************************/
    ros::ServiceClient client = nh.serviceClient<dobot_bringup::GetPose>("GetPose");
    dobot_bringup::GetPose srv;
#ifdef
    // 无参       GetPose.srv文件里参数
#else
    int user, tool;
    srv.request.user.push_back(user);    // srv.request.user为数组  GetPose.srv文件里参数
    srv.request.tool.push_back(tool);    // srv.request.tool为数组  GetPose.srv文件里参数
#endif
    client.call(srv);
    ROS_INFO("Result: %d", srv.response.res);

    /******************************
     * 指令： ModbusCreate
     * 功能：创建modbus主站
     ******************************/
    ros::ServiceClient client = nh.serviceClient<dobot_bringup::ModbusCreate>("ModbusCreate");
    dobot_bringup::ModbusCreate srv;
#ifdef
    std::string ip;
    int port, slave_id, isRTU;
    srv.request.ip = ip;    // srv.request.ip  ModbusCreate.srv文件里参数
    srv.request.port = port;
    srv.request.slave_id = slave_id;
#else
    std::string ip;
    int port, slave_id, isRTU;
    srv.request.ip = ip;    //
    srv.request.port = port;
    srv.request.slave_id = slave_id;
    srv.request.isRTU.push_back(isRTU);    //  srv.request.isRTU   为int-数组   ModbusCreate.srv文件里参数
#endif
    client.call(srv);
    ROS_INFO("Result: %d", srv.response.res);

    /******************************
     * 指令： GetHoldRegs
     * 功能：读保持寄存器。
     ******************************/
    ros::ServiceClient client = nh.serviceClient<dobot_bringup::GetHoldRegs>("GetHoldRegs");
    dobot_bringup::GetHoldRegs srv;
#ifdef
    int index, addr, count;
    std::string valType;
    srv.request.index = index;    //
    srv.request.addr = addr;
    srv.request.count = count;
#else
    int index, addr, count;
    std::string valType;
    srv.request.index = index;    //
    srv.request.addr = addr;
    srv.request.count = count;
    srv.request.valType.push_back(valType);    //  srv.request.isRTU   为String-数组   GetHoldRegs.srv文件里参数
#endif
    client.call(srv);
    ROS_INFO("Result: %d", srv.response.res);

    /******************************
     * 指令： DOGroup
     * 功能：设置输出组端口状态  (最大支持64个参数)
     ******************************/
    ros::ServiceClient client = nh.serviceClient<dobot_bringup::DOGroup>("DOGroup");
    dobot_bringup::DOGroup srv;
#ifdef
    int index1, value1, index2, value2, index32, value32;
    srv.request.args.push_back(index1);    //  srv.request.args   为int-数组   DOGroup.srv文件里参数
    srv.request.args.push_back(value1);
#else
    int index1, value1, index2, value2, index32, value32;
    srv.request.args.push_back(index1);    //  srv.request.args   为int-数组   DOGroup.srv文件里参数
    srv.request.args.push_back(value1);
    //...
    srv.request.args.push_back(index32);
    srv.request.args.push_back(value32);
#endif
    client.call(srv);
    ROS_INFO("Result: %d", srv.response.res);

    /******************************
     * 指令： SetCollideDrag
     * 功能：设置是否强制进入拖拽（报错状态下也能进入拖拽）
     ******************************/

    ros::ServiceClient client = nh.serviceClient<dobot_bringup::SetCollideDrag>("SetCollideDrag");
    dobot_bringup::SetCollideDrag srv;
    int status;
    srv.request.status = status;    //  srv.request.args   为int-数组   DOGroup.srv文件里参数
    client.call(srv);
    ROS_INFO("Result: %d", srv.response.res);

    /******************************
     * 指令： MovL
     * 功能：功能：点到点运动，目标点位为笛卡尔点位
     ******************************/
    ros::ServiceClient client = nh.serviceClient<dobot_bringup::MovL>("MovL");
    dobot_bringup::MovL srv;
#ifdef
    double x, y, z, a, b, c;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z;
    srv.request.a = a;
    srv.request.b = b;
    srv.request.c = c;
#else
    double x, y, z, a, b, c;
    std::string userparam{ "User=1" };
    std::string toolparam{ "Tool=1" };
    std::string speedlparam{ "SpeedL=1" };
    std::string acclparam{ "AccL=1" };
    std::string cpparam{ "CP=1" };
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z;
    srv.request.a = a;
    srv.request.b = b;
    srv.request.c = c;
    srv.request.paramValue.push_back(userparam);    //  srv.request.paramValue 为string-数组  MovL.srv文件里参数
                                                    //  1个可选参数
    srv.request.paramValue.push_back(toolparam);    // 2个可选参数     参数数量和顺序可灵活使用
    srv.request.paramValue.push_back(speedlparam);    // 3个可选参数  参数数量和顺序可灵活使用
    srv.request.paramValue.push_back(acclparam);      // 4个可选参数   参数数量和顺序可灵活使用
    srv.request.paramValue.push_back(cpparam);        // 5个可选参数   参数数量和顺序可灵活使用

#endif
    client.call(srv);
    ROS_INFO("Result: %d", srv.response.res);

    /******************************
     * 指令： Arc
     * 功能：：从当前位置以圆弧插补方式移动至笛卡尔坐标系下的目标位置。
    ​		该指令需结合其他运动指令确定圆弧起始点。
     ******************************/
    ros::ServiceClient client = nh.serviceClient<dobot_bringup::Arc>("Arc");
    dobot_bringup::Arc srv;
#ifdef
    double x1, y1, z1, a1, b1, c1;
    double x2, y2, z2, a2, b2, c2;
    srv.request.x1 = x1;
    srv.request.y1 = y1;
    srv.request.z1 = z1;
    srv.request.a1 = a1;
    srv.request.b1 = b1;
    srv.request.c1 = c1;
    srv.request.x2 = x2;
    srv.request.y2 = y2;
    srv.request.z2 = z2;
    srv.request.a2 = a2;
    srv.request.b2 = b2;
    srv.request.c2 = c2;
#else
    double x1, y1, z1, a1, b1, c1;
    double x2, y2, z2, a2, b2, c2;
    std::string userparam{ "User=1" };
    std::string toolparam{ "Tool=1" };
    std::string speedlparam{ "SpeedL=1" };
    std::string acclparam{ "AccL=1" };
    srv.request.x1 = x1;
    srv.request.y1 = y1;
    srv.request.z1 = z1;
    srv.request.a1 = a1;
    srv.request.b1 = b1;
    srv.request.c1 = c1;
    srv.request.x2 = x2;
    srv.request.y2 = y2;
    srv.request.z2 = z2;
    srv.request.a2 = a2;
    srv.request.b2 = b2;
    srv.request.c2 = c2;
    srv.request.paramValue.push_back(userparam);    //  srv.request.paramValue 为string-数组  Arc.srv文件里参数
                                                    //  1个可选参数
    srv.request.paramValue.push_back(toolparam);    // 2个可选参数     参数数量和顺序可灵活使用
    srv.request.paramValue.push_back(speedlparam);    // 3个可选参数  参数数量和顺序可灵活使用
    srv.request.paramValue.push_back(acclparam);      // 4个可选参数   参数数量和顺序可灵活使用

#endif
    client.call(srv);
    ROS_INFO("Result: %d", srv.response.res);

    /******************************
     * 指令： MovLIO
     * 功能：在直线运动时并行设置数字输出端口状态，目标点位为笛卡尔点位。
     ******************************/
    ros::ServiceClient client = nh.serviceClient<dobot_bringup::MovLIO>("MovLIO");
    dobot_bringup::MovLIO srv;
#ifdef
    double x, y, z, a, b, c;
    int Mode, Distance, Index, Status;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z;
    srv.request.a = a;
    srv.request.b = b;
    srv.request.c = c;
    srv.request.Mode = Mode;
    srv.request.Distance = Distance;
    srv.request.Index = Index;
    srv.request.Status = Status;
#else
    double x, y, z, a, b, c;
    int Mode, Distance, Index, Status;
    std::string userparam{ "User=1" };
    std::string toolparam{ "Tool=1" };
    std::string speedlparam{ "SpeedL=1" };
    std::string acclparam{ "AccL=1" };
    std::string cpparam{ "Cp=1" };
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z;
    srv.request.a = a;
    srv.request.b = b;
    srv.request.c = c;
    srv.request.Mode = Mode;
    srv.request.Distance = Distance;
    srv.request.Index = Index;
    srv.request.Status = Status;
    srv.request.paramValue.push_back(userparam);    //  srv.request.paramValue 为string-数组  MovLIO.srv文件里参数
                                                    //  1个可选参数
    srv.request.paramValue.push_back(toolparam);      // 2个可选参数     参数数量和顺序可灵活使用
    srv.request.paramValue.push_back(speedlparam);    // 3个可选参数  参数数量和顺序可灵活使用
    srv.request.paramValue.push_back(acclparam);      // 4个可选参数   参数数量和顺序可灵活使用
    srv.request.paramValue.push_back(cpparam);        // 5个可选参数   参数数量和顺序可灵活使用

#endif
    client.call(srv);
    ROS_INFO("Result: %d", srv.response.res);

    /******************************
     * 指令： MoveJog
     * 功能：点动运动，不固定距离运动
     ******************************/
    ros::ServiceClient client = nh.serviceClient<dobot_bringup::MoveJog>("MoveJog");
    dobot_bringup::MoveJog srv;
#ifdef
    std::string axisID;
    srv.request.axisID = axisID;
#else
    std::string axisID, CoordType, userparam, toolparam;
    srv.request.axisID = axisID;
    srv.request.paramValue.push_back(CoordType);    //  1个可选参数
    srv.request.paramValue.push_back(userparam);    // 2个可选参数     参数数量和顺序可灵活使用
    srv.request.paramValue.push_back(toolparam);    // 3个可选参数  参数数量和顺序可灵活使用
#endif
    client.call(srv);
    ROS_INFO("Result: %d", srv.response.res);

    /******************************
     * 指令： Circle
     * 功能：整圆运动，仅对笛卡尔点位生效。
     ******************************/
    ros::ServiceClient client = nh.serviceClient<dobot_bringup::Circle>("Circle");
    dobot_bringup::Circle srv;
#ifdef
    double x1, y1, z1, a1, b1, c1;
    double x2, y2, z2, a2, b2, c2;
    int count;
    srv.request.count = count;
    srv.request.x1 = x1;
    srv.request.y1 = y1;
    srv.request.z1 = z1;
    srv.request.a1 = a1;
    srv.request.b1 = b1;
    srv.request.c1 = c1;
    srv.request.x2 = x2;
    srv.request.y2 = y2;
    srv.request.z2 = z2;
    srv.request.a2 = a2;
    srv.request.b2 = b2;
    srv.request.c2 = c2;
#else
    double x1, y1, z1, a1, b1, c1;
    double x2, y2, z2, a2, b2, c2;
    int count;
    std::string userparam{ "User=1" };
    std::string toolparam{ "Tool=1" };
    std::string speedlparam{ "SpeedL=1" };
    std::string acclparam{ "AccL=1" };
    srv.request.count = count;
    srv.request.x1 = x1;
    srv.request.y1 = y1;
    srv.request.z1 = z1;
    srv.request.a1 = a1;
    srv.request.b1 = b1;
    srv.request.c1 = c1;
    srv.request.x2 = x2;
    srv.request.y2 = y2;
    srv.request.z2 = z2;
    srv.request.a2 = a2;
    srv.request.b2 = b2;
    srv.request.c2 = c2;
    srv.request.paramValue.push_back(userparam);    //  srv.request.paramValue 为string-数组  Circle.srv文件里参数
                                                    //  1个可选参数
    srv.request.paramValue.push_back(toolparam);      // 2个可选参数     参数数量和顺序可灵活使用
    srv.request.paramValue.push_back(speedlparam);    // 3个可选参数  参数数量和顺序可灵活使用
    srv.request.paramValue.push_back(acclparam);      // 4个可选参数   参数数量和顺序可灵活使用

#endif
    client.call(srv);
    ROS_INFO("Result: %d", srv.response.res);

    ros::spin();

    return 0;
}
