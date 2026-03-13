/**
 ***********************************************************************************************************************
 *
 * @author YangXiBo
 * @date   2023/08/18
 *
 *
 ***********************************************************************************************************************
 */

#pragma once

#include <ros/ros.h>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <algorithm>
#include <regex>
#include <assert.h>
#include <cstring>
#include <dobot_v4_bringup/tcp_socket.h>

#pragma pack(push, 1)
// 数据 按照 8 字节 以及  48 字节对齐的模式,
// 大小设计为  30 * 8 * 6 = 30 *6*sizeof(double) = 30 * sizeof(double)
typedef struct RealTimeData_t
{
    //
    uint16_t len;                   // 0000 ~ 0001  字符长度
    uint16_t Reserve[3];            // 0002 ~ 0007  占位符
    uint64_t digital_input_bits;    // 0008 ~ 0015  DI 按照bit 进行计算的
    uint64_t digital_outputs;       // 0016 ~ 0023  DO 按照bit 进行计算的
    uint64_t robot_mode;            // 0024 ~ 0031  机器人模式
    uint64_t controller_timer;      // 0032 ~ 0039  机器人时间 1970年到现在的时间  单位是 ms
    uint64_t run_time;              // 0040 ~ 0047  机器人开机时间 单位是ms
    // 0048 ~ 0095                       //
    uint64_t test_value;            // 0048 ~ 0055  内存结构测试标准值  0x0123 4567 89AB CDEF
    double safety_mode;             // 0056 ~ 0063  (弃用字段)
    double speed_scaling;           // 0064 ~ 0071  全局速率
    double linear_momentum_norm;    // 0072 ~ 0079  机器人当前动量(未实现)
    double v_main;                  // 0080 ~ 0087  控制板电压值(未实现)
    double v_robot;                 // 0088 ~ 0095  机器人电压(48V)
    // 0096 ~ 0143                       //
    double i_robot;                         // 0096 ~ 0103 机器人电流
    double program_state;                   // 0104 ~ 0111 脚本运行状态
    double safety_status;                   // 0112 ~ 0119 安全状态（未实现）
    double tool_accelerometer_values[3];    // 0120 ~ 0143 tcp加速度（未实现）
    // 0144 ~ 0191                       //
    double elbow_position[3];    // 0144 ~ 0167 肘位置（未实现）
    double elbow_velocity[3];    // 0168 ~ 0191 肘速度（未实现）
    // 0192 ~ ...                        //
    double q_target[6];              // 0192 ~ 0239  // 目标关节位置
    double qd_target[6];             // 0240 ~ 0287  // 目标关节速度
    double qdd_target[6];            // 0288 ~ 0335  // 目标关节加速度
    double i_target[6];              // 0336 ~ 0383  // 目标关节电流
    double m_target[6];              // 0384 ~ 0431  // 目标关节扭矩
    double q_actual[6];              // 0432 ~ 0479  // 实际关节位置
    double qd_actual[6];             // 0480 ~ 0527  // 实际关节速度
    double i_actual[6];              // 0528 ~ 0575  // 实际电流
    double i_control[6];             // 0576 ~ 0623  // TCP传感器力值（未实现）
    double tool_vector_actual[6];    // 0624 ~ 0671  // TCP实际坐标 (TCP: 末端工具中心点 terminal central point)
    double TCP_speed_actual[6];      // 0672 ~ 0719  // TCP速度
    double TCP_force[6];             // 0720 ~ 0767  // TCP力值  (电流环计算)
    double tool_vector_target[6];    // 0768 ~ 0815  // TCP目标坐标
    double TCP_speed_target[6];      // 0816 ~ 0863  // TCP目标速度
    double motor_temperatures[6];    // 0864 ~ 0911  // 关节温度
    double joint_modes[6];           // 0912 ~ 0959  // 关节控制模式
    double v_actual[6];              // 960  ~ 1007  // 关节电压
    int8_t handtype[4];              // 1008,1009,1010,1011 R、D、N、cfg   手系信息  新版已经删除????
    int8_t userCoordinate;           // 1012 用户坐标系ID
    int8_t toolCoordinate;           // 1013 工具坐标系ID
    int8_t isRunQueuedCmd;           // 1014 算法队列运行标志
    int8_t isPauseCmdFlag;           // 1015 算法队列暂停标志
    int8_t velocityRatio;            // 1016 关节速度比例
    int8_t accelerationRatio;        // 1017 关节加速度比例
    int8_t jerkRatio;                // 1018 关节加加速度比例（未实现）
    int8_t xyzVelocityRatio;         // 1019 笛卡尔位置速度比例 (x,y,z  单位是 距离每秒)
    int8_t rVelocityRatio;           // 1020 笛卡尔姿态速度比例 (rx,ry,rz 单位是 角度每秒)
    int8_t xyzAccelerationRatio;     // 1021 笛卡尔位置加速度比例
    int8_t rAccelerationRatio;       // 1022 笛卡尔姿态加速度比例
    int8_t xyzJerkRatio;             // 1023 笛卡尔位置加加速度比例（未实现）
    int8_t rJerkRatio;               // 1024 笛卡尔姿态加加速度比例（未实现）
    int8_t BrakeStatus;              // 1025 机器人抱闸状态
    int8_t EnableStatus;             // 1026 机器人使能状态
    int8_t DragStatus;               // 1027 机器人拖拽状态
    int8_t RunningStatus;            // 1028 机器人运行状态
    int8_t ErrorStatus;              // 1029 机器人报警状态
    int8_t JogStatus;                // 1030 机器人点动状态
    int8_t RobotType;                // 1031 M1机型手系
    int8_t DragButtonSignal;         // 1032 按钮板拖拽信号
    int8_t EnableButtonSignal;       // 1033 按钮板使能信号
    int8_t RecordButtonSignal;       // 1034 按钮板录制信号
    int8_t ReappearButtonSignal;     // 1035 按钮板复现信号
    int8_t JawButtonSignal;          // 1036 按钮板夹爪控制信号
    int8_t SixForceOnline;           // 1037 六维力在线状态（未实现）
    int8_t CollisionStates;          // 1038 碰撞状态
    int8_t ArmApproachState;         // 1039 小臂接近暂停状态
    int8_t J4ApproachState;          // 1040 J4接近暂停状态
    int8_t J5ApproachState;          // 1041 J5接近暂停状态
    int8_t J6ApproachState;          // 1042 J6接近暂停状态
    int8_t Reserve2[61];             // 1043 ~ 1103   预留
    double vibrationDisZ;            // 1104 ~ 1111 加速度计测量Z轴抖动位移
    uint64_t currentCommandId;       // 1112 ~ 1119 当前运动队列id
    double m_actual[6];              // 1120 ~ 1167 实际扭矩
    double load;                     // 1168 ~ 1175 负载质量
    double centerX;                  // 1176 ~ 1183 负载 X 偏心距离
    double centerY;                  // 1184 ~ 1191 负载 Y 偏心 距离
    double centerZ;                  // 1192 ~ 1199 负载 Z 偏心距离
    double user[6];                  // 1200 ~ 1247 用户坐标系值
    double tool[6];                  // 1248 ~ 1295 工具坐标系值
    double TraceIndex;               // 1296 ~ 1303 轨迹复现索引 （未实现）
    double SixForceValue[6];         // 1304 ~ 1351 六维力传感器原始值
    double TargetQuaternion[4];      // 1352 ~ 1383 目标四元数
    double ActualQuaternion[4];      // 1384 ~ 1415 实际四元数
    uint16_t AutoManualMode;         // 1416 ~ 1417 手自动模式 0: 未开启 1: manual 2:auto
    int8_t Reserve3[22];             // 1418 ~ 1439
} RealTimeData;
#pragma pack(pop)

/**
 * URCommander
 */
class CR5Commander
{
protected:
    static constexpr double PI = 3.1415926;

private:
    std::mutex mutex_;
    double current_joint_[6];
    double tool_vector_[6];
    RealTimeData real_time_data_;
    std::atomic<bool> is_running_;
    std::unique_ptr<std::thread> thread_;
    std::shared_ptr<TcpClient> real_time_tcp_;
    std::shared_ptr<TcpClient> dash_board_tcp_;

public:
    explicit CR5Commander(const std::string& ip)
        : current_joint_{}, tool_vector_{}, real_time_data_{}, is_running_(false)
    {
        is_running_ = false;
        real_time_tcp_ = std::make_shared<TcpClient>(ip, 30004);
        dash_board_tcp_ = std::make_shared<TcpClient>(ip, 29999);
    }

    ~CR5Commander()
    {
        is_running_ = false;
        thread_->join();
    }

    void getCurrentJointStatus(double* joint)
    {
        mutex_.lock();
        memcpy(joint, current_joint_, sizeof(current_joint_));
        mutex_.unlock();
    }

    void getToolVectorActual(double* val)
    {
        mutex_.lock();
        memcpy(val, tool_vector_, sizeof(tool_vector_));
        mutex_.unlock();
    }

    void recvTask()
    {
        uint32_t has_read;
        while (is_running_) {
            if (real_time_tcp_->isConnect()) {
                try {
                    if (real_time_tcp_->tcpRecv(&real_time_data_, sizeof(real_time_data_), has_read, 5000)) {
                        if (real_time_data_.len != 1440)
                            continue;

                        mutex_.lock();
                        for (uint32_t i = 0; i < 6; i++)
                            current_joint_[i] = deg2Rad(real_time_data_.q_actual[i]);

                        memcpy(tool_vector_, real_time_data_.tool_vector_actual, sizeof(tool_vector_));
                        mutex_.unlock();
                    } else {
                        //                        ROS_WARN("tcp recv timeout");
                    }
                } catch (const TcpClientException& err) {
                    real_time_tcp_->disConnect();
                    ROS_ERROR("tcp recv error : %s", err.what());
                }
            } else {
                try {
                    real_time_tcp_->connect();
                } catch (const TcpClientException& err) {
                    ROS_ERROR("tcp recv error : %s", err.what());
                    sleep(3);
                }
            }

            if (!dash_board_tcp_->isConnect()) {
                try {
                    dash_board_tcp_->connect();
                } catch (const TcpClientException& err) {
                    ROS_ERROR("tcp recv error : %s", err.what());
                    sleep(3);
                }
            }
        }
    }

    void init()
    {
        try {
            is_running_ = true;
            thread_ = std::unique_ptr<std::thread>(new std::thread(&CR5Commander::recvTask, this));
        } catch (const TcpClientException& err) {
            ROS_ERROR("Commander : %s", err.what());
        }
    }

    bool isEnable() const
    {
        return real_time_data_.robot_mode == 5;
    }

    bool isConnected() const
    {
        return dash_board_tcp_->isConnect() && real_time_tcp_->isConnect();
    }

    void enableRobot()
    {
        const char* cmd = "EnableRobot()";
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void disableRobot()
    {
        const char* cmd = "DisableRobot()";
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void clearError()
    {
        const char* cmd = "ClearError()";
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void speedFactor(int ratio)
    {
        char cmd[100];
        sprintf(cmd, "SpeedFactor(%d)", ratio);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void user(int index)
    {
        char cmd[100];
        sprintf(cmd, "User(%d)", index);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void tool(int index)
    {
        char cmd[100];
        sprintf(cmd, "Tool(%d)", index);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void setPayload(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "SetPayload(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void Do(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "DO(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void doInstant(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "DOInstant(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void toolDO(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "ToolDO(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void toolDOInstant(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "ToolDOInstant(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void AO(const int32_t index, const int32_t value)
    {
        char cmd[100];
        sprintf(cmd, "AO(%d, %d)", index, value);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void AOInstant(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "AOInstant(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void AccJ(int r)
    {
        char cmd[100];
        sprintf(cmd, "AccJ(%d)", r);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void AccL(int r)
    {
        char cmd[100];
        sprintf(cmd, "AccL(%d)", r);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void VelJ(int r)
    {
        char cmd[100];
        sprintf(cmd, "VelJ(%d)", r);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void VelL(int r)
    {
        char cmd[100];
        sprintf(cmd, "VelL(%d)", r);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void CP(int r)
    {
        char cmd[100];
        sprintf(cmd, "CP(%d)", r);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void PowerOn()
    {
        const char* cmd = "PowerOn()";
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void RunScript(const std::string& projectName)
    {
        char cmd[100];
        sprintf(cmd, "RunScript(%s)", projectName.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void Stop()
    {
        const char* cmd = "Stop()";
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void Pause()
    {
        const char* cmd = "Pause()";
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void Continue()
    {
        const char* cmd = "Continue()";
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void EnableSafeSkin(int status)
    {
        char cmd[100];
        sprintf(cmd, "EnableSafeSkin(%d)", status);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void SetSafeSkin(int part, int status)
    {
        char cmd[100];
        sprintf(cmd, "SetSafeSkin(%d,%d)", part, status);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void GetStartPose(const std::string& traceName)
    {
        char cmd[100];
        sprintf(cmd, "GetStartPose(%s)", traceName.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void StartPath(const std::string& traceName)
    {
        char cmd[100];
        sprintf(cmd, "StartPath(%s)", traceName.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void PositiveKin(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "PositiveKin(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void InverseKin(const std::string& parameter)
    {
        char cmd[1000];
        sprintf(cmd, "InverseKin(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void SetCollisionLevel(int level)
    {
        char cmd[100];
        sprintf(cmd, "SetCollisionLevel(%d)", level);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void GetAngle()
    {
        const char* cmd = "GetAngle()";
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void GetPose()
    {
        const char* cmd = "GetPose()";
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }
    void GetPose(std::string arg)
    {
        char cmd[100];
        sprintf(cmd, "GetPose(%s)", arg.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void EmergencyStop(int value)
    {
        char cmd[100];
        sprintf(cmd, "EmergencyStop(%d)", value);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void ModbusRTUCreate(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "ModbusRTUCreate(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void ModbusCreate(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "ModbusCreate(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void ModbusClose(int index)
    {
        char cmd[100];
        sprintf(cmd, "ModbusClose(%d)", index);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void GetInBits(const std::string& parameter)
    {
        char cmd[100];
        sprintf(cmd, "GetInBits(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void GetInRegs(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "GetInRegs(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void GetCoils(int index, int addr, int count)
    {
        char cmd[100];
        sprintf(cmd, "GetCoils(%d,%d,%d)", index, addr, count);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void SetCoils(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "SetCoils(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void GetHoldRegs(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "GetHoldRegs(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void SetHoldRegs(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "SetHoldRegs(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void GetErrorID()
    {
        const char* cmd = "GetErrorID()";
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void DI(int index)
    {
        char cmd[100];
        sprintf(cmd, "DI(%d)", index);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void ToolDI(int index)
    {
        char cmd[100];
        sprintf(cmd, "ToolDI(%d)", index);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void AI(int index)
    {
        char cmd[100];
        sprintf(cmd, "AI(%d)", index);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void ToolAI(int index)
    {
        char cmd[100];
        sprintf(cmd, "ToolAI(%d)", index);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void DIGroup(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "DIGroup(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void DOGroup(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "DOGroup(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void BrakeControl(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "BrakeControl(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void StartDrag()
    {
        const char* cmd = "StartDrag()";
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void StopDrag()
    {
        const char* cmd = "StopDrag()";
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void DragSensivity(const std::string& parameter)
    {
        char cmd[100];
        sprintf(cmd, "DragSensivity(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void GetDO(int index)
    {
        char cmd[100];
        sprintf(cmd, "GetDO(%d)", index);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void GetAO(int index)
    {
        char cmd[100];
        sprintf(cmd, "GetAO(%d)", index);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void GetDOGroup(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "GetDOGroup(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void SetTool485(const std::string& parameter)
    {
        std::stringstream ss;
        ss << "SetTool485(" << parameter << ")";
        std::string arg(ss.str());
        dash_board_tcp_->tcpSend(arg.c_str(), arg.size());
    }

    void SetSafeWallEnable(int index, int value)
    {
        char cmd[100];
        sprintf(cmd, "SetSafeWallEnable(%d,%d)", index, value);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void SetToolPower(int status)
    {
        char cmd[100];
        sprintf(cmd, "SetToolPower(%d)", status);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void SetToolMode(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "SetToolMode(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void SetBackDistance(double distance)
    {
        char cmd[100];
        sprintf(cmd, "SetBackDistance(%0.3f)", distance);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void SetPostCollisionMode(int mode)
    {
        char cmd[100];
        sprintf(cmd, "SetPostCollisionMode(%d)", mode);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void SetUser(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "SetUser(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void SetTool(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "SetTool(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void CalcUser(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "CalcUser(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void CalcTool(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "CalcTool(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void GetInputBool(int address)
    {
        char cmd[100];
        sprintf(cmd, "GetInputBool(%d)", address);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void GetInputInt(int address)
    {
        char cmd[100];
        sprintf(cmd, "GetInputInt(%d)", address);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void GetInputFloat(int address)
    {
        char cmd[100];
        sprintf(cmd, "GetInputFloat(%d)", address);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void GetOutputBool(int address)
    {
        char cmd[100];
        sprintf(cmd, "GetOutputBool(%d)", address);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void GetOutputInt(int address)
    {
        char cmd[100];
        sprintf(cmd, "GetOutputInt(%d)", address);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void GetOutputFloat(int address)
    {
        char cmd[100];
        sprintf(cmd, "GetOutputFloat(%d)", address);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void SetOutputBool(int address, int value)
    {
        char cmd[100];
        sprintf(cmd, "SetOutputBool(%d,%d)", address, value);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void SetOutputInt(int address, int value)
    {
        char cmd[100];
        sprintf(cmd, "SetOutputInt(%d,%d)", address, value);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void SetOutputFloat(int address, double value)
    {
        char cmd[100];
        sprintf(cmd, "SetOutputFloat(%d,%0.3f)", address, value);
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void movJ(const std::string& jointorpose)
    {
        char cmd[100];
        sprintf(cmd, "MovJ(%s)", jointorpose.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void movL(const std::string& jointorpose)
    {
        char cmd[100];
        sprintf(cmd, "MovL(%s)", jointorpose.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void MovLIO(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "MovLIO(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void MovJIO(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "MovJIO(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void Arc(const std::string& parameter)
    {
        char cmd[1000];
        sprintf(cmd, "Arc(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void Circle(const std::string& parameter)
    {
        char cmd[1000];
        sprintf(cmd, "Circle(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void moveJog(const std::string& axis)
    {
        char cmd[100];
        sprintf(cmd, "MoveJog(%s)", axis.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void RelMovJTool(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "RelMovJTool(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void RelMovLTool(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "RelMovLTool(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void RelMovJUser(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "RelMovJUser(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void RelMovLUser(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "RelMovLUser(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void RelJointMovJ(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "RelJointMovJ(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void GetCurrentCommandId()
    {
        const char* cmd = "GetCurrentCommandId()";
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void ServoJ(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "ServoJ(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void ServoP(const std::string& parameter)
    {
        char cmd[100];
        assert(parameter.size() < 100);
        sprintf(cmd, "ServoP(%s)", parameter.c_str());
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    void dashSendCmd(const char* cmd, uint32_t len)
    {
        dash_board_tcp_->tcpSend(cmd, strlen(cmd));
    }

    bool dashRecvCmd(char* cmd, uint32_t len, uint32_t timeout)
    {
        uint32_t has_read;
        dash_board_tcp_->tcpRecv(cmd, len, has_read, timeout);
        return has_read != 0;
    }

    void motionDoCmd(const char* cmd, int32_t& err_id)
    {
        std::vector<std::string> result;
        doTcpCmd(dash_board_tcp_, cmd, err_id, result);
    }

    void motionDoCmd(const char* cmd, int32_t& err_id, std::vector<std::string>& result)
    {
        doTcpCmd(dash_board_tcp_, cmd, err_id, result);
    }

    bool callRosService(const std::string cmd, int32_t& err_id)
    {
        try {
            std::vector<std::string> result_;
            doTcpCmd(this->dash_board_tcp_, cmd.c_str(), err_id, result_);
            return true;
        } catch (const TcpClientException& err) {
            ROS_ERROR("%s", err.what());
            err_id = -1;
            return false;
        }
    }
    bool callRosService(const std::string cmd, int32_t& err_id, std::vector<std::string>& result_)
    {
        try {
            doTcpCmd(this->dash_board_tcp_, cmd.c_str(), err_id, result_);
            return true;
        } catch (const TcpClientException& err) {
            ROS_ERROR("%s", err.what());
            err_id = -1;
            return false;
        }
    }

    uint16_t getRobotMode() const
    {
        return real_time_data_.robot_mode;
    }

    const RealTimeData* getRealData() const
    {
        return &real_time_data_;
    }

private:
    static void doTcpCmd(std::shared_ptr<TcpClient>& tcp, const char* cmd, int32_t& err_id,
                         std::vector<std::string>& result)
    {
        try {
            uint32_t has_read;
            char buf[1024];
            memset(buf, 0, sizeof(buf));

            ROS_INFO("tcp send cmd : %s", cmd);
            tcp->tcpSend(cmd, strlen(cmd));

            char* recv_ptr = buf;

            while (true) {
                bool err = tcp->tcpRecv(recv_ptr, 1024, has_read, 0);
                if (!err) {
                    sleep(0.01);
                    continue;
                }
                if (*(recv_ptr + strlen(recv_ptr) - 1) == ';')
                    break;

                recv_ptr = recv_ptr + strlen(recv_ptr);
            }
            result = regexRecv(std::string(buf));
            if (result.size() >= 2U) {
                if (stoi(result[0]) == 0) {
                    err_id = stoi(result[1]);
                } else {
                    err_id = 2147483647;    // int-max
                }
            }

            ROS_INFO("tcp recv feedback : %s", buf);    // FIXME parse the buf may be better
        } catch (const std::logic_error& err) {
            ROS_ERROR("tcpDoCmd failed : %s", err.what());
        }
    }

    static std::string parseString(const std::string& str)
    {
        std::string returnInfomation = "ErrorID: ";

        std::size_t pos = str.find(',');
        if (pos == std::string::npos)
            throw std::logic_error(std::string("Has no ',' found : ") + str);
        returnInfomation += str.substr(0, pos);
        returnInfomation += " ReturnValue: ";

        // parse result
        std::size_t start_pos = str.find('{');
        if (start_pos == std::string::npos)
            throw std::logic_error(std::string("Has no '{': ") + str);
        std::size_t end_pos = str.find('}');
        if (end_pos == std::string::npos)
            throw std::logic_error(std::string("Has no '}': ") + str);

        assert(end_pos > start_pos);
        returnInfomation += str.substr(start_pos + 1, end_pos - start_pos - 1);

        returnInfomation += " cmd = ";
        returnInfomation += str.substr(end_pos + 1);
        return returnInfomation;
    }

    static inline double rad2Deg(double rad)
    {
        return rad * 180.0 / PI;
    }

    static inline double deg2Rad(double deg)
    {
        return deg * PI / 180.0;
    }

    static std::vector<std::string> regexRecv(std::string getRecvInfo)
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
};
