/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/07
 *
 * <h2><center>&copy; COPYRIGHT 2021 Dobot CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <ros/ros.h>
#include <ros/param.h>
#include <dobot_bringup/cr5_robot.h>
#include <sensor_msgs/JointState.h>
#include <dobot_bringup/ToolVectorActual.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "CR5Robot");

    try
    {
        ros::NodeHandle node;
        ros::NodeHandle private_node("~");

        ros::AsyncSpinner async_spinner(1);
        async_spinner.start();

        sensor_msgs::JointState joint_state_msg;
        ros::Publisher joint_state_pub = private_node.advertise<sensor_msgs::JointState>("/joint_states", 100);
        dobot_bringup::RobotStatus robot_status_msg;
        ros::Publisher robot_status_pub = private_node.advertise<dobot_bringup::RobotStatus>("/dobot_bringup/msg/RobotStatus", 100);

        dobot_bringup::ToolVectorActual tool_vector_actual_msg;
        ros::Publisher tool_vector_pub =
            private_node.advertise<dobot_bringup::ToolVectorActual>("/dobot_bringup/msg/ToolVectorActual", 100);
        string z ="/";
        const char* robot_type = getenv("DOBOT_TYPE");
        string a = robot_type == nullptr ? "cr5" : robot_type;
        string b = "_robot/joint_controller/follow_joint_trajectory";
        string ss =  z + a+ b ;
        for (uint32_t i = 0; i < 6; i++)
        {
            joint_state_msg.position.push_back(0.0);
            joint_state_msg.name.push_back(std::string("joint") + std::to_string(i + 1));
        }

        CR5Robot robot(private_node, ss);

        double rate_vale = private_node.param("JointStatePublishRate", 10);

        robot.init();
        ros::Rate rate(rate_vale);
        double position[6];
        while (ros::ok())
        {
            //
            // publish joint state
            //
            robot.getJointState(position);
            joint_state_msg.header.stamp = ros::Time::now();
            joint_state_msg.header.frame_id = "dummy_link";
            for (uint32_t i = 0; i < 6; i++)
                joint_state_msg.position[i] = position[i];
            joint_state_pub.publish(joint_state_msg);

            double val[6];
            robot.getToolVectorActual(val);
            tool_vector_actual_msg.x = val[0];
            tool_vector_actual_msg.y = val[1];
            tool_vector_actual_msg.z = val[2];
            tool_vector_actual_msg.rx = val[3];
            tool_vector_actual_msg.ry = val[4];
            tool_vector_actual_msg.rz = val[5];
            tool_vector_pub.publish(tool_vector_actual_msg);

            //
            // publish robot status
            //
            robot_status_msg.is_enable = robot.isEnable();
            robot_status_msg.is_connected = robot.isConnected();
            robot_status_pub.publish(robot_status_msg);

            rate.sleep();
        }
    }
    catch (const std::exception& err)
    {
        ROS_ERROR("%s", err.what());
        return -1;
    }

    return 0;
}
