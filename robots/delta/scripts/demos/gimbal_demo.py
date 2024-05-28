#!/usr/bin/env python

# rosrun delta gimbal_demo.py __ns:=delta

import rospy
from sensor_msgs.msg import JointState

class gimbalDemo:
    def __init__(self):
        self.gimbals_ctrl_pub_ = rospy.Publisher("gimbals_ctrl", JointState, queue_size=1)
        self.gimbals_ctrl_msg_ = JointState()
        self.gimbals_ctrl_msg_.name = ["gimbal1", "gimbal2", "gimbal3"]
        self.gimbals_ctrl_msg_.position = [0, 0, 0]
        self.gimbal_angle_max_abs = 1.0
        self.gimbal_period = 2.0
        self.d_gimbal_angle = 2.0 * self.gimbal_angle_max_abs / self.gimbal_period / 40

        self.timer = rospy.Timer(rospy.Duration(0.025), self.timerCallback)

    def timerCallback(self, event):
        for i in range(len(self.gimbals_ctrl_msg_.position)):
            self.gimbals_ctrl_msg_.position[i] = self.gimbals_ctrl_msg_.position[i] + self.d_gimbal_angle
        if abs(self.gimbals_ctrl_msg_.position[i]) > self.gimbal_angle_max_abs:
            self.d_gimbal_angle = -self.d_gimbal_angle
        self.gimbals_ctrl_pub_.publish(self.gimbals_ctrl_msg_)

if __name__ == "__main__":
    rospy.init_node("gimbal_demo")
    gimbalDemo()
    rospy.spin()
