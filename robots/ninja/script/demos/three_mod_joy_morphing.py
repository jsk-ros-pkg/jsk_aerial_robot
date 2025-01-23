#!/usr/bin/env python
from sensor_msgs.msg import Joy

import rospy
import numpy as np
import tf
import rosgraph
import sys, select, termios, tty, time
from sensor_msgs.msg import JointState
import math


class ThreeModJoyMorph():
    def __init__(self):
        master = rosgraph.Master('/rostopic')
        try:
            _, subs, _ = master.getSystemState()
        except socket.error:
            raise ROSTopicIOException("Unable to communicate with master!")

        teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]

        self.joy_sub = rospy.Subscriber("/ninja1/joy",Joy, self.joyCb)
        self.joint_control_pub = rospy.Publisher("/assembly/target_joint_pos", JointState, queue_size=10)

        self.nav_rate = rospy.Rate(40)
        
        self.joy = Joy()

        self.morph_flag_positive = False
        self.morph_flag_negative = False
        self.morph_delta = 0.01
        self.current_angle = 1.5

        self.desire_joint = JointState()
        self.desire_joint.name = ['mod1/yaw','mod2/yaw']
        self.desire_joint.velocity = [0.08]
        
        time.sleep(0.5)

    def joyCb(self, msg):
        self.joy = msg
        if msg.axes[3] == -1:
            if not self.morph_flag_positive:
                self.morph_flag_positive = True
                self.current_angle = self.current_angle + self.morph_delta
                self.desire_joint.position = [self.current_angle,self.current_angle]
                self.joint_control_pub.publish(self.desire_joint)
        elif msg.axes[3] == 1:
            self.morph_flag_positive = False

        if msg.axes[4] == -1:
            if not self.morph_flag_negative:
                self.morph_flag_negative = True
                self.current_angle = self.current_angle - self.morph_delta
                self.desire_joint.position = [self.current_angle,self.current_angle]
                self.joint_control_pub.publish(self.desire_joint)
        elif msg.axes[4] == 1:
            self.morph_flag_negative = False            
            
    def main(self):
        rospy.spin()

if __name__ == "__main__":

  rospy.init_node("ninja_asm_joy")

  joy = ThreeModJoyMorph()
  joy.main()



