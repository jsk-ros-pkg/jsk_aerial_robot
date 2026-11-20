#!/usr/bin/env python

from __future__ import print_function # for print function in python2
import sys, select, termios, tty

import rospy
from std_msgs.msg import Empty
from spinal.msg import ServoControlCmd
from aerial_robot_msgs.msg import FlightNav
import rosgraph




msg = """
Instruction:

---------------------------

w a d i k

r

s

Please don't have caps lock on.
CTRL+c to quit
---------------------------
"""

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def printMsg(msg, msg_len = 50):
        print(msg.ljust(msg_len) + "\r", end="")

if __name__=="__main__":
        settings = termios.tcgetattr(sys.stdin)
        rospy.init_node("servo_control_publisher")
        pub = rospy.Publisher('/servo/target_states', ServoControlCmd, queue_size=10)
        print(msg)

        rospy.sleep(0.01)
        pos_init = 2047
        wave_large = 1024
        wave_small = 200
        current_pos_0 = pos_init
        current_pos_1 = pos_init
        current_pos_2 = pos_init
        running = False


        try:
                while(True):
                        key = getKey()

                        msg = ""

                        if key == 'l':

                                msg = "send land command"

                        if key == 'w':
                            # running = True
                            for i in range(2):
                                pub.publish(ServoControlCmd(index=[0], cmd=[current_pos_0 + wave_large]))
                                rospy.sleep(0.5)
                                pub.publish(ServoControlCmd(index=[0], cmd=[current_pos_0 - wave_large]))
                                rospy.sleep(0.5)  # Sleep for 1 second between iterations
                            pub.publish(ServoControlCmd(index=[0], cmd=[current_pos_0]))
                            msg = "step forward"
                            # running = False

                        if key == 's':
                            pub.publish(ServoControlCmd(index=[0, 1, 2, 3], cmd=[pos_init, pos_init, pos_init, pos_init]))
                            current_pos_0 = pos_init
                            current_pos_1 = pos_init
                            current_pos_2 = pos_init
                            current_pos_3 = pos_init
                            msg = "back to initial position"

                        if key == 'a':
                            # running = True
                            for i in range(2):
                                pub.publish(ServoControlCmd(index=[0], cmd=[current_pos_0 - 1024]))
                                rospy.sleep(0.4)
                                pub.publish(ServoControlCmd(index=[0], cmd=[current_pos_0]))
                                rospy.sleep(0.4)
                            pub.publish(ServoControlCmd(index=[0], cmd=[current_pos_0]))
                            msg = "step turn left"
                            # running = False

                        if key == 'd':
                            for i in range(2):
                                pub.publish(ServoControlCmd(index=[0], cmd=[current_pos_0 + 1024]))
                                rospy.sleep(0.4)
                                pub.publish(ServoControlCmd(index=[0], cmd=[current_pos_0]))
                                rospy.sleep(0.4)
                            pub.publish(ServoControlCmd(index=[0], cmd=[current_pos_0]))
                            msg = "step turn right"

                        if key == 'f':
                            current_pos_0 -= 10240
                            running = True
                            pub.publish(ServoControlCmd(index=[0], cmd=[current_pos_0]))
                            rospy.sleep(0.3)
                            msg = "continue forward 3 rounds"

                        if key == 'r':
                            current_pos_0 += 200
                            # running = True
                            pub.publish(ServoControlCmd(index=[0], cmd=[current_pos_0]))
                            rospy.sleep(0.05)
                            msg = "continue forward"

                        if key == 'k':
                            if current_pos_2 > 3072:
                                continue
                            current_pos_2 += 200
                            print(current_pos_2)
                            pub.publish(ServoControlCmd(index=[2], cmd=[current_pos_2]))
                            rospy.sleep(0.3)
                            msg = "tail up"

                        if key == 'i':
                            if current_pos_2 <= 1024:
                                continue
                            current_pos_2 -= 200
                            pub.publish(ServoControlCmd(index=[2], cmd=[current_pos_2]))
                            rospy.sleep(0.3)
                            msg = "tail down"

                        if key == '\x03':
                                break

                        printMsg(msg)
                        rospy.sleep(0.001)

        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
