#!/usr/bin/env python

from __future__ import print_function # for print function in python2
import sys, select, termios, tty

import rospy
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav
import rosgraph
from spinal.msg import ServoControlCmd



msg = """
Instruction:

---------------------------

i:  init

Please don't have caps lock on.
CTRL+c to quit
---------------------------
"""

## id
## 3        4
## 1        0
##      2

dest_pos = [2047, 2047, 2047, 2047, 2047]
reverse = [-1, -1, -1, -1, -1]

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def printMsg(msg, msg_len = 50):
        print(msg.ljust(msg_len) + "\r", end="")

def min_cut(pos, min):
    for i in range(len(pos)):
        if pos[i] < min:
            pos[i] = min
    return pos

if __name__=="__main__":
        settings = termios.tcgetattr(sys.stdin)
        rospy.init_node("tail_command")
        print(msg)

        tail_pub = rospy.Publisher("servo/target_states", ServoControlCmd, queue_size=1)
        
        rate = rospy.Rate(10)
        speed = 200

        try:
            while(True):
                tail_msg = ServoControlCmd()
                tail_msg.index = [3, 4, 5, 6, 7]
                # tail_msg.index = [0, 1, 2, 3, 4]

                key = getKey()

                if key == 'i':
                    dest_pos = [2047, 2047, 2047, 2047, 2047]
                    tail_msg.angles = dest_pos
                    tail_pub.publish(tail_msg)
                    print("init")
                if key == 'a': ## left
                    dest_pos[0] = dest_pos[0] + speed * reverse[0] * 2
                    dest_pos[1] = dest_pos[1] - speed * reverse[1]
                    dest_pos[2] = dest_pos[2] + speed * reverse[2]
                    dest_pos[3] = dest_pos[3] - speed * reverse[3]
                    dest_pos[4] = dest_pos[4] + speed * reverse[4] * 2
                    dest_pos = min_cut(dest_pos, -2047)
                    tail_msg.angles = dest_pos
                    tail_pub.publish(tail_msg)
                    print("left")
                if key == 'd': ## right
                    dest_pos[0] = dest_pos[0] - speed * reverse[0]
                    dest_pos[1] = dest_pos[1] + speed * reverse[1] * 2
                    dest_pos[2] = dest_pos[2] + speed * reverse[2]
                    dest_pos[3] = dest_pos[3] + speed * reverse[3] * 2
                    dest_pos[4] = dest_pos[4] - speed * reverse[4]
                    dest_pos = min_cut(dest_pos, -2047)
                    tail_msg.angles = dest_pos
                    tail_pub.publish(tail_msg)
                    print("right")
                if key == 'w': ## up
                    dest_pos[0] = dest_pos[0] + speed * reverse[0] * 2
                    dest_pos[1] = dest_pos[1] + speed * reverse[1] * 2
                    dest_pos[2] = dest_pos[2] - speed * reverse[2]
                    dest_pos[3] = dest_pos[3] - speed * reverse[3]
                    dest_pos[4] = dest_pos[4] - speed * reverse[4]
                    dest_pos = min_cut(dest_pos, -2047)
                    tail_msg.angles = dest_pos
                    tail_pub.publish(tail_msg)
                    print("up")
                if key == 's': ## down
                    dest_pos[0] = dest_pos[0] - speed * reverse[0] 
                    dest_pos[1] = dest_pos[1] - speed * reverse[1] 
                    dest_pos[2] = dest_pos[2] + speed * reverse[2] * 2
                    dest_pos[3] = dest_pos[3] + speed * reverse[3] * 2
                    dest_pos[4] = dest_pos[4] + speed * reverse[4] * 2
                    dest_pos = min_cut(dest_pos, -2047)
                    tail_msg.angles = dest_pos
                    tail_pub.publish(tail_msg)
                    print("down")
                if key == '\x03':
                        break

                rospy.sleep(0.001)

        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


