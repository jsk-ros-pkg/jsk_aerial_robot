#!/usr/bin/env python
import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from std_msgs.msg import UInt8
import rosgraph

import sys, select, termios, tty

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

if __name__=="__main__":
        settings = termios.tcgetattr(sys.stdin)
        rospy.init_node("keyboard_command")

        ns = "/gimbalrotor/teleop_command"
        ns1 = "/gimbalrotor1/teleop_command"
        ns2 = "/gimbalrotor2/teleop_command"
        land_pub_1 = rospy.Publisher(ns1 + '/land', Empty, queue_size=1)
        land_pub_2 = rospy.Publisher(ns2 + '/land', Empty, queue_size=1)
        halt_pub = rospy.Publisher(ns + '/halt', Empty, queue_size=1)
        start_pub_1 = rospy.Publisher(ns1 + '/start', Empty, queue_size=1)
        start_pub_2 = rospy.Publisher(ns2 + '/start', Empty, queue_size=1)
        takeoff_pub_1 = rospy.Publisher(ns1 + '/takeoff', Empty, queue_size=1)
        takeoff_pub_2 = rospy.Publisher(ns2 + '/takeoff', Empty, queue_size=1)
        force_landing_pub = rospy.Publisher(ns + '/force_landing', Empty, queue_size=1)
        ctrl_mode_pub = rospy.Publisher(ns + '/ctrl_mode', Int8, queue_size=1)
        motion_start_pub = rospy.Publisher('task_start', Empty, queue_size=1)


        #the way to write publisher in python
        comm=Int8()
        gain=UInt16()
        try:
                while(True):
                        key = getKey()
                        print("the key value is {}".format(ord(key)))
                        # takeoff and landing
                        if key == 'l':
                                land_pub_1.publish(Empty())
                                land_pub_2.publish(Empty())
                                #for hydra joints
                        if key == 'r':
                                start_pub_1.publish(Empty())
                                start_pub_2.publish(Empty())
                                #for hydra joints
                        if key == 'h':
                                halt_pub.publish(Empty())
                                 #for hydra joints
                        if key == 'f':
                                force_landing_pub.publish(Empty())
                        if key == 't':
                                takeoff_pub_1.publish(Empty())
                                takeoff_pub_2.publish(Empty())
                        if key == 'u':
                                stair_pub.publish(Empty())
                        if key == 'x':
                                motion_start_pub.publish()
                        if key == 'v':
                                comm.data = 1
                                ctrl_mode_pub.publish(comm)
                        if key == 'p':
                                comm.data = 0
                                ctrl_mode_pub.publish(comm)
                        if key == '\x03':
                                break
                        rospy.sleep(0.001)

        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


