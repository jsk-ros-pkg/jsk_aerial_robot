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

        master = rosgraph.Master('/rostopic')
        try:
                _, subs, _ = master.getSystemState()

        except socket.error:
                raise ROSTopicIOException("Unable to communicate with master!")

        teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
        robot_names = [topic.split('/teleop')[0] for topic in teleop_topics]

        land_pubs = []
        halt_pubs = []
        start_pubs = []
        takeoff_pubs = []
        force_landing_pubs = []
        ctrl_mode_pubs = []
        motion_start_pubs =[]
        for name in robot_names:
                ns = name + "/teleop_command"
                land_pubs.append(rospy.Publisher(ns + '/land', Empty, queue_size=1))
                halt_pubs.append(rospy.Publisher(ns + '/halt', Empty, queue_size=1))
                start_pubs.append(rospy.Publisher(ns + '/start', Empty, queue_size=1))
                takeoff_pubs.append(rospy.Publisher(ns + '/takeoff', Empty, queue_size=1))
                force_landing_pubs.append(rospy.Publisher(ns + '/force_landing', Empty, queue_size=1))
                ctrl_mode_pubs.append(rospy.Publisher(ns + '/ctrl_mode', Int8, queue_size=1))
                motion_start_pubs.append(rospy.Publisher('task_start', Empty, queue_size=1))

        #the way to write publisher in python
        comm=Int8()
        gain=UInt16()
        try:
                while(True):
                        key = getKey()
                        print("the key value is {}".format(ord(key)))
                        # takeoff and landing
                        if key == 'l':
                                for land_pub in land_pubs:
                                        land_pub.publish(Empty())
                                        #for hydra joints
                        if key == 'r':
                                for start_pub in start_pubs:
                                        start_pub.publish(Empty())
                                        #for hydra joints
                        if key == 'h':
                                for halt_pub in halt_pubs:
                                        halt_pub.publish(Empty())
                                        #for hydra joints
                        if key == 'f':
                                for force_landing_pub in force_landing_pubs:
                                        force_landing_pub.publish(Empty())
                        if key == 't':
                                for takeoff_pub in takeoff_pubs:
                                        takeoff_pub.publish(Empty())
                        if key == 'x':
                                for motion_start_pub in motion_start_pubs:
                                        motion_start_pub.publish()
                        if key == 'v':
                                comm.data = 1
                                for ctrl_mode_pub in ctrl_mode_pubs:
                                        ctrl_mode_pub.publish(comm)
                        if key == 'p':
                                comm.data = 0
                                for ctrl_mode_pub in ctrl_mode_pubs:
                                        ctrl_mode_pub.publish(comm)
                        if key == '\x03':
                                break
                        rospy.sleep(0.001)

        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


