#!/usr/bin/env python
from __future__ import print_function # for print function in pytho2
import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from std_msgs.msg import UInt8
from aerial_robot_msgs.msg import FlightNav
import rosgraph

import sys, select, termios, tty

msg = """
Instruction:

---------------------------

r:  arming motor (please do before takeoff)
t:  takeoff
l:  land
f:  force landing
h:  halt (force stop motor)
x:  motion start
y:  motion interrupt
z:  motion force transition

     q           w           e           [
(turn left)  (forward)  (turn right)  (move up)

     a           s           d           ]
(move left)  (backward) (move right) (move down)


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
        rospy.init_node("keyboard_command")
        print(msg)

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
        nav_pubs = []

        xy_vel   = rospy.get_param("xy_vel", 0.2)
        z_vel    = rospy.get_param("z_vel", 0.2)
        yaw_vel  = rospy.get_param("yaw_vel", 0.2)

        for name in robot_names:
                ns = name + "/teleop_command"
                land_pubs.append(rospy.Publisher(ns + '/land', Empty, queue_size=1))
                halt_pubs.append(rospy.Publisher(ns + '/halt', Empty, queue_size=1))
                start_pubs.append(rospy.Publisher(ns + '/start', Empty, queue_size=1))
                takeoff_pubs.append(rospy.Publisher(ns + '/takeoff', Empty, queue_size=1))
                force_landing_pubs.append(rospy.Publisher(ns + '/force_landing', Empty, queue_size=1))
                nav_pubs.append(rospy.Publisher(name + '/uav/nav', FlightNav, queue_size=1))

                motion_start_pub = rospy.Publisher('motion_start', Empty, queue_size=1)
                motion_interrupt_pub = rospy.Publisher("/emergency_assembly_interuption",Empty,queue_size=1)
                motion_force_trans_pub = rospy.Publisher('/force_switching', Empty, queue_size=1)

        try:
                while(True):
                        nav_msg = FlightNav()
                        nav_msg.control_frame = FlightNav.WORLD_FRAME
                        nav_msg.target = FlightNav.COG
                        key = getKey()
                        msg = ""

                        if key == 'l':
                                for land_pub in land_pubs:
                                        land_pub.publish(Empty())
                                        #for hydra joints
                                msg = "send land command"
                        if key == 'r':
                                for start_pub in start_pubs:
                                        start_pub.publish(Empty())
                                        #for hydra joints
                                msg = "send motor-arming command"
                        if key == 'h':
                                for halt_pub in halt_pubs:
                                        halt_pub.publish(Empty())
                                        #for hydra joints
                                msg = "send motor-disarming (halt) command"
                        if key == 'f':
                                for force_landing_pub in force_landing_pubs:
                                        force_landing_pub.publish(Empty())
                                msg = "send force landing command"
                        if key == 't':
                                for takeoff_pub in takeoff_pubs:
                                        takeoff_pub.publish(Empty())
                                msg = "send takeoff command"
                        if key == 'x':
                                motion_start_pub.publish()
                                msg = "send task-start command"
                        if key == 'y':
                                motion_interrupt_pub.publish()
                                msg = "send task-interrupt command"
                        if key == 'z':
                                motion_force_trans_pub.publish()
                                msg = "send task-force-transition command"
                        if key == 'w':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_x = xy_vel
                                for nav_pub in nav_pubs:
                                        nav_pub.publish(nav_msg)
                                msg = "send +x vel command"
                        if key == 's':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_x = -xy_vel
                                for nav_pub in nav_pubs:
                                        nav_pub.publish(nav_msg)
                                msg = "send -x vel command"
                        if key == 'a':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_y = xy_vel
                                for nav_pub in nav_pubs:
                                        nav_pub.publish(nav_msg)
                                msg = "send +y vel command"
                        if key == 'd':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_y = -xy_vel
                                for nav_pub in nav_pubs:
                                        nav_pub.publish(nav_msg)
                                msg = "send -y vel command"
                        if key == 'q':
                                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_omega_z = yaw_vel
                                for nav_pub in nav_pubs:
                                        nav_pub.publish(nav_msg)
                                msg = "send +yaw vel command"
                        if key == 'e':
                                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_omega_z = -yaw_vel
                                for nav_pub in nav_pubs:
                                        nav_pub.publish(nav_msg)                                
                                msg = "send -yaw vel command"

                        if key == '[':
                                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_z = z_vel
                                for nav_pub in nav_pubs:
                                        nav_pub.publish(nav_msg)
                                msg = "send +z vel command"
                        if key == ']':
                                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_z = -z_vel
                                for nav_pub in nav_pubs:
                                        nav_pub.publish(nav_msg)
                                msg = "send -z vel command"
                        if key == '\x03':
                                break
                        printMsg(msg)
                        rospy.sleep(0.001)

        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


