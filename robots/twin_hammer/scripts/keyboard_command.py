#!/usr/bin/env python

from __future__ import print_function # for print function in python2
import sys, select, termios, tty

import rospy
from std_msgs.msg import Empty, String
from aerial_robot_msgs.msg import FlightNav
from std_msgs.msg import Int8
from geometry_msgs.msg import WrenchStamped
import rosgraph

msg = """
Instruction:

---------------------------

r:  arming motor (please do before takeoff)
t:  takeoff
l:  land
f:  force landing
h:  halt (force stop motor)

     q           w           e           [
(turn left)  (forward)  (turn right)  (move up)

     a           s           d           ]
(move left)  (backward) (move right) (move down)

To haptics feedback:
1. press number key
2. press direction
3. press "m"

number key:  wrench value

force  :  x          y           z
torque :  o (roll)   p (pitch)   i (yaw)

m:  haptics force on
n:  haptics force off

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
        rospy.init_node("keyboard_command_for_twin_hammer")
        # robot_ns = rospy.get_param("~robot_ns", "")
        robot_ns = "/twin_hammer"
        print(msg)

        if not robot_ns:
                master = rosgraph.Master('/rostopic')
                try:
                        _, subs, _ = master.getSystemState()

                except socket.error:
                        raise ROSTopicIOException("Unable to communicate with master!")

                teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
                if len(teleop_topics) == 1:
                        robot_ns = teleop_topics[0].split('/teleop')[0]

        ns = robot_ns + "/teleop_command"
        land_pub = rospy.Publisher(ns + '/land', Empty, queue_size=1)
        halt_pub = rospy.Publisher(ns + '/halt', Empty, queue_size=1)
        start_pub = rospy.Publisher(ns + '/start', Empty, queue_size=1)
        takeoff_pub = rospy.Publisher(ns + '/takeoff', Empty, queue_size=1)
        force_landing_pub = rospy.Publisher(ns + '/force_landing', Empty, queue_size=1)
        nav_pub = rospy.Publisher(robot_ns + '/uav/nav', FlightNav, queue_size=1)
        haptics_switch_pub = rospy.Publisher(robot_ns + '/haptics_switch', Int8, queue_size=1)
        haptics_wrench_pub = rospy.Publisher(robot_ns + '/haptics_wrench', WrenchStamped, queue_size=1)
        teleop_mode_pub = rospy.Publisher(robot_ns + '/teleop_mode', String, queue_size=1)

        xy_vel   = rospy.get_param("xy_vel", 0.2)
        z_vel    = rospy.get_param("z_vel", 0.2)
        yaw_vel  = rospy.get_param("yaw_vel", 0.2)

        motion_start_pub = rospy.Publisher('task_start', Empty, queue_size=1)

        input_num = 0
        wrench_scale = 1
        desire_wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        try:
                while(True):
                        nav_msg = FlightNav()
                        nav_msg.control_frame = FlightNav.WORLD_FRAME
                        nav_msg.target = FlightNav.COG
                        haptics_switch_msg = Int8()
                        haptics_wrench_msg = WrenchStamped()
                        teleop_mode_msg = String()

                        key = getKey()

                        msg = ""

                        if key == 'l':
                                land_pub.publish(Empty())
                                msg = "send land command"
                        if key == 'r':
                                start_pub.publish(Empty())
                                msg = "send motor-arming command"
                        if key == 'h':
                                halt_pub.publish(Empty())
                                msg = "send motor-disarming (halt) command"
                        if key == 'f':
                                force_landing_pub.publish(Empty())
                                msg = "send force landing command"
                        if key == 't':
                                takeoff_pub.publish(Empty())
                                msg = "send takeoff command"
                        # if key == 'x':
                        #         motion_start_pub.publish()
                        #         msg = "send task-start command"
                        if key == 'w':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_x = xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +x vel command"
                        if key == 's':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_x = -xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send -x vel command"
                        if key == 'a':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_y = xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +y vel command"
                        if key == 'd':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_y = -xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send -y vel command"
                        if key == 'q':
                                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_omega_z = yaw_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +yaw vel command"
                        if key == 'e':
                                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_omega_z = -yaw_vel
                                msg = "send -yaw vel command"
                                nav_pub.publish(nav_msg)
                        if key == '[':
                                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_z = z_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +z vel command"
                        if key == ']':
                                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_z = -z_vel
                                nav_pub.publish(nav_msg)
                                msg = "send -z vel command"

                        if key == 'm':
                                haptics_switch_msg.data = 1
                                haptics_switch_pub.publish(haptics_switch_msg)
                                msg = "haptics force on"
                        if key == 'n':
                                haptics_switch_msg.data = 0
                                haptics_switch_pub.publish(haptics_switch_msg)
                                msg = "haptics force off"

                        if key == 'b':
                                teleop_mode_msg.data = 'pos'
                                teleop_mode_pub.publish(teleop_mode_msg)
                                msg = "POS mode"
                        if key == 'v':
                                teleop_mode_msg.data = 'vel'
                                teleop_mode_pub.publish(teleop_mode_msg)
                                msg = "VEL mode"

                        if key == '0':
                                input_num = 0.0
                        if key == '1':
                                input_num = 1.0
                        if key == '2':
                                input_num = 2.0
                        if key == '3':
                                input_num = 3.0
                        if key == '4':
                                input_num = 4.0
                        if key == '5':
                                input_num = 5.0
                        if key == '6':
                                input_num = -4.0
                        if key == '7':
                                input_num = -3.0
                        if key == '8':
                                input_num = -2.0
                        if key == '9':
                                input_num = -1.0
                        des_input = input_num * wrench_scale
                        # msg = "input is "+str(des_input)+"N"

                        if key == 'x':
                                desire_wrench[0] = des_input
                                msg = "force_x"
                                print(desire_wrench)
                        if key == 'y':
                                desire_wrench[1] = des_input
                                msg = "force_y"
                                print(desire_wrench)
                        if key == 'z':
                                desire_wrench[2] = des_input
                                msg = "force_z"
                                print(desire_wrench)
                        if key == 'o':
                                desire_wrench[3] = des_input
                                msg = "torque_x"
                                print(desire_wrench)
                        if key == 'p':
                                desire_wrench[4] = des_input
                                msg = "torque_y"
                                print(desire_wrench)
                        if key == 'i':
                                desire_wrench[5] = des_input
                                msg = "torque_z"
                                print(desire_wrench)

                        haptics_wrench_msg.wrench.force.x = desire_wrench[0]
                        haptics_wrench_msg.wrench.force.y = desire_wrench[1]
                        haptics_wrench_msg.wrench.force.z = desire_wrench[2]
                        haptics_wrench_msg.wrench.torque.x = desire_wrench[3]
                        haptics_wrench_msg.wrench.torque.y = desire_wrench[4]
                        haptics_wrench_msg.wrench.torque.z = desire_wrench[5]
                        if key == 'm':
                                haptics_wrench_pub.publish(haptics_wrench_msg)
                        if key == 'n' or key == 'l':
                                desire_wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                                haptics_wrench_pub.publish(haptics_wrench_msg)

                        if key == '\x03':
                                break

                        printMsg(msg)
                        rospy.sleep(0.001)

        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


