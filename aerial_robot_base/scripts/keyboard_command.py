#!/usr/bin/env python
import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from std_msgs.msg import UInt8

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
up/down:       move forward/backward
left/right:    move left/right
w/s:           increase/decrease altitude
a/d:           turn left/right
t/l:           takeoff/land
r:             reset (toggle emergency state)
p:             switch to fly in stairs
anything else: stop

please don't have caps lock on.
CTRL+c to quit
"""

move_bindings = {
        68:('linear', 'y', 0.1), #left
        67:('linear', 'y', -0.1), #right
        65:('linear', 'x', 0.05), #forward
        66:('linear', 'x', -0.05), #back
        'w':('linear', 'z', 0.1),
        's':('linear', 'z', -0.1),
        'a':('angular', 'z', 0.5),
        'd':('angular', 'z', -0.5),
        }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	print msg
	
	#pub = rospy.Publisher('cmd_vel', Twist)
	land_pub = rospy.Publisher('/teleop_command/land', Empty)
        halt_pub = rospy.Publisher('/teleop_command/halt', Empty)
	start_pub = rospy.Publisher('/teleop_command/start', Empty)
	takeoff_pub = rospy.Publisher('/teleop_command/takeoff', Empty)
	roll_pub = rospy.Publisher('/teleop_command/roll', Int8)
	pitch_pub =  rospy.Publisher('/teleop_command/pitch', Int8)
	yaw_pub =  rospy.Publisher('/teleop_command/yaw', Int8)
	thrust_pub = rospy.Publisher('/teleop_command/throttle', Int8)
	stair_pub = rospy.Publisher('/teleop_command/stair', Empty)
	ctrl_mode_pub = rospy.Publisher('/teleop_command/ctrl_mode', Int8)

        #for hydra joints
        joints_ctrl_pub = rospy.Publisher('/teleop_command/joints_ctrl', Int8)


        motion_start_pub = rospy.Publisher('task_start', Empty)
        stop_realtime_lqi_pub = rospy.Publisher('realtime_control', UInt8)

	rospy.init_node('keyboard_command')
        #the way to write publisher in python
	comm=Int8() 
	gain=UInt16() 

	try:
		while(True):
			key = getKey()
			print "the key value is %d" % ord(key)
			# takeoff and landing
			if key == 'l':
				land_pub.publish(Empty())
                                #for hydra joints
			if key == 'r':
				start_pub.publish(Empty())
                                #for hydra joints
			if key == 'h':
				halt_pub.publish(Empty())
                                 #for hydra joints
			if key == 't':
				takeoff_pub.publish(Empty())
			if key == 'u':
				stair_pub.publish(Empty())
			if key == 'm':
                                #for motor bias set
                                comm.data = 0
                                motor_bias_set_pub.publish(comm)
			if key == '1':
                                #for hydra joints
                                comm.data = 1
                                joints_ctrl_pub.publish(comm)
			if key == '2':
                                #for hydra joints
                                comm.data = 2
                                joints_ctrl_pub.publish(comm)
			if key == '3':
                                #for hydra joints
                                comm.data = 3
                                joints_ctrl_pub.publish(comm)
			if key == '4':
                                #for hydra joints
                                comm.data = 4
                                joints_ctrl_pub.publish(comm)
			if key == '5':
                                #for hydra joints
                                comm.data = 5
                                joints_ctrl_pub.publish(comm)
			if key == '6':
                                #for hydra joints
                                comm.data = 6
                                joints_ctrl_pub.publish(comm)
			if key == 'x':
                                stop_real = UInt8()
				stop_real.data = 0
                                #stop_realtime_lqi_pub.publish(stop_real)
                                motion_start_pub.publish()
			if key == 'a':
				comm.data = 1
				yaw_pub.publish(comm)
				print "command"
			if key == 'd':
				comm.data = -1
				yaw_pub.publish(comm)
			if key == 'z':
				comm.data = 2
				yaw_pub.publish(comm)
				print "command"
			if key == 'c':
				comm.data = -2
				yaw_pub.publish(comm)
			if key == 'w':
				comm.data = 1
				thrust_pub.publish(comm)
			if key == 's':
				comm.data = -1
				thrust_pub.publish(comm)
                        #new function for control mode changing
                        if key == 'v':
				comm.data = 1
				ctrl_mode_pub.publish(comm)
                        #new function for control mode changing
			if key == 'p':
				comm.data = 0
				ctrl_mode_pub.publish(comm)
                        if ord(key) == 27:
                                key = getKey()
                                print "the key value is %d" % ord(key)
                                key = getKey()
                                print "the key value is %d" % ord(key)

                                if ord(key) == 68:
                                        comm.data = -1
                                        roll_pub.publish(comm)
                                        print "68"
                                elif ord(key) == 67:
                                        comm.data = 1
                                        roll_pub.publish(comm)
                                        print "67"
                                elif ord(key) == 65:
                                        comm.data = 1
                                        pitch_pub.publish(comm)
                                        print "65"
                                elif ord(key) == 66:
                                        comm.data = -1
                                        pitch_pub.publish(comm)
                                        print "66"
                                else:
                                        print "the key value is %d" % ord(key)
                                        print "no valid command"
			if ord(key) in move_bindings.keys():
                                #key = ord(key)
				msg ="""clear"""
			else:
				if (key == '\x03'):
					break
			rospy.sleep(0.001)

	except Exception as e:
		print e
		print repr(e)

	finally:
                
    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


