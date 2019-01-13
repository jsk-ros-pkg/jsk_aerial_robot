#!/usr/bin/env python
import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from std_msgs.msg import UInt8

import sys, select, termios, tty

msg = """

    r: motor arming (please do this before takeoff)
    t: takeoff (this can be received by robot only after motor arming)
    l: landing
    f: force landing (without xy position control, robot will descend slowly)
    h: halt (stop motor immediately )
    p: switch to position control mode
    v: switch to velocity control mode

    please don't have caps lock on.
    CTRL+c to quit
"""

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
	land_pub = rospy.Publisher('/teleop_command/land', Empty, queue_size=1)
        halt_pub = rospy.Publisher('/teleop_command/halt', Empty, queue_size=1)
	start_pub = rospy.Publisher('/teleop_command/start', Empty, queue_size=1)
	takeoff_pub = rospy.Publisher('/teleop_command/takeoff', Empty, queue_size=1)
        force_landing_pub = rospy.Publisher('/teleop_command/force_landing', Empty, queue_size=1)
	ctrl_mode_pub = rospy.Publisher('/teleop_command/ctrl_mode', Int8, queue_size=1)
        motion_start_pub = rospy.Publisher('task_start', Empty, queue_size=1)

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
			if key == 'f':
			        force_landing_pub.publish(Empty())
			if key == 't':
				takeoff_pub.publish(Empty())
			if key == 'u':
				stair_pub.publish(Empty())
			if key == 'x':
                                motion_start_pub.publish()
                        if key == 'v':
				comm.data = 1
				ctrl_mode_pub.publish(comm)
                        #new function for control mode changing
			if key == 'p':
				comm.data = 0
				ctrl_mode_pub.publish(comm)
                        if (key == '\x03'):
				break
                        '''
                        if ord(key) == 27:
                                key = getKey()
                                key = getKey()
                                if ord(key) == 68:
                                        print "68"
                                else:
			if ord(key) in move_bindings.keys():
				msg ="""clear"""
			else:
				if (key == '\x03'):
					break
                        '''
			rospy.sleep(0.001)

	except Exception as e:
		print e
		print repr(e)

	finally:
                
    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


