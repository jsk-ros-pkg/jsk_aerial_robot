#!/usr/bin/env python
import roslib; roslib.load_manifest('jsk_quadcopter')
import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from jsk_quadcopter.msg import fourAxisPidDebug

import sys, select, termios, tty, math

def callback(data):
        global start_flag
        if start_flag == True:
                global msg_cnt
                global throttle_sum
                global pitch_sum
                global roll_sum
                global yaw_sum
                msg_cnt = msg_cnt + 1
                throttle_sum = throttle_sum + data.throttle.posErrNoTransform * data.throttle.posErrNoTransform
                pitch_sum = pitch_sum + data.pitch.posErrTransform * data.pitch.posErrTransform
                roll_sum = roll_sum + data.roll.posErrTransform * data.roll.posErrTransform
                yaw_sum = yaw_sum + data.yaw.posErrNoTransform * data.yaw.posErrNoTransform

                throttle_rms = math.sqrt(throttle_sum / msg_cnt)
                pitch_rms = math.sqrt(pitch_sum / msg_cnt)
                roll_rms = math.sqrt(roll_sum / msg_cnt)
                yaw_rms = math.sqrt(yaw_sum / msg_cnt)
                rospy.loginfo("throttle rms:%f , pitch rms: %f, roll rms: %f, yaw rms: %f", throttle_rms, pitch_rms, roll_rms, yaw_rms)
        #rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)

        start_flag = False
	msg_cnt = 0
        yaw_sum = 0
        pitch_sum = 0
        roll_sum = 0
        throttle_sum = 0

        rospy.Subscriber("/controller/debug", fourAxisPidDebug, callback)



	rospy.init_node('keyboard_command')

        

	#gain=UInt16() 

	try:
		while(True):
			key = getKey()
			print "the key value is %d" % ord(key)
			# takeoff and landing
			if key == 's':
                                start_flag = True
			if key == 'h':
                                start_flag = False
			else:
				if (key == '\x03'):
					break
			rospy.sleep(0.001)

	except Exception as e:
		print e
		print repr(e)

	finally:
                
    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


