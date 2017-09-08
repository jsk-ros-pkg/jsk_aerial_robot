#!/usr/bin/env python
import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from aerial_robot_base.msg import FlatnessPid, States

import sys, select, termios, tty, math

def pos_err_callback(data):
        global start_flag
        if start_flag == True:
                global msg_cnt
                global throttle_sum
                global pitch_sum
                global roll_sum
                global yaw_sum
                msg_cnt = msg_cnt + 1
                throttle_sum = throttle_sum + data.throttle.pos_err * data.throttle.pos_err
                pitch_sum = pitch_sum + data.pitch.pos_err * data.pitch.pos_err
                roll_sum = roll_sum + data.roll.pos_err * data.roll.pos_err
                yaw_sum = yaw_sum + data.yaw.pos_err * data.yaw.pos_err

                throttle_rms = math.sqrt(throttle_sum / msg_cnt)
                pitch_rms = math.sqrt(pitch_sum / msg_cnt)
                roll_rms = math.sqrt(roll_sum / msg_cnt)
                yaw_rms = math.sqrt(yaw_sum / msg_cnt)
                #rospy.loginfo("pos: throttle rms:%f , pitch rms: %f, roll rms: %f, yaw rms: %f \n", throttle_rms, pitch_rms, roll_rms, yaw_rms)
                #print u"pos: throttle rms:%f , pitch rms: %f, roll rms: %f, yaw rms: %f" % (throttle_rms, pitch_rms, roll_rms, yaw_rms)

def att_err_callback(data):
        global start_flag
        if start_flag == True:
                global att_msg_cnt
                global att_pitch_sum
                global att_roll_sum

                att_msg_cnt = att_msg_cnt + 1
                att_pitch_sum = att_pitch_sum + data.states[7].state[2].x * data.states[7].state[2].x
                att_roll_sum = att_roll_sum + data.states[6].state[2].x * data.states[6].state[2].x

                pitch_rms = math.sqrt(att_pitch_sum / att_msg_cnt)
                roll_rms = math.sqrt(att_roll_sum / att_msg_cnt)

                #rospy.loginfo("att: pitch rms: %f, roll rms: %f\n", pitch_rms, roll_rms)
                #print u"att: pitch rms: %f, roll rms: %f" % (pitch_rms, roll_rms)

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

        att_msg_cnt = 0
        att_pitch_sum = 0
        att_roll_sum = 0
        rospy.Subscriber("/controller/debug", FlatnessPid, pos_err_callback)
        rospy.Subscriber("/uav/full_state", States, att_err_callback)

	rospy.init_node('keyboard_command')


	try:
		while(True):
			key = getKey()
			print "the key value is %d" % ord(key)
			# takeoff and landing
			if key == 's':
                                start_flag = True
			if key == 'h':

                                if att_msg_cnt > 0:
                                        pitch_rms = math.sqrt(att_pitch_sum / att_msg_cnt)
                                        roll_rms = math.sqrt(att_roll_sum / att_msg_cnt)
                                        print u"att: pitch rms: %f, roll rms: %f" % (pitch_rms, roll_rms)

                                if msg_cnt > 0:
                                        throttle_rms = math.sqrt(throttle_sum / msg_cnt)
                                        pitch_rms = math.sqrt(pitch_sum / msg_cnt)
                                        roll_rms = math.sqrt(roll_sum / msg_cnt)
                                        yaw_rms = math.sqrt(yaw_sum / msg_cnt)
                                        print u"pos: throttle rms:%f , pitch rms: %f, roll rms: %f, yaw rms: %f" % (throttle_rms, pitch_rms, roll_rms, yaw_rms)

                                start_flag = False
                                msg_cnt = 0
                                yaw_sum = 0
                                pitch_sum = 0
                                roll_sum = 0
                                throttle_sum = 0
			else:
				if (key == '\x03'):
					break
			rospy.sleep(0.001)

	except Exception as e:
		print e
		print repr(e)

	finally:
                
    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


