#!/usr/bin/env python

import sys
import rospy
from spinal.msg import ServoControlCmd
from pynput import keyboard
import time


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


rospy.init_node('servo_control_publisher')
pub = rospy.Publisher('/servo/target_states', ServoControlCmd, queue_size=10)
rospy.sleep(0.5)
pos_init = 2047
wave_large = 1024
wave_small = 200
current_pos_0 = pos_init
current_pos_1 = pos_init
running = False


print(msg)

# Motion Functions

def get_back():
    global current_pos_0, current_pos_1
    pub.publish(ServoControlCmd(index=[0,1,2,3], angles = [pos_init, pos_init, pos_init, pos_init]))
    current_pos_0 = pos_init
    current_pos_1 = pos_init
    

def move_forward(): # step move forward
    global running
    running = True
    for i in range(2):
        pub.publish(ServoControlCmd(index=[0], angles=[current_pos_0 + wave_large]))
        rospy.sleep(0.5)
        pub.publish(ServoControlCmd(index=[0], angles=[current_pos_0 - wave_large]))
        rospy.sleep(0.5)  # Sleep for 1 second between iterations
    pub.publish(ServoControlCmd(index=[0], angles=[current_pos_0]))


def turn_left():
    global running
    running = True
    for i in range(2):
        pub.publish(ServoControlCmd(index=[0], angles=[current_pos_0 - 1024]))
        rospy.sleep(0.4)
        pub.publish(ServoControlCmd(index=[0], angles=[current_pos_0]))
        rospy.sleep(0.4)
    pub.publish(ServoControlCmd(index=[0], angles=[current_pos_0]))    


def turn_right():
    global running
    running = True
    for i in range(2):
        pub.publish(ServoControlCmd(index=[0], angles=[current_pos_0 + 1024]))
        rospy.sleep(0.4)
        pub.publish(ServoControlCmd(index=[0], angles=[current_pos_0]))
        rospy.sleep(0.4)
    pub.publish(ServoControlCmd(index=[0], angles=[current_pos_0]))    
    

def move_forward_continue_cw():
    global current_pos_0, running
    current_pos_0 += 200
    #running = True
    pub.publish(ServoControlCmd(index=[0], angles=[current_pos_0]))
    rospy.sleep(0.05)


def move_forward_continue_ccw():
    global current_pos_0, running
    current_pos_0 -= 20480
    running = True
    pub.publish(ServoControlCmd(index=[0], angles=[current_pos_0]))
    rospy.sleep(0.3)
    

def tail_up():
    global current_pos_1
    if current_pos_1 > 3072:
        return
    current_pos_1 += 200
    print(current_pos_1)
    pub.publish(ServoControlCmd(index=[1], angles=[current_pos_1]))
    rospy.sleep(0.3)


def tail_down():
    global current_pos_1
    if  current_pos_1 <= 1024:
        return
    current_pos_1 -= 200
    pub.publish(ServoControlCmd(index=[1], angles=[current_pos_1]))
    rospy.sleep(0.3)
    

def on_press(key):
    try:
        if key.char == 'w':
            if not running:
                move_forward()
        elif key.char == 'r':
            if not running:
                move_forward_continue_cw()
        elif key.char == 'f':
            if not running:
                move_forward_continue_ccw()
        elif key.char == 'a':
            if not running:
                turn_left()
        elif key.char == 'd':
            if not running:
                turn_right() 
        elif key.char =='s':
            get_back()
        elif key.char == 'k':
            tail_up()
        elif key.char == 'i':
            tail_down()
    except AttributeError:
    	pass
    	
def on_release(key):
    global running
    try:
        if key.char == 'w':
            running = False
        elif key.char == 'r':
            running = False
        elif key.char == 'f':
            running = False
        elif key.char == 'a':
            running = False
        elif key.char == 'd':
            running = False
    except AttributeError:
        pass

with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    rospy.loginfo("Listening for keyboard events...")
    rospy.spin()
            

