#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8
from spinal.msg import PwmState
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy

class Perching():
    def __init__(self):
        self.flight_state_sub = rospy.Subscriber('/quadrotor/flight_state',UInt8,self.flight_state_cb)
        self.flight_state_msg = UInt8()
        self.flight_state_flag = False

        self.perching_state_sub = rospy.Subscriber('/quadrotor/perching_state',UInt8,self.perching_state_cb)
        self.perching_state_msg = UInt8()
        self.perching_state_flag = False

        self.PWM_pub = rospy.Publisher('/quadrotor/pwm_indiv_test',PwmState,queue_size=1)
        self.PWM_msg = PwmState()
        self.PWM = 0.6
        self.init_PWM = 0.6
        
        # MODE and LARGE and STBY
        self.PWM_msg.index = [5]
        self.PWM_msg.percentage = [1.0]

        self.halt_pub = rospy.Publisher('/quadrotor/teleop_command/halt',Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher('/quadrotor/teleop_command/takeoff',Empty, queue_size=1)
        #self.forceland_pub = rospy.Publisher('/quadrotor/teleop_command/forcelanding',Empty, queue_size=1)

        self.joy_sub = rospy.Subscriber('/quadrotor/joy', Joy, self.joy_cb)
        self.joy = Joy()
        self.ready_perching_flag = False
        self.perching_flag = False
        self.stop_flag = False

        self.takeoff_wainting_time = 5.0
        self.perching_time_before_halt = 1.5
        self.perching_time_after_halt = 2.0
        ##self.timer = rospy.Timer(rospy.Duration(0.05),self.timerCallback)

    def flight_state_cb(self,msg):
        self.flight_state_msg = msg

    def perching_state_cb(self,msg):
        self.perching_state_msg = msg
        if self.perching_state_msg == 1:
            self.perching_flag = True

    def joy_cb(self,msg):
        self.joy = msg
        if self.joy.buttons[2] == 1:
            self.perching_flag = True
        else:
            self.perching_flag = False

        if self.joy.buttons[0] == 1:
            self.ready_perching_flag = True
        if self.joy.buttons[7] == 1:
            self.stop_flag = True
    def flight_state(self):
        if self.flight_state_msg.data == 2:
            self.flight_state_flag = True
        else:
            self.flight_state_flag = False


    def start_pump(self):
        # PWM of pump
        self.PWM_msg.index = [4]
        self.PWM_msg.percentage = [0.8]

        print(self.PWM_msg.percentage)
        self.PWM_pub.publish(self.PWM_msg)

    def stop_pump(self):
        # PWM of pump
        self.PWM_msg.index = [4]
        self.PWM_msg.percentage = [0.0]

        print(self.PWM_msg.percentage)
        self.PWM_pub.publish(self.PWM_msg)

    def start_solenoid_valve(self):
        # PWM of solenoid valve
        self.PWM_msg.index = [6]
        self.PWM_msg.percentage = [1.0]

        print(self.PWM_msg.percentage)
        self.PWM_pub.publish(self.PWM_msg)

    def stop_solenoid_valve(self):
        # PWM of solenoid valve
        self.PWM_msg.index = [6]
        self.PWM_msg.percentage = [0.0]

        print(self.PWM_msg.percentage)
        self.PWM_pub.publish(self.PWM_msg)

    def wait(self):
        rospy.sleep(0.3)


    def main(self):
        while not rospy.is_shutdown():
            self.flight_state()
            if self.flight_state_flag:
                rospy.sleep(self.takeoff_wainting_time)
                self.stop_solenoid_valve()
                print("stop_solenoid")
                rospy.sleep(1.0)
                self.takeoff_pub.publish(Empty())
                break
            else:
                # if self.ready_perching_flag:
                if self.perching_state_flag:
                    print("ready perching")
                    self.wait()
                    self.stop_solenoid_valve()
                    self.wait()
                    self.start_pump() 
                    # if self.perching_flag:
                    print("start perching")
                    self.wait()
                    self.start_solenoid_valve()
                    rospy.sleep(self.perching_time_before_halt) #2.0
                    self.halt_pub.publish(Empty())
                    #self.forceland_pub.publish(Empty())
                    print("halt")
                    rospy.sleep(0.1)
                    self.start_solenoid_valve()
                    rospy.sleep(0.1)
                    self.start_pump()
                    rospy.sleep(self.perching_time_after_halt)
                    self.stop_pump() #push button and stop
                    self.wait()
                    self.ready_perching_flag = False
                elif self.stop_flag:
                    print("stop perching")
                    rospy.sleep(0.1)
                    self.stop_solenoid_valve() #push button and stop
                    rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node("Perching")
    node = Perching()
    node.main()
