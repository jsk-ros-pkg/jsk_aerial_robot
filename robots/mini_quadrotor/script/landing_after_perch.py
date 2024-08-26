#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8
from spinal.msg import PwmState
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy

class Perching():
    def __init__(self):
        #Air pressure sensor
        self.air_pressure_sub = rospy.Subscriber('/sensor1',UInt8,self.air_pressure_sensor_cb)
        self.air_pressure_sensor_msg = UInt8()
        self.max_pressure = 50
        self.ready_pressure = 30

        #flight state
        self.flight_state_sub = rospy.Subscriber('/quadrotor/flight_state',UInt8,self.flight_state_cb)
        self.flight_state_msg = UInt8()
        self.flight_state_flag = False

        #perching state
        self.perching_state_sub = rospy.Subscriber('/perching_state',UInt8,self.perching_state_cb)
        self.perching_state_msg = UInt8()
        self.perching_flag = False

        #PWM
        self.PWM_pub = rospy.Publisher('/quadrotor/pwm_indiv_test',PwmState,queue_size=1)
        self.PWM_msg = PwmState()
        self.PWM = 0.6
        self.init_PWM = 0.6

        # MODE and LARGE and STBY
        self.PWM_msg.index = [5]
        self.PWM_msg.percentage = [1.0]
        self.arming_on_pub = rospy.Publisher('/quadrotor/teleop_command/start',Empty,queue_size=1)
        self.halt_pub = rospy.Publisher('/quadrotor/teleop_command/halt',Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher('/quadrotor/teleop_command/takeoff',Empty, queue_size=1)

        self.joy_sub = rospy.Subscriber('/quadrotor/joy', Joy, self.joy_cb)
        self.joy = Joy()
        self.perching_flag = 0

        self.takeoff_wainting_time = 5.0
        self.perching_time_before_halt = 1.0
        self.perching_time_after_halt = 4.0
        ##self.timer = rospy.Timer(rospy.Duration(0.05),self.timerCallback)

    #perching flag == 0 #initialize
    #perching flag == 1 #ready
    #perching flag == 2 #start_perching
    #perching flag == 3 #keep_perching
    #perching flag == 4 #deperching
    def flight_state_cb(self,msg):
        self.flight_state_msg = msg
        if self.flight_state_msg.data == 5: #2
            self.flight_state_flag = True


    def perching_state_cb(self,msg):
        self.perching_state_msg = msg
        if self.perching_state_msg.data == 1:
            self.perching_flag = 1

    def air_pressure_sensor_cb(self,msg):
        self.air_pressure_sensor_msg = msg


    def joy_cb(self,msg):
        self.joy = msg
        if self.joy.buttons[4] == 1: #leftup
            self.perching_flag = 1
        if self.joy.buttons[7] == 1: #rightdown
            self.perching_flag = 2
        if self.joy.buttons[5] == 1: #rightup
            self.perching_flag = 3
        if self.joy.buttons[6] == 1: #leftdown
            self.perching_flag = 4
        if self.joy.buttons[3] == 1: #triangle
            self.perching_flag = 0

    def start_pump(self):
        # PWM of pump
        self.wait()
        self.PWM_msg.index = [4]
        self.PWM_msg.percentage = [0.8]

        print(self.PWM_msg.percentage)
        self.PWM_pub.publish(self.PWM_msg)

    def max_work_pump(self):
        self.wait()
        # PWM of pump
        self.PWM_msg.index = [4]
        self.PWM_msg.percentage = [1.0]

        # print(self.PWM_msg.percentage)
        self.PWM_pub.publish(self.PWM_msg)


    def stop_pump(self):
        # PWM of pump
        self.wait()
        self.PWM_msg.index = [4]
        self.PWM_msg.percentage = [0.0]

        print(self.PWM_msg.percentage)
        self.PWM_pub.publish(self.PWM_msg)

    def start_solenoid_valve(self):
        # PWM of solenoid valve
        self.wait()
        self.PWM_msg.index = [5]
        self.PWM_msg.percentage = [1.0]

        print(self.PWM_msg.percentage)
        self.PWM_pub.publish(self.PWM_msg)

    def stop_solenoid_valve(self):
        # PWM of solenoid valve
        self.wait()
        self.PWM_msg.index = [5]
        self.PWM_msg.percentage = [0.0]

        print(self.PWM_msg.percentage)
        self.PWM_pub.publish(self.PWM_msg)

    def wait(self):
        rospy.sleep(0.3)


    def initialize(self):
        print("stop perching")
        self.stop_solenoid_valve()
        self.stop_pump()
        #add bottom actuator

    def ready_perching(self):
        print("ready perching")
        # rospy.sleep(2.0) #2.0 #air pressure
        if self.air_pressure_sensor_msg.data < self.ready_pressure:
            self.start_solenoid_valve()
            self.start_pump() #it is better to use [if] to force stop
        else:
            self.stop_pump()
            self.perching_flag = 2

    def start_perching(self):
        if self.flight_state_flag:
            print("start perching")
            self.halt_pub.publish(Empty())
            self.flight_state_flag = False
        else:
            print("not flight")
        print("halt")
        if self.air_pressure_sensor_msg.data < self.max_pressure:
            self.start_solenoid_valve()
            self.max_work_pump()
        else:
            self.stop_pump()
            self.perching_flag = 3

    def keep_perching(self):
        print("keep perching")
        self.start_solenoid_valve()
        if self.air_pressure_sensor_msg.data < self.max_pressure:
            self.start_pump()
        else:
            self.stop_pump()

    def deperching(self):
        self.arming_on_pub.publish(Empty())
        rospy.sleep(2.0)
        self.stop_solenoid_valve()
        print("stop_solenoid")
        if not self.flight_state_flag:
            print("take_off!!!")
            rospy.sleep(1.0)
            self.takeoff_pub.publish(Empty())
        else:
            print("not take off")

    def main(self):
        while not rospy.is_shutdown():
            rospy.loginfo("perch: %s",self.perching_flag)
            rospy.loginfo("air_pressure: %s",self.air_pressure_sensor_msg.data)
            #print(self.perching_state_msg)
            #else:
               # if self.ready_perching_flag:
            if self.perching_flag == 1:
                self.ready_perching()
            elif self.perching_flag == 2:
                self.start_perching()
            elif self.perching_flag == 3:
                self.keep_perching()
            elif self.perching_flag == 4:
                self.deperching()
                self.perching_flag = 0
            else:
                self.initialize()

if __name__ == '__main__':
    rospy.init_node("Perching")
    node = Perching()
    node.main()
