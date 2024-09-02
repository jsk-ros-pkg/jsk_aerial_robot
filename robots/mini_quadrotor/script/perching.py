#!/usr/bin/env python

# This program is to check air actuator by controller 
import rospy
from std_msgs.msg import Int8
from spinal.msg import PwmState
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy

class Perching():
    def __init__(self):
        #Air Pressure Sensor
        self.air_pressure_sub = rospy.Subscriber('/sensor',Int8,self.air_pressure_sensor_cb)
        self.air_pressure_sensor_msg = Int8()
        self.ready_pressure = 20
        self.max_pressure = 40

        #PWMstate
        self.PWM_pub = rospy.Publisher('/pwm_indiv_test',PwmState,queue_size=1) #/quadrotor/pwm_indiv_test
        self.PWM_msg = PwmState()
        self.PWM = 0.6
        self.init_PWM = 0.6

        # MODE and LARGE and STBY
        rospy.sleep(0.3)
        self.PWM_msg.index = [4]
        self.PWM_msg.percentage = [1.0]

        self.halt_pub = rospy.Publisher('/quadrotor/teleop_command/halt',Empty, queue_size=1)
        #self.forceland_pub = rospy.Publisher('/quadrotor/teleop_command/force_landing',Empty, queue_size=1)

        self.joy_sub = rospy.Subscriber('/quadrotor/joy', Joy, self.joy_cb)
        self.joy = Joy()

        self.perching_flag = 0

        ##self.timer = rospy.Timer(rospy.Duration(0.05),self.timerCallback)

    def air_pressure_sensor_cb(self,msg):
        self.air_pressure_sensor_msg = msg

    def joy_cb(self,msg):
        self.joy = msg
        if self.joy.buttons[2] == 1:
            self.perching_flag = 1
        if self.joy.buttons[0] == 1:
            self.perching_flag = 2
        if self.joy.buttons[5] == 1:
            self.perching_flag = 3
        if self.joy.buttons[6] == 1:
            self.perching_flag = 4
        if self.joy.buttons[4] == 1:
            self.perching_flag = 0

    def start_pump(self):
        self.wait()
        # PWM of pump
        self.PWM_msg.index = [4]
        self.PWM_msg.percentage = [0.8]

        # print(self.PWM_msg.percentage)
        self.PWM_pub.publish(self.PWM_msg)

    def max_work_pump(self):
        self.wait()
        # PWM of pump
        self.PWM_msg.index = [4]
        self.PWM_msg.percentage = [1.0]

        # print(self.PWM_msg.percentage)
        self.PWM_pub.publish(self.PWM_msg)

    def stop_pump(self):
        self.wait()
        # PWM of pump
        self.PWM_msg.index = [4]
        self.PWM_msg.percentage = [0.0]

        # print(self.PWM_msg.percentage)
        self.PWM_pub.publish(self.PWM_msg)

    def start_solenoid_valve(self):
        self.wait()
        # PWM of solenoid valve
        self.PWM_msg.index = [5]
        self.PWM_msg.percentage = [1.0]

        print(self.PWM_msg.percentage)
        self.PWM_pub.publish(self.PWM_msg)

    def stop_solenoid_valve(self):
        self.wait()
        # PWM of solenoid valve
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
        print("start perching")
        # self.halt_pub.publish(Empty())
        # self.forceland_pub.publish(Empty())
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
        #rospy.sleep(self.perching_time_after_halt) #air pressure

    def deperching(self):
        rospy.sleep(2.0)
        self.stop_solenoid_valve()
        print("stop_solenoid")
        rospy.sleep(1.0)
        print("take_off!!!")



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

    # def main(self):
    #     while not rospy.is_shutdown():
    #         if self.ready_perching_flag:
    #             print("ready")
    #             rospy.sleep(0.3)
    #             self.stop_solenoid_valve()
    #             rospy.sleep(0.5)
    #             self.start_pump()
    #             if self.perching_flag:
    #                 rospy.sleep(0.5)
    #                 self.start_solenoid_valve()
    #                 rospy.sleep(0.1)
    #                 self.start_solenoid_valve()
    #                 rospy.sleep(0.1)
    #                 self.start_pump()
    #                 rospy.sleep(9.0)
    #                 self.stop_pump() #push button and stop 
    #                 rospy.sleep(1.0)
    #                 self.ready_perching_flag = False
    #                 break
    #             else:
    #                 continue

    #         if self.stop_flag:
    #             rospy.sleep(0.3)
    #             self.stop_solenoid_valve()
    #             rospy.sleep(0.3)
    #             self.stop_pump()
    #             print("stop")

if __name__ == '__main__':
    rospy.init_node("Perching")
    node = Perching()
    node.main()
