#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
from spinal.msg import PwmTest
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

class Perching():
    def __init__(self):
        #Air pressure sensor
        self.air_pressure_sub = rospy.Subscriber('/sensor',Int8,self.air_pressure_sensor_cb)
        self.air_pressure1_sub = rospy.Subscriber('/sensor_1',Int8,self.air_pressure_sensor1_cb)
        self.joy_sub = rospy.Subscriber('/quadrotor/joy',Joy,self.joy_cb)
        self.joy = Joy()
        self.stable_sub = rospy.Subscriber('/stable',Bool,self.air_stable_cb) 
        self.air_pressure_sensor_msg = Int8()
        self.air_pressure_sensor1_msg = Int8()
        self.stop_flag = False
        self.max_pressure = 50
        self.ready_pressure = 30

        #PWM
        self.PWM_pub = rospy.Publisher('/pwm_test',PwmTest,queue_size=1)
        self.stop_pub = rospy.Publisher('/stop',Bool,queue_size=1)
        self.PWM_msg = PwmTest()
        self.PWM = 0.6
        self.init_PWM = 0.6

        self.perching_flag = 0
        self.output = 0.0
        self.base_pressure = 50


        # self.takeoff_wainting_time = 5.0
        # self.perching_time_before_halt = 1.0
        # self.perching_time_after_halt = 4.0
        ##self.timer = rospy.Timer(rospy.Duration(0.05),self.timerCallback)

    #perching flag == 0 #initialize
    #perching flag == 1 #ready
    #perching flag == 2 #start_perching
    #perching flag == 3 #keep_perching
    #perching flag == 4 #deperching

    # def flight_state_cb(self,msg):
    #     self.flight_state_msg = msg
    #     if self.flight_state_msg.data == 5: #2
    #         self.flight_state_flag = True


    def perching_state_cb(self,msg):
        self.perching_state_msg = msg
        if self.perching_state_msg.data == 1:
            self.perching_flag = 1

    def air_pressure_sensor_cb(self,msg):
        self.air_pressure_sensor_msg = msg

    def air_pressure_sensor1_cb(self,msg):
        self.air_pressure_sensor1_msg = msg

    def air_stable_cb(self,msg):
         self.stable_flag = msg


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

    def start_pump_10(self):
        # PWM of pump
        self.wait()
        self.PWM_msg.motor_index = [4,6]
        self.PWM_msg.pwms = [0.21,0.21]

        print(self.PWM_msg.pwms)
        self.PWM_pub.publish(self.PWM_msg)


    def start_pump_20(self):
        # PWM of pump
        self.wait()
        self.PWM_msg.motor_index = [4,6]
        self.PWM_msg.pwms = [0.28,0.28]

        print(self.PWM_msg.pwms)
        self.PWM_pub.publish(self.PWM_msg)

    def start_pump_30(self):
        # PWM of pump
        self.wait()
        self.PWM_msg.motor_index = [4,6]
        self.PWM_msg.pwms = [0.25,0.25] #[0.3,0.3]

        print(self.PWM_msg.pwms)
        self.PWM_pub.publish(self.PWM_msg)

    def start_pump_40(self):
        # PWM of pump
        self.wait()
        self.PWM_msg.motor_index = [4,6]
        self.PWM_msg.pwms = [0.3,0.3]

        print(self.PWM_msg.pwms)
        self.PWM_pub.publish(self.PWM_msg)

    def start_pump_50(self):
        # PWM of pump
        self.wait()
        self.PWM_msg.motor_index = [4,6]
        self.PWM_msg.pwms = [0.25,0.25]

        print(self.PWM_msg.pwms)
        self.PWM_pub.publish(self.PWM_msg)

    def start_pump_60(self):
        # PWM of pump
        self.wait()
        self.PWM_msg.motor_index = [4,6]
        self.PWM_msg.pwms = [0.45,0.45]

        print(self.PWM_msg.pwms)
        self.PWM_pub.publish(self.PWM_msg)


    def keep_work_pump(self):
        self.wait()
        # PWM of pump
        self.PWM_msg.motor_index = [4,6]
        self.PWM_msg.pwms = [0.3,0.3]

        # print(self.PWM_msg.percentage)
        self.PWM_pub.publish(self.PWM_msg)

    def stop_pump(self):
        # PWM of pump
        self.wait()
        self.PWM_msg.motor_index = [4,6]
        self.PWM_msg.pwms = [0.0,0.0]

        print(self.PWM_msg.pwms)
        self.PWM_pub.publish(self.PWM_msg)

    def start_solenoid_valve(self):
        # PWM of solenoid valve
        self.wait()
        self.PWM_msg.motor_index = [5]
        self.PWM_msg.pwms = [1.0]

        print(self.PWM_msg.pwms)
        self.PWM_pub.publish(self.PWM_msg)

    def stop_solenoid_valve(self):
        # PWM of solenoid valve
        self.wait()
        self.PWM_msg.motor_index = [5]
        self.PWM_msg.pwms = [0.0]

        print(self.PWM_msg.pwms)
        self.PWM_pub.publish(self.PWM_msg)

    def start_solenoid_valve1(self):
        # PWM of solenoid valve
        self.wait()
        self.PWM_msg.motor_index = [7]
        self.PWM_msg.pwms = [1.0]

        print(self.PWM_msg.pwms)
        self.PWM_pub.publish(self.PWM_msg)

    def stop_solenoid_valve1(self):
        # PWM of solenoid valve
        self.wait()
        self.PWM_msg.motor_index = [7]
        self.PWM_msg.pwms = [0.0]

        print(self.PWM_msg.pwms)
        self.PWM_pub.publish(self.PWM_msg)

    def wait(self):
        rospy.sleep(0.3)

    def initialize(self):
        print("stop")
        self.stop_solenoid_valve()
        self.stop_pump()
        #add bottom actuator

    # def ready_perching(self):
    #     print("ready perching")
    #     # rospy.sleep(2.0) #2.0 #air pressure
    #     if self.air_pressure_sensor_msg.data < self.ready_pressure:
    #         self.start_solenoid_valve()
    #         self.start_pump() #it is better to use [if] to force stop
    #     else:
    #         self.stop_pump()
    #         self.perching_flag = 2

    # def start_perching(self):
    #     if self.flight_state_flag:
    #         print("start perching")
    #         self.halt_pub.publish(Empty())
    #         self.flight_state_flag = False
    #     else:
    #         print("not flight")
    #     print("halt")
    #     if self.air_pressure_sensor_msg.data < self.max_pressure:
    #         self.start_solenoid_valve()
    #         self.max_work_pump()
    #     else:
    #         self.stop_pump()
    #         self.perching_flag = 3

    # def keep_perching(self):
    #     print("keep perching")
    #     self.start_solenoid_valve()
    #     if self.air_pressure_sensor_msg.data < self.max_pressure:
    #         self.start_pump()
    #     else:
    #         self.stop_pump()

    # def deperching(self):
    #     self.arming_on_pub.publish(Empty())
    #     rospy.sleep(2.0)
    #     self.stop_solenoid_valve()
    #     print("stop_solenoid")
    #     if not self.flight_state_flag:
    #         print("take_off!!!")
    #         rospy.sleep(1.0)
    #         self.takeoff_pub.publish(Empty())
    #     else:
    #         print("not take off")

    def pressure_60mpa(self):
        print("60mpa")
        # rospy.sleep(2.0) #2.0 #air pressure
        self.start_solenoid_valve()
        if self.air_pressure_sensor_msg.data < 60:
            self.start_pump_60() #it is better to use [if] to force stop
        elif self.air_pressure_sensor_msg.data == 60:
            self.keep_work_pump()
        else:
            self.stop_pump()

    def pressure_50mpa(self):
        print("50mpa")
        # rospy.sleep(2.0) #2.0 #air pressure
        self.start_pump_50() #it is better to use [if] to force stop
        if self.air_pressure_sensor_msg.data < 50:
             self.start_solenoid_valve() #it is better to use [if] to force stop
        elif self.air_pressure_sensor_msg.data == 50:
            self.start_solenoid_valve()
        else:
            self.stop_solenoid_valve()

    def pressure_40mpa(self):
        print("40mpa")
        # rospy.sleep(2.0) #2.0 #air pressure
        self.start_pump_40()
        if self.air_pressure_sensor_msg.data < 40:
            self.start_solenoid_valve() #it is better to use [if] to force stop
        elif self.air_pressure_sensor_msg.data == 40:
            self.start_solenoid_valve()
        else:
            self.stop_solenoid_valve()

    def pressure_30mpa(self):
        print("30mpa")
        # rospy.sleep(2.0) #2.0 #air pressure
        self.start_pump_30()
        if self.air_pressure_sensor_msg.data < 30:
            self.start_solenoid_valve()
        elif self.air_pressure_sensor_msg.data == 30:
            self.start_solenoid_valve()
        else:
            self.stop_solenoid_valve()

    def pressure_20mpa(self):
        print("20mpa")
        # rospy.sleep(2.0) #2.0 #air pressure
        self.start_solenoid_valve()
        if self.air_pressure_sensor_msg.data < 20:
            self.start_pump_20() #it is better to use [if] to force stop
        elif self.air_pressure_sensor_msg.data == 20:
            self.keep_work_pump()
        else:
            self.stop_pump()

    def pressure_10mpa(self):
        print("10mpa")
        # rospy.sleep(2.0) #2.0 #air pressure
        self.start_solenoid_valve()
        if self.air_pressure_sensor_msg.data < 10:
            self.start_pump_10() #it is better to use [if] to force stop
        elif self.air_pressure_sensor_msg.data == 10:
            self.stop_pump()
        else:
            self.stop_pump()

    def bottom_pressure_5mpa(self):
        print("bottom_5mpa")
        # rospy.sleep(2.0) #2.0 #air pressure
        if self.air_pressure_sensor1_msg.data <= 5:
            self.start_solenoid_valve1() #it is better to use [if] to force stop
        elif self.air_pressure_sensor1_msg.data == 5:
            self.start_solenoid_valve1() #it is better to use [if] to force stop
        else:
            self.stop_solenoid_valve1()

    def bottom_pressure_10mpa(self):
        print("bottom_10mpa")
        # rospy.sleep(2.0) #2.0 #air pressure
        if self.air_pressure_sensor1_msg.data < 10:
            self.start_solenoid_valve1() #it is better to use [if] to force stop
        elif self.air_pressure_sensor1_msg.data == 10:
            self.stop_solenoid_valve1() #it is better to use [if] to force stop
        else:
            self.stop_solenoid_valve1()

    def bottom_pressure_20mpa(self):
        print("bottom_20mpa")
        # rospy.sleep(2.0) #2.0 #air pressure
        if self.air_pressure_sensor1_msg.data < 20:
            self.start_solenoid_valve1() #it is better to use [if] to force stop
        elif self.air_pressure_sensor1_msg.data == 20:
            self.stop_solenoid_valve1() #it is better to use [if] to force stop
        else:
            self.stop_solenoid_valve1()
    def bottom_pressure_30mpa(self):
        print("bottom_30mpa")
        # rospy.sleep(2.0) #2.0 #air pressure
        if self.air_pressure_sensor1_msg.data < 30:
            self.start_solenoid_valve1() #it is better to use [if] to force stop
        elif self.air_pressure_sensor1_msg.data == 30:
            self.stop_solenoid_valve1() #it is better to use [if] to force stop
        else:
            self.stop_solenoid_valve1()

    def bottom_pressure_40mpa(self):
        print("bottom_40mpa")
        # rospy.sleep(2.0) #2.0 #air pressure
        if self.air_pressure_sensor1_msg.data < 40:
            self.start_solenoid_valve1() #it is better to use [if] to force stop
        elif self.air_pressure_sensor1_msg.data == 40:
            self.stop_solenoid_valve1() #it is better to use [if] to force stop
        else:
            self.stop_solenoid_valve1()
    def bottom_pressure_50mpa(self):
        print("bottom_50mpa")
        # rospy.sleep(2.0) #2.0 #air pressure
        if self.air_pressure_sensor1_msg.data < 50:
            self.start_solenoid_valve1() #it is better to use [if] to force stop
        elif self.air_pressure_sensor1_msg.data == 50:
            self.stop_solenoid_valve1() #it is better to use [if] to force stop
        else:
            self.stop_solenoid_valve1()

    def calculate(self):
        self.output = 0.5 # 0.8 * (self.base_pressure - self.air_pressure_sensor_msg.data) / self.base_pressure 10_0.21, 20_0.25, 30_0.35 40_0.4, 50_0.42, 60_0.45 70_0.5

    def main(self):
        while not rospy.is_shutdown():
            self.bottom_pressure_50mpa()
            if self.perching_flag == 1:
                self.pressure_60mpa()
                self.stop_flag = False
            elif self.perching_flag == 2:
                self.pressure_40mpa()
                self.stop_flag = False
            elif self.perching_flag == 3:
                self.pressure_50mpa()
                self.stop_flag = False
            elif self.perching_flag == 4:
                self.pressure_30mpa()
                self.stop_flag = False
            else:
                self.initialize()
                self.stop_flag = True
                self.stop_pub.publish(self.stop_flag)

            # if(self.stable_flag == True):
            #     self.stop_pump()
            #     rospy.loginfo("true")
            # else:
            #     self.start_pump()
            rospy.loginfo("air_pressure: %s",self.air_pressure_sensor_msg.data)
            rospy.loginfo("flag: %s",self.perching_flag)
            rospy.loginfo("air_pressure_bottom: %s",self.air_pressure_sensor1_msg.data)

if __name__ == '__main__':
    rospy.init_node("Perching")
    node = Perching()
    node.main()
