#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt8, Int8, Empty
from spinal.msg import PwmTest
from sensor_msgs.msg import Joy

class Perching:
    def __init__(self):
        # Air pressure sensors
        self.air_pressure_sub = rospy.Subscriber('/sensor', Int8, self.air_pressure_sensor_cb)
        self.air_pressure1_sub = rospy.Subscriber('/sensor_1', Int8, self.air_pressure_sensor1_cb)
        self.air_pressure_sensor_msg = Int8()
        self.air_pressure_sensor1_msg = Int8()
        self.max_pressure = 50
        self.ready_pressure = 30
        self.perching_pressure = 30
        self.bottom_usual_pressure = 20

        # Flight state
        self.flight_state_sub = rospy.Subscriber('/quadrotor/flight_state', UInt8, self.flight_state_cb)
        self.flight_state_msg = UInt8()
        self.flight_state_flag = False

        # Perching state
        self.perching_state_sub = rospy.Subscriber('/perching_state', UInt8, self.perching_state_cb)
        self.perching_state_msg = UInt8()
        self.perching_flag = 0
        self.perching_sub_flag = False
        self.deperching_pub = rospy.Publisher('/perching_state',UInt8,queue_size = 1)
        self.deperching_msg = UInt8()
        self.deperching_msg.data = 2

        # PWM
        self.PWM_pub = rospy.Publisher('/quadrotor/pwm_test', PwmTest, queue_size=1)
        self.PWM_msg = PwmTest()
        self.output = 0.0

        # Commands
        self.arming_on_pub = rospy.Publisher('/quadrotor/teleop_command/start', Empty, queue_size=1)
        self.halt_pub = rospy.Publisher('/quadrotor/teleop_command/halt', Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher('/quadrotor/teleop_command/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=1)

        # Joystick
        self.joy_sub = rospy.Subscriber('/quadrotor/joy', Joy, self.joy_cb)
        self.joy = Joy()

        # Flags
        # self.prepare_bottom_flag = False
        self.mode = True #"True" is demo mode and "False" is test mode
        self.cnt = 0
        self.halt_flag = True

    def flight_state_cb(self, msg):
        self.flight_state_msg = msg
        self.flight_state_flag = self.flight_state_msg.data == 5

    def perching_state_cb(self, msg):
        self.perching_state_msg = msg
        if self.perching_state_msg.data == 1:
            self.perching_sub_flag = True

    def air_pressure_sensor_cb(self, msg):
        self.air_pressure_sensor_msg = msg

    def air_pressure_sensor1_cb(self, msg):
        self.air_pressure_sensor1_msg = msg

    def joy_cb(self, msg):
        self.joy = msg
        buttons = self.joy.buttons
        if buttons[4] == 1:  # left up
            self.perching_flag = 1
        elif buttons[7] == 1:  # right down
            self.perching_flag = 2
        elif buttons[5] == 1:  # right up
            self.perching_flag = 3
        elif buttons[6] == 1:  # left down

            self.perching_flag = 4
        elif buttons[3] == 1:  # triangle
            self.perching_flag = 5

    def publish_pwm(self, motor_indices, pwm_values):
        self.PWM_msg.motor_index = motor_indices
        self.PWM_msg.pwms = pwm_values
        self.PWM_pub.publish(self.PWM_msg)

    def start_pump(self):
        self.publish_pwm([4, 6], [0.5, 0.5])

    def keep_work_pump(self):
        self.publish_pwm([4, 6], [0.3, 0.3])

    def adjust_pump(self):
        self.publish_pwm([4, 6], [self.output, self.output])

    def max_work_pump(self):
        self.publish_pwm([4, 6], [0.8, 0.8])

    def stop_pump(self):
        self.publish_pwm([4, 6], [0.0, 0.0])

    def start_solenoid_valve(self):
        self.publish_pwm([5], [1.0])

    def stop_solenoid_valve(self):
        self.publish_pwm([5], [0.0])

    def start_solenoid_valve1(self):
        self.publish_pwm([7], [1.0])

    def stop_solenoid_valve1(self):
        self.publish_pwm([7], [0.0])

    def stop_solenoid_valve_all(self):
        self.publish_pwm([5,7], [0.0,0.0])
    def start_solenoid_valve_all(self):
        self.publish_pwm([5,7], [1.0,1.0])    

    def cal_pressure(self, target_pressure, sensor_index):
        sensor_data = self.air_pressure_sensor_msg.data if sensor_index == 0 else self.air_pressure_sensor1_msg.data
        if (target_pressure - sensor_data) > 0:
            self.output = max((target_pressure - sensor_data) * 0.01 + 0.28, 0.0)
        else:
            self.output = 0.0

    def initialize(self):
        self.stop_pump()
        self.stop_solenoid_valve()
        self.start_solenoid_valve1()

    def bottom_pressure_prepare(self):
        self.start_solenoid_valve_all()
        print("bottom prepare")
        if self.air_pressure_sensor1_msg.data < self.bottom_usual_pressure:
            self.cal_pressure(self.bottom_usual_pressure, 1)
            self.adjust_pump()
            # self.prepare_bottom_flag = False
        if self.air_pressure_sensor1_msg.data >= self.bottom_usual_pressure:
            self.stop_pump()
            # self.prepare_bottom_flag = True

    def ready_perching(self):
        if self.air_pressure_sensor1_msg.data <= self.ready_pressure:
            self.cal_pressure(self.ready_pressure, 1)
            self.adjust_pump()
        if self.air_pressure_sensor1_msg.data >= self.ready_pressure:
            rospy.logwarn("==============ready perching==================")
            self.stop_pump()
            self.stop_solenoid_valve_all()
            rospy.logwarn("joint to bottom!!!")
            self.perching_sub_flag = False
            self.perching_flag = 2

    def start_perching(self):
        # if self.flight_state_flag:
        rospy.logwarn("==============perching==================")
        if self.air_pressure_sensor_msg.data <= self.max_pressure:
            if self.air_pressure_sensor_msg.data >= 18 and self.halt_flag:
                self.max_work_pump()
                rospy.logwarn("halt")
                self.halt_pub.publish(Empty())
                self.max_work_pump()
                self.halt_flag = False
                #self.max_work_pump()
                self.flight_state_flag = False
            else:
                print("joint!!!")
                self.stop_solenoid_valve_all()
                self.max_work_pump()
            if self.air_pressure_sensor_msg.data == self.max_pressure:
                self.stop_pump()
                self.perching_flag = 3
        else:
            self.stop_pump()
            self.perching_flag = 3
        #     rospy.logwarn("==============not flight==================")

    def keep_perching(self):
        print(self.cnt)
        self.cnt +=1
        if self.cnt >= 20:
            rospy.sleep(1.0)
            print("deperch ready")
            self.perching_flag = 4
        else:
            if self.air_pressure_sensor1_msg.data <= 5:
                self.start_solenoid_valve()
                self.cal_pressure(5, 1)
                self.adjust_pump()
            elif self.air_pressure_sensor_msg.data < self.perching_pressure:
                self.cal_pressure(self.perching_pressure, 0)
                self.adjust_pump()
            else:
                self.stop_pump()

    def deperching(self):
        rospy.logwarn("==============deperching==================")
        self.deperching_pub.publish(self.deperching_msg)
        rospy.sleep(2.0)
        self.arming_on_pub.publish(Empty())
        rospy.sleep(2.0)
        self.stop_pump()
        self.stop_solenoid_valve()
        self.start_solenoid_valve1()

        if not self.flight_state_flag:
            rospy.sleep(1.0)
            self.start_solenoid_valve1()
            self.takeoff_pub.publish(Empty())
            rospy.sleep(12.0)
            self.land_pub.publish(Empty())

    def main(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rospy.loginfo(f"Perch flag: {self.perching_flag}, Air pressure: {self.air_pressure_sensor_msg.data}, Bottom pressure: {self.air_pressure_sensor1_msg.data}, Output: {self.output}")
            # if self.mode:
            if self.perching_flag == 1 or self.perching_sub_flag:
                if self.air_pressure_sensor1_msg.data < self.bottom_usual_pressure:
                    self.bottom_pressure_prepare()
                else:
                    #40kPa bottom -> max pump 
                    self.ready_perching()
            elif self.perching_flag == 2:
                # halt and perching
                self.start_perching()
            elif self.perching_flag == 3:
                self.keep_perching()
            elif self.perching_flag == 4:
                self.deperching()
                self.perching_flag = 0
            elif self.perching_flag == 5:
                self.initialize()
            else:
                self.bottom_pressure_prepare()
            # else:
            #     if self.perching_flag == 1:
            #             self.ready_perching()
            #     elif self.perching_flag == 2:
            #         self.start_perching()
            #     elif self.perching_flag == 3:
            #         self.keep_perching()
            #     elif self.perching_flag == 4:
            #         self.deperching()
            #         self.perching_flag = 0
            #     else:
            #         self.initialize()

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("Perching")
    node = Perching()
    node.main()
