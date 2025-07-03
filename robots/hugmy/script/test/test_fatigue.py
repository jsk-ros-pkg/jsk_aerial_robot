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
        self.bottom_pressure = 0
        self.joint_pressure = 20

        self.perching_flag = 0

        # PWM
        self.PWM_pub = rospy.Publisher('/pwm_test', PwmTest, queue_size=1)
        self.PWM_msg = PwmTest()
        self.output = 0.0

        # Joystick
        self.joy_sub = rospy.Subscriber('/quadrotor/joy', Joy, self.joy_cb)
        self.joy = Joy()

        # Flags
        # self.prepare_bottom_flag = False

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
        self.publish_pwm([4, 6], [0.9, 0.9])

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
            self.output = (target_pressure - sensor_data) * 0.01 + 0.25
        else:
            self.output = 0.0

    def change_pressure(self):
        if self.air_pressure_sensor_msg.data < self.joint_pressure:
            rospy.loginfo("joint")
            self.stop_solenoid_valve()
            self.cal_pressure(self.joint_pressure, 1)
            self.adjust_pump()
        else:
            self.stop_pump()
            #self.start_solenoid_valve()
            # if self.air_pressure_sensor1_msg.data < self.bottom_pressure:
            #     rospy.loginfo("bottom")
            #     self.start_solenoid_valve()
            #     self.cal_pressure(self.bottom_pressure, 1)
            #     self.adjust_pump()
            # else:

    def initialize(self):
        self.stop_pump()
        self.stop_solenoid_valve()
        self.start_solenoid_valve1()

    # def bottom_pressure_prepare(self):
    #     self.start_solenoid_valve_all()
    #     rospy.loginfo("bottom prepare")
    #     if self.air_pressure_sensor1_msg.data < self.bottom_usual_pressure:
    #         self.cal_pressure(self.bottom_usual_pressure, 1)
    #         self.adjust_pump()
    #         # self.prepare_bottom_flag = False
    #     elif self.air_pressure_sensor1_msg.data >= self.bottom_usual_pressure:
    #         self.stop_pump()
    #         # self.prepare_bottom_flag = True

    # def ready_perching(self):
    #     if self.air_pressure_sensor1_msg.data < self.ready_pressure:
    #         self.cal_pressure(self.ready_pressure, 1)
    #         self.adjust_pump()
    #     elif self.air_pressure_sensor1_msg.data >= self.ready_pressure:
    #         rospy.logwarn("==============ready perching==================")
    #         self.stop_pump()
    #         self.stop_solenoid_valve_all()
    #         rospy.logwarn("joint to bottom!!!")
    #         rospy.sleep(2.0)
    #         self.perching_sub_flag = False
    #         self.perching_flag = 2

    # def start_perching(self):
    #     # if self.flight_state_flag:
    #     rospy.logwarn("==============perching==================")
    #     if self.air_pressure_sensor_msg.data <= self.max_pressure:
    #         if self.air_pressure_sensor_msg.data >= 21 and self.halt_flag:
    #             self.max_work_pump()
    #             rospy.logwarn("halt")
    #             self.halt_pub.publish(Empty())
    #             self.max_work_pump()
    #             self.halt_flag = False
    #             #self.max_work_pump()
    #             self.flight_state_flag = False
    #         else:
    #             rospy.loginfo("joint!!!")
    #             self.stop_solenoid_valve_all()
    #             self.max_work_pump()
    #         if self.air_pressure_sensor_msg.data == self.max_pressure:
    #             self.stop_pump()
    #             self.perching_flag = 3
    #     else:
    #         self.stop_pump()
    #         self.perching_flag = 3
    #     #     rospy.logwarn("==============not flight==================")

    # def keep_perching(self):
    #     rospy.loginfo(self.cnt)
    #     self.cnt +=1
    #     if self.cnt >= 50:
    #         rospy.sleep(1.0)
    #         rospy.loginfo("deperch ready")
    #         self.perching_flag = 4
    #     else:
    #         if self.air_pressure_sensor1_msg.data <= 5:
    #             self.start_solenoid_valve()
    #             self.cal_pressure(5, 1)
    #             self.adjust_pump()
    #         elif self.air_pressure_sensor_msg.data < self.perching_pressure:
    #             self.cal_pressure(self.perching_pressure, 0)
    #             self.adjust_pump()
    #         else:
    #             self.stop_pump()

    # def deperching(self):
    #     rospy.logwarn("==============deperching==================")
    #     self.deperching_pub.publish(self.deperching_msg)
    #     rospy.sleep(2.0)
    #     self.arming_on_pub.publish(Empty())
    #     rospy.sleep(2.0)
    #     self.stop_pump()
    #     self.stop_solenoid_valve()
    #     self.start_solenoid_valve1()

    #     if not self.flight_state_flag:
    #         rospy.sleep(1.0)
    #         self.start_solenoid_valve1()
    #         self.takeoff_pub.publish(Empty())
    #         rospy.sleep(12.0)
    #         self.land_pub.publish(Empty())

    def main(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rospy.loginfo(f"Air pressure: {self.air_pressure_sensor_msg.data}, Bottom pressure: {self.air_pressure_sensor1_msg.data}, Output: {self.output}")
            # if self.mode:
            print(self.perching_flag)
            if self.perching_flag == 2:
                self.change_pressure()
            elif self.perching_flag == 5:
                self.stop_pump()

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("Perching")
    node = Perching()
    node.main()
