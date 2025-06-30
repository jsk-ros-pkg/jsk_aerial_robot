#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8
from spinal.msg import PwmTest

class AirPressureController:
    def __init__(self):
        self.air_pressure_sub = rospy.Subscriber('/sensor', Int8, self.air_pressure_cb)
        self.air_pressure1_sub = rospy.Subscriber('/sensor_1', Int8, self.air_pressure1_cb)
        self.air_pressure_sensor_msg = Int8()
        self.air_pressure_sensor1_msg = Int8()

        self.pwm_pub = rospy.Publisher('/quadrotor/pwm_test', PwmTest, queue_size=1)
        self.output = 0.0

        self.max_pressure = 50
        self.perching_pressure = 40
        self.bottom_usual_pressure = 10

    def air_pressure_cb(self, msg):
        self.air_pressure_sensor_msg = msg

    def air_pressure1_cb(self, msg):
        self.air_pressure_sensor1_msg = msg

    def publish_pwm(self, motor_indices, pwm_values):
        msg = PwmTest()
        msg.motor_index = motor_indices
        msg.pwms = pwm_values
        self.pwm_pub.publish(msg)

    def cal_pressure(self, target_pressure, sensor_index):
        sensor_data = self.air_pressure_sensor_msg.data if sensor_index == 0 else self.air_pressure_sensor1_msg.data
        thre = 1
        error = target_pressure - sensor_data
        if error > thre:
            self.output = min(error * 0.02 + 0.3, 0.7)
        else:
            self.output = max(error * 0.01 + 0.32, 0.0)

    def adjust_pump(self):
        self.publish_pwm([4, 6], [self.output, self.output])

    # def start_pump(self):
    #     self.publish_pwm([4, 6], [0.5, 0.5])

    # def keep_work_pump(self):
    #     self.publish_pwm([4, 6], [0.3, 0.3])

    # def max_work_pump(self):
    #     self.publish_pwm([4, 6], [0.9, 0.9])

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
        thre= 1
        error = target_pressure - sensor_data
        if error > thre:
            self.output = min(error* 0.02 + 0.3, 0.7)  # Adjusted output to prevent overshoot
        else:
            self.output = max(error* 0.01 + 0.32, 0.0)

    def initialize(self):
        self.stop_pump()
        rospy.sleep(0.3)
        self.stop_solenoid_valve()
        self.start_solenoid_valve1()

    def adjust_pressure(self):
        self.stop_solenoid_valve1()
        if self.air_pressure_sensor1_msg.data <= self.bottom_usual_pressure:
            self.start_solenoid_valve()
            self.cal_pressure(self.bottom_usual_pressure, 1)
            self.adjust_pump()
        else:
            if self.air_pressure_sensor_msg.data <= self.perching_pressure:
                self.stop_solenoid_valve()
                self.cal_pressure(self.perching_pressure, 0)
                self.adjust_pump()
            else:
                self.stop_pump()

        if self.air_pressure_sensor1_msg.data >= self.bottom_usual_pressure and self.air_pressure_sensor_msg.data >= self.perching_pressure:
            self.stop_pump()


    def main(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rospy.loginfo(f"Air: {self.air_pressure_sensor_msg.data}, Bottom: {self.air_pressure_sensor1_msg.data}, Out: {self.output}")
            self.adjust_pressure()

            if self.air_pressure_sensor1_msg.data > 50 or self.air_pressure_sensor_msg.data > 60:
                rospy.logwarn("Overpressure! Initializing system.")
                self.publish_pwm([4, 6], [0.0, 0.0])
                self.publish_pwm([5, 7], [0.0, 0.0])

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("air_pressure_controller")
    node = AirPressureController()
    node.main()