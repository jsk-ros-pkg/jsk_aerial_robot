#!/usr/bin/env python3
import tf
import rospy
import numpy as np
from scipy.optimize import nnls
from std_msgs.msg import Empty
from spinal.msg import PwmTest
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs import Int8

from visualization_msgs.msg import Marker

class Haptics:
    def __init__(self):
        self.motor_output = 0.5

        # Flight state
        # self.flight_state_sub = rospy.Subscriber('/quadrotor/flight_state', UInt8, self.flight_state_cb)
        # self.flight_state_msg = UInt8()
        # self.flight_state_flag = False

        # 0: stop 1: manual 2: automatic
        self.control_mode = 0

        # Perching state
        # self.perching_state_sub = rospy.Subscriber('/perching_state', UInt8, self.perching_state_cb)
        # self.perching_state_msg = UInt8()
        # self.deperching_pub = rospy.Publisher('/perching_state',UInt8,queue_size = 1)
        # self.deperching_msg = UInt8()
        # self.deperching_msg.data = 2

        self.pressure_flag_sub = rospy.Subscriber('/pressure_flag', Int8, self.pressure_flag_cb)
        self.pressure_flag = Int8()

        self.odom_sub = rospy.Subscriber('/quadrotor/uav/cog/odom',Odometry,self.odom_cb)
        self.pose = Pose()
        self.euler = Vector3()

        # PWM
        self.PWM_pub = rospy.Publisher('/quadrotor/pwm_test', PwmTest, queue_size=1)
        self.PWM_msg = PwmTest()
        self.output = 0.0
        self.pwm_cal_pub = rospy.Publisher('/pwm_cal', Float32MultiArray, queue_size=1)
        self.pwm_cal_pub_msg = Float32MultiArray()

        # Commands
        self.arming_on_pub = rospy.Publisher('/quadrotor/teleop_command/start', Empty, queue_size=1)
        self.halt_pub = rospy.Publisher('/quadrotor/teleop_command/halt', Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher('/quadrotor/teleop_command/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/quadrotor/teleop_command/land', Empty, queue_size=1)

        # Joystick
        self.joy_sub = rospy.Subscriber('/quadrotor/joy', Joy, self.joy_cb)
        self.joy = Joy()

        #position
        self.pos_flag = True
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_pos = Vector3()

        # Flags
        self.cnt = 0

        # Rviz
        self.marker_pub = rospy.Publisher('/target_marker', Marker, queue_size=1)

    # def flight_state_cb(self, msg):
    #     self.flight_state_msg = msg
    #     self.flight_state_flag = self.flight_state_msg.data == 5

    def pressure_flag_cb(self, msg):
        self.pressure_flag = msg.data

    def odom_cb(self,msg):
        self.pose = msg.pose.pose
        self.height = self.pose.position.z
        quaternion_x = self.pose.orientation.x
        quaternion_y = self.pose.orientation.y
        quaternion_z = self.pose.orientation.z
        quaternion_w = self.pose.orientation.w
        euler = tf.transformations.euler_from_quaternion((quaternion_x,quaternion_y,quaternion_z,quaternion_w))
        self.euler = Vector3(x=euler[0],y=euler[1],z=euler[2])

    def joy_cb(self, msg):
        self.joy = msg
        buttons = self.joy.buttons

        if buttons[1] == 1:  # batu
            self.control_mode = 0
            rospy.loginfo("Switched to STOP mode")
        elif buttons[2] == 1:  # maru
            self.control_mode = 1
            rospy.loginfo("Switched to MANUAL mode")
        elif buttons[0] == 1:  # shikaku
            self.control_mode = 2
            rospy.loginfo("Switched to AUTO mode")
        elif buttons[3] == 1:  # triangle
            self.initialize()
            self.halt_pub.publish(Empty())
            rospy.loginfo("Emergency stop")

    def publish_target_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.target_x
        marker.pose.position.y = self.target_y
        marker.pose.position.z = 0.8  # 高さを少し持たせて見えやすく
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def publish_pwm(self, motor_indices, pwm_values):
        self.PWM_msg.motor_index = motor_indices
        self.PWM_msg.pwms = pwm_values
        self.PWM_pub.publish(self.PWM_msg)

    def cal_thrust_power(self, strength_norm):
        thrust = 4.0*abs(strength_norm) #max 9.0
        pwm = -0.000679 * thrust**2 + 0.044878* thrust + 0.5
        return min(pwm, 0.65)

    # def control_motor(self):
    #     rospy.loginfo("manual mode start")
    #     x = - self.joy.axes[0]
    #     y = self.joy.axes[1]
    #     print(x)

    #     #deadzone
    #     deadzone = 0.05
    #     x = x if abs(x) > deadzone else 0.0
    #     y = y if abs(y) > deadzone else 0.0

    #     if x ==0.0 and y==0.0:
    #         motor_index = list(range(4))
    #         motor_pwms = [0.5] * 4
    #     else:
    #         norm = np.linalg.norm([x,y])
    #         pwm_x = self.cal_thrust_power(x)
    #         pwm_y = self.cal_thrust_power(y)
    #         pwm_dia = self.cal_thrust_power(norm)

    #         active_motors = []
    #         if x > 0: active_motors.extend([(0, pwm_x), (1, pwm_x)])  # 右移動
    #         if x < 0: active_motors.extend([(2, pwm_x), (3, pwm_x)])  # 左移動
    #         if y > 0: active_motors.extend([(0, pwm_y), (3, pwm_y)])  # 前進
    #         if y < 0: active_motors.extend([(1, pwm_y), (2, pwm_y)])  # 後退

    #         pwm_dict = {}
    #         for idx, pwm in active_motors:
    #             if idx in pwm_dict:
    #                 pwm_dict[idx] = pwm_dia  # 重複があれば合成PWMに置き換え
    #             else:
    #                 pwm_dict[idx] = pwm

    #         motor_index = list(pwm_dict.keys())
    #         motor_pwms = list(pwm_dict.values())

    #     self.publish_pwm(motor_index, motor_pwms)
    #     rospy.loginfo(f"Publish index: {motor_index}, Pwms: {motor_pwms}")

    def control_motor(self):
        rospy.loginfo("Manual mode start")


        x = -self.joy.axes[0]
        y = self.joy.axes[1]

        # deadzone
        deadzone = 0.05
        x = x if abs(x) > deadzone else 0.0
        y = y if abs(y) > deadzone else 0.0

        if x == 0.0 and y == 0.0:
            motor_index = list(range(4))
            motor_pwms = [0.5] * 4
        else:
            input_vec = np.array([x, y])
            norm = np.linalg.norm(input_vec)

            if norm > 1e-6:
                direction = input_vec / norm  # 単位ベクトルに正規化
            else:
                direction = np.array([0.0, 0.0])
                norm = 0.0

            motor_dirs = np.array([
                [1, -1, -1, 1],
                [-1, -1, 1, 1]
            ])

            # 疑似逆行列による寄与率（非負制約付き最小二乗解）
            alpha, rnorm = nnls(motor_dirs.T, direction)

            alpha_scaled = alpha * norm

            motor_pwms = []
            for i in range(4):
                pwm = self.cal_thrust_power(alpha_scaled[i])
                motor_pwms.append(pwm)

            motor_index = list(range(4))
            self.pwm_cal_pub_msg = motor_pwms
            self.pwm_cal_pub.publish(self.pwm_cal_pub_msg)
            self.publish_pwm(motor_index, motor_pwms)
            rospy.loginfo(f"Publish index: {motor_index}, Pwms: {motor_pwms}")


    def control_motor_to_target(self):
        rospy.loginfo("AUTO mode start")
        if self.pos_flag:
            self.target_x = self.pose.position.x + 0.5
            self.target_y = self.pose.position.y + 0.5
            self.pos_flag = False
            rospy.loginfo(f"Target position set to: ({self.target_x}, {self.target_y})")

        dx = self.target_x - self.pose.position.x
        dy = self.target_y - self.pose.position.y
        rospy.loginfo(f"dx: {dx}, dy: {dy}")

        cos_yaw = np.cos(-self.euler.z)
        sin_yaw = np.sin(-self.euler.z)
        dx_body = dx * cos_yaw + dy * sin_yaw
        dy_body = -dx * sin_yaw + dy * cos_yaw

        if  - 0.1 < dx_body < 0.1 and - 0.1 < dy_body < 0.1:
            motor_index = list(range(4))
            motor_pwms = [0.5] * 4
        else:
            direction = np.array([dx_body,dy_body])
            norm = np.hypot(dx_body, dy_body)
            if norm > 1e-6:
                direction_normalized = direction / norm
            else:
                direction_normalized = np.array([0.0, 0.0])
            pwm_x = self.cal_thrust_power(direction_normalized[0])
            pwm_y = self.cal_thrust_power(direction_normalized[1])
            pwm_dia = self.cal_thrust_power(norm)

            active_motors = []
            if direction[0] > 0.1:  # +x方向
                active_motors.extend([(0, pwm_x), (3, pwm_x)])
            elif direction[0] < -0.1:  # -x方向
                active_motors.extend([(1, pwm_x), (2, pwm_x)])

            if direction[1] > 0.1:  # +y方向
                active_motors.extend([(2, pwm_y), (3, pwm_y)])
            elif direction[1] < -0.1:  # -y方向
                active_motors.extend([(0, pwm_y), (3, pwm_y)])

            pwm_dict = {}
            for idx, pwm in active_motors:
                if idx in pwm_dict:
                    pwm_dict[idx] = pwm_dia
                else:
                    pwm_dict[idx] = pwm

            motor_index = list(pwm_dict.keys())
            motor_pwms = list(pwm_dict.values())

        self.publish_pwm(motor_index, motor_pwms)
        rospy.loginfo(f"Publish index: {motor_index}, Pwms: {motor_pwms}")

    def stop_all(self):
        rospy.loginfo("Stopping all motors and pumps.")
        self.stop_pump()
        rospy.sleep(0.3)
        self.publish_pwm([0,1,2,3], [0.5,0.5,0.5,0.5])


    def main(self):
        rate = rospy.Rate(100)  # 100 Hz
        # bottom_ready = self.air_pressure_sensor1_msg.data >= 5 # (self.bottom_usual_pressure - 5)
        # joint_ready = self.air_pressure_sensor_msg.data >= 35 # (self.perching_pressure - 5)
        while not rospy.is_shutdown():
            rospy.loginfo(f"Mode: {self.control_mode} Output: {self.output}")
            if pressure_flag:
                if self.control_mode == 1:
                    self.control_motor()
                elif self.control_mode == 2:
                    self.control_motor_to_target()
                elif self.control_mode == 0:
                    self.stop_all()
            else:
                self.stop_all()
            
            self.publish_target_marker()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("Haptics")
    node = Haptics()
    node.main()
