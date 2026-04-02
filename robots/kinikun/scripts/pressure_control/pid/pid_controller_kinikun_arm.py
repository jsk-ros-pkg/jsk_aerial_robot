#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import message_filters
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from dynamic_reconfigure.server import Server
from kinikun.cfg import RelativePIDConfig

def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class PID:
    def __init__(self):
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.integral_limit = 5.0

    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0

        self.integral += error * dt
        self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)

        derivative = (error - self.prev_error) / dt

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.prev_error = error
        return output

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

class RelativePoseController:
    def __init__(self):
        rospy.init_node("relative_pose_pid")

        self.rate_hz = rospy.get_param("~rate", 50.0)
        self.pressure_base = 0.1
        self.dac_scale = 4551.1  # 4096 / 0.9
        self.pressure_max = rospy.get_param("~pressure_max", 0.4)


        # ===== Roll & Yaw targets =====
        self.target_roll = 0.0
        self.target_yaw  = 0.0

        # ===== Roll & Yaw PID =====
        self.pid_roll = PID()
        self.pid_yaw  = PID()

        self.last_time = rospy.Time.now()
        self.current_rpy = [0.0, 0.0, 0.0]
        self.relative_pos = [0.0, 0.0, 0.0]
        self.pub_cmd = rospy.Publisher("mpa_cmd", Quaternion, queue_size=1)
        self.pub_debug_rpy = rospy.Publisher("debug_relative_rpy", Vector3, queue_size=1)

        self.srv = Server(RelativePIDConfig, self.reconfigure_cb)

        body_sub = message_filters.Subscriber('/arm_kinniku/base/pose', PoseStamped)
        arm_sub  = message_filters.Subscriber('/arm_kinniku/ee/pose', PoseStamped)

        self.ts = message_filters.ApproximateTimeSynchronizer([body_sub, arm_sub], 10, 0.05)
        self.ts.registerCallback(self.pose_callback)

        rospy.loginfo("Relative Pose PID Controller Started (ROLL + YAW).")

    def reconfigure_cb(self, config, level):
        self.pid_roll.kp = config.roll_kp
        self.pid_roll.ki = config.roll_ki
        self.pid_roll.kd = config.roll_kd

        self.pid_yaw.kp = config.yaw_kp
        self.pid_yaw.ki = config.yaw_ki
        self.pid_yaw.kd = config.yaw_kd

        self.target_roll = math.radians(config.target_roll)
        self.target_yaw  = math.radians(config.target_yaw)

        self.pressure_base = config.pressure_base
        self.dac_scale = config.dac_scale

        rospy.loginfo(f"Config Updated: Tgt R={config.target_roll:.1f}deg, Y={config.target_yaw:.1f}deg")
        return config

    def pose_callback(self, body_msg, arm_msg):
        curr_time = rospy.Time.now()
        dt = (curr_time - self.last_time).to_sec()
        self.last_time = curr_time

        # World -> Body
        trans_b = [body_msg.pose.position.x, body_msg.pose.position.y, body_msg.pose.position.z]
        quat_b  = [body_msg.pose.orientation.x, body_msg.pose.orientation.y,
                   body_msg.pose.orientation.z, body_msg.pose.orientation.w]
        T_w_b = tft.concatenate_matrices(tft.translation_matrix(trans_b), tft.quaternion_matrix(quat_b))

        # World -> Arm
        trans_a = [arm_msg.pose.position.x, arm_msg.pose.position.y, arm_msg.pose.position.z]
        quat_a  = [arm_msg.pose.orientation.x, arm_msg.pose.orientation.y,
                   arm_msg.pose.orientation.z, arm_msg.pose.orientation.w]
        T_w_a = tft.concatenate_matrices(tft.translation_matrix(trans_a), tft.quaternion_matrix(quat_a))

        # T_b_a = (T_w_b)^-1 * T_w_a
        T_b_w = tft.inverse_matrix(T_w_b)
        T_b_a = tft.concatenate_matrices(T_b_w, T_w_a)

        # r=roll, p=pitch, y=yaw
        r, p, y = tft.euler_from_matrix(T_b_a, axes='sxyz')

        self.current_rpy = [r, p, y]
        self.pub_debug_rpy.publish(Vector3(r, p, y))

        err_roll = normalize_angle(self.target_roll - r)
        err_yaw  = normalize_angle(self.target_yaw  - y)

        u_roll = self.pid_roll.update(err_roll, dt)
        u_yaw  = self.pid_yaw.update(err_yaw, dt)

        p1 = clamp(self.pressure_base + u_roll, 0.0, self.pressure_max)
        p2 = clamp(self.pressure_base - u_roll, 0.0, self.pressure_max)
        p3 = clamp(self.pressure_base + u_yaw,  0.0, self.pressure_max)
        p4 = clamp(self.pressure_base - u_yaw,  0.0, self.pressure_max)

        v1 = clamp(p1 * self.dac_scale, 0, 4096)
        v2 = clamp(p2 * self.dac_scale, 0, 4096)
        v3 = clamp(p3 * self.dac_scale, 0, 4096)
        v4 = clamp(p4 * self.dac_scale, 0, 4096)

        cmd_msg = Quaternion(x=v1, y=v2, z=v3, w=v4)
        self.pub_cmd.publish(cmd_msg)

        if rospy.get_time() % 1.0 < 0.1:
            rospy.loginfo(
                f"Roll: Tgt={self.target_roll:.2f} Cur={r:.2f} u={u_roll:.2f} | "
                f"Yaw:  Tgt={self.target_yaw:.2f} Cur={y:.2f} u={u_yaw:.2f}"
            )

if __name__ == "__main__":
    try:
        RelativePoseController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
