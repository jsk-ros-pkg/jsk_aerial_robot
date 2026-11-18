#!/usr/bin/env python3

from __future__ import print_function # for print function in python2

import threading
import time

import rospy
import math

from spinal.msg import ServoControlCmd, ServoStates, ServoExtendedStates, ServoExtendedCmd, ServoExtendedCmds
from spinal.msg import Imu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class VerticalAuto():
    def __init__(self):

        self.control_lock = threading.RLock()

        self.joy_dead_zone = rospy.get_param('~joy_dead_zone', 0.1)
        self.max_val = rospy.get_param('~max_val', 250)
        self.servo_equ_angles = rospy.get_param('~servos_angle', 0)
        self.servo_cmd = rospy.get_param('~servo_cmd', [0, 0]) # 0 for servo, 1 for motor
        self.motor_equ_angles = rospy.get_param('~motor_angle', 0.0)
        self.motor_angle_raw = rospy.get_param('~motor_angle_raw', 0.0)
        self.motor_angle_ref = rospy.get_param('~motor_angle_ref', 720)
        self.imu_angle = rospy.get_param('imu_angle', 0.0)
        self.imu_angle_raw = rospy.get_param('imu_angle_raw', 0.0)
        self.tar_angle_ref = rospy.get_param('~tar_angle_ref', 0.0)
        self.servo_axis = 0.0

        self.motor_acc = 0.0
        self.motor_acc_rate = rospy.get_param('~motor_acc_rate', 1.0)
        self.motor_cruise_speed = 0.0
        self.motor_speed_axis_prev = 0.0
        self.motor_vari_enable = True # True: variable speed, False: constant speed
        self.motor_vari_cmd = rospy.get_param('~motor_vari_cmd', 0.0)
        self.motor_vari_rate = rospy.get_param('~motor_vari_rate', 0.8)
        self.motor_vari_min_rate = rospy.get_param('~motor_vari_min_rate', 0.05)
        self.motor_vari_dec_speed = rospy.get_param('~motor_vari_dec_speed', 6.0 * math.pi)
        self.motor_vari_speed = rospy.get_param('~motor_vari_speed', 10.0 * math.pi)
        self.motor_pos_control = False
        self.auto_control_mode = False
        self.auto_pattern = 'stop'
        self.auto_button_prev = 0
        self.turn_button_prev = 0
        self.fast_turn_tar_angle = 2048.0
        self.auto_start_time = rospy.Time.now()

        # Vertical task: servo side +1 -> 3072, side -1 -> 1024.
        self.vertical_auto_mode = False
        self.vertical_state = 'manual'
        self.vertical_side = -1
        self.vertical_reversals = 0
        self.vertical_reverse_limit = rospy.get_param('~auto_reversals', 6)
        self.vertical_motor_speed = rospy.get_param('~auto_motor_speed', 4.0 * math.pi)
        self.vertical_motor_direction = rospy.get_param('~auto_motor_direction', 1)
        self.vertical_hold_seconds = rospy.get_param('~auto_hold_seconds', 3.0)
        self.vertical_prepare_seconds = rospy.get_param('~auto_prepare_seconds', 4.0)
        self.vertical_prepare_fast_start = rospy.get_param('~auto_prepare_fast_start', False)
        self.vertical_up_prepare_seconds = rospy.get_param('~auto_up_prepare_seconds', 10.0)
        self.vertical_up_seconds = rospy.get_param('~auto_up_seconds', 20.0)
        self.vertical_reverse_seconds = rospy.get_param('~auto_reverse_seconds', 0.5)
        self.vertical_gait_timeout = rospy.get_param('~auto_gait_timeout', 20.0)
        self.vertical_reverse_timeout = rospy.get_param('~auto_reverse_timeout', 2.5)
        self.vertical_state_time = 0.0
        self.vertical_initial_reverse = False
        self.vertical_nose_armed = False
        self.vertical_peak_armed = False
        self.vertical_peak_time = None
        self.vertical_window_time = None
        self.vertical_decision_sample = 0
        self.vertical_decision_time = 0.0
        self.vertical_trigger_axis = 0
        self.vertical_trigger_mode = ''
        self.vertical_trigger_time = None
        self.vertical_trigger_latched = False

        self.joy_timeout = rospy.get_param('~joy_timeout', 0.5)
        self.feedback_timeout = rospy.get_param('~feedback_timeout', 0.5)
        self.joy_time = None
        self.servo_state_time = None
        self.motor_state_time = None
        self.imu_time = None
        self.imu_sample_time = None
        self.imu_sample = 0
        self.imu_valid = False
        self.imu_geometry_valid = False
        self.imu_nose_angle = 90.0
        self.imu_nose_rate = 0.0
        self.imu_roll_projection = 0.0
        self.imu_sin_roll = 0.0
        self.imu_sin_rate = 0.0
        self.imu_sin_acc = 0.0
        self.imu_previous_nose = 90.0
        self.imu_previous_sin = 0.0

        positive_parameters = [self.vertical_motor_speed, self.vertical_hold_seconds,
                               self.vertical_prepare_seconds, self.vertical_up_prepare_seconds,
                               self.vertical_up_seconds,
                               self.vertical_reverse_seconds, self.vertical_gait_timeout,
                               self.vertical_reverse_timeout, self.joy_timeout, self.feedback_timeout]
        if any(not math.isfinite(value) or value <= 0 for value in positive_parameters):
            raise ValueError('Vertical control times and speed must be finite and positive')
        if (isinstance(self.vertical_reverse_limit, bool)
                or not isinstance(self.vertical_reverse_limit, int) or self.vertical_reverse_limit < 1):
            raise ValueError('auto_reversals must be a positive integer')
        if self.vertical_motor_direction not in (-1, 1):
            raise ValueError('auto_motor_direction must be -1 or 1')
        if self.vertical_reverse_timeout <= self.vertical_reverse_seconds:
            raise ValueError('auto_reverse_timeout must exceed auto_reverse_seconds')

        self.servo_cmd_pub = rospy.Publisher('/servo/target_states', ServoControlCmd, queue_size=1)
        self.motor_cmd_pub = rospy.Publisher('/servo/extended_cmds', ServoExtendedCmds, queue_size=1)

        self.joy_sub = rospy.Subscriber('/joy', Joy, self._joyCallback)
        self.twist_sub = rospy.Subscriber('/cmd_vel', Twist, self._twistCallback)
        self.servo_pos_sub = rospy.Subscriber('/servo/states', ServoStates, self._servoStateCallback)
        self.motor_state_sub = rospy.Subscriber('/servo/extended_states', ServoExtendedStates, self._motorStateCallback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self._imuCallback)

        self.control_timer = rospy.Timer(rospy.Duration(0.02), self._controlLoop)
        rospy.on_shutdown(self.shutdown)


    def _servoStateCallback(self, msg):
        with self.control_lock:
            if not msg.servos or not math.isfinite(msg.servos[0].angle):
                return
            self.servo_equ_angles = msg.servos[0].angle % 4096
            self.servo_state_time = time.monotonic()

    def _imuCallback(self, msg):
        with self.control_lock:
            # IMU quaternion [x, y, z, w], body to world; +X forward, +Y left, +Z up.
            now = time.monotonic()
            quaternion = list(msg.quaternion)
            if len(quaternion) != 4 or not all(math.isfinite(value) for value in quaternion):
                self.imu_valid = False
                return
            norm = math.sqrt(sum(value * value for value in quaternion))
            if not math.isfinite(norm) or norm < 1e-6:
                self.imu_valid = False
                return
            x, y, z, w = [value / norm for value in quaternion]
            sample_time = msg.stamp.to_sec() if msg.stamp.to_sec() > 0 else now
            if self.imu_sample_time is not None and sample_time <= self.imu_sample_time:
                return

            self.imu_angle_raw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            self.imu_angle = self.normalize_angle(self.imu_angle_raw - self.tar_angle_ref)
            nose_z = 2.0 * (x * z - w * y)
            side_z = 2.0 * (y * z + w * x)
            back_z = 1.0 - 2.0 * (x * x + y * y)
            nose_angle = math.degrees(math.acos(max(-1.0, min(-nose_z, 1.0))))
            horizontal = math.hypot(side_z, back_z)
            geometry_valid = horizontal >= 0.20
            sin_roll = side_z / horizontal if geometry_valid else 0.0
            dt = sample_time - self.imu_sample_time if self.imu_sample_time is not None else None

            if dt is None or dt > self.feedback_timeout or not self.imu_valid:
                self.imu_nose_angle = nose_angle
                self.imu_nose_rate = 0.0
                self.imu_sin_roll = sin_roll
                self.imu_sin_rate = 0.0
                self.imu_sin_acc = 0.0
                self.vertical_peak_armed = False
                self.vertical_peak_time = None
                self.vertical_window_time = None
            else:
                # Quaternion differences replace the simulator's angular-velocity input.
                nose_rate = (nose_angle - self.imu_previous_nose) / dt
                sin_rate = ((sin_roll - self.imu_previous_sin) / dt
                            if geometry_valid and self.imu_geometry_valid else 0.0)
                self.imu_nose_angle += (1.0 - math.exp(-dt / 0.08)) * (nose_angle - self.imu_nose_angle)
                self.imu_nose_rate += (1.0 - math.exp(-dt / 0.04)) * (nose_rate - self.imu_nose_rate)
                self.imu_sin_roll += (1.0 - math.exp(-dt / 0.04)) * (sin_roll - self.imu_sin_roll)
                previous_rate = self.imu_sin_rate
                self.imu_sin_rate += (1.0 - math.exp(-dt / 0.08)) * (sin_rate - self.imu_sin_rate)
                self.imu_sin_acc += (1.0 - math.exp(-dt / 0.12)) * ((self.imu_sin_rate - previous_rate) / dt - self.imu_sin_acc)

            self.imu_roll_projection = side_z
            self.imu_previous_nose = nose_angle
            self.imu_previous_sin = sin_roll
            self.imu_geometry_valid = geometry_valid
            self.imu_time = now
            self.imu_sample_time = sample_time
            self.imu_sample += 1
            self.imu_valid = True
            if self.vertical_auto_mode and self.vertical_state == 'gait':
                self.update_vertical_peak(now)

    def _motorStateCallback(self, msg):
        with self.control_lock:
            if not msg.servos or not math.isfinite(msg.servos[0].angle):
                return
            self.motor_angle_raw = msg.servos[0].angle % 4096
            # Logical 2048 is raw motor_angle_ref; position targets use this corrected frame.
            self.motor_equ_angles = self.normalize_servo_angle(self.motor_angle_raw - self.motor_angle_ref + 2048.0)
            self.motor_state_time = time.monotonic()

    def initialize(self):
        self.tar_angle_ref = self.imu_angle_raw
        self.motor_angle_ref = self.motor_angle_raw
        self.motor_equ_angles = 2048.0
        rospy.loginfo("initialize fish: imu ref %s, motor ref %s", self.tar_angle_ref, self.motor_angle_ref)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def normalize_servo_angle(self, angle):
        while angle >= 4096:
            angle -= 4096
        while angle < 0:
            angle += 4096
        return angle

    def pos_control(self, servo_no, tar, rate):
        if servo_no == 1:
            pos = self.motor_equ_angles
            val = 2 * math.pi
            kp = 2.8
            deadzone = 50
        else:
            pos = self.servo_equ_angles
            val = self.max_val
            kp = 2.8
            deadzone = 50
        if math.fabs(tar - pos) > 2048:
            if tar-pos < 0:
                tar += 4096
            else:
                tar -= 4096
        if tar-pos < 0:
            direction = -1
        else:
            direction = 1


        if math.fabs(tar-pos) > 700:
            cmd = direction * rate * val
        elif math.fabs(tar-pos) > deadzone:
            cmd = (tar-pos)/2048 * kp * val
        else:
            cmd = 0.0
        self.servo_cmd[servo_no] = cmd

    def start_auto(self):
        self.motor_pos_control = False
        self.auto_control_mode = True
        self.auto_pattern = 'fast_start'
        self.auto_start_time = rospy.Time.now()
        rospy.loginfo("start auto mode")

    def start_fast_turn(self, tar_angle):
        self.motor_pos_control = False
        self.auto_control_mode = True
        self.auto_pattern = 'fast_turn'
        self.fast_turn_tar_angle = tar_angle
        self.auto_start_time = rospy.Time.now()
        rospy.loginfo("start fast turn")

    def start_cruise(self):
        self.auto_control_mode = False
        if not self.auto_pattern == 'cruise':
            rospy.loginfo("start cruise")
        self.auto_pattern = 'cruise'
        self.motor_pos_control = False
        self.motor_cruise_speed = 3.0 * math.pi
        self.servo_cmd[1] = self.motor_cruise_speed

    def motor_vari(self):
        speed = self.motor_cruise_speed
        if speed == 0:
            return 0.0

        phase = ((self.motor_equ_angles - 1024.0) % 2048.0) / 2048.0
        if speed < 0:
            phase = ((1024.0 - self.motor_equ_angles) % 2048.0) / 2048.0

        rate = max(0.0, min(self.motor_vari_rate, 0.99))
        min_rate = max(0.0, min(self.motor_vari_min_rate, rate))
        speed_range = max(self.motor_vari_speed - self.motor_vari_dec_speed, 0.001)
        speed_rate = max(0.0, min((math.fabs(speed) - self.motor_vari_dec_speed) / speed_range, 1.0))
        rate += (min_rate - rate) * speed_rate
        # Constant angular deceleration: speed squared is linear in angle.
        return speed * (math.sqrt(1.0 + rate * (1.0 - 2.0 * phase)) - 1.0)


    def stop_auto(self):
        self.vertical_auto_mode = False
        self.vertical_state = 'manual'
        self.vertical_trigger_time = None
        self.motor_pos_control = False
        self.auto_control_mode = False
        if not self.auto_pattern == 'stop':
            rospy.loginfo("motor stop.")
        self.auto_pattern = 'stop'
        self.stop_control()

    def manual_control(self):
        servo_tar = 2048.0 - self.servo_axis * 1024.0
        if servo_tar > 3072.0:
            servo_tar = 3072.0
        if servo_tar < 1024.0:
            servo_tar = 1024.0

        self.pos_control(0, servo_tar, 1.0)
        self.motor_cruise_speed += self.motor_acc * self.motor_acc_rate

    def stop_control(self):
        self.pos_control(0, 2048, 1.0)
        self.pos_control(1, 2048,1.0)

    def fast_start(self):
        t = (rospy.Time.now() - self.auto_start_time).to_sec()

        if t < 2.0:
            self.servo_cmd[0] = 250
            self.servo_cmd[1] = 2.0 * math.pi
        else:
            self.start_cruise()

    def fast_turn(self):
        t = (rospy.Time.now() - self.auto_start_time).to_sec()

        if t < 0.75:
            self.pos_control(0, 2048.0, 0.33)
            self.pos_control(1, 2048.0, 0.33)
        elif t < 1.0:
            self.pos_control(0, self.fast_turn_tar_angle, 1.0)
            self.pos_control(1, self.fast_turn_tar_angle, 3.0)
        else:
            self.start_cruise()

    def cruise(self):
        self.manual_control()

    def auto_control(self):
        if self.vertical_auto_mode:
            if self.auto_pattern == 'vertical_up_auto':
                self.vertical_up_control()
            else:
                self.vertical_control()
        elif self.auto_pattern == 'fast_start':
            self.fast_start()
        elif self.auto_pattern == 'fast_turn':
            self.fast_turn()
        else:
            self.auto_control_mode = False
            self.servo_cmd[0] = 0
            self.servo_cmd[1] = 0

    def publish_cmd(self):
        servo_msg = ServoControlCmd()
        servo_msg.index = [0]
        servo_msg.cmd.append(int(self.servo_cmd[0]))
        self.servo_cmd_pub.publish(servo_msg)

        motor = ServoExtendedCmd()
        motor.index = 0
        motor.mode = 1
        self.motor_vari_cmd = 0.0
        if self.auto_pattern == 'cruise' and not self.motor_pos_control:
            if self.motor_vari_enable:
                self.motor_vari_cmd = self.motor_vari()
            self.servo_cmd[1] = self.motor_cruise_speed + self.motor_vari_cmd
        motor.cmd = -self.servo_cmd[1]

        motor_msg = ServoExtendedCmds()
        motor_msg.stamp.secs = 0
        motor_msg.stamp.nsecs = 0
        motor_msg.servos.append(motor)
        self.motor_cmd_pub.publish(motor_msg)

    def feedback_ready(self, now):
        return (self.servo_state_time is not None and self.motor_state_time is not None
                and now - self.servo_state_time <= self.feedback_timeout
                and now - self.motor_state_time <= self.feedback_timeout)

    def update_vertical_trigger(self, now):
        if (self.joy_time is None or now - self.joy_time > self.joy_timeout
                or self.vertical_trigger_mode == ''):
            self.vertical_trigger_time = None
            return
        if self.vertical_trigger_latched:
            return
        if self.vertical_trigger_time is None:
            self.vertical_trigger_time = now
        if now - self.vertical_trigger_time >= self.vertical_hold_seconds:
            self.vertical_trigger_latched = True
            if not self.auto_control_mode:
                if self.vertical_trigger_mode == 'vertical_up_auto':
                    self.start_vertical_up_auto(now)
                else:
                    self.start_vertical_auto(now, -self.vertical_trigger_axis)

    def start_vertical_auto(self, now, side):
        if (not self.feedback_ready(now) or not self.imu_valid or self.imu_time is None
                or now - self.imu_time > self.feedback_timeout):
            rospy.logwarn('vertical auto needs fresh IMU, servo and motor feedback')
            return
        self.motor_pos_control = False
        self.auto_control_mode = True
        self.auto_pattern = 'vertical_auto'
        self.vertical_auto_mode = True
        self.vertical_side = side
        self.vertical_reversals = 0
        self.vertical_initial_reverse = True
        self.motor_acc = 0.0
        self.motor_cruise_speed = 0.0
        self.set_vertical_state('prepare', now)
        rospy.loginfo('start vertical auto: %s reversals', self.vertical_reverse_limit)

    def start_vertical_up_auto(self, now):
        if not self.feedback_ready(now):
            rospy.logwarn('vertical up auto needs fresh servo and motor feedback')
            return
        self.auto_control_mode = True
        self.auto_pattern = 'vertical_up_auto'
        self.vertical_auto_mode = True
        self.vertical_side = -1
        self.vertical_reversals = 0
        self.motor_acc = 0.0
        self.motor_cruise_speed = 0.0
        self.set_vertical_state('prepare', now)
        rospy.loginfo('start vertical up auto')

    def set_vertical_state(self, state, now):
        self.vertical_state = state
        self.vertical_state_time = now
        self.vertical_nose_armed = self.imu_nose_angle <= 88.0
        self.vertical_peak_armed = False
        self.vertical_peak_time = None
        self.vertical_window_time = None
        self.vertical_decision_sample = self.imu_sample
        self.vertical_decision_time = now
        self.motor_pos_control = (state == 'prepare'
                                  and (self.auto_pattern == 'vertical_up_auto'
                                       or not self.vertical_prepare_fast_start))
        rospy.loginfo('vertical auto: %s, side %s, reversals %s/%s',
                      state, self.vertical_side, self.vertical_reversals, self.vertical_reverse_limit)

    def finish_vertical_auto(self, reason):
        rospy.loginfo('finish vertical auto: %s; return to manual stop', reason)
        self.motor_acc = 0.0
        self.motor_cruise_speed = 0.0
        self.servo_axis = 0.0
        self.motor_pos_control = False
        self.stop_auto()

    def update_vertical_peak(self, now):
        # Filter before applying side, so switching side creates no derivative pulse.
        height = -self.vertical_side * self.imu_sin_roll
        rate = -self.vertical_side * self.imu_sin_rate
        minimum = math.sin(math.radians(25.0))
        if self.imu_geometry_valid and height >= minimum and rate >= 0.10:
            self.vertical_peak_armed = True
            self.vertical_peak_time = None
        if self.vertical_peak_armed and rate <= 0.0:
            self.vertical_peak_time = now if self.imu_geometry_valid and height >= minimum else None
            self.vertical_peak_armed = False
        if not self.imu_geometry_valid or height < 0.0:
            self.vertical_peak_armed = False
            self.vertical_peak_time = None

    def vertical_reverse_ready(self, now):
        height = -self.vertical_side * self.imu_sin_roll
        rate = -self.vertical_side * self.imu_sin_rate
        acceleration = -self.vertical_side * self.imu_sin_acc
        minimum_peak = math.sin(math.radians(30.0))
        observed_peak = (self.vertical_peak_time is not None
                         and 0.0 <= now - self.vertical_peak_time <= 0.08 and height >= minimum_peak)
        predicted_peak = False
        if rate > 0.0 and acceleration < -0.10:
            peak_time = -rate / acceleration
            peak_height = height + rate * peak_time + 0.5 * acceleration * peak_time * peak_time
            predicted_peak = (self.vertical_peak_armed and 0.0 <= peak_time <= 0.80
                              and abs(peak_time - 0.25) <= 0.10 and peak_height >= minimum_peak)
        return (self.imu_geometry_valid and self.vertical_nose_armed
                and height >= math.sin(math.radians(25.0))
                and -self.vertical_side * self.imu_roll_projection >= 0.20
                and self.imu_nose_angle >= 70.0 and self.imu_nose_rate >= -90.0
                and (observed_peak or predicted_peak))

    def vertical_control(self):
        now = time.monotonic()
        if (not self.imu_valid or self.imu_time is None
                or now - self.imu_time > self.feedback_timeout):
            self.finish_vertical_auto('IMU unavailable')
            return
        elapsed = now - self.vertical_state_time

        if self.vertical_state == 'prepare':
            # Hold the servo position; optionally start the motor at swimming speed.
            servo_tar = 2048.0 + self.vertical_side * 1024.0
            self.pos_control(0, servo_tar, 1.0)
            if self.vertical_prepare_fast_start:
                self.servo_cmd[1] = self.vertical_side * self.vertical_motor_direction * self.vertical_motor_speed
            else:
                self.pos_control(1, servo_tar, 1.0)
            prepare_seconds = self.vertical_prepare_seconds
            if (elapsed >= prepare_seconds and abs(self.servo_equ_angles - servo_tar) <= 50.0
                    and (self.vertical_prepare_fast_start or abs(self.motor_equ_angles - servo_tar) <= 50.0)):
                self.vertical_side *= -1
                self.set_vertical_state('reverse', now)
            elif elapsed > prepare_seconds + self.vertical_reverse_timeout:
                self.finish_vertical_auto('preparation position timeout')
                return
            else:
                return

        # Switch both outputs on the same control tick.
        servo_tar = 2048.0 + self.vertical_side * 1024.0
        self.pos_control(0, servo_tar, 1.0)
        self.servo_cmd[1] = self.vertical_side * self.vertical_motor_direction * self.vertical_motor_speed

        if self.vertical_state == 'reverse':
            elapsed = now - self.vertical_state_time
            if elapsed >= self.vertical_reverse_seconds and abs(self.servo_equ_angles - servo_tar) <= 50.0:
                if self.vertical_initial_reverse:
                    self.vertical_initial_reverse = False
                else:
                    self.vertical_reversals += 1
                    if self.vertical_reversals >= self.vertical_reverse_limit:
                        self.finish_vertical_auto('requested reversals completed')
                        return
                self.set_vertical_state('gait', now)
            elif elapsed > self.vertical_reverse_timeout:
                self.finish_vertical_auto('servo reversal timeout')
                
        elif self.vertical_state == 'gait':
            if elapsed > self.vertical_gait_timeout:
                self.finish_vertical_auto('no roll reversal window')
                return
            # Repeated timer ticks must not confirm one old IMU observation.
            if self.vertical_decision_sample == self.imu_sample:
                return
            self.vertical_decision_sample = self.imu_sample
            if now - self.vertical_decision_time > 0.10:
                self.vertical_window_time = None
            self.vertical_decision_time = now
            self.vertical_nose_armed |= self.imu_nose_angle <= 88.0
            if elapsed >= 1.5 and self.vertical_reverse_ready(now):
                if self.vertical_window_time is None:
                    self.vertical_window_time = now
                if now - self.vertical_window_time >= 0.04 - 1e-8:
                    self.vertical_side *= -1
                    self.set_vertical_state('reverse', now)
                    self.pos_control(0, 2048.0 + self.vertical_side * 1024.0, 1.0)
                    self.servo_cmd[1] = self.vertical_side * self.vertical_motor_direction * self.vertical_motor_speed
            else:
                self.vertical_window_time = None

    def vertical_up_control(self):
        now = time.monotonic()
        elapsed = now - self.vertical_state_time
        self.pos_control(0, 1024.0, 1.0)
        if self.vertical_state == 'prepare':
            self.pos_control(1, 1024.0, 1.0)
            if elapsed < self.vertical_up_prepare_seconds:
                return
            self.set_vertical_state('up', now)
        if self.vertical_state == 'up':
            if now - self.vertical_state_time >= self.vertical_up_seconds:
                self.finish_vertical_auto('vertical up duration completed')
                return
            self.servo_cmd[1] = self.vertical_side * self.vertical_motor_direction * self.vertical_motor_speed

    def _controlLoop(self, event):
        with self.control_lock:
            now = time.monotonic()
            self.update_vertical_trigger(now)
            if not self.feedback_ready(now):
                if self.vertical_auto_mode:
                    self.finish_vertical_auto('actuator feedback unavailable')
                else:
                    self.stop_auto()
                self.servo_cmd = [0.0, 0.0]
                self.publish_cmd()
                return
            if self.auto_control_mode:
                self.auto_control()
            elif self.auto_pattern == 'stop' and not self.motor_pos_control:
                # Keep homing from onboard feedback even when the joystick is offline.
                self.stop_control()
            elif self.joy_time is None or now - self.joy_time > self.joy_timeout:
                self.motor_acc = 0.0
                self.motor_cruise_speed = 0.0
                self.stop_auto()
            elif self.auto_pattern == 'cruise' and not self.motor_pos_control:
                self.pos_control(0, 2048.0 - self.servo_axis * 1024.0, 1.0)
            self.publish_cmd()

    def shutdown(self):
        self.control_timer.shutdown()
        with self.control_lock:
            self.auto_control_mode = False
            self.vertical_auto_mode = False
            self.auto_pattern = 'stop'
            self.servo_cmd = [0.0, 0.0]
            self.publish_cmd()

    def _joyCallback(self, msg):
        with self.control_lock:
            if len(msg.axes) < 11 or len(msg.buttons) < 8:
                rospy.logwarn_throttle(5.0, 'joystick needs at least 11 axes and 8 buttons')
                return
            if not all(math.isfinite(value) for value in msg.axes):
                return
            now = time.monotonic()
            if self.joy_time is None or now - self.joy_time > self.joy_timeout:
                self.vertical_trigger_time = None
            self.joy_time = now
            trigger_axis = 1 if msg.axes[9] > 0 else -1 if msg.axes[9] < 0 else 0
            trigger_mode = 'vertical_auto' if trigger_axis != 0 else ''
            if msg.axes[10] == 1:
                trigger_mode = 'vertical_up_auto'
                trigger_axis = 0
            if trigger_mode == '':
                self.vertical_trigger_time = None
                self.vertical_trigger_latched = False
            elif not self.vertical_trigger_latched:
                if (trigger_mode != self.vertical_trigger_mode or trigger_axis != self.vertical_trigger_axis
                        or self.vertical_trigger_time is None):
                    self.vertical_trigger_time = now
            self.vertical_trigger_axis = trigger_axis
            self.vertical_trigger_mode = trigger_mode

            motor_speed_step = 0.0
            if msg.axes[10] != self.motor_speed_axis_prev:
                if msg.axes[10] == 1 or msg.axes[10] == -1:
                    motor_speed_step = msg.axes[10]
            self.motor_speed_axis_prev = msg.axes[10]

            if msg.buttons[1] == 1:
                self.vertical_trigger_latched = self.vertical_trigger_mode != ''
                if self.vertical_auto_mode:
                    self.finish_vertical_auto('joystick stop')
                else:
                    self.stop_auto()
                if not self.feedback_ready(now):
                    self.servo_cmd = [0.0, 0.0]
                self.publish_cmd()
                return

            if self.auto_control_mode or not self.feedback_ready(now):
                return

            if msg.buttons[0] == 1:
                self.start_cruise()

            if msg.buttons[2] == 1 and self.feedback_ready(now) and self.imu_valid:
                self.initialize()

            # servo
            self.servo_axis = msg.axes[2]
            if math.fabs(self.servo_axis) < self.joy_dead_zone:
                self.servo_axis = 0

            # auto trajectory
            if msg.buttons[3] == 1 and self.auto_button_prev == 0:
                self.start_auto()
                self.auto_button_prev = msg.buttons[3]
                return
            self.auto_button_prev = msg.buttons[3]

            self.motor_pos_control = msg.buttons[5] == 1
            if msg.buttons[5] == 1:
                turn_tar = 2048.0 - self.servo_axis * 1024.0
                if turn_tar > 3072.0:
                    turn_tar = 3072.0
                if turn_tar < 1024.0:
                    turn_tar = 1024.0

                if msg.buttons[4] == 1 and self.turn_button_prev == 0:
                    if self.servo_axis > 0:
                        self.start_fast_turn(1024.0)
                        self.turn_button_prev = msg.buttons[4]
                        return
                    elif self.servo_axis < 0:
                        self.start_fast_turn(3072.0)
                        self.turn_button_prev = msg.buttons[4]
                        return
                self.turn_button_prev = msg.buttons[4]
                self.pos_control(0, turn_tar, 1.0)
                self.pos_control(1, turn_tar, 1.0)
                self.publish_cmd()
                return
            if msg.buttons[4] == 0:
                self.turn_button_prev = 0

            # motor acceleration and deceleration
            if msg.buttons[6] == 1 or msg.buttons[7] == 1:
                self.motor_acc = 0.5 * (msg.axes[3] - 1.0) - 0.5 * (msg.axes[4] - 1.0)
                if math.fabs(self.motor_acc) < self.joy_dead_zone:
                    self.motor_acc = 0
                rospy.loginfo("accelerate, acc = %.2f, cruise speed = %.2f", self.motor_acc, self.motor_cruise_speed / math.pi)
            else:
                self.motor_acc = 0

            if self.auto_pattern == 'stop':
                self.stop_control()
                self.publish_cmd()
            elif self.auto_pattern == 'cruise':
                self.cruise()
                if motor_speed_step != 0:
                    self.motor_cruise_speed = (round(self.motor_cruise_speed / math.pi) + motor_speed_step) * math.pi
                    rospy.loginfo("cruise speed = %.3f rad/s (%.0f * pi)", self.motor_cruise_speed, self.motor_cruise_speed / math.pi)
                self.publish_cmd()


    def _twistCallback(self, msg):
        with self.control_lock:
            if (self.vertical_auto_mode or not self.feedback_ready(time.monotonic())
                    or not math.isfinite(msg.linear.x)):
                return

            if self.auto_pattern != 'stop':
                self.servo_cmd[1] = msg.linear.x
                if self.auto_pattern == 'cruise':
                    self.motor_cruise_speed = msg.linear.x
                self.publish_cmd()



if __name__=="__main__":

    rospy.init_node("vertical_auto")

    teleop_node = VerticalAuto()

    rospy.spin()
