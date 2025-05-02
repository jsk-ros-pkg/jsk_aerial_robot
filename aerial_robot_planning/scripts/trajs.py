"""
 Created by li-jinjie on 24-1-5.
"""
import numpy as np
from typing import Tuple
import tf_conversions as tf


class BaseTraj:
    def __init__(self, loop_num: int = np.inf) -> None:
        self.T = float()
        self.loop_num = loop_num
        self.use_constant_ref = False

    def check_finished(self, t: float) -> bool:
        return t > self.T * self.loop_num

    def get_3d_pt(self, t: float) -> Tuple[float, float, float, float, float, float, float, float, float]:
        x, y, z, vx, vy, vz, ax, ay, az = 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        return x, y, z, vx, vy, vz, ax, ay, az

    def get_3d_orientation(self, t: float) -> Tuple[
        float, float, float, float, float, float, float, float, float, float]:
        qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
        roll_rate, pitch_rate, yaw_rate = 0.0, 0.0, 0.0
        roll_acc, pitch_acc, yaw_acc = 0.0, 0.0, 0.0
        return qw, qx, qy, qz, roll_rate, pitch_rate, yaw_rate, roll_acc, pitch_acc, yaw_acc


class BaseTrajwFixedRotor(BaseTraj):
    def __init__(self, loop_num: int = np.inf) -> None:
        super().__init__(loop_num)
        self.use_fix_rotor_flag = False

    def get_fixed_rotor(self, t: float):
        rotor_id = 0
        ft_fixed = 7.0
        alpha_fixed = 0.0
        return rotor_id, ft_fixed, alpha_fixed


class CircleTraj(BaseTraj):
    def __init__(self, loop_num) -> None:
        super().__init__(loop_num)
        self.r = 1  # radius in meters
        self.T = 10  # period in seconds
        self.omega = 2 * np.pi / self.T  # angular velocity

    def get_2d_pt(self, t: float) -> Tuple[float, float, float, float, float, float]:
        x = self.r * np.cos(self.omega * t) - 1.0
        y = self.r * np.sin(self.omega * t)
        vx = -self.r * self.omega * np.sin(self.omega * t)
        vy = self.r * self.omega * np.cos(self.omega * t)
        ax = -self.r * self.omega ** 2 * np.cos(self.omega * t)
        ay = -self.r * self.omega ** 2 * np.sin(self.omega * t)
        return x, y, vx, vy, ax, ay

    def get_3d_pt(self, t: float) -> Tuple[float, float, float, float, float, float, float, float, float]:
        x = self.r * np.cos(self.omega * t) - 1.0
        y = self.r * np.sin(self.omega * t)
        z = 0.5
        vx = -self.r * self.omega * np.sin(self.omega * t)
        vy = self.r * self.omega * np.cos(self.omega * t)
        vz = 0.0
        ax = -self.r * self.omega ** 2 * np.cos(self.omega * t)
        ay = -self.r * self.omega ** 2 * np.sin(self.omega * t)
        az = 0.0
        return x, y, z, vx, vy, vz, ax, ay, az


class LemniscateTraj(BaseTraj):
    def __init__(self, loop_num) -> None:
        super().__init__(loop_num)
        self.a = 1.0  # parameter determining the size of the Lemniscate
        self.z_range = 0.3  # range of z
        self.T = 20  # period in seconds
        self.omega = 2 * np.pi / self.T  # angular velocity

    def get_2d_pt(self, t: float) -> Tuple[float, float, float, float, float, float]:
        t = t + self.T / 4  # shift the phase to make the trajectory start at the origin

        x = self.a * np.cos(self.omega * t)
        y = self.a * np.sin(2 * self.omega * t)

        vx = -self.a * self.omega * np.sin(self.omega * t)
        vy = 2 * self.a * self.omega * np.cos(2 * self.omega * t)

        ax = -self.a * self.omega ** 2 * np.cos(self.omega * t)
        ay = -4 * self.a * self.omega ** 2 * np.sin(2 * self.omega * t)

        return x, y, vx, vy, ax, ay

    def get_3d_pt(self, t: float) -> Tuple[float, float, float, float, float, float, float, float, float]:
        t = t + self.T / 4  # shift the phase to make the trajectory start at the origin

        x = self.a * np.cos(self.omega * t)
        y = self.a * np.sin(2 * self.omega * t) / 2
        z = self.z_range * np.sin(2 * self.omega * t + np.pi / 2) + 1.0

        vx = -self.a * self.omega * np.sin(self.omega * t)
        vy = 2 * self.a * self.omega * np.cos(2 * self.omega * t) / 2
        vz = 2 * self.z_range * self.omega * np.cos(2 * self.omega * t + np.pi)

        ax = -self.a * self.omega ** 2 * np.cos(self.omega * t)
        ay = -4 * self.a * self.omega ** 2 * np.sin(2 * self.omega * t) / 2
        az = -4 * self.z_range * self.omega ** 2 * np.sin(2 * self.omega * t + np.pi)

        return x, y, z, vx, vy, vz, ax, ay, az


class LemniscateTrajOmni(LemniscateTraj):
    def __init__(self, loop_num) -> None:
        super().__init__(loop_num)
        self.a_orientation = 0.5

    def get_3d_orientation(self, t: float) -> Tuple[
        float, float, float, float, float, float, float, float, float, float]:
        t = t + self.T * 1 / 4

        roll = -2 * self.a_orientation * np.sin(2 * self.omega * t) / 2
        pitch = self.a_orientation * np.cos(self.omega * t)
        yaw = np.pi / 2 * np.sin(self.omega * t + np.pi) + np.pi / 2
        (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        roll_rate = -2 * 2 * self.a_orientation * self.omega * np.cos(2 * self.omega * t) / 2
        pitch_rate = -self.a_orientation * self.omega * np.sin(self.omega * t)
        yaw_rate = np.pi / 2 * self.omega * np.cos(self.omega * t + np.pi / 2)

        roll_acc = -2 * -4 * self.a_orientation * self.omega ** 2 * np.sin(2 * self.omega * t) / 2
        pitch_acc = -self.a_orientation * self.omega ** 2 * np.cos(self.omega * t)
        yaw_acc = -np.pi / 2 * self.omega ** 2 * np.sin(self.omega * t + np.pi / 2)

        return qw, qx, qy, qz, roll_rate, pitch_rate, yaw_rate, roll_acc, pitch_acc, yaw_acc


class SetPointTraj(BaseTraj):
    def __init__(self, loop_num) -> None:
        super().__init__(loop_num)
        self.pos = np.array([0.0, 0.0, 0.7])
        self.vel = np.array([0.0, 0.0, 0.0])
        self.acc = np.array([0.0, 0.0, 0.0])

        self.att = np.array([0.0, 0.0, 0.0])
        self.att_rate = np.array([0.0, 0.0, 0.0])
        self.att_acc = np.array([0.0, 0.0, 0.0])

        self.t_converge = 8.0
        self.T = 4 * self.t_converge

    def get_3d_pt(self, t: float) -> Tuple[float, float, float, float, float, float, float, float, float]:
        x, y, z = self.pos
        vx, vy, vz = self.vel
        ax, ay, az = self.acc

        if 3 * self.t_converge > t > self.t_converge:
            x = 0.3
            y = 0.2
            z = 1.2

        if 3 * self.t_converge > t > 2 * self.t_converge:
            x = -0.3
            y = 0.0
            z = 1.0

        return x, y, z, vx, vy, vz, ax, ay, az

    def get_3d_orientation(self, t: float) -> Tuple[
        float, float, float, float, float, float, float, float, float, float]:
        roll, pitch, yaw = self.att
        roll_rate, pitch_rate, yaw_rate = self.att_rate
        roll_acc, pitch_acc, yaw_acc = self.att_acc

        if 3 * self.t_converge > t > self.t_converge:
            roll = 0.5
            yaw = 0.3

        if 3 * self.t_converge > t > 2 * self.t_converge:
            pitch = 0.5
            yaw = -0.3

        (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        return qw, qx, qy, qz, roll_rate, pitch_rate, yaw_rate, roll_acc, pitch_acc, yaw_acc


class PitchRotationTraj(BaseTraj):
    def __init__(self, loop_num) -> None:
        super().__init__(loop_num)
        self.T = 10  # total time for one full rotation cycle (0 to -2.5 and back to 0)
        self.omega = 2 * np.pi / self.T  # angular velocity

    def get_3d_orientation(self, t: float) -> Tuple[
        float, float, float, float, float, float, float, float, float, float]:
        # Calculate the pitch angle based on time
        t = t - np.floor(t / self.T) * self.T  # make t in the range of [0, T]

        max_pitch = 3.2

        if t <= self.T / 2:
            pitch = max_pitch * (2 * t / self.T)  # from 0 to max_pitch rad
        else:
            pitch = max_pitch * (2 - 2 * t / self.T)  # from max_pitch to 0 rad

        roll = 0.0
        yaw = 0.0

        (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        if t <= self.T / 2:
            pitch_rate = max_pitch * 2 / self.T
        else:
            pitch_rate = -max_pitch * 2 / self.T

        roll_rate = 0.0
        yaw_rate = 0.0

        roll_acc = 0.0
        pitch_acc = 0.0
        yaw_acc = 0.0

        return qw, qx, qy, qz, roll_rate, pitch_rate, yaw_rate, roll_acc, pitch_acc, yaw_acc


class PitchContinuousRotationTraj(BaseTrajwFixedRotor):
    def __init__(self, loop_num) -> None:
        super().__init__(loop_num)
        self.T = 20  # total time for one full rotation cycle (0 to -2.5 and back to 0)

    def get_3d_orientation(self, t: float) -> Tuple[
        float, float, float, float, float, float, float, float, float, float]:
        # Calculate the pitch angle based on time
        t = t - np.floor(t / self.T) * self.T  # make t in the range of [0, T]

        max_pitch = np.pi * 2
        pitch = max_pitch * (t / self.T)  # from 0 to max_pitch rad
        roll = 0.0
        yaw = 0.0

        (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        pitch_rate = max_pitch / self.T
        roll_rate = 0.0
        yaw_rate = 0.0

        roll_acc = 0.0
        pitch_acc = 0.0
        yaw_acc = 0.0

        return qw, qx, qy, qz, roll_rate, pitch_rate, yaw_rate, roll_acc, pitch_acc, yaw_acc

    def get_fixed_rotor(self, t: float):
        rotor_id = 0
        ft_fixed = 7.0
        alpha_fixed = 0.0

        t_servo_change = 1.0
        min_ft = 0.5

        if self.T / 2 - 2 * t_servo_change >= t:
            self.use_fix_rotor_flag = False

        if self.T / 2 - 1 * t_servo_change >= t > self.T / 2 - 2 * t_servo_change:
            rotor_id = 1
            ft_fixed = min_ft
            alpha_fixed = np.pi
            self.use_fix_rotor_flag = True

        if self.T / 2 >= t > self.T / 2 - 1 * t_servo_change:
            rotor_id = 3
            ft_fixed = min_ft
            alpha_fixed = -np.pi
            self.use_fix_rotor_flag = True

        if self.T / 2 + 1 * t_servo_change >= t > self.T / 2:
            rotor_id = 0
            ft_fixed = min_ft
            alpha_fixed = np.pi
            self.use_fix_rotor_flag = True

        if self.T / 2 + 2 * t_servo_change >= t > self.T / 2 + 1 * t_servo_change:
            rotor_id = 2
            ft_fixed = min_ft
            alpha_fixed = -np.pi
            self.use_fix_rotor_flag = True

        if t > self.T / 2 + 2 * t_servo_change:
            self.use_fix_rotor_flag = False

        return rotor_id, ft_fixed, alpha_fixed


class PitchRotationTrajOpposite(PitchRotationTraj):
    def get_3d_orientation(self, t: float) -> Tuple[
        float, float, float, float, float, float, float, float, float, float]:
        qw, qx, qy, qz, roll_rate, pitch_rate, yaw_rate, roll_acc, pitch_acc, yaw_acc = super().get_3d_orientation(t)
        return qw, qx, -qy, qz, roll_rate, -pitch_rate, yaw_rate, roll_acc, -pitch_acc, yaw_acc


class Continuous45DegRotationTraj(BaseTraj):
    def __init__(self, loop_num) -> None:
        super().__init__(loop_num)
        self.T = 10  # total time for one full rotation cycle (0 to -2.5 and back to 0)
        self.omega = 2 * np.pi / self.T  # angular velocity

    def get_3d_orientation(self, t: float) -> Tuple[
        float, float, float, float, float, float, float, float, float, float]:
        roll = np.arctan2(np.sin(self.omega * t) / np.sqrt(2), np.cos(self.omega * t))
        pitch = np.arcsin(np.sin(self.omega * t) / np.sqrt(2))
        yaw = np.arctan(np.tan(self.omega * t / 2) ** 2)

        (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        roll_rate = 0.0
        pitch_rate = 0.0
        yaw_rate = 0.0

        roll_acc = 0.0
        pitch_acc = 0.0
        yaw_acc = 0.0

        return qw, qx, qy, qz, roll_rate, pitch_rate, yaw_rate, roll_acc, pitch_acc, yaw_acc


class RollRotationTraj(BaseTraj):
    def __init__(self, loop_num) -> None:
        super().__init__(loop_num)
        self.T = 10  # total time for one full rotation cycle (0 to -2.5 and back to 0)
        self.omega = 2 * np.pi / self.T  # angular velocity

    def get_3d_orientation(self, t: float) -> Tuple[
        float, float, float, float, float, float, float, float, float, float]:
        # Calculate the pitch angle based on time
        t = t - np.floor(t / self.T) * self.T  # make t in the range of [0, T]

        max_roll = 3.2

        if t <= self.T / 2:
            roll = max_roll * (2 * t / self.T)  # from 0 to max_roll rad
        else:
            roll = max_roll * (2 - 2 * t / self.T)  # from max_roll to 0 rad

        pitch = 0.0
        yaw = 0.0

        (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        if t <= self.T / 2:
            roll_rate = max_roll * 2 / self.T
        else:
            roll_rate = -max_roll * 2 / self.T

        pitch_rate = 0.0
        yaw_rate = 0.0

        roll_acc = 0.0
        pitch_acc = 0.0
        yaw_acc = 0.0

        return qw, qx, qy, qz, roll_rate, pitch_rate, yaw_rate, roll_acc, pitch_acc, yaw_acc


class RollRotationYaw045dTraj(BaseTraj):
    def __init__(self, loop_num) -> None:
        super().__init__(loop_num)
        self.T = 10  # total time for one full rotation cycle (0 to -2.5 and back to 0)
        self.omega = 2 * np.pi / self.T  # angular velocity

    def get_3d_orientation(self, t: float) -> Tuple[
        float, float, float, float, float, float, float, float, float, float]:
        # Calculate the pitch angle based on time
        t = t - np.floor(t / self.T) * self.T  # make t in the range of [0, T]

        max_roll = np.pi

        if t <= self.T / 2:
            roll = max_roll * (2 * t / self.T)  # from 0 to max_roll rad
        else:
            roll = max_roll * (2 - 2 * t / self.T)  # from max_roll to 0 rad

        pitch = 0.0
        yaw = np.pi / 4.0

        (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(roll, pitch, yaw, axes='rxyz')

        if t <= self.T / 2:
            roll_rate = max_roll * 2 / self.T
        else:
            roll_rate = -max_roll * 2 / self.T

        pitch_rate = 0.0
        yaw_rate = 0.0

        roll_acc = 0.0
        pitch_acc = 0.0
        yaw_acc = 0.0

        return qw, qx, qy, qz, roll_rate, pitch_rate, yaw_rate, roll_acc, pitch_acc, yaw_acc


class PitchSetPtTraj(BaseTraj):
    def __init__(self, loop_num) -> None:
        super().__init__(loop_num)
        self.t_converge = 8.0
        self.pitch_values = [
            0.0,
            0.5,
            1.0,
            1.5,
            2.0,
            2.5,
            2.0,
            1.5,
            1.0,
            0.5,
            0.0,
            -0.5,
            -1.0,
            -1.5,
            -2.0,
            -2.5,
            -2.0,
            -1.5,
            -1.0,
            -0.5,
            0.0,
        ]
        self.T = len(self.pitch_values) * self.t_converge

    def get_3d_orientation(self, t: float) -> Tuple[
        float, float, float, float, float, float, float, float, float, float]:
        roll, yaw = 0.0, 0.0
        roll_rate, yaw_rate = 0.0, 0.0
        roll_acc, yaw_acc = 0.0, 0.0

        index = min(int(t // self.t_converge), len(self.pitch_values) - 1)
        pitch = self.pitch_values[index]

        (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        pitch_rate = 0.0
        pitch_acc = 0.0

        return qw, qx, qy, qz, roll_rate, pitch_rate, yaw_rate, roll_acc, pitch_acc, yaw_acc


class YawRotationRoll090dTraj(BaseTraj):
    def __init__(self, loop_num) -> None:
        super().__init__(loop_num)
        self.T = 30  # total time for one full rotation cycle
        self.omega = 2 * np.pi / self.T  # angular velocity

    def get_3d_orientation(self, t: float) -> Tuple[
        float, float, float, float, float, float, float, float, float, float]:
        # Calculate the yaw angle based on time

        yaw = self.omega * t

        roll = np.pi / 2.0
        pitch = 0.0

        (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(roll, pitch, yaw, axes='rxyz')

        roll_rate = 0.0
        pitch_rate = 0.0
        yaw_rate = self.omega

        roll_acc = 0.0
        pitch_acc = 0.0
        yaw_acc = 0.0

        return qw, qx, qy, qz, roll_rate, pitch_rate, yaw_rate, roll_acc, pitch_acc, yaw_acc


class YawRotationRoll045dTraj(BaseTraj):
    def __init__(self, loop_num) -> None:
        super().__init__(loop_num)
        self.T = 30  # total time for one full rotation cycle
        self.omega = 2 * np.pi / self.T  # angular velocity

    def get_3d_orientation(self, t: float) -> Tuple[
        float, float, float, float, float, float, float, float, float, float]:
        # Calculate the yaw angle based on time

        yaw = self.omega * t

        roll = np.pi / 4.0
        pitch = 0.0

        (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(roll, pitch, yaw, axes='rxyz')

        roll_rate = 0.0
        pitch_rate = 0.0
        yaw_rate = self.omega

        roll_acc = 0.0
        pitch_acc = 0.0
        yaw_acc = 0.0

        return qw, qx, qy, qz, roll_rate, pitch_rate, yaw_rate, roll_acc, pitch_acc, yaw_acc


class YawRotationRoll135dTraj(BaseTraj):
    def __init__(self, loop_num) -> None:
        super().__init__(loop_num)
        self.T = 30  # total time for one full rotation cycle
        self.omega = 2 * np.pi / self.T  # angular velocity

    def get_3d_orientation(self, t: float) -> Tuple[
        float, float, float, float, float, float, float, float, float, float]:
        # Calculate the yaw angle based on time

        yaw = self.omega * t

        roll = np.pi * 3.0 / 4.0
        pitch = 0.0

        (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(roll, pitch, yaw, axes='rxyz')

        roll_rate = 0.0
        pitch_rate = 0.0
        yaw_rate = self.omega

        roll_acc = 0.0
        pitch_acc = 0.0
        yaw_acc = 0.0

        return qw, qx, qy, qz, roll_rate, pitch_rate, yaw_rate, roll_acc, pitch_acc, yaw_acc


class SingularityPointTraj(BaseTraj):
    def __init__(self, loop_num) -> None:
        super().__init__(loop_num)
        self.att = np.array([0.0, 0.0, 0.0])
        self.att_rate = np.array([0.0, 0.0, 0.0])
        self.att_acc = np.array([0.0, 0.0, 0.0])

        self.t_converge = 8.0
        self.T = 8 * self.t_converge

    def get_3d_orientation(self, t: float) -> Tuple[
        float, float, float, float, float, float, float, float, float, float]:

        yaw = np.pi / 4.0
        roll = np.pi / 2.0
        pitch = 0.0

        if 2 * self.t_converge >= t > self.t_converge:
            yaw = np.pi * 3.0 / 4.0

        if 3 * self.t_converge >= t > 2 * self.t_converge:
            yaw = np.pi * 5.0 / 4.0

        if 4 * self.t_converge >= t > 3 * self.t_converge:
            yaw = np.pi * 7.0 / 4.0

        if 5 * self.t_converge >= t > 4 * self.t_converge:
            yaw = np.pi * 1.0 / 4.0

        if 6 * self.t_converge >= t > 5 * self.t_converge:
            yaw = np.pi * 1.0 / 4.0 + 0.01

        if 7 * self.t_converge >= t > 6 * self.t_converge:
            yaw = np.pi * 1.0 / 4.0 - 0.01

        if 8 * self.t_converge >= t > 7 * self.t_converge:
            yaw = np.pi * 1.0 / 4.0 + 0.1

        (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(roll, pitch, yaw, axes='rxyz')

        roll_rate, pitch_rate, yaw_rate = self.att_rate
        roll_acc, pitch_acc, yaw_acc = self.att_acc

        return qw, qx, qy, qz, roll_rate, pitch_rate, yaw_rate, roll_acc, pitch_acc, yaw_acc


class TestFixedRotorTraj(BaseTrajwFixedRotor):
    def __init__(self, loop_num) -> None:
        super().__init__(loop_num)
        self.t_converge = 4.0
        self.T = 7 * self.t_converge

    def get_fixed_rotor(self, t: float):
        rotor_id = 0
        ft_fixed = 7.0
        alpha_fixed = 0.0

        if 0.0 >= t:
            self.use_fix_rotor_flag = False

        if self.t_converge >= t > 0.0:
            ft_fixed = 1.0
            self.use_fix_rotor_flag = True

        if 2 * self.t_converge >= t > self.t_converge:
            ft_fixed = 2.0
            self.use_fix_rotor_flag = True

        if 3 * self.t_converge >= t > 2 * self.t_converge:
            ft_fixed = 3.0
            self.use_fix_rotor_flag = True

        if 4 * self.t_converge >= t > 3 * self.t_converge:
            ft_fixed = 4.0
            self.use_fix_rotor_flag = True

        if 5 * self.t_converge >= t > 4 * self.t_converge:
            ft_fixed = 5.0
            self.use_fix_rotor_flag = True

        if 6 * self.t_converge >= t > 5 * self.t_converge:
            ft_fixed = 6.0
            self.use_fix_rotor_flag = True

        if 7 * self.t_converge >= t > 6 * self.t_converge:
            ft_fixed = 7.0
            self.use_fix_rotor_flag = True

        if t > 7 * self.t_converge:
            self.use_fix_rotor_flag = False

        return rotor_id, ft_fixed, alpha_fixed


class HappyBirthdayFixedRotorTraj(BaseTrajwFixedRotor):
    def __init__(self, loop_num: int = 1, beat: float = 1.0) -> None:
        super().__init__(loop_num)

        # beats of Happy Birthday (use strings for dot-notation)
        # note that the last beat is half note so it is doubled
        self.notes: list[str] = [
            "5", "5", "6", "5", ".1", "7", "7",
            "5", "5", "6", "5", ".2", ".1", ".1",
            "5", "5", ".5", ".3", ".1", "7", "6", "6",
            ".4", ".4", ".3", ".1", ".2", ".1", ".1"
        ]

        self.beat = beat  # for quarter note
        self.t_total = len(self.notes) * self.beat
        self.T = self.t_total * self.loop_num

        self.min_thrust = 0.5  # TODO: if 0N is stable, change this place to 0.0

    def _parse_note(self, note: str) -> float:
        if note == "0":  # rest
            return self.min_thrust

        # upper octave
        if note.startswith(".") and note[1:].isdigit():
            return float(int(note[1:]) + 7)

        # lower octave
        if note.endswith(".") and note[:-1].isdigit():
            return max(0.0, float(int(note[:-1]) - 7))

        # middle octave
        if note.isdigit():
            return float(int(note))

        raise ValueError(f"Unrecognized note token: {note!r}")

    def get_fixed_rotor(self, t: float):
        rotor_id = 0
        alpha_fixed = 0.0

        # Song finished or not yet started → no fixed rotor
        if t < 0.0 or t >= self.T:
            self.use_fix_rotor_flag = False
            return rotor_id, self.min_thrust, alpha_fixed

        # Current beat index
        beat_idx = int(t / self.beat) % len(self.notes)
        note = self.notes[beat_idx]
        ft_fixed = 13 - self._parse_note(note)

        # Rest → disable flag
        self.use_fix_rotor_flag = ft_fixed != self.min_thrust
        return rotor_id, ft_fixed, alpha_fixed
