"""
 Created by li-jinjie on 24-1-5.
"""
import numpy as np
from typing import Tuple


class BaseTraj:
    def __init__(self) -> None:
        pass

    def get_2d_pt(self, t: float) -> Tuple[float, float, float, float, float, float]:
        x, y, vx, vy, ax, ay = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        return x, y, vx, vy, ax, ay

    def get_3d_pt(self, t: float) -> Tuple[float, float, float, float, float, float, float, float, float]:
        x, y, z, vx, vy, vz, ax, ay, az = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        return x, y, z, vx, vy, vz, ax, ay, az


class SetPointTraj(BaseTraj):
    def __init__(self) -> None:
        super().__init__()
        self.pos = np.array([0.0, 0.0, 0.7])
        self.vel = np.array([0.0, 0.0, 0.0])
        self.acc = np.array([0.0, 0.0, 0.0])

        self.att = np.array([0.0, 0.0, 0.0])
        self.att_rate = np.array([0.0, 0.0, 0.0])
        self.att_acc = np.array([0.0, 0.0, 0.0])

        self.t_converge = 5.0

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
            y = -0.2
            z = 1.0

        return x, y, z, vx, vy, vz, ax, ay, az

    def get_3d_orientation(self, t: float) -> Tuple[float, float, float, float, float, float, float, float, float]:
        roll, pitch, yaw = self.att
        roll_rate, pitch_rate, yaw_rate = self.att_rate
        roll_acc, pitch_acc, yaw_acc = self.att_acc

        if 3 * self.t_converge > t > self.t_converge:
            roll = 0.5
            yaw = 0.3

        if 3 * self.t_converge > t > 2 * self.t_converge:
            pitch = 0.5
            yaw = -0.3

        return roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, roll_acc, pitch_acc, yaw_acc


class CircleTraj(BaseTraj):
    def __init__(self) -> None:
        super().__init__()
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
    def __init__(self) -> None:
        super().__init__()
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
    def __init__(self) -> None:
        super().__init__()
        self.a_orientation = 0.5

    def get_3d_orientation(self, t: float) -> Tuple[float, float, float, float, float, float, float, float, float]:
        t = t + self.T * 1 / 4

        roll = -2 * self.a_orientation * np.sin(2 * self.omega * t) / 2
        pitch = self.a_orientation * np.cos(self.omega * t)
        yaw = np.pi / 2 * np.sin(self.omega * t + np.pi) + np.pi / 2

        roll_rate = -2 * 2 * self.a_orientation * self.omega * np.cos(2 * self.omega * t) / 2
        pitch_rate = -self.a_orientation * self.omega * np.sin(self.omega * t)
        yaw_rate = np.pi / 2 * self.omega * np.cos(self.omega * t + np.pi / 2)

        roll_acc = -2 * -4 * self.a_orientation * self.omega ** 2 * np.sin(2 * self.omega * t) / 2
        pitch_acc = -self.a_orientation * self.omega ** 2 * np.cos(self.omega * t)
        yaw_acc = - np.pi / 2 * self.omega ** 2 * np.sin(self.omega * t + np.pi / 2)

        return roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, roll_acc, pitch_acc, yaw_acc
