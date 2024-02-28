"""
 Created by li-jinjie on 24-1-5.
"""
import numpy as np
from typing import Tuple


class BaseTraj2D:
    def __init__(self) -> None:
        pass

    def get_2d_pt(self, t: float) -> Tuple[float, float, float, float, float, float]:
        x, y, vx, vy, ax, ay = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        return x, y, vx, vy, ax, ay


class CircleTraj2D(BaseTraj2D):
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


class LemniscateTraj2D(BaseTraj2D):
    def __init__(self) -> None:
        super().__init__()
        self.a = 0.5  # parameter determining the size of the Lemniscate
        self.T = 15  # period in seconds
        self.omega = 2 * np.pi / self.T  # angular velocity

        self.z = 0.6

    def get_2d_pt(self, t: float) -> Tuple[float, float, float, float, float, float]:
        x = self.a * np.cos(self.omega * t)
        y = self.a * np.sin(2 * self.omega * t)
        vx = -self.a * self.omega * np.sin(self.omega * t)
        vy = 2 * self.a * self.omega * np.cos(2 * self.omega * t)
        ax = -self.a * self.omega ** 2 * np.cos(self.omega * t)
        ay = -4 * self.a * self.omega ** 2 * np.sin(2 * self.omega * t)
        return x, y, vx, vy, ax, ay

    def get_3d_pt(self, t: float) -> Tuple[float, float, float, float, float, float, float, float, float]:
        x = self.a * np.cos(self.omega * t)
        y = self.a * np.sin(2 * self.omega * t) / 2
        vx = -self.a * self.omega * np.sin(self.omega * t)
        vy = 2 * self.a * self.omega * np.cos(2 * self.omega * t)
        ax = -self.a * self.omega ** 2 * np.cos(self.omega * t)
        ay = -4 * self.a * self.omega ** 2 * np.sin(2 * self.omega * t)
        return x, y, self.z, vx, vy, 0.0, ax, ay, 0.0
