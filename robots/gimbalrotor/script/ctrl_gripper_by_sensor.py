#!/usr/bin/env python3


import sys
import tty
import termios
import select
import time
import threading
from collections import deque
from statistics import median
import rospy
from std_msgs.msg import UInt16MultiArray
from spinal.msg import ServoControlCmd, ServoStates


class CtrlGripper:
    def __init__(self):
        self.lock = threading.RLock()
        self.pub = rospy.Publisher("/gimbalrotor/servo/target_states",ServoControlCmd, queue_size=1)
        self.first_th = 350
        self.strong_th = 440
        self.reset_th = 340
        self.load_th = 100

        self.state = "open"         # States: open, checking, gripped
        self.armed = False
        self.sensor_filter = [deque(maxlen=3) for _ in range(5)]
        self.load_filter = deque(maxlen=3)
        self.reset_since = None
        self.first_since = None
        self.strong_since = None
        self.load_since = None
        self.close_time = 0

        rospy.Subscriber("/udts/proximities_intensity/raw", UInt16MultiArray, self.sensor_cb)
        rospy.Subscriber("/gimbalrotor/servo/states", ServoStates, self.servo_cb)
        rospy.Timer(rospy.Duration(0.1), self.check_timeout)
        rospy.Timer(rospy.Duration(0.5), self.update_params)

    def update_params(self, _):
        with self.lock:
            self.first_th = rospy.get_param("~first_threshold", 350)
            self.strong_th = rospy.get_param("~strong_threshold", 440)
            self.reset_th = rospy.get_param("~reset_threshold", 340)
            self.load_th = rospy.get_param("~load_threshold", 100)

    def send(self, value):
        msg = ServoControlCmd()
        msg.index = [6]
        msg.angles = [value]
        self.pub.publish(msg)

    def grip(self):
        with self.lock:
            self.send(900)
            self.state = "checking"
            self.armed = False
            self.close_time = time.monotonic()
            self.load_since = None
            self.load_filter.clear()
            rospy.loginfo("Closing gripper")

    def open(self):
        with self.lock:
            self.send(3500)
            self.state = "open"
            self.armed = False
            self.reset_since = None
            self.first_since = None
            self.strong_since = None
            self.load_since = None
            rospy.loginfo("Opening gripper")

    def sensor_cb(self, msg):
        now = time.monotonic()
        values = [v for v in msg.data if v != 65535]

        if len(values) != 5:
            return

        with self.lock:
            for f, v in zip(self.sensor_filter, values):
                f.append(v)

            if any(len(f) < 3 for f in self.sensor_filter):
                return

            values = [median(f) for f in self.sensor_filter]

            if self.state != "open":
                return

            if max(values) <= self.reset_th:
                if self.reset_since is None:
                    self.reset_since = now
                if now - self.reset_since >= 0.3:
                    self.armed = True
            else:
                self.reset_since = None

            if not self.armed:
                return

            if values[0] >= self.first_th:
                if self.first_since is None:
                    self.first_since = now
            else:
                self.first_since = None

            if max(values) >= self.strong_th:
                if self.strong_since is None:
                    self.strong_since = now
            else:
                self.strong_since = None

            first_detected = (
                self.first_since is not None and
                now - self.first_since >= 0.12
            )
            strong_detected = (
                self.strong_since is not None and
                now - self.strong_since >= 0.08
            )

            if first_detected or strong_detected:
                self.grip()

    def servo_cb(self, msg):
        now = time.monotonic()

        with self.lock:
            if self.state != "checking":
                return

            if now - self.close_time < 0.5:
                return # 0.5 may be changed by sensor

            for servo in msg.servos:
                if servo.index != 6:
                    continue

                self.load_filter.append(abs(servo.load))

                if len(self.load_filter) < 3:
                    return

                load = median(self.load_filter)

                if load >= self.load_th:
                    if self.load_since is None:
                        self.load_since = now

                    if now - self.load_since >= 0.2:
                        self.state = "gripped"
                        rospy.loginfo("Grip success! load=%.1f", load)
                else:
                    self.load_since = None

                return

    def check_timeout(self, _):
        with self.lock:
            if (self.state == "checking" and
                    time.monotonic() - self.close_time >= 2.0):
                rospy.logwarn("Grip failed -> open")
                self.open()

    def run(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)
            print("g: grip / o: open / Ctrl+C: exit")

            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()
                    if key == "g":
                        self.grip()
                    elif key == "o":
                        self.open()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


if __name__ == "__main__":
    rospy.init_node("ctrl_gripper")
    CtrlGripper().run()
