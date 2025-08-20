#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Publish phase-shifted servo commands to /servo/target_states (ROS 1).
Each servo follows: start at 2048 -> go down to 0 -> up to 4096 -> back to 0,
implemented as a -sin waveform so the very first movement goes DOWN.
Per-servo phase shift highlights independent control.

Usage:
  rosrun <your_pkg> phase_shift_servos.py
Optional args:
  _indices:="1,2,3,4"         # servo indices
  _topic:="/servo/target_states"
  _rate:=50                    # Hz
  _period:=6.0                 # seconds per full cycle
  _phase_deg:=90               # phase shift between adjacent servos (degrees)
  _min:=0 _max:=4096
"""

import math
import time
import rospy
from spinal.msg import ServoControlCmd  # Message type: index: int32[], angles: int32[]


def parse_indices(s):
    return [int(x) for x in s.split(",") if x.strip()]


def clamp(v, vmin, vmax):
    return max(vmin, min(v, vmax))


def main():
    rospy.init_node("phase_shift_servos")

    # ---- Parameters ----
    indices_str = rospy.get_param("~indices", "0,1,2,3")
    indices = parse_indices(indices_str)
    topic = rospy.get_param("~topic", "/servo/target_states")
    rate_hz = float(rospy.get_param("~rate", 50))
    period = float(rospy.get_param("~period", 12.0))  # seconds for a full cycle
    phase_deg = float(rospy.get_param("~phase_deg", 45))  # per-servo phase offset (degrees)
    vmin = int(rospy.get_param("~min", 0))
    vmax = int(rospy.get_param("~max", 4096))

    # ---- Waveform setup ----
    # Angle(t) = base - amp * sin(omega * t + phi_i)
    # Start at 2048 and go DOWN first (toward 0), then up to 4096, then back.
    base = (vmin + vmax) / 2.0  # 2048 when min=0, max=4096
    amp = (vmax - vmin) / 2.0  # 2048
    omega = 2.0 * math.pi / period  # rad/s

    # Per-servo phase shift (radians)
    phase_step = math.radians(phase_deg)
    phases = [i * phase_step for i in range(len(indices))]

    pub = rospy.Publisher(topic, ServoControlCmd, queue_size=10)
    rate = rospy.Rate(rate_hz)

    rospy.loginfo("Starting phase-shifted servo publisher:")
    rospy.loginfo(
        "  indices=%s  topic=%s  rate=%.1f Hz  period=%.2f s  phase=%.1f deg/servo",
        indices,
        topic,
        rate_hz,
        period,
        phase_deg,
    )

    t0 = time.time()
    while not rospy.is_shutdown():
        t = time.time() - t0
        angles = []
        for phi in phases:
            val = base - amp * math.sin(omega * t + phi)  # go down first
            val = int(round(clamp(val, vmin, vmax)))
            angles.append(val)

        msg = ServoControlCmd()
        msg.index = indices
        msg.angles = angles

        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
