#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interpolates a sine curve trajectory with:
  - CubicSpline (SciPy) to ensure continuity of velocity, acceleration, and jerk
  - FlightNav (uav/nav) to publish position, velocity, yaw, and yaw rate at 50Hz
  - Latch publishes an RViz Path on /ninja1/reference_path for visualization

Dependencies:
  $ sudo apt-get install python3-scipy python3-numpy
  or
  $ pip3 install scipy numpy
"""
import rospy
import numpy as np
from scipy.interpolate import CubicSpline
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


def generate_spline(x_dist, A, omega, z_fixed, duration, rate_hz):
    """
    Compute raw sine curve on time samples and
    create splines with continuous velocity and acceleration using CubicSpline

    Returns:
      cs_x, cs_y, cs_z: CubicSpline instances for x, y, z
      t_samples: array of sampling times
    """
    # Time samples
    N = int(duration * rate_hz) + 1
    t_samples = np.linspace(0.0, duration, N)
    # Raw positions
    x_raw = x_dist * t_samples / duration
    y_raw = A * np.sin(omega * t_samples)
    z_raw = np.full_like(t_samples, z_fixed)
    # Raw velocities (boundary conditions)
    vx_start = x_dist / duration
    vx_end   = vx_start
    vy_start = A * omega * np.cos(0.0)
    vy_end   = A * omega * np.cos(omega * duration)
    # Create splines
    cs_x = CubicSpline(t_samples, x_raw, bc_type=((1, vx_start), (1, vx_end)))
    cs_y = CubicSpline(t_samples, y_raw, bc_type=((1, vy_start), (1, vy_end)))
    cs_z = CubicSpline(t_samples, z_raw, bc_type='natural')
    return cs_x, cs_y, cs_z, t_samples


def main():
    rospy.init_node('sine_flightnav_spline')

    # Retrieve parameters
    duration    = rospy.get_param('~duration', 20.0)
    n_cycles    = rospy.get_param('~n_cycles', 2)
    A           = rospy.get_param('~amplitude', 1.5)
    z_fixed     = rospy.get_param('~z_fixed', 0.7)
    rate_hz     = rospy.get_param('~rate_hz', 100)
    x_distance  = rospy.get_param('~x_distance', 4.0)

    # Start position offset
    start_x = rospy.get_param('~start_x', -2.0)  # X: -2m
    start_y = rospy.get_param('~start_y',  0.0)  # Y:  0m

    # Calculate angular frequency based on number of cycles
    omega = 2 * np.pi * n_cycles / duration

    # Additional: yaw rate deadband / saturation control
    max_yaw_rate             = rospy.get_param('~max_yaw_rate', 1.0)               # [rad/s]
    yaw_rate_deadband_speed  = rospy.get_param('~yaw_rate_deadband_speed', 0.2)    # [m/s]

    # Prepare splines
    cs_x, cs_y, cs_z, t_samples = generate_spline(
        x_distance, A, omega, z_fixed, duration, rate_hz)

    # Publishers
    nav_pub  = rospy.Publisher('/ninja1/uav/nav', FlightNav, queue_size=1)
    path_pub = rospy.Publisher('/ninja1/reference_path', Path, queue_size=1, latch=True)
    rate = rospy.Rate(rate_hz)

    # Create and publish RViz Path once
    path_msg = Path()
    path_msg.header.frame_id = 'world'
    t0 = rospy.Time.now() + rospy.Duration(0.01)
    for t in t_samples:
        px = float(cs_x(t)) + start_x
        py = float(cs_y(t)) + start_y
        pz = float(cs_z(t))
        # Create PoseStamped
        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.header.stamp = t0 + rospy.Duration.from_sec(t)
        ps.pose.position.x = px
        ps.pose.position.y = py
        ps.pose.position.z = pz
        # Yaw angle and quaternion
        vx = float(cs_x(t, 1)); vy = float(cs_y(t, 1))
        yaw = np.arctan2(vy, vx)
        qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        path_msg.poses.append(ps)
    path_pub.publish(path_msg)
    rospy.loginfo('Published spline reference path: %d points (%.1fs) starting at (%.1f, %.1f)',
                  len(t_samples), duration, start_x, start_y)

    # Flight command loop
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec()
        if elapsed > duration:
            break

        # Get state from splines
        px = float(cs_x(elapsed)) + start_x
        py = float(cs_y(elapsed)) + start_y
        pz = float(cs_z(elapsed))
        vx = float(cs_x(elapsed, 1)); vy = float(cs_y(elapsed, 1))
        ax = float(cs_x(elapsed, 2)); ay = float(cs_y(elapsed, 2))

        # Yaw angle and yaw rate (saturation + deadband)
        yaw = np.arctan2(vy, vx)
        denom = vx * vx + vy * vy
        raw_yaw_rate = float((vx * ay - vy * ax) / denom) if denom > 1e-6 else 0.0
        speed = np.hypot(vx, vy)
        if speed < yaw_rate_deadband_speed:
            yaw_rate = 0.0
        else:
            yaw_rate = float(np.clip(raw_yaw_rate, -max_yaw_rate, max_yaw_rate))

        # FlightNav message
        msg = FlightNav()
        msg.header.stamp = rospy.Time.now()
        # XY
        msg.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
        msg.target_pos_x    = px
        msg.target_pos_y    = py
        msg.target_vel_x    = vx
        msg.target_vel_y    = vy
        # Z
        msg.pos_z_nav_mode  = FlightNav.POS_MODE
        msg.target_pos_z    = pz
        # Yaw
        msg.yaw_nav_mode    = FlightNav.POS_VEL_MODE
        msg.target_yaw      = yaw
        msg.target_omega_z  = yaw_rate

        nav_pub.publish(msg)
        rate.sleep()

    rospy.loginfo('Spline-based sine trajectory completed')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
