#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3D Lemniscate trajectory with continuous yaw, pitch, and roll alignment:
  - CubicSpline (SciPy) for smooth velocity/acceleration
  - FlightNav publishes position, velocity, yaw, pitch, and roll
  - Pre-flight phase: move to initial point and smoothly transition orientation
  - Post-flight phase: smoothly reset orientation to level

Roll control ensures the UAV’s top surface always faces the negative Y-axis by banking direction,
with a configurable bank factor to increase tilt.

Dependencies:
  sudo apt-get install python3-scipy python3-numpy
  or
  pip3 install scipy numpy
"""
import rospy
import numpy as np
from scipy.interpolate import CubicSpline
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def generate_lemniscate_spline(a, c, z_offset, n_cycles, duration, rate_hz):
    N = int(duration * rate_hz) + 1
    t = np.linspace(0.0, duration, N)
    θ = 2 * np.pi * n_cycles * t / duration

    x_raw = a * np.sin(θ) / (1.0 + np.cos(θ)**2)
    y_raw = a * np.sin(θ) * np.cos(θ) / (1.0 + np.cos(θ)**2)
    z_raw = c * np.sin(2.0 * θ) + z_offset

    dt = t[1] - t[0]
    vx_start = (x_raw[1] - x_raw[0]) / dt
    vx_end   = (x_raw[-1] - x_raw[-2]) / dt
    vy_start = (y_raw[1] - y_raw[0]) / dt
    vy_end   = (y_raw[-1] - y_raw[-2]) / dt

    cs_x = CubicSpline(t, x_raw, bc_type=((1, vx_start), (1, vx_end)))
    cs_y = CubicSpline(t, y_raw, bc_type=((1, vy_start), (1, vy_end)))
    cs_z = CubicSpline(t, z_raw, bc_type='natural')
    return cs_x, cs_y, cs_z, t

def main():
    rospy.init_node('lemniscate_spline_with_roll')

    # parameters
    duration         = rospy.get_param('~duration',        45.0)
    n_cycles         = rospy.get_param('~n_cycles',         3)
    a                = rospy.get_param('~amplitude',       1.5)
    c                = rospy.get_param('~twist_amp',       0.3)
    z_offset         = rospy.get_param('~z_offset',        1.0)
    rate_hz          = rospy.get_param('~rate_hz',        100)
    start_x          = rospy.get_param('~start_x',       0.0)
    start_y          = rospy.get_param('~start_y',       0.0)
    wait_duration    = rospy.get_param('~wait_duration',   5.0)   # seconds
    reset_duration   = rospy.get_param('~reset_duration',  5.0)   # seconds to reset orientation
    bank_factor      = rospy.get_param('~bank_factor',     10.0)  # scale factor for roll tilt

    max_yaw_rate            = rospy.get_param('~max_yaw_rate',           1.0)
    yaw_rate_deadband_speed = rospy.get_param('~yaw_rate_deadband_speed', 0.2)
    g = 9.81

    # generate spline trajectory
    cs_x, cs_y, cs_z, t_samples = generate_lemniscate_spline(
        a, c, z_offset, n_cycles, duration, rate_hz)

    # initial orientation from trajectory
    vx0 = float(cs_x(0.0, 1)); vy0 = float(cs_y(0.0, 1)); vz0 = float(cs_z(0.0, 1))
    ax0 = float(cs_x(0.0, 2)); ay0 = float(cs_y(0.0, 2))
    yaw0 = np.arctan2(vy0, vx0)
    horiz0 = np.hypot(vx0, vy0)
    pitch0 = -np.arctan2(vz0, horiz0)
    raw_yaw_rate0 = (vx0 * ay0 - vy0 * ax0) / (vx0*vx0 + vy0*vy0) if (vx0*vx0 + vy0*vy0)>1e-6 else 0.0
    a_lat0 = abs(horiz0 * raw_yaw_rate0)
    roll0 = np.arctan2(bank_factor * a_lat0, g)

    # publishers
    nav_pub  = rospy.Publisher('/ninja1/uav/nav', FlightNav, queue_size=1)
    path_pub = rospy.Publisher('/ninja1/reference_path', Path, queue_size=1, latch=True)
    rate = rospy.Rate(rate_hz)

    # publish RViz Path
    path = Path(); path.header.frame_id = 'world'
    t0 = rospy.Time.now() + rospy.Duration(0.01)
    for ti in t_samples:
        ps = PoseStamped(); ps.header.frame_id='world'; ps.header.stamp = t0 + rospy.Duration.from_sec(ti)
        ps.pose.position.x = float(cs_x(ti)) + start_x
        ps.pose.position.y = float(cs_y(ti)) + start_y
        ps.pose.position.z = float(cs_z(ti))
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    path_pub.publish(path)
    rospy.loginfo('Published path with %d pts', len(t_samples))

    # Pre-flight: move to start and smoothly adjust orientation
    init_x = float(cs_x(0.0)) + start_x
    init_y = float(cs_y(0.0)) + start_y
    init_z = float(cs_z(0.0))
    rospy.loginfo('Pre-flight: moving to (%.2f, %.2f, %.2f) and orienting', init_x, init_y, init_z)
    start_t = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - start_t).to_sec() < wait_duration:
        elapsed = (rospy.Time.now() - start_t).to_sec()
        frac = elapsed / wait_duration
        yaw = yaw0 * frac
        pitch = pitch0 * frac
        roll = roll0 * frac
        msg = FlightNav()
        msg.header.stamp      = rospy.Time.now()
        msg.pos_xy_nav_mode   = FlightNav.POS_MODE; msg.target_pos_x  = init_x; msg.target_pos_y = init_y
        msg.pos_z_nav_mode    = FlightNav.POS_MODE; msg.target_pos_z  = init_z
        msg.yaw_nav_mode      = FlightNav.POS_MODE; msg.target_yaw    = yaw
        msg.pitch_nav_mode    = FlightNav.POS_MODE; msg.target_pitch  = pitch
        msg.roll_nav_mode     = FlightNav.POS_MODE; msg.target_roll   = roll
        nav_pub.publish(msg)
        rate.sleep()
    rospy.loginfo('Pre-flight orientation complete')

    # main flight loop
    start_time = rospy.Time.now()
    last_yaw = yaw0; last_pitch = pitch0; last_roll = roll0
    last_px = init_x; last_py = init_y; last_pz = init_z
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec()
        if elapsed > duration:
            # capture final state
            last_px = float(cs_x(duration)) + start_x
            last_py = float(cs_y(duration)) + start_y
            last_pz = float(cs_z(duration))
            last_vx = float(cs_x(duration,1)); last_vy = float(cs_y(duration,1)); last_vz = float(cs_z(duration,1))
            last_yaw = np.arctan2(last_vy, last_vx)
            horiz = np.hypot(last_vx, last_vy)
            last_pitch = -np.arctan2(last_vz, horiz)
            raw_rate = (last_vx*float(cs_y(duration,2)) - last_vy*float(cs_x(duration,2)))/(last_vx**2+last_vy**2) if (last_vx**2+last_vy**2)>1e-6 else 0.0
            last_roll = np.arctan2(bank_factor * abs(horiz * raw_rate), g)
            break

        px = float(cs_x(elapsed)) + start_x; py = float(cs_y(elapsed)) + start_y; pz = float(cs_z(elapsed))
        vx = float(cs_x(elapsed,1)); vy = float(cs_y(elapsed,1)); vz = float(cs_z(elapsed,1))
        ax = float(cs_x(elapsed,2)); ay = float(cs_y(elapsed,2))
        yaw = np.arctan2(vy, vx)
        horiz = np.hypot(vx, vy)
        pitch = -np.arctan2(vz, horiz)
        raw_yaw_rate = (vx*ay - vy*ax)/(vx*vx+vy*vy) if (vx*vx+vy*vy)>1e-6 else 0.0
        roll = np.arctan2(bank_factor * abs(horiz * raw_yaw_rate), g)
        yaw_rate = 0.0 if horiz<yaw_rate_deadband_speed else np.clip(raw_yaw_rate, -max_yaw_rate, max_yaw_rate)
        last_px, last_py, last_pz = px, py, pz
        last_yaw, last_pitch, last_roll = yaw, pitch, roll

        msg = FlightNav()
        msg.header.stamp      = rospy.Time.now()
        msg.pos_xy_nav_mode   = FlightNav.POS_VEL_MODE; msg.target_pos_x  = px; msg.target_pos_y = py
        msg.target_vel_x      = vx; msg.target_vel_y    = vy
        msg.pos_z_nav_mode    = FlightNav.POS_MODE; msg.target_pos_z  = pz
        msg.yaw_nav_mode      = FlightNav.POS_VEL_MODE; msg.target_yaw    = yaw; msg.target_omega_z = yaw_rate
        msg.pitch_nav_mode    = FlightNav.POS_MODE; msg.target_pitch  = pitch
        msg.roll_nav_mode     = FlightNav.POS_MODE; msg.target_roll   = roll
        nav_pub.publish(msg)
        rate.sleep()

    rospy.loginfo('Trajectory completed, entering post-flight reset')
    # Post-flight: smoothly reset orientation to level at final position
    reset_start = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - reset_start).to_sec() < reset_duration:
        t = (rospy.Time.now() - reset_start).to_sec()
        frac = t / reset_duration
        yaw = last_yaw * (1 - frac)
        pitch = last_pitch * (1 - frac)
        roll = last_roll * (1 - frac)
        msg = FlightNav()
        msg.header.stamp      = rospy.Time.now()
        msg.pos_xy_nav_mode   = FlightNav.POS_MODE; msg.target_pos_x  = last_px; msg.target_pos_y = last_py
        msg.pos_z_nav_mode    = FlightNav.POS_MODE; msg.target_pos_z  = last_pz
        msg.yaw_nav_mode      = FlightNav.POS_MODE; msg.target_yaw    = yaw
        msg.pitch_nav_mode    = FlightNav.POS_MODE; msg.target_pitch  = pitch
        msg.roll_nav_mode     = FlightNav.POS_MODE; msg.target_roll   = roll
        nav_pub.publish(msg)
        rate.sleep()
    rospy.loginfo('Post-flight reset complete')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
