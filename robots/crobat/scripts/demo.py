#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Run a roll demo, move to the square start point, and fly the square."""

import rospy

from aerial_robot_msgs.msg import FlightNav


def lerp(start, end, fraction):
    return start + (end - start) * fraction


def make_nav_message(x, y, h, vx, vy, roll, pitch):
    msg = FlightNav()
    msg.header.stamp = rospy.Time.now()
    msg.control_frame = FlightNav.WORLD_FRAME
    msg.target = FlightNav.COG

    msg.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
    msg.target_pos_x = x
    msg.target_pos_y = y
    msg.target_vel_x = vx
    msg.target_vel_y = vy

    msg.pos_z_nav_mode = FlightNav.POS_MODE
    msg.target_pos_z = h
    msg.target_vel_z = 0.0

    msg.roll_nav_mode = FlightNav.POS_MODE
    msg.pitch_nav_mode = FlightNav.POS_MODE
    msg.target_roll = roll
    msg.target_pitch = pitch

    return msg


def transition(nav_pub, rate, duration, start, goal, velocity=(0.0, 0.0)):
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec()
        fraction = min(elapsed / duration, 1.0)

        x = lerp(start[0], goal[0], fraction)
        y = lerp(start[1], goal[1], fraction)
        roll = lerp(start[2], goal[2], fraction)
        pitch = lerp(start[3], goal[3], fraction)
        nav_pub.publish(
            make_nav_message(x, y, 0.5, velocity[0], velocity[1], roll, pitch)
        )

        if fraction >= 1.0:
            break
        rate.sleep()


def main():
    rospy.init_node("demo")

    # Parameters used by square_pose.py.
    a = float(rospy.get_param("~a", 1.0))
    b = float(rospy.get_param("~b", 0.3))
    edge_duration = float(rospy.get_param("~flight_time", 20.0))
    t = float(rospy.get_param("~t", 1.0))
    rate_hz = float(rospy.get_param("~rate_hz", 40.0))
    reset_duration = float(rospy.get_param("~reset_duration", 2.0))

    nav_pub = rospy.Publisher("/crobat/uav/nav", FlightNav, queue_size=1)
    rate = rospy.Rate(rate_hz)

    # Keep (x, y, z) = (0, 0, 0.5) and move only roll.
    transition(nav_pub, rate, 3.0, (0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.4, 0.0))
    transition(nav_pub, rate, 2.0, (0.0, 0.0, 0.4, 0.0), (0.0, 0.0, 0.4, 0.0))
    transition(nav_pub, rate, 6.0, (0.0, 0.0, 0.4, 0.0), (0.0, 0.0, -0.4, 0.0))
    transition(nav_pub, rate, 2.0, (0.0, 0.0, -0.4, 0.0), (0.0, 0.0, -0.4, 0.0))
    transition(nav_pub, rate, 3.0, (0.0, 0.0, -0.4, 0.0), (0.0, 0.0, 0.0, 0.0))
    transition(nav_pub, rate, 3.0, (0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))

    # Move to the starting point of the square in three seconds.
    move_duration = 3.0
    transition(
        nav_pub,
        rate,
        move_duration,
        (0.0, 0.0, 0.0, 0.0),
        (-0.5, -0.5, 0.0, 0.0),
        (-0.5 / move_duration, -0.5 / move_duration),
    )
    transition(
        nav_pub,
        rate,
        2.0,
        (-0.5, -0.5, 0.0, 0.0),
        (-0.5, -0.5, 0.0, 0.0),
    )

    # The following square motion is the same as square_pose.py.
    start_x = -0.5
    start_y = -0.5
    h = 0.5

    corners = (
        (0.0, 0.0),
        (0.0, a),
        (a, a),
        (a, 0.0),
        (0.0, 0.0),
    )

    attitudes = (
        (0.0, 0.0),
        (-b, 0.0),
        (0.0, b),
        (b, 0.0),
        (0.0, -b),
    )

    speed = a / edge_duration
    velocities = (
        (0.0, speed),
        (speed, 0.0),
        (0.0, -speed),
        (-speed, 0.0),
    )

    start_time = rospy.Time.now()
    edge_cycle = edge_duration + t
    square_duration = 4.0 * edge_cycle

    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec()
        if elapsed >= square_duration:
            break

        edge = min(int(elapsed / edge_cycle), 3)
        edge_elapsed = elapsed - edge * edge_cycle

        if edge_elapsed < edge_duration:
            fraction = edge_elapsed / edge_duration
            x = start_x + lerp(corners[edge][0], corners[edge + 1][0], fraction)
            y = start_y + lerp(corners[edge][1], corners[edge + 1][1], fraction)
            roll = lerp(attitudes[edge][0], attitudes[edge + 1][0], fraction)
            pitch = lerp(attitudes[edge][1], attitudes[edge + 1][1], fraction)
            vx, vy = velocities[edge]
        else:
            x = start_x + corners[edge + 1][0]
            y = start_y + corners[edge + 1][1]
            roll, pitch = attitudes[edge + 1]
            vx, vy = 0.0, 0.0

        nav_pub.publish(make_nav_message(x, y, h, vx, vy, roll, pitch))
        rate.sleep()

    reset_start = rospy.Time.now()
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - reset_start).to_sec()
        fraction = min(elapsed / reset_duration, 1.0) if reset_duration > 0.0 else 1.0
        pitch = lerp(-b, 0.0, fraction)

        nav_pub.publish(
            make_nav_message(start_x, start_y, h, 0.0, 0.0, 0.0, pitch)
        )

        if fraction >= 1.0:
            break
        rate.sleep()

    nav_pub.publish(make_nav_message(start_x, start_y, h, 0.0, 0.0, 0.0, 0.0))


if __name__ == "__main__":
    main()
