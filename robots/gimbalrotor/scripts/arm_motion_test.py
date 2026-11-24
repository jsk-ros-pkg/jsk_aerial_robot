#!/usr/bin/env python3
"""Sweep each Grape arm joint through its allowed range, then return all axes to zero."""

import math
import time
import xml.etree.ElementTree as ET

import rospy
from sensor_msgs.msg import JointState


ARM_JOINTS = tuple(f"arm_joint{index}_joint1" for index in range(1, 6))


def read_positions(topic, names, timeout):
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        msg = rospy.wait_for_message(topic, JointState, timeout=deadline - time.monotonic())
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions and math.isfinite(positions[name]) for name in names):
            return [positions[name] for name in names]
    raise RuntimeError(f"No complete arm feedback on {topic}")


def joint_bounds(model, joint):
    limit = joint.find("limit")
    if joint.get("type") == "continuous":
        lower, upper = -math.inf, math.inf
    else:
        lower, upper = float(limit.get("lower")), float(limit.get("upper"))
    for follower in model.findall("joint"):
        mimic = follower.find("mimic")
        limit = follower.find("limit")
        if mimic is None or limit is None or mimic.get("joint") != joint.get("name"):
            continue
        multiplier = float(mimic.get("multiplier", 1))
        if multiplier == 0:
            continue
        offset = float(mimic.get("offset", 0))
        if limit.get("lower") is not None and limit.get("upper") is not None:
            bounds = sorted((float(limit.get(key)) - offset) / multiplier for key in ("lower", "upper"))
            lower, upper = max(lower, bounds[0]), min(upper, bounds[1])
    return lower, upper


def command_and_check(publisher, topic, name, target, bounds, move_time, timeout, tolerance):
    measured, = read_positions(topic, [name], timeout)
    start_angle = min(max(measured, bounds[0]), bounds[1])
    started = rospy.Time.now()
    while True:
        now = rospy.Time.now()
        fraction = min(1.0, (now - started).to_sec() / move_time)
        angle = target if fraction >= 1.0 else start_angle + fraction * (target - start_angle)
        command = JointState(name=[name], position=[angle])
        command.header.stamp = now
        publisher.publish(command)
        if fraction >= 1.0:
            break
        rospy.sleep(min(0.05, (1.0 - fraction) * move_time))

    deadline = time.monotonic() + timeout
    measured = None
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        try:
            measured, = read_positions(topic, [name], max(0.001, deadline - time.monotonic()))
        except rospy.ROSException:
            break
        if abs(measured - target) <= tolerance:
            rospy.loginfo("%s reached %.3f rad (target %.3f rad)", name, measured, target)
            return
    actual = "no feedback" if measured is None else f"last measured {measured:.3f} rad"
    raise RuntimeError(f"{name} did not reach {target:.3f} rad within {timeout:g} s ({actual})")


def main(gripper_only=False):
    rospy.init_node("gripper_motion_test" if gripper_only else "arm_motion_test")
    timeout = float(rospy.get_param("~timeout", 8.0))
    tolerance = float(rospy.get_param("~tolerance", 0.08))
    move_time = float(rospy.get_param("~move_time", 2.0))
    full_range = rospy.get_param("~full_range", True)
    if not all(math.isfinite(value) and value > 0 for value in (timeout, tolerance, move_time)):
        raise ValueError("~timeout, ~tolerance, and ~move_time must be positive and finite")

    namespace = rospy.get_param("~robot_ns", "/gimbalrotor").rstrip("/")
    topic = namespace + "/joint_states"
    model = ET.fromstring(rospy.get_param(namespace + "/robot_description"))
    joints = {joint.get("name"): joint for joint in model.findall("joint")
              if joint.get("name") in ARM_JOINTS and joint.get("type") in ("revolute", "continuous")}
    dof = rospy.get_param("~dof", len(joints))
    if dof not in (3, 5):
        raise ValueError(f"Expected a 3DoF or 5DoF Grape arm, found {len(joints)} joints (requested {dof})")
    names = ARM_JOINTS[-dof:]
    if joints.keys() != set(names):
        raise RuntimeError(f"Expected {dof} arm joints, found {', '.join(sorted(joints)) or 'none'}")
    if gripper_only:
        names = (ARM_JOINTS[-1],)

    limits = {name: joint_bounds(model, joints[name]) for name in names}
    for name, (lower, upper) in limits.items():
        if not lower <= 0.0 <= upper:
            raise ValueError(f"Zero is outside the allowed range for {name}: [{lower}, {upper}]")

    publisher = rospy.Publisher(namespace + "/joints_ctrl", JointState, queue_size=1)
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown() and not publisher.get_num_connections():
        if time.monotonic() >= deadline:
            raise RuntimeError(f"No subscriber on {publisher.resolved_name}")
        time.sleep(0.05)

    read_positions(topic, names, timeout)
    incomplete = []
    for name in names:
        initial, = read_positions(topic, [name], timeout)
        bounds = limits[name]
        lower, upper = bounds
        if not lower <= initial <= upper:
            rospy.logwarn("Skipping %s: measured %.3f rad, allowed [%.3f, %.3f] rad",
                          name, initial, lower, upper)
            incomplete.append(name)
            continue
        if full_range:
            if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
                rospy.logwarn("Skipping %s: no finite range to sweep", name)
                incomplete.append(name)
                continue
            targets = (lower, upper)
        else:
            step = 0.50 if name == ARM_JOINTS[-1] else 0.20
            target = next((angle for angle in (initial + step, initial - step)
                           if lower <= angle <= upper), None)
            if target is None:
                rospy.logwarn("Skipping %s: no %.3f rad motion inside [%.3f, %.3f] rad",
                              name, step, lower, upper)
                incomplete.append(name)
                continue
            targets = (target,)
        sequence = " -> ".join(f"{angle:.3f}" for angle in (initial, *targets, initial))
        rospy.loginfo("%s: %s rad (%.2f s per segment)", name, sequence, move_time)
        try:
            for target in targets:
                command_and_check(publisher, topic, name, target, bounds, move_time, timeout, tolerance)
        except RuntimeError:
            try:
                command_and_check(publisher, topic, name, initial, bounds, move_time, timeout, tolerance)
            except RuntimeError as return_error:
                rospy.logerr("Return to the initial angle also failed: %s", return_error)
            raise
        command_and_check(publisher, topic, name, initial, bounds, move_time, timeout, tolerance)
    if incomplete:
        raise RuntimeError(f"Motion test incomplete. Skipped: {', '.join(incomplete)}")
    for name in names:
        command_and_check(publisher, topic, name, 0.0, limits[name], move_time, timeout, tolerance)
    rospy.loginfo("%d tested axes reached their test positions and returned to zero", len(names))


def run(gripper_only=False):
    try:
        main(gripper_only)
    except rospy.ROSInterruptException:
        pass
    except (RuntimeError, rospy.ROSException, ValueError, KeyError) as error:
        rospy.logerr("%s", error)
        raise SystemExit(1)


if __name__ == "__main__":
    run()
