#!/usr/bin/env python

import subprocess
import sys

import rospy


def main():
    rospy.init_node("run_all_benchmarks")

    benchmarks = rospy.get_param("~benchmarks")

    extra_args = rospy.get_param("~benchmark_args", "")
    extra_args = extra_args.split() if extra_args else []

    failures = []
    for name in benchmarks:
        rospy.loginfo("==== running benchmark: %s ====", name)
        cmd = ["rosrun", "aerial_robot_dynamics", name] + extra_args
        ret = subprocess.call(cmd)
        if ret != 0:
            rospy.logerr("benchmark %s exited with code %d", name, ret)
            failures.append(name)

    if failures:
        rospy.logerr("failed benchmarks: %s", ", ".join(failures))
        sys.exit(1)

    rospy.loginfo("all benchmarks finished")


if __name__ == "__main__":
    main()
