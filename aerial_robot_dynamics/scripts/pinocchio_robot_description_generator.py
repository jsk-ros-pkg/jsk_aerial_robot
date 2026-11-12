#!/usr/bin/env python

import rospy

from aerial_robot_dynamics.robot_description import make_pinocchio_robot_description


def get_robot_description():
    robot_description = rospy.get_param("robot_description")
    return robot_description


def generate_pinocchio_robot_description(robot_description):
    return make_pinocchio_robot_description(robot_description, logger=rospy.loginfo)


def main():
    rospy.init_node("pinocchio_robot_description_generator")

    robot_description = get_robot_description()

    pinocchio_robot_description = generate_pinocchio_robot_description(robot_description)

    rospy.set_param("pinocchio_robot_description", pinocchio_robot_description)


if __name__ == "__main__":
    main()
