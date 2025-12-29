#!/usr/bin/env python

import rospy
import xml.etree.ElementTree as ET
from std_msgs.msg import String


def get_robot_description():
    robot_description = rospy.get_param("robot_description")
    return robot_description


def generate_pinocchio_robot_description(robot_description):
    root = ET.fromstring(robot_description)
    for joint in root.findall("joint"):
        name = joint.attrib.get("name")
        if "rotor" in name:
            joint.attrib["type"] = "fixed"
            rospy.loginfo("{} is modified to fixed joint".format(name))

            # remove limit tag
            existing_limit = joint.find("limit")
            if existing_limit is not None:
                joint.remove(existing_limit)
    return ET.tostring(root, encoding="unicode")


def main():
    rospy.init_node("pinocchio_robot_description_generator")

    robot_description = get_robot_description()

    pinocchio_robot_description = generate_pinocchio_robot_description(robot_description)

    rospy.set_param("pinocchio_robot_description", pinocchio_robot_description)


if __name__ == "__main__":
    main()
