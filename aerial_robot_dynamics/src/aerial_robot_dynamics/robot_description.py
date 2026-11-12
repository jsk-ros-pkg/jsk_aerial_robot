import xml.etree.ElementTree as ET

ROTOR_JOINT_KEYWORD = "rotor"


def make_pinocchio_robot_description(robot_description, keyword=ROTOR_JOINT_KEYWORD, logger=None):
    root = ET.fromstring(robot_description)
    for joint in root.findall("joint"):
        name = joint.attrib.get("name")
        if name is None or keyword not in name:
            continue
        joint.attrib["type"] = "fixed"
        if logger is not None:
            logger("{} is modified to fixed joint".format(name))

        # remove limit tag
        existing_limit = joint.find("limit")
        if existing_limit is not None:
            joint.remove(existing_limit)
    return ET.tostring(root, encoding="unicode")
