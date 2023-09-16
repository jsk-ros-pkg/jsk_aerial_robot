#!/usr/bin/env python

import xml.etree.ElementTree as ET
from xml.dom import minidom
import subprocess
import yaml
import rospkg
import os


rotor_list = []
joint_list = []
rospack = rospkg.RosPack()


def get_filename(filepath):
    return filepath.rsplit("/", 1)[1]


def get_directory(filepath):
    return filepath.rsplit("/", 1)[0]


def remove_extension(filename):
    before_ext, ext = os.path.splitext(filename)
    return before_ext


def run_xacro(input_path, output_path):
    cmd = "rosrun xacro xacro {} > {}".format(input_path, output_path)
    proc = subprocess.run(cmd, shell=True)


def process_urdf(package, urdf_path, workdir_path):
    global rotor_list
    global joint_list
    rotor_list = []
    joint_list = []
    urdf_tree = ET.parse(urdf_path)
    urdf_root = urdf_tree.getroot()

    # add mujoco config
    mujoco = ET.Element('mujoco')
    compiler = ET.SubElement(mujoco, 'compiler')
    option = ET.SubElement(mujoco, 'option')
    compiler.set('balanceinertia', 'true')
    urdf_root.append(mujoco)

    # fix mesh path in visual tag
    for link in urdf_root.findall("link"):
        for link_visual in link.findall("visual"):
            for link_visual_geometry in link_visual.findall("geometry"):
                for link_visual_geometry_mesh in link_visual_geometry.findall("mesh"):
                    filepath = link_visual_geometry_mesh.attrib["filename"]
                    search_string = "package://"
                    index = filepath.find(search_string)
                    filename = filepath[index + len(search_string):]
                    filename = filename[filename.find("/"):]
                    filepath = rospack.get_path(package) + filename

                    # modify extention
                    filename, ex = os.path.splitext(filepath)
                    filepath = filename + ".stl"
                    filename = get_filename(filepath)

                    # copy stl to working directory
                    cmd = "cp {} {}".format(filepath, workdir_path)
                    proc = subprocess.run(cmd, shell=True)

                    # add geometry in visual tag
                    geometry_elem = ET.Element('geometry')
                    mesh_elem = ET.Element("mesh")
                    mesh_elem.set("filename", filename)
                    geometry_elem.append(mesh_elem)
                    link_visual.remove(link_visual_geometry)
                    link_visual.append(geometry_elem)

    # replace collision tag by mesh
    ## remove initial collision
    for link in urdf_root.findall("link"):
        for link_collision in link.findall("collision"):
            link.remove(link_collision)

    ## copy from visual
    for link in urdf_root.findall("link"):
        collision_tag = ET.Element("collision")
        link_name = link.attrib["name"]
        collision_tag.set("name", link_name)
        visual_exist = False
        for link_visual in link.findall("visual"):
            visual_exist = True
            for link_visual_elem in link_visual:
                collision_tag.append(link_visual_elem)
        if visual_exist:
            link.append(collision_tag)

    # get actuator list
    for transmission in urdf_root.findall("transmission"):
        for transmission_joint in transmission.findall("joint"):
            name = transmission_joint.attrib["name"]
            for transmission_joint_hardwareinterface in transmission_joint.findall("hardwareInterface"):
                if transmission_joint_hardwareinterface.text == "RotorInterface":
                    rotor_list.append(name)
                if transmission_joint_hardwareinterface.text == "hardware_interface/EffortJointInterface":
                    joint_list.append(name)
        urdf_root.remove(transmission)

    # output modified urdf
    xmlstr = minidom.parseString(ET.tostring(urdf_root)).toprettyxml(indent="  ")
    with open(urdf_path, 'w') as f:
        f.write(xmlstr)

    # remove blank lines in urdf
    cmd = "sed -i '/^[[:space:]]*$/d' {}".format(urdf_path)
    proc = subprocess.run(cmd, shell=True)


def generate_xml(urdf_path, mujoco_path):
    # compile by mujoco
    aerial_robot_simulation_path = rospack.get_path("aerial_robot_simulation")
    mujoco_compile_path = aerial_robot_simulation_path + "/build/mujoco-2.3.7/bin/compile"
    cmd = "{} {} {}".format(mujoco_compile_path, urdf_path, mujoco_path)
    proc = subprocess.run(cmd, shell=True)

def process_xml(urdf_path, mujoco_path):
    mujoco_tree = ET.parse(mujoco_path)
    mujoco_root = mujoco_tree.getroot()
    urdf_tree = ET.parse(urdf_path)
    urdf_root = urdf_tree.getroot()

    # (child, parent)形式のマップを生成する方法
    mujoco_parent_map = dict((c, p) for p in mujoco_tree.iter() for c in p)
    urdf_parent_map = dict((c, p) for p in urdf_tree.iter() for c in p)

    # get m_f_rate from urdf
    m_f_rate = 0.0
    for m_f_rate_elem in urdf_root.iter("m_f_rate"):
        m_f_rate = m_f_rate_elem.attrib["value"]

    compiler = mujoco_root.find("compiler")
    compiler.set("balanceinertia", "true")

    # process joints
    thrusts = ""
    rotor_axis_dict = {}
    for joint in mujoco_root.iter("joint"):
        ## for joint
        if "joint" in joint.attrib["name"]:
            joint.set("damping", "150") # modify damping
        ## for rotor
        if "rotor" in joint.attrib["name"]:
            ### get control range
            thrusts = joint.attrib["range"]

            ### get rotor axis for counter torque
            axis = joint.attrib["axis"].split()[2]
            rotor_axis_dict[joint.attrib["name"]] = axis

            ### add site at rotor
            parent_body = mujoco_parent_map[joint]
            site_elem = ET.Element("site")
            site_elem.set("name", joint.attrib["name"])
            site_elem.set("pos", "0 0 0")
            parent_body.remove(joint)
            parent_body.append(site_elem)

    # surround root link by body tag
    ## find root link
    root_link_name = ""
    for joint in urdf_root.iter("joint"):
        is_root_joint = False
        for joint_elem in joint:
            if joint_elem.tag == "parent" and joint_elem.attrib["link"] == "root":
                is_root_joint = True
        if is_root_joint:
            for joint_elem in joint:
                if joint_elem.tag == "child":
                    root_link_name = joint_elem.attrib["link"]

    ## get root link inertial
    root_link_mass = ""
    root_link_origin = ""
    root_link_inertia = ""
    for link in urdf_root.iter("link"):
        if link.attrib["name"] == root_link_name:
            for link_elem in link:
                if(link_elem.tag == "inertial"):
                    link_inertial = link_elem
                    for link_inertial_elem in link_inertial:
                        if link_inertial_elem.tag == "mass":
                            root_link_mass = link_inertial_elem.attrib["value"]
                        if link_inertial_elem.tag == "origin":
                            root_link_origin = link_inertial_elem.attrib["xyz"]
                        if link_inertial_elem.tag == "inertia":
                            root_link_inertia = link_inertial_elem.attrib["ixx"] + " " + link_inertial_elem.attrib["iyy"] + " " + link_inertial_elem.attrib["izz"] + " " + link_inertial_elem.attrib["ixy"] + " " + link_inertial_elem.attrib["ixz"] + " " + link_inertial_elem.attrib["iyz"]

    ## add root link body
    worldbody_elem_list = []
    for worldbody in mujoco_root.iter("worldbody"):
        for worldbody_elem in worldbody:
            worldbody_elem_list.append(worldbody_elem)
        for worldbody_elem in  worldbody_elem_list:
            worldbody.remove(worldbody_elem)

        root_link_elem = ET.Element("body")
        root_link_elem.set("name", root_link_name)
        root_link_elem.set("pos", "0 0 0.2")
        inertial_elem = ET.Element("inertial")
        inertial_elem.set("pos", root_link_origin)
        inertial_elem.set("mass", root_link_mass)
        inertial_elem.set("fullinertia", root_link_inertia)
        root_link_elem.append(inertial_elem)
        freejoint_elem = ET.Element("freejoint")
        root_link_elem.append(freejoint_elem)

        for worldbody_elem in worldbody_elem_list:
            root_link_elem.append(worldbody_elem)
        worldbody.append(root_link_elem)

    # add site at fc
    fc_name = ""
    fc_joint = ""
    fc_parent = ""
    fc_rel_pos = ""
    ## find fc name
    for baselink in urdf_root.iter("baselink"):
        fc_name = baselink.attrib["name"]
    ## find parent link
    for child in urdf_root.iter("child"):
        if child.attrib["link"] == fc_name:
            fc_joint = urdf_parent_map[child]
            for fc_joint_elem in fc_joint:
                if fc_joint_elem.tag == "parent":
                    fc_parent = fc_joint_elem.attrib["link"]
                if fc_joint_elem.tag == "origin":
                    fc_rel_pos = fc_joint_elem.attrib["xyz"]
    ## add site fc
    for mujoco_body in mujoco_root.iter("body"):
        if mujoco_body.attrib["name"] == fc_parent:
            site_elem = ET.Element("site")
            site_elem.set("name", fc_name)
            site_elem.set("pos", fc_rel_pos)
            mujoco_body.append(site_elem)


    # add inertial to fixed links connected to root link
    ## search urdf joint
    ### ルートリンクからfixedなジョイントでつながっていて、visualとinertialを持っているリンクの、メッシュの名前とinertialをとってくる。
    child_link_name_list = []
    #### fixedなジョイントでつながっているリンク名をとってくる
    for urdf_joint in urdf_root.iter("joint"):
        if urdf_joint.attrib["type"] == "fixed":
            is_parent_root = False
            parent_link_name = ""
            child_link_name = ""
            child_link_origin_xyz = ""
            child_link_origin_rpy = ""
            for urdf_joint_elem in urdf_joint:
                if urdf_joint_elem.tag == "parent":
                    parent_link_name = urdf_joint_elem.attrib["link"]
                    if parent_link_name == root_link_name:
                        is_parent_root = True
                if urdf_joint_elem.tag == "child":
                    child_link_name = urdf_joint_elem.attrib["link"]
            if is_parent_root:
                child_link_name_list.append(child_link_name)


    #### visualとinertialを探す
    mesh_list = []
    mass_list = []
    origin_pos_list = []
    inertia_list = []
    link_list = []
    for child_link_name in child_link_name_list:
        for link in urdf_root.iter("link"):
            if link.attrib["name"] == child_link_name:
                is_exist_inertial = False
                is_exist_visual = False
                inertial_mass = ""
                inertial_origin_pos = ""
                inertial_inertia = ""
                for link_elem in link:
                    if link_elem.tag == "visual":
                        is_exist_visual = True
                    if link_elem.tag == "inertial":
                        is_exist_inertial = True
                        for link_inertial_elem in link_elem:
                            if link_inertial_elem.tag == "mass":
                                inertial_mass = link_inertial_elem.attrib["value"]
                            if link_inertial_elem.tag == "origin":
                                inertial_origin_pos = link_inertial_elem.attrib["xyz"]
                            if link_inertial_elem.tag == "inertia":
                                inertial_inertia = link_inertial_elem.attrib["ixx"] + " " + link_inertial_elem.attrib["iyy"] + " " + link_inertial_elem.attrib["izz"] + " " + link_inertial_elem.attrib["ixy"] + " " + link_inertial_elem.attrib["ixz"] + " " + link_inertial_elem.attrib["iyz"]

                if is_exist_inertial and is_exist_visual:
                    mass_list.append(inertial_mass)
                    origin_pos_list.append(inertial_origin_pos)
                    inertia_list.append(inertial_inertia)
                    link_list.append(link.attrib["name"])

    # output modified mujoco model
    xmlstr = minidom.parseString(ET.tostring(mujoco_root)).toprettyxml(indent="  ")
    with open(mujoco_path, "w") as f:
        f.write(xmlstr)

    # remove brank line in xml
    cmd = "sed -i '/^[[:space:]]*$/d' {}".format(mujoco_path)
    proc = subprocess.run(cmd, shell=True)

    # reload mujoco model
    mujoco_tree = ET.parse(mujoco_path)
    mujoco_root = mujoco_tree.getroot()
    mujoco_parent_map = dict((c, p) for p in mujoco_tree.iter() for c in p)

    ### xmlを修正
    for geom in mujoco_root.iter("geom"):
        if geom.attrib["name"] in link_list:
            geom_pos = "0 0 0"
            geom_quat = "0 0 0 1"
            if "pos" in geom.attrib:
                geom_pos = geom.attrib["pos"]
            if "quat" in geom.attrib:
                geom_quat = geom.attrib["quat"]
            parent_body = mujoco_parent_map[geom]
            index = link_list.index(geom.attrib["name"])
            body_elem = ET.Element("body")
            body_elem.set("name", link_list[index])
            body_elem.set("pos", geom_pos)
            body_elem.set("quat", geom_quat)
            inertial_elem = ET.Element("inertial")
            inertial_elem.set("pos", origin_pos_list[index])
            inertial_elem.set("mass", mass_list[index])
            inertial_elem.set("fullinertia", inertia_list[index])
            body_elem.append(inertial_elem)
            parent_body.append(body_elem)

    # process geoms
    for geom in mujoco_root.iter("geom"):
        geom.set("contype", "1")
        geom.set("conaffinity", "0")

    # actuators
    global rotor_list
    global joint_list
    actuator_elem = ET.Element("actuator")
    ## rotors
    for rotor in rotor_list:
        rotor_elem = ET.Element("motor")
        rotor_elem.set("name", rotor)
        rotor_elem.set("ctrllimited", "true")
        rotor_elem.set("ctrlrange", thrusts)
        rotor_elem.set("gear", "0 0 1 0 0 " + str(float(m_f_rate) * float(rotor_axis_dict[rotor])))
        rotor_elem.set("site", rotor)
        actuator_elem.append(rotor_elem)
    ## joints
    for joint in joint_list:
        joint_elem = ET.Element("position")
        joint_elem.set("name", joint)
        joint_elem.set("kp", "40")
        joint_elem.set("joint", joint)
        actuator_elem.append(joint_elem)
    mujoco_root.append(actuator_elem)

    # sensors
    sensor_elem = ET.Element("sensor")
    acc = ET.Element("accelerometer")
    acc.set("name", "acc")
    acc.set("site", "fc")
    gyro = ET.Element("gyro")
    gyro.set("name", "gyro")
    gyro.set("site", "fc")
    mag = ET.Element("magnetometer")
    mag.set("name", "mag")
    mag.set("site", "fc")
    sensor_elem.append(acc)
    sensor_elem.append(gyro)
    sensor_elem.append(mag)
    mujoco_root.append(sensor_elem)

    # include world
    aerial_robot_simulation_path = rospack.get_path("aerial_robot_simulation")
    world_path = aerial_robot_simulation_path + "/mujoco/world.xml"
    rel_path = os.path.relpath(world_path, get_directory(mujoco_path))
    include_elem = ET.Element("include")
    include_elem.set("file", rel_path)
    mujoco_root.append(include_elem)

    # output modified mujoco model
    xmlstr = minidom.parseString(ET.tostring(mujoco_root)).toprettyxml(indent="  ")
    with open(mujoco_path, "w") as f:
        f.write(xmlstr)

    # remove brank line in xml
    cmd = "sed -i '/^[[:space:]]*$/d' {}".format(mujoco_path)
    proc = subprocess.run(cmd, shell=True)

    # remove intermediate urdf file
    # os.remove(urdf_path)


def convert_dae2stl(meshdir):
    aerial_robot_simulation_path = rospack.get_path("aerial_robot_simulation")
    cmd = "blender -b -P {} -- {}".format(aerial_robot_simulation_path + "/scripts/convert.py", meshdir)
    proc = subprocess.run(cmd, shell=True)


base = os.path.dirname(os.path.abspath(__file__))
config_path = os.path.normpath(os.path.join(base, "../config/model_param.yaml"))

with open(config_path) as file:
    obj = yaml.safe_load(file)
    for package in obj["package"]:
        print(package)
        pkg_path = rospack.get_path(package)
        meshdir = pkg_path + obj[package]["meshdir"]
        convert_dae2stl(meshdir)
        cmd = "rm -r {}".format(pkg_path + "/mujoco")
        proc = subprocess.run(cmd, shell=True)
        for (input_path, filename) in zip(obj[package]["input"], obj[package]["filename"]):
            input_xacro_path = pkg_path + input_path
            workdir_path = pkg_path + "/mujoco/" + filename
            output_urdf_path = workdir_path + "/robot.urdf"

            cmd = "mkdir -p {}".format(workdir_path)
            proc = subprocess.run(cmd, shell=True)

            run_xacro(input_xacro_path, output_urdf_path)

            process_urdf(package, output_urdf_path, workdir_path)

            mujoco_path = workdir_path + "/robot.xml"
            generate_xml(output_urdf_path, mujoco_path)

            process_xml(output_urdf_path, mujoco_path)
