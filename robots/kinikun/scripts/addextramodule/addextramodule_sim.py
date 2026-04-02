#!/usr/bin/env python

import os, tf, math, rospy
import xml.etree.ElementTree as ET
from geometry_msgs.msg import Transform, Quaternion, Pose, Point
from std_srvs.srv import SetBool
from aerial_robot_model.srv import AddExtraModule, AddExtraModuleRequest
from gazebo_msgs.srv import SpawnModel, DeleteModel

def euler_to_quaternion(roll, pitch, yaw):
    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

def spawn_object(module_name, model_path, link_name):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        tree = ET.parse(model_path)
        root = tree.getroot()
        pose_element = root.find('.//pose')
        
        if pose_element is not None:
            pose_values = list(map(float, pose_element.text.split()))
            x, y, z = pose_values[0:3]
            roll, pitch, yaw = pose_values[3:6]
            pose = Point(x, y, z)
            orientation = euler_to_quaternion(roll, pitch, yaw)
            initial_pose = Pose(pose, orientation)
            print(f"\033[1;32m[Msg] Initial pose set.\033[0m")
        else:
            print("\033[1;91m[Warn] No <pose> tag found in the model file.\033[0m")
            return None, None, None

        inertia_element = root.find('.//inertia')
        if inertia_element is not None:
            inertia = {
                'm': float(inertia_element.find('./mass').text),
                'ixx': float(inertia_element.find('./ixx').text),
                'ixy': float(inertia_element.find('./ixy').text),
                'ixz': float(inertia_element.find('./ixz').text),
                'iyy': float(inertia_element.find('./iyy').text),
                'iyz': float(inertia_element.find('./iyz').text),
                'izz': float(inertia_element.find('./izz').text)
            }
            print(f"\033[1;32m[Msg] Inertia set.\033[0m")
        else:
            print("\033[1;91m[Warn] No <inertia> tag found in the model file.\033[0m")
            return None, None, None

        config = {
            'module_name': module_name,
            'transform': Transform(translation=pose, rotation=orientation),
            'inertia': inertia
        }

        with open(model_path, 'r') as model_file:
            model_sdf = model_file.read()

        return model_sdf, initial_pose, config

    except Exception as e:
        rospy.logerr(f"\033[1;91m[Error] Failed to parse SDF file: {e}\033[0m")
        return None, None, None

def delete_object(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        response = delete_model_service(model_name)
        if response.success:
            print(f"\033[1;32m[Msg] {model_name} has been deleted successfully!\033[0m")
        else:
            print(f"\033[1;91m[Warning] Failed to delete {model_name}.\033[0m")
    except rospy.ServiceException as e:
        print(f"\033[1;91m[Warning] Service call failed: {e}\033[0m")

def call_add_extra_module(action, link_name, config):
    rospy.wait_for_service('/kinikun/add_extra_module')
    try:
        add_extra_module = rospy.ServiceProxy('/kinikun/add_extra_module', AddExtraModule)
        request = AddExtraModuleRequest()
        request.action = action
        request.module_name = config['module_name']
        request.parent_link_name = link_name
        request.transform = config['transform']
        request.inertia.m = config['inertia']['m']
        request.inertia.ixx = config['inertia']['ixx']
        request.inertia.ixy = config['inertia']['ixy']
        request.inertia.ixz = config['inertia']['ixz']
        request.inertia.iyy = config['inertia']['iyy']
        request.inertia.iyz = config['inertia']['iyz']
        request.inertia.izz = config['inertia']['izz']

        response = add_extra_module(request)
        if response.status:
            print("\033[1;32m[Msg] Module added/deleted successfully!\033[0m")
            return True
        else:
            print("\033[1;91m[Error] Failed to add/delete module!\033[0m")
            return False
    except rospy.ServiceException as e:
        print(f"\033[1;91m[Error] Service call failed: {e}\033[0m")
        return False

def main():
    rospy.init_node('add_module_client')
    rospy.sleep(1)

    while True:
        print("\033[1mSelect parent link (e.g., dummy_link_1, dummy_link_2): \033[0m")
        link_name = input().strip()
        if link_name:  # ダミーリンク名が入力された場合
            break
        else:
            print("\033[1;91m[Warn] Please enter a valid link name.\033[0m")

    while True:
        print("\033[1mEnter the module name: \033[0m")
        module_name = input().strip()
        model_path = f'/home/kan/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/models/{module_name}.sdf'  # SDFファイルのパスを指定
        if not os.path.exists(model_path):
            print(f"\033[1;91m[Warning] '{module_name}' model not found. Please enter a valid module name.\033[0m")
        else:
            model_sdf, initial_pose, config = spawn_object(module_name, model_path, link_name)
            if model_sdf is not None:
                break

    while True:
        print("\033[1mEnter 'add' or 'delete': \033[0m")
        operation = input().strip().lower()
        if operation == "add":
            action = 1
            try:
                spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
                spawn_model(module_name, model_sdf, '', initial_pose, 'root')
                if call_add_extra_module(action, link_name, config):
                    print(f"\033[1;32m[Msg] {module_name} model spawned and added successfully!\033[0m")
                    break
            except rospy.ServiceException as e:
                print(f"\033[1;91m[Error] Spawn service call failed: {e}\033[0m")
        elif operation == "delete":
            action = -1
            if call_add_extra_module(action, link_name, config):
                rospy.sleep(2)
                delete_object(module_name)
                break
        else:
            print("\033[1;91m[Error] Invalid operation. Please enter 'add' or 'delete'.\033[0m")

if __name__ == "__main__":
    main()