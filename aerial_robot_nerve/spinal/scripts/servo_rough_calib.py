#!/usr/bin/env python
import rospy
import rospkg
from std_srvs.srv import SetBool, SetBoolRequest
from spinal.srv import SetBoardConfig, SetBoardConfigRequest, GetBoardInfo
from sensor_msgs.msg import JointState
import socket
import rosgraph
import math

class JointRoughCalib:
    def __init__(self):

        # roslaunch parameters declared inside a <node> are private parameters.
        # Normalize the configured namespace so topic and parameter names are
        # absolute (for example, "dragon" becomes "/dragon").
        robot_ns = rospy.get_param("~robot_ns", "")
        robot_ns = robot_ns.strip('/')
        robot_ns = '/' + robot_ns if robot_ns else ''

        if not robot_ns:
            # Fall back to discovering the namespace from get_board_info. The
            # rosserial bridge may still be registering when this node starts,
            # so wait without busy-looping and remain responsive to Ctrl+C.
            master = rosgraph.Master(rospy.get_name())
            board_info_srvs = []
            while not board_info_srvs and not rospy.is_shutdown():
                try:
                    _, _, srvs = master.getSystemState()
                    board_info_srvs = sorted(
                        srv[0] for srv in srvs
                        if srv[0].endswith('/get_board_info'))
                except socket.error as error:
                    rospy.logwarn_throttle(
                        5.0, "unable to communicate with ROS master: %s", error)

                if not board_info_srvs:
                    rospy.loginfo_throttle(
                        5.0, "wait for rosserial get_board_info service...")
                    rospy.sleep(0.2)

            if rospy.is_shutdown():
                return

            # Choose the first robot namespace. A root-level service produces
            # an empty namespace, which correctly resolves to /joint_states.
            robot_ns = board_info_srvs[0][:-len('/get_board_info')]

        rospy.loginfo('robot name is {}'.format(robot_ns))

        # capture the joint angles
        self.joint_states = None
        joint_states_topic = robot_ns + '/joint_states'
        self.joint_sub = rospy.Subscriber(
            joint_states_topic, JointState, self.cb, queue_size=1)
        while self.joint_states is None and not rospy.is_shutdown():
            rospy.loginfo_throttle(
                5.0, "wait for joint states on %s...", joint_states_topic)
            rospy.sleep(0.2)

        if rospy.is_shutdown():
            return

        config = rospy.get_param(robot_ns + '/servo_controller/joints')
        angle_scale = config['angle_scale']
        zero_point_offset = config['zero_point_offset']

        # get calib angles
        calib_angles = rospy.get_param(robot_ns + '/joints/calib_angles', None) # dict

        if calib_angles is None:
            # set calib angles according to

            rospy.loginfo('get joint calib angles from rosparameter ({})'.format(robot_ns + '/servo_controller/joints'))

            calib_angles = dict()
            for conf in [v for k, v in config.items() if 'controller' in k]:

                if 'simulation' not in conf:
                    continue

                simulation_config = conf['simulation']

                if 'init_value' in simulation_config:
                    calib_angles[conf['name']] = simulation_config['init_value']
        else:
            if not isinstance(calib_angles, dict):
                rospy.logwarn('type calib_angles should be dictionary, should not be {}'.format(type(calib_angles)))
                return;


        calib_joints = dict()
        for k, v in calib_angles.items():
            calib_joints[k] = {'calib_angle': v}

        # get neuron info for servo
        joint_id_name_map = {v['id']: v['name'] for k, v in config.items() if 'controller' in k}
        get_board_info_client = rospy.ServiceProxy(robot_ns + '/get_board_info', GetBoardInfo)
        try:
            res = get_board_info_client()
            c = -1
            for i, b in enumerate(res.boards):
                for j, s in enumerate(b.servos):
                    c += 1
                    if c not in joint_id_name_map.keys():
                        # not joint type (maybe gimbal)
                        continue

                    name = joint_id_name_map[c]

                    if name not in calib_joints.keys():
                        # is not in calib target
                        continue

                    config = calib_joints[name]
                    config["slave_id"] = b.slave_id
                    config["servo_id"] = j


        except rospy.ServiceException as e:
            rospy.logerr("/get_board_info service call failed: %s"%e)


        rospy.loginfo('calib joints are {}, angle_scale: {}, zero_point_offset: {}'.format(calib_joints, angle_scale, zero_point_offset))


        # disable joint servo
        rospy.loginfo('disable joint servo')
        self.joint_servo_client_ = rospy.ServiceProxy(robot_ns + '/joints/torque_enable', SetBool)

        try:
            req = SetBoolRequest(False)
            res = self.joint_servo_client_(req)
        except rospy.ServiceException as e:
            print('/joints/torque_enable call failed: {}'.format(e))
        rospy.loginfo('succeed')

        # joint angle calib (update round offset)
        set_board_config_client = rospy.ServiceProxy(robot_ns + '/set_board_config', SetBoardConfig)
        req = SetBoardConfigRequest()
        for k, v in calib_joints.items():
            if k not in self.joint_states.name:
                rospy.logerr("there is no {} in joint states".format(k))
                continue

            index = self.joint_states.name.index(k)
            angle = self.joint_states.position[index]
            ref = v['calib_angle']

            diff = math.fabs(angle - ref)
            thresh = 0.2 # 0.2 rad as a hard-coding angle

            if diff > thresh:
                rospy.loginfo("calibrate {} for large round offset, current: {}, ref: {}".format(k, angle, ref))
                req = SetBoardConfigRequest()
                req.command = 11 # WIP: please refer to spinal/mcu_project/lib/Jsk_Lib/CAN/can_constants.h
                req.data.append(int(v['slave_id']))
                req.data.append(int(v['servo_id']))
                raw_ref = ref / angle_scale + zero_point_offset
                req.data.append(int(raw_ref))
                rospy.loginfo('call service')
                rospy.loginfo('command: ' + str(req.command))
                rospy.loginfo('data: ' + str(req.data))
                try:
                    res = set_board_config_client(req)
                    rospy.loginfo("the result is {}".format(bool(res.success)))
                    timeout = 3
                    rospy.sleep(timeout) # WIP: wait for finish update in spinal-neuron
                    rospy.loginfo('wait {} [sec] for update angle'.format(timeout))

                    # check the joint angle update
                    angle = self.joint_states.position[index]
                    diff = math.fabs(angle - ref)
                    if diff > thresh:
                        rospy.logerr("calibrate fail for {}, current: {}, ref: {}".format(k, angle, ref))
                        return
                except rospy.ServiceException as e:
                    print("/set_board_config service call failed: {}".format(e))


        # enable joint servo again
        rospy.loginfo('enable joint servo')
        self.joint_servo_client_ = rospy.ServiceProxy(robot_ns + '/joints/torque_enable', SetBool)
        try:
            res = self.joint_servo_client_(SetBoolRequest(True))
        except rospy.ServiceException as e:
            print('/joints/torque_enable call failed: {}'.format(e))
        rospy.loginfo('succeed')

    def cb(self, msg):
        self.joint_states = msg

if __name__=="__main__":

    rospy.init_node("joint_rough_calib")

    node = JointRoughCalib()
