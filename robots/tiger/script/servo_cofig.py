#!/usr/bin/env python

import os
import rospy
from spinal.srv import *
import rosgraph
from argparse import ArgumentParser


rospy.init_node("tiger_neuron_configure")

parser = ArgumentParser()
parser.add_argument('-n', nargs='+', dest="neuron_id", help='board id(s) to configure')
args = parser.parse_args()

robot_ns = ''
if robot_ns == '':
    master = rosgraph.Master('/rostopic')
    try:
        _, subs, _ = master.getSystemState()
    except socket.error:
        raise ROSTopicIOException("Unable to communicate with master!")

    teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
    if len(teleop_topics) == 1:
        robot_ns = teleop_topics[0].split('/teleop')[0]


get_board_info_client = rospy.ServiceProxy(robot_ns + '/get_board_info', GetBoardInfo)
set_board_config_client = rospy.ServiceProxy(robot_ns + '/set_board_config', SetBoardConfig)

configure_commands = ['SET_SERVO_SEND_DATA_FLAG',
                      'SET_SERVO_PID_GAIN',
                      'SET_SERVO_PROFILE_VEL',
                      'SET_SERVO_CURRENT_LIMIT',
                      'SET_SERVO_EXTERNAL_ENCODER_FLAG',
                      'SET_SERVO_RESOLUTION_RATIO']

configure_commands = ['SET_SERVO_PID_GAIN']

req = SetBoardConfigRequest()
for neuron_id in args.neuron_id:
    neuron_id = int(neuron_id)
    for servo_id in range(4):
        for command in configure_commands:
            req.command = getattr(req, command)

            if command == 'SET_SERVO_SEND_DATA_FLAG':
                req.data = [neuron_id, servo_id, 1]

            if command == 'SET_SERVO_PID_GAIN':
                if servo_id == 0 or servo_id == 1: #joints
                    req.data = [neuron_id, servo_id, 3000, 0, 2500]
                elif servo_id == 2: # gimbal roll
                    req.data = [neuron_id, servo_id, 1200, 0, 10000]
                else: # gimbal pitch
                    req.data = [neuron_id, servo_id, 3000, 200, 2500]

            if command == 'SET_SERVO_PROFILE_VEL':
                if servo_id == 0 or servo_id == 1: # joint yaw/pitch
                    req.data = [neuron_id, servo_id, 30]
                else:
                    req.data = [neuron_id, servo_id, 0]

            if command == 'SET_SERVO_CURRENT_LIMIT':
                if servo_id == 0 or servo_id == 1: # joint yaw/pitch
                    req.data = [neuron_id, servo_id, 350]
                else:
                    req.data = [neuron_id, servo_id, 0]

            if command == 'SET_SERVO_EXTERNAL_ENCODER_FLAG':
                req.data = [neuron_id, servo_id, 0]

            if command ==  'SET_SERVO_RESOLUTION_RATIO':
                req.data = [neuron_id, servo_id, 1, 1]

            try:
                rospy.loginfo('neuron: {}, servo: {}, command: {} ({})'.format(neuron_id, servo_id, command, req.command))
                rospy.loginfo('data: {}'.format(req.data))

                res = set_board_config_client(req)
                rospy.loginfo(bool(res.success))
            except rospy.ServiceException as e:
                print("/set_board_config service call failed: {}".format(e))
            rospy.sleep(5)
