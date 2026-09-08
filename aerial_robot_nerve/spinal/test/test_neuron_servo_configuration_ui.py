#!/usr/bin/env python3

import unittest
from types import SimpleNamespace
from unittest import mock

from spinal import board_configurator
from spinal import servo_monitor
from spinal.msg import ServoInfo
from spinal.srv import SetDirectServoConfigRequest


class FakeTreeView(object):
    def setModel(self, model):
        self.model = model

    def setUniformRowHeights(self, _enabled):
        pass

    def setFirstColumnSpanned(self, *_args):
        pass

    def rootIndex(self):
        return None

    def setColumnWidth(self, *_args):
        pass

    def expandAll(self):
        pass

    def show(self):
        pass


def make_neuron_response():
    servo = SimpleNamespace(
        id=2,
        configured=True,
        bus_type=ServoInfo.BUS_TTL,
        mode=ServoInfo.MODE_MIT,
        servo_mode=ServoInfo.SERVO_MODE_CURRENT,
        kp_integer=3,
        kp_decimal=2500,
        kd_integer=0,
        kd_decimal=5000,
        p_gain=1200,
        i_gain=0,
        d_gain=500,
        profile_velocity=0,
        current_limit=2352,
        send_data_flag=True,
        external_encoder_flag=False,
        joint_resolution=1,
        servo_resolution=1,
    )
    board = SimpleNamespace(
        slave_id=1,
        imu_send_data_flag=True,
        dynamixel_ttl_rs485_mixed=True,
        servo_pulley_skip_thresh=65535,
        servo_feedback_frequency=200,
        servo_target_watchdog_timeout=0.1,
        servos=[servo],
    )
    return SimpleNamespace(boards=[board])


def item_children(item):
    return {
        item.child(row, 0).text(): item.child(row, 1).text()
        for row in range(item.rowCount())
        if item.child(row, 1) is not None
    }


class NeuronServoConfigurationUiTest(unittest.TestCase):
    def test_board_configurator_displays_neuron_configuration(self):
        tree = FakeTreeView()
        fake = SimpleNamespace(
            get_board_info_client_=make_neuron_response,
            _widget=SimpleNamespace(boardInfoTreeView=tree),
            joint_id_name_map={},
        )

        board_configurator.BoardConfigurator.updateButtonCallback(fake)

        board_item = fake.model.item(0, 0)
        board_values = item_children(board_item)
        self.assertEqual(board_values['servo_feedback_frequency'], '200')
        self.assertEqual(board_values['servo_target_watchdog'], '0.1')

        servo_group = next(
            board_item.child(row, 0)
            for row in range(board_item.rowCount())
            if board_item.child(row, 0).text().startswith('servo (')
        )
        servo_values = item_children(servo_group.child(0, 0))
        self.assertEqual(servo_values['configured'], 'True')
        self.assertEqual(servo_values['bus_type'], 'T')
        self.assertEqual(servo_values['mode'], 'MIT')
        self.assertEqual(servo_values['servo_mode'], 'Current')
        self.assertEqual(servo_values['mit_pd'], '3.2500, 0.5000')

    def test_servo_monitor_displays_neuron_configuration(self):
        headers = [
            'torque', 'configured', 'bus', 'mode', 'servo_mode', 'mit_pd',
            'joint name', 'board', 'index', 'id', 'angle', 'temperature',
            'load', 'error', 'pid_gains', 'profile_velocity',
            'current_limit', 'send_data_flag'
        ]
        fake = SimpleNamespace(
            get_board_info_client_=make_neuron_response,
            joint_id_name_map={},
            _headers=headers,
        )

        servo_monitor.ServoMonitor.updateButtonCallback(fake)

        row = fake._table_data[0]
        self.assertEqual(row[headers.index('configured')], 'True')
        self.assertEqual(row[headers.index('bus')], 'T')
        self.assertEqual(row[headers.index('mode')], 'MIT')
        self.assertEqual(row[headers.index('servo_mode')], 'Current')
        self.assertEqual(row[headers.index('mit_pd')], 'Kp=3.2500, Kd=0.5000')

    def test_neuron_servo_configuration_uses_addressed_service(self):
        cases = [
            ('bus_type', 'T', 'SET_SERVO_BUS_TYPE',
             [ServoInfo.BUS_TTL]),
            ('mode', 'MIT', 'SET_SERVO_CONTROL_MODE',
             [ServoInfo.MODE_MIT]),
            ('mit_pd', '3.25, 0.5', 'SET_SERVO_MIT_PD',
             [3, 2500, 0, 5000]),
            ('pid_gain', '1200, 0, 500', 'SET_SERVO_PID_GAIN',
             [1200, 0, 500]),
            ('profile_velocity', '10', 'SET_SERVO_PROFILE_VEL', [10]),
            ('send_data_flag', 'true', 'SET_SERVO_SEND_DATA_FLAG', [1]),
            ('current_limit', '2352', 'SET_SERVO_CURRENT_LIMIT', [2352]),
            ('external_encoder_flag', 'false',
             'SET_SERVO_EXTERNAL_ENCODER_FLAG', [0]),
            ('resolution[joint:servo]', '1:1',
             'SET_SERVO_RESOLUTION_RATIO', [1, 1]),
            ('servo_feedback_frequency', '200',
             'SET_SERVO_FEEDBACK_FREQUENCY', [200]),
            ('servo_target_watchdog', '0.1',
             'SET_SERVO_TARGET_WATCHDOG_TIMEOUT', [100]),
            ('dynamixel_ttl_rs485_mixed', 'true',
             'SET_DYNAMIXEL_TTL_RS485_MIXED', [1]),
        ]

        for command, text, constant_name, expected_data in cases:
            with self.subTest(command=command):
                requests = []

                def direct_config_client(request):
                    requests.append(request)
                    return SimpleNamespace(success=True)

                def legacy_config_client(_request):
                    self.fail(
                        'Neuron servo configuration used legacy '
                        'set_board_config')

                fake = SimpleNamespace(
                    _board_id='1',
                    _servo_index='0',
                    _raw_servo_id='2',
                    _current_servo_serial_index=0,
                    _command=command,
                    _widget=SimpleNamespace(
                        lineEdit=SimpleNamespace(text=lambda: text)
                    ),
                    direct_servo_config_client_=direct_config_client,
                    set_board_config_client_=legacy_config_client,
                    disableServoTorque=lambda: None,
                    updateButtonCallback=lambda: None,
                )

                with mock.patch.object(board_configurator.rospy, 'sleep'):
                    board_configurator.BoardConfigurator.configureButtonCallback(
                        fake)

                self.assertEqual(len(requests), 1)
                request = requests[0]
                self.assertIsInstance(request, SetDirectServoConfigRequest)
                self.assertEqual(request.board_id, 1)
                self.assertEqual(request.servo_index, 0)
                self.assertEqual(
                    request.command, getattr(request, constant_name))
                self.assertEqual(list(request.data), expected_data)

    def test_neuron_board_configuration_keeps_legacy_service(self):
        direct_requests = []
        legacy_requests = []
        fake = SimpleNamespace(
            _board_id='1',
            _servo_index=None,
            _raw_servo_id=None,
            _current_servo_serial_index=None,
            _command='imu_send_data_flag',
            _widget=SimpleNamespace(
                lineEdit=SimpleNamespace(text=lambda: 'true')
            ),
            direct_servo_config_client_=lambda request: direct_requests.append(
                request),
            set_board_config_client_=lambda request: (
                legacy_requests.append(request) or SimpleNamespace(success=True)
            ),
            disableServoTorque=lambda: None,
            updateButtonCallback=lambda: None,
        )

        with mock.patch.object(board_configurator.rospy, 'sleep'):
            board_configurator.BoardConfigurator.configureButtonCallback(fake)

        self.assertEqual(direct_requests, [])
        self.assertEqual(len(legacy_requests), 1)
        self.assertEqual(list(legacy_requests[0].data), [1, 1])

    def test_servo_monitor_neuron_calibration_uses_addressed_service(self):
        headers = [
            'torque', 'configured', 'bus', 'mode', 'servo_mode', 'mit_pd',
            'joint name', 'board', 'index', 'id', 'angle', 'temperature',
            'load', 'error', 'pid_gains', 'profile_velocity',
            'current_limit', 'send_data_flag'
        ]
        values = {'board': '1', 'index': '0', 'id': '2'}
        requests = []

        class FakeTable(object):
            def currentIndex(self):
                return SimpleNamespace(row=lambda: 0)

            def item(self, _row, column):
                return SimpleNamespace(text=lambda: values[headers[column]])

        fake = SimpleNamespace(
            _headers=headers,
            _widget=SimpleNamespace(
                servoTableWidget=FakeTable(),
                homingOffsetLineEdit=SimpleNamespace(text=lambda: '1234'),
            ),
            servo_torque_pub_=SimpleNamespace(publish=lambda _msg: None),
            set_direct_servo_config_client_=lambda request: (
                requests.append(request) or SimpleNamespace(success=True)
            ),
            set_board_config_client_=lambda _request: self.fail(
                'Neuron calibration used legacy set_board_config'),
        )

        with mock.patch.object(servo_monitor.rospy, 'sleep'):
            servo_monitor.ServoMonitor.jointCalib(fake)

        self.assertEqual(len(requests), 1)
        self.assertEqual(requests[0].board_id, 1)
        self.assertEqual(requests[0].servo_index, 0)
        self.assertEqual(list(requests[0].data), [1234])


if __name__ == '__main__':
    unittest.main()
