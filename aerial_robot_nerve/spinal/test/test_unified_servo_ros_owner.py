#!/usr/bin/env python3

import pathlib
import unittest


SPINAL_DIR = pathlib.Path(__file__).resolve().parents[1]
DIRECT_SERVO_CPP = (
    SPINAL_DIR / 'mcu_project/lib/Jsk_Lib/servo/servo.cpp'
).read_text()
SPINE_CPP = (
    SPINAL_DIR / 'mcu_project/lib/Hydrus_Lib/Spine/spine.cpp'
).read_text()
H7_NODE_HANDLE = (
    SPINAL_DIR / 'mcu_project/boards/stm32H7/ros_lib/ros/node_handle.h'
).read_text()
H7_V2_NODE_HANDLE = (
    SPINAL_DIR / 'mcu_project/boards/stm32H7_v2/ros_lib/ros/node_handle.h'
).read_text()
H7_MAIN = (
    SPINAL_DIR / 'mcu_project/boards/stm32H7/Src/main.c'
).read_text()
H7_V2_MAIN = (
    SPINAL_DIR / 'mcu_project/boards/stm32H7_v2/Src/main.c'
).read_text()
DIRECT_SERVO_HEADER = (
    SPINAL_DIR / 'mcu_project/lib/Jsk_Lib/servo/servo.h'
).read_text()
SERVO_BRIDGE_CPP = (
    SPINAL_DIR.parents[1] /
    'aerial_robot_model/src/servo_bridge/servo_bridge.cpp'
).read_text()
ROSSERIAL_TOPIC_HANDLERS = (
    SPINAL_DIR.parents[2] /
    'rosserial/rosserial_server/include/rosserial_server/topic_handlers.h'
).read_text()


class UnifiedServoRosOwnerTest(unittest.TestCase):

    def test_direct_servo_does_not_register_shared_ros_interfaces(self):
        shared_registrations = (
            'nh_->subscribe(servo_ctrl_sub_)',
            'nh_->subscribe(servo_torque_ctrl_sub_)',
            'nh_->advertise(servo_state_pub_)',
            'nh_->advertise(servo_torque_state_pub_)',
            'nh_->advertiseService(servo_config_srv_)',
            'nh_->advertiseService(board_info_srv_)',
        )
        for registration in shared_registrations:
            self.assertNotIn(registration, DIRECT_SERVO_CPP)

    def test_spine_board_info_always_contains_board_zero(self):
        self.assertIn(
            'board_info_res_.boards_length = slave_num_ + 1', SPINE_CPP)
        self.assertIn('fillDirectServoBoardInfo', SPINE_CPP)

    def test_spine_dispatches_direct_servo_configuration(self):
        self.assertIn(
            'req.board_id == CAN::MASTER_ID', SPINE_CPP)
        self.assertIn('requestConfigByIndex', SPINE_CPP)

    def test_empty_spinal_board_reports_startup_snapshot(self):
        self.assertIn('DirectServoBoardSettings', DIRECT_SERVO_HEADER)
        self.assertIn('getBoardSettings()', DIRECT_SERVO_HEADER)
        self.assertIn('direct_servo_settings_', SPINE_CPP)
        for main in (H7_MAIN, H7_V2_MAIN):
            self.assertIn('servo_.getBoardSettings()', main)
            self.assertIn('&direct_servo_settings', main)

    def test_empty_spinal_board_rejects_configuration(self):
        self.assertNotIn('configureEmptyDirectServoBoard', SPINE_CPP)
        callback = SPINE_CPP.split('void servoConfigCallback', 1)[1]
        callback = callback.split('\n  bool init(', 1)[0]
        self.assertIn('if (direct_servo_ == nullptr) return;', callback)

    def test_startup_configuration_skips_boards_without_servos(self):
        self.assertGreaterEqual(
            SERVO_BRIDGE_CPP.count(
                'if (detected_board.servos.empty()) continue;'),
            3)

    def test_neuron_servo_configuration_does_not_pause_all_can_tx(self):
        callback = SPINE_CPP.split('void servoConfigCallback', 1)[1]
        callback = callback.split('\n  bool init(', 1)[0]
        self.assertNotIn('can_tx_idle_start_time_', callback)

    def test_spine_global_index_includes_direct_servos_first(self):
        self.assertIn('direct_servo_num_', SPINE_CPP)
        self.assertIn('setDirectServoTargets', SPINE_CPP)
        self.assertIn('setDirectServoTorque', SPINE_CPP)

    def test_neuron_feedback_frequency_controls_ros_publish_period(self):
        publish = SPINE_CPP.split('void servoPublish()', 1)[1]
        self.assertIn('getServoFeedbackFrequency()', publish)
        self.assertNotIn('SERVO_PUB_INTERVAL', publish)

    def test_rosserial_mcu_validates_subscriber_topic_id(self):
        for node_handle in (H7_NODE_HANDLE, H7_V2_NODE_HANDLE):
            self.assertIn('topic_ >= 100', node_handle)
            self.assertIn('topic_ < 100 + MAX_SUBSCRIBERS', node_handle)

    def test_rosserial_service_is_advertised_only_when_ready(self):
        self.assertIn('request_endpoint_ready_', ROSSERIAL_TOPIC_HANDLERS)
        self.assertIn('response_endpoint_ready_', ROSSERIAL_TOPIC_HANDLERS)
        self.assertIn('advertiseIfReady', ROSSERIAL_TOPIC_HANDLERS)

    def test_startup_configuration_is_one_shot_and_rate_limited(self):
        self.assertIn('direct_servo_config_timer_', SERVO_BRIDGE_CPP)
        self.assertNotIn('direct_servo_config_retry_interval',
                         SERVO_BRIDGE_CPP)
        self.assertIn('DIRECT_SERVO_CONFIG_CALL_INTERVAL = 0.02',
                      SERVO_BRIDGE_CPP)
        self.assertIn('DIRECT_SERVO_CONFIG_STARTUP_DELAY = 1.0',
                      SERVO_BRIDGE_CPP)
        self.assertNotIn('param("direct_servo_config_call_interval"',
                         SERVO_BRIDGE_CPP)
        self.assertNotIn('param("direct_servo_config_startup_delay"',
                         SERVO_BRIDGE_CPP)

    def test_startup_configuration_does_not_gate_control(self):
        self.assertNotIn('disableServoTorqueForConfiguration',
                         SERVO_BRIDGE_CPP)
        self.assertNotIn('direct_servo_config_ready_', SERVO_BRIDGE_CPP)

if __name__ == '__main__':
    unittest.main()
