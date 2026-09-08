#!/usr/bin/env python3

import pathlib
import unittest


REPO = pathlib.Path(__file__).resolve().parents[3]


def read(path):
    return (REPO / path).read_text(encoding='utf-8')


class ServoCurrentBasedPositionModeTest(unittest.TestCase):

    def test_ros_message_has_distinct_logical_and_dynamixel_modes(self):
        servo_info = read('aerial_robot_nerve/spinal/msg/ServoInfo.msg')
        self.assertIn('uint8 MODE_CBP=6', servo_info)
        self.assertIn(
            'uint8 SERVO_MODE_CURRENT_BASED_POSITION=5', servo_info)

    def test_servo_bridge_parses_cbp_and_expects_operating_mode_five(self):
        bridge = read('aerial_robot_model/src/servo_bridge/servo_bridge.cpp')
        self.assertIn(
            'if (mode == "CBP") return spinal::ServoInfo::MODE_CBP;',
            bridge)
        self.assertIn('case spinal::ServoInfo::MODE_CBP:', bridge)
        self.assertIn(
            'return spinal::ServoInfo::SERVO_MODE_CURRENT_BASED_POSITION;',
            bridge)

    def test_direct_servo_writes_current_and_position_for_cbp(self):
        base = read(
            'aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/servo/'
            'drivers/servo_base_driver.h')
        header = read(
            'aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/servo/'
            'drivers/Dynamixel/dynamixel_serial.h')
        source = read(
            'aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/servo/'
            'drivers/Dynamixel/dynamixel_serial.cpp')
        self.assertIn('DIRECT_SERVO_MODE_CBP = 6', base)
        self.assertIn(
            'DYNAMIXEL_CURRENT_BASED_POSITION_CONTROL_MODE 5', header)
        goal_write = source.split(
            'void DynamixelSerial::cmdBulkWriteGoalStates(', 1)[1].split(
                'void DynamixelSerial::recordGoalWrite', 1)[0]
        self.assertIn('case DIRECT_SERVO_MODE_CBP:', goal_write)
        self.assertIn('CTRL_GOAL_CURRENT', goal_write)
        self.assertIn('CTRL_GOAL_POSITION', goal_write)
        self.assertIn('clampGoalCurrentMagnitude', goal_write)
        self.assertIn('ServoMask cbp_mask = 0;', goal_write)
        self.assertIn('param_idx = 0;', goal_write)
        self.assertEqual(goal_write.count('COMMAND_BULK_WRITE'), 2)

    def test_neuron_writes_current_and_position_for_cbp(self):
        header = read(
            'aerial_robot_nerve/neuron/neuronlib/Servo/Dynamixel/'
            'dynamixel_serial.h')
        source = read(
            'aerial_robot_nerve/neuron/neuronlib/Servo/Dynamixel/'
            'dynamixel_serial.cpp')
        self.assertIn('NEURON_SERVO_MODE_CBP = 6', header)
        self.assertIn(
            'case NEURON_SERVO_MODE_CBP: '
            'return CURRENT_BASE_POSITION_CONTROL_MODE;', source)
        goal_write = source.split(
            'void DynamixelSerial::cmdBulkWriteGoalStates()', 1)[1].split(
                'void DynamixelSerial::cmdSyncWriteGoalCurrent', 1)[0]
        self.assertIn('case NEURON_SERVO_MODE_CBP:', goal_write)
        self.assertIn('CTRL_GOAL_CURRENT', goal_write)
        self.assertIn('CTRL_GOAL_POSITION', goal_write)
        self.assertIn('clampGoalCurrentMagnitude', goal_write)
        self.assertIn('uint8_t cbp_mask = 0;', goal_write)
        self.assertIn('param_idx = 0;', goal_write)
        self.assertEqual(goal_write.count('COMMAND_BULK_WRITE'), 2)

    def test_spinal_caches_default_cbp_current_limit_for_neuron(self):
        can_servo = read(
            'aerial_robot_nerve/spinal/mcu_project/lib/Hydrus_Lib/'
            'CANDevice/servo/can_servo.cpp')
        self.assertIn('SERVO_CONTROL_MODE_CBP = 6', can_servo)
        self.assertIn(
            'CAN::SERVO_CONFIG_SET_CONTROL_MODE ||', can_servo)
        self.assertIn(
            'CAN::SERVO_CONFIG_SET_CURRENT_LIMIT)', can_servo)
        self.assertIn('servo.goal_current_ = static_cast<int16_t>', can_servo)

    def test_rqt_displays_cbp_and_detects_mode_mismatch(self):
        configurator = read(
            'aerial_robot_nerve/spinal/src/spinal/board_configurator.py')
        monitor = read(
            'aerial_robot_nerve/spinal/src/spinal/servo_monitor.py')
        self.assertIn("ServoInfo.MODE_CBP: 'CbP'", configurator)
        self.assertIn(
            "ServoInfo.SERVO_MODE_CURRENT_BASED_POSITION: "
            "'Current-based Position'", configurator)
        self.assertIn("'CbP': 'Current-based Position'", monitor)


if __name__ == '__main__':
    unittest.main()
