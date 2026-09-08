#!/usr/bin/env python3

import pathlib
import unittest


REPO = pathlib.Path(__file__).resolve().parents[3]
NEURON_SERVO_CPP = (
    REPO / 'aerial_robot_nerve/neuron/neuronlib/Servo/servo.cpp'
).read_text()
NEURON_SERVO_H = (
    REPO / 'aerial_robot_nerve/neuron/neuronlib/Servo/servo.h'
).read_text()
NEURON_DXL_CPP = (
    REPO /
    'aerial_robot_nerve/neuron/neuronlib/Servo/Dynamixel/dynamixel_serial.cpp'
).read_text()
NEURON_DXL_H = (
    REPO /
    'aerial_robot_nerve/neuron/neuronlib/Servo/Dynamixel/dynamixel_serial.h'
).read_text()
NEURON_INITIALIZER_CPP = (
    REPO / 'aerial_robot_nerve/neuron/neuronlib/Initializer/initializer.cpp'
).read_text()
SPINAL_CAN_SERVO_H = (
    REPO / 'aerial_robot_nerve/spinal/mcu_project/lib/Hydrus_Lib/'
    'CANDevice/servo/can_servo.h'
).read_text()
SPINAL_CAN_SERVO_CPP = (
    REPO / 'aerial_robot_nerve/spinal/mcu_project/lib/Hydrus_Lib/'
    'CANDevice/servo/can_servo.cpp'
).read_text()
SPINE_CPP = (
    REPO / 'aerial_robot_nerve/spinal/mcu_project/lib/Hydrus_Lib/'
    'Spine/spine.cpp'
).read_text()
DIRECT_SERVO_CPP = (
    REPO / 'aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/'
    'servo/servo.cpp'
).read_text()


def function_body(source, signature, next_signature):
    return source.split(signature, 1)[1].split(next_signature, 1)[0]


class NeuronServoFirmwareSafetyTest(unittest.TestCase):

    def test_can_rx_only_queues_dynamixel_configuration(self):
        callback = function_body(
            NEURON_SERVO_CPP,
            'void Servo::receiveDataCallback',
            '#else')
        self.assertIn('queueConfigRequest', callback)
        self.assertNotIn('servo_handler_.configure(', callback)
        self.assertIn('processConfigRequest', NEURON_SERVO_CPP)
        self.assertIn('config_request_busy_', NEURON_SERVO_H)

    def test_legacy_can_configuration_also_uses_servo_task(self):
        self.assertNotIn(
            'servo_.servo_handler_.configure(', NEURON_INITIALIZER_CPP)
        self.assertIn('servo_.queueConfigRequest(', NEURON_INITIALIZER_CPP)

    def test_hardware_configuration_writes_are_checked(self):
        for method in (
                'setHomingOffset', 'setPositionGains',
                'setProfileVelocity', 'setCurrentLimit'):
            self.assertIn('bool %s(uint8_t servo_index)' % method,
                          NEURON_DXL_H)
            self.assertIn('bool DynamixelSerial::%s' % method,
                          NEURON_DXL_CPP)
            self.assertIn('if (!%s(servo_index))' % method,
                          NEURON_DXL_CPP)
        self.assertIn('old_p_gain', NEURON_DXL_CPP)
        self.assertIn('old_profile_velocity', NEURON_DXL_CPP)
        self.assertIn('old_current_limit', NEURON_DXL_CPP)

    def test_bus_type_is_not_required_before_unrelated_configuration(self):
        configure = function_body(
            NEURON_DXL_CPP,
            'uint8_t DynamixelSerial::configure',
            'void DynamixelSerial::updateTargetWatchdogs')
        self.assertIn('findOrCreateServoFlashConfig(servo_index)', configure)
        self.assertNotIn(
            'if (config == nullptr) return CAN::SERVO_CONFIG_RESULT_INVALID;',
            configure)
        valid_flash = function_body(
            NEURON_DXL_CPP,
            'bool DynamixelSerial::validFlashConfig',
            'NeuronServoFlashConfig* DynamixelSerial::findServoFlashConfig')
        self.assertIn('NEURON_SERVO_BUS_UNKNOWN', valid_flash)

    def test_one_unconfigured_servo_does_not_block_configured_servos(self):
        set_torque = function_body(
            NEURON_DXL_CPP,
            'void DynamixelSerial::setTorque',
            'void DynamixelSerial::startTargetCycle')
        bulk_write = function_body(
            NEURON_DXL_CPP,
            'void DynamixelSerial::cmdBulkWriteGoalStates',
            'void DynamixelSerial::cmdReadPresentCVP')
        self.assertNotIn('busMapValid()', set_torque)
        self.assertNotIn('if (!busMapValid()) return;', bulk_write)
        self.assertIn('if (!servo.configured()) continue;', bulk_write)

    def test_neuron_feedback_frequency_accepts_supported_range(self):
        configure = function_body(
            NEURON_DXL_CPP,
            'uint8_t DynamixelSerial::configure',
            'void DynamixelSerial::updateTargetWatchdogs')
        self.assertIn('data[0] <= 0 || data[0] > 200', configure)
        self.assertNotIn(
            'data[0] != NEURON_SERVO_FEEDBACK_FREQUENCY_HZ', configure)

    def test_status_packet_and_ping_have_bounds_checks(self):
        parser = function_body(
            NEURON_DXL_CPP,
            'int8_t DynamixelSerial::readStatusPacket',
            'void DynamixelSerial::cmdPing')
        self.assertIn('parameter_len > STATUS_PACKET_SIZE - 2', parser)
        self.assertIn('parameter_index >= STATUS_PACKET_SIZE', parser)
        self.assertIn('loop_count >= STATUS_PACKET_SIZE', parser)
        self.assertIn('servo_num_ >= MAX_SERVO_NUM', parser)
        self.assertIn('s != servo_.end()', parser)

    def test_torque_flag_is_applied_before_targets_from_same_frame(self):
        callback = function_body(
            NEURON_SERVO_CPP,
            'void Servo::receiveDataCallback',
            '#else')
        torque_pos = callback.index('servo_handler_.setTorque(')
        position_pos = callback.index('bool has_position')
        self.assertLess(torque_pos, position_pos)

    def test_neuron_servo_requires_explicit_torque_enable(self):
        self.assertIn('torque_command_(false)', SPINAL_CAN_SERVO_H)
        self.assertIn('torque_feedback_(false)', SPINAL_CAN_SERVO_H)
        position_callback = function_body(
            SPINE_CPP, 'void servoPositionCallback',
            'void servoCurrentCallback')
        current_callback = function_body(
            SPINE_CPP, 'void servoCurrentCallback',
            'void servoTorqueControlCallback')
        torque_callback = function_body(
            SPINE_CPP, 'void servoTorqueControlCallback',
            'void boardConfigCallback')
        self.assertNotIn('servo.setTorqueEnable(true)', position_callback)
        self.assertNotIn('servo.setTorqueEnable(true)', current_callback)
        self.assertIn('servo.setTorqueEnable(enable)', torque_callback)

    def test_neuron_torque_feedback_cannot_overwrite_command(self):
        set_torque = function_body(
            SPINAL_CAN_SERVO_CPP, 'void Servo::setTorqueEnable',
            'uint8_t CANServo::requestConfig')
        send_data = function_body(
            SPINAL_CAN_SERVO_CPP, 'void CANServo::sendData',
            'void CANServo::receiveDataCallback')
        receive_data = SPINAL_CAN_SERVO_CPP.split(
            'void CANServo::receiveDataCallback', 1)[1]
        self.assertIn(
            'if (torque_enable && force_servo_off_) return;', set_torque)
        self.assertNotIn('if (force_servo_off_) return;', set_torque)
        self.assertIn('torque_command_ = torque_enable', set_torque)
        self.assertIn(
            'servo.torque_command_ && !servo.force_servo_off_', send_data)
        self.assertIn('servo.torque_feedback_ =', receive_data)
        self.assertNotIn(
            'servo.torque_command_ = servo.torque_feedback_', receive_data)
        self.assertIn(
            'if (servo.force_servo_off_) servo.torque_command_ = false;',
            receive_data)

    def test_direct_servo_target_keeps_debug_auto_enable_behavior(self):
        set_targets = function_body(
            DIRECT_SERVO_CPP, 'bool DirectServo::setGoalTargets',
            'bool DirectServo::setTorqueEnable')
        self.assertIn('if (!servo.torque_enable_)', set_targets)
        self.assertIn('servo_handler_.setTorque(index)', set_targets)


if __name__ == '__main__':
    unittest.main()
