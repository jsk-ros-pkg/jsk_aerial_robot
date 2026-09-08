#!/usr/bin/env python3

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
NEURON = ROOT / "aerial_robot_nerve/neuron"
SPINAL = ROOT / "aerial_robot_nerve/spinal"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


class CanTransportSafetyTest(unittest.TestCase):
    def setUp(self):
        self.cores = [
            read(NEURON / "neuronlib/CAN/can_core.cpp"),
            read(SPINAL / "mcu_project/lib/Jsk_Lib/CAN/can_core.cpp"),
        ]
        self.core_headers = [
            read(NEURON / "neuronlib/CAN/can_core.h"),
            read(SPINAL / "mcu_project/lib/Jsk_Lib/CAN/can_core.h"),
        ]
        self.managers = [
            read(NEURON / "neuronlib/CAN/can_device_manager.cpp"),
            read(SPINAL / "mcu_project/lib/Jsk_Lib/CAN/can_device_manager.cpp"),
        ]
        self.manager_headers = [
            read(NEURON / "neuronlib/CAN/can_device_manager.h"),
            read(SPINAL / "mcu_project/lib/Jsk_Lib/CAN/can_device_manager.h"),
        ]

    def test_tx_uses_call_local_header_and_payload(self):
        for source in self.cores:
            self.assertNotRegex(source, r"\n\s*FDCAN_TxHeaderTypeDef tx_header_;" )
            self.assertNotRegex(source, r"\n\s*uint8_t tx_data_\[64\];")
            self.assertIn("FDCAN_TxHeaderTypeDef tx_header = {};", source)
            self.assertIn("uint8_t tx_data[64] = {};", source)

    def test_tx_reports_failure_and_counts_drops(self):
        for header, source in zip(self.core_headers, self.cores):
            self.assertRegex(header, r"bool sendMessage\(")
            self.assertIn("uint32_t getTxDropCount();", header)
            self.assertIn("tx_drop_count", source)
            self.assertIn("return false;", source)

    def test_short_messages_follow_debug_classic_can_behavior(self):
        for source in self.cores:
            self.assertRegex(
                source,
                r"(?s)if \(dlc <= 8\).*?FDCAN_CLASSIC_CAN.*?FDCAN_BRS_OFF",
            )
            self.assertRegex(
                source,
                r"(?s)else.*?FDCAN_FD_CAN.*?FDCAN_BRS_ON",
            )

    def test_rx_accepts_classic_short_and_requires_brs_for_fd(self):
        expected = (
            "return (rx_header.FDFormat == FDCAN_CLASSIC_CAN && dlc <= 8) ||"
        )
        for header, source in zip(self.core_headers, self.managers):
            self.assertIn("bool validFrameFormat", header)
            self.assertIn(expected, header)
            self.assertIn("CAN::validFrameFormat(msg.rx_header, dlc)", source)

    def test_rx_queue_drops_are_counted(self):
        for header, source in zip(self.manager_headers, self.managers):
            self.assertIn("uint32_t getRxDropCount();", header)
            self.assertIn("rx_drop_count", source)
            self.assertGreaterEqual(source.count("recordRxDrop();"), 2)

    def test_neuron_can_tx_is_triggered_by_target_reception(self):
        main = read(NEURON / "neuron_g4/Core/Src/main.c")
        callback = re.search(
            r"void CANDeviceManager::userReceiveMessagesCallback.*?\n}\n",
            main,
            re.DOTALL,
        )
        self.assertIsNotNone(callback)
        self.assertIn("MESSAGEID_RECEIVE_SERVO_TARGET", callback.group(0))
        self.assertNotIn("MESSAGEID_RECEIVE_SERVO_ANGLE", callback.group(0))
        self.assertIn("osSemaphoreRelease(canTxSemHandle)", callback.group(0))

        task = re.search(
            r"void servoTaskCallback.*?\n}\n",
            main,
            re.DOTALL,
        )
        self.assertIsNotNone(task)
        self.assertNotIn("osSemaphoreRelease(canTxSemHandle)", task.group(0))

    def test_neuron_torque_enable_also_controls_servo_led(self):
        header = read(
            NEURON / "neuronlib/Servo/Dynamixel/dynamixel_serial.h"
        )
        source = read(
            NEURON / "neuronlib/Servo/Dynamixel/dynamixel_serial.cpp"
        )

        self.assertIn(
            "#define TORQUE_LED_BYTE_LEN", header
        )
        self.assertIn(
            "cmdWriteTorqueEnableWithLED(uint8_t servo_index,",
            header,
        )

        torque_instruction = re.search(
            r"case INST_SET_TORQUE:.*?break;", source, re.DOTALL
        )
        self.assertIsNotNone(torque_instruction)
        self.assertRegex(
            torque_instruction.group(0),
            r"cmdWriteTorqueEnableWithLED\(\s*servo_index,",
        )

        helper = re.search(
            r"void DynamixelSerial::cmdWriteTorqueEnableWithLED\(.*?\n}",
            source,
            re.DOTALL,
        )
        self.assertIsNotNone(helper)
        self.assertIn(
            "uint8_t parameters[TORQUE_LED_BYTE_LEN] = "
            "{enable_value, enable_value};",
            helper.group(0),
        )
        self.assertIn("servo_[servo_index].led_ = torque_enable;", helper.group(0))
        self.assertIn("CTRL_TORQUE_ENABLE, parameters,", helper.group(0))
        self.assertIn("TORQUE_LED_BYTE_LEN", helper.group(0))

    def test_spinal_rx_queue_holds_one_complete_four_servo_snapshot(self):
        for relative in (
            "mcu_project/boards/stm32H7/Src/main.c",
            "mcu_project/boards/stm32H7_v2/Src/main.c",
        ):
            main = read(SPINAL / relative)
            match = re.search(r"osMailQDef\(CanMail,\s*(\d+),\s*can_msg\)", main)
            self.assertIsNotNone(match)
            self.assertGreaterEqual(int(match.group(1)), 20)

    def test_servo_config_fails_immediately_when_can_enqueue_fails(self):
        can_servo = read(
            SPINAL / "mcu_project/lib/Hydrus_Lib/CANDevice/servo/can_servo.cpp"
        )
        request = can_servo.split("uint8_t CANServo::requestConfig(", 1)[1]
        request = request.split("uint8_t CANServo::requestConfigByIndex", 1)[0]
        self.assertRegex(
            request,
            r"if \(!sendMessage\(CAN::MESSAGEID_RECEIVE_SERVO_CONFIG",
        )
        self.assertIn("config_pending_ = false;", request)


if __name__ == "__main__":
    unittest.main()
