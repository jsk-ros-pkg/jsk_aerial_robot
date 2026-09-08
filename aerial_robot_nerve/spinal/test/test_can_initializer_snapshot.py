#!/usr/bin/env python3

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
NEURON = ROOT / "aerial_robot_nerve/neuron"
SPINAL = ROOT / "aerial_robot_nerve/spinal"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


class CanInitializerSnapshotTest(unittest.TestCase):
    def setUp(self):
        self.initializer = read(
            SPINAL
            / "mcu_project/lib/Hydrus_Lib/CANDevice/initializer/can_initializer.cpp"
        )
        self.neuron = read(
            SPINAL / "mcu_project/lib/Hydrus_Lib/Neuron/neuron.h"
        )
        self.sender = read(NEURON / "neuronlib/Initializer/initializer.cpp")

    def test_snapshot_is_staged_until_every_servo_fragment_arrives(self):
        self.assertIn("beginConfiguration", self.neuron)
        self.assertIn("markConfigurationFragment", self.neuron)
        self.assertIn("commitConfiguration", self.neuron)
        self.assertIn("expected_config_mask_", self.neuron)
        self.assertNotIn("setInitialized", self.initializer)

    def test_legacy_and_fd_snapshots_have_explicit_completion_masks(self):
        self.assertIn("extended_config ? 0x0F : 0x07", self.neuron)
        self.assertIn("CANFD_PROTOCOL_VERSION", self.initializer)

    def test_each_fragment_validates_dlc_and_servo_index(self):
        for message, minimum_dlc in (
            ("MESSAGEID_SEND_INITIAL_CONFIG_1", 8),
            ("MESSAGEID_SEND_INITIAL_CONFIG_2", 8),
            ("MESSAGEID_SEND_INITIAL_CONFIG_3", 8),
            ("MESSAGEID_SEND_INITIAL_CONFIG_4", 16),
        ):
            case_start = self.initializer.index(f"case CAN::{message}")
            next_case = self.initializer.find("case CAN::", case_start + 10)
            body = self.initializer[case_start:next_case]
            self.assertIn(f"DLC < {minimum_dlc}", body)
            self.assertIn("pendingServo", body)
            self.assertIn("markConfigurationFragment", body)

    def test_initial_configuration_retries_and_discards_incomplete_boards(self):
        self.assertIn("CONFIG_REQUEST_MAX_ATTEMPTS = 3", self.initializer)
        self.assertIn("getConfigGeneration", self.initializer)
        self.assertIn("remove_if", self.initializer)
        self.assertIn("!neuron.getInitialized()", self.initializer)

    def test_snapshot_sender_stops_after_a_transport_failure(self):
        self.assertIn("bool Initializer::sendBoardConfig()", self.sender)
        self.assertIn("if (!sendMessage", self.sender)
        self.assertIn("return false;", self.sender)


if __name__ == "__main__":
    unittest.main()
