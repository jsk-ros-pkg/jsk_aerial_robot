#!/usr/bin/env python3

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SOURCE = (ROOT / "aerial_robot_model/src/servo_bridge/servo_bridge.cpp").read_text(
    encoding="utf-8"
)
HEADER = (ROOT / "aerial_robot_model/include/aerial_robot_model/servo_bridge.h").read_text(
    encoding="utf-8"
)


class ServoStartupConfigurationTest(unittest.TestCase):
    def test_startup_does_not_disable_all_torque(self):
        self.assertNotIn("disableServoTorqueForConfiguration", SOURCE)
        self.assertNotIn("ServoTorqueStates", SOURCE)
        self.assertNotIn("direct_servo_torque_off_settle", SOURCE)

    def test_failed_configuration_never_blocks_control_callbacks(self):
        self.assertNotIn("direct_servo_config_ready_", SOURCE)
        self.assertNotIn("direct_servo_config_ready_", HEADER)
        self.assertNotIn("configuration is not ready", SOURCE)

    def test_configuration_is_one_shot_and_calls_remain_rate_limited(self):
        self.assertIn("direct_servo_config_timer_", SOURCE)
        self.assertNotIn("direct_servo_config_retry_interval", SOURCE)
        self.assertIn("ros::WallDuration(call_interval).sleep()", SOURCE)
        self.assertIn("control remains available", SOURCE)

    def test_yaml_mapping_uses_explicit_board_local_address(self):
        self.assertIn("uint8_t board_id", SOURCE)
        self.assertIn("uint8_t servo_index", SOURCE)
        self.assertIn('getInheritedValue(controller, group, "board_id"', SOURCE)
        self.assertIn('getInheritedValue(controller, group, "servo_index"', SOURCE)


if __name__ == "__main__":
    unittest.main()
