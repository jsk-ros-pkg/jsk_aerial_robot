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
YAML = (ROOT / "aerial_robot_nerve/spinal/launch/Servo.yaml").read_text(
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
        self.assertIn("DIRECT_SERVO_CONFIG_CALL_INTERVAL = 0.02", SOURCE)
        self.assertIn("DIRECT_SERVO_CONFIG_STARTUP_DELAY = 1.0", SOURCE)
        self.assertNotIn('param("direct_servo_config_call_interval"', SOURCE)
        self.assertNotIn('param("direct_servo_config_startup_delay"', SOURCE)
        self.assertIn("control remains available", SOURCE)

    def test_yaml_mapping_uses_only_the_global_servo_id(self):
        self.assertIn("resolveGlobalServoAddress", SOURCE)
        self.assertNotIn('getInheritedValue(controller, group, "board_id"', SOURCE)
        self.assertNotIn('getInheritedValue(controller, group, "servo_index"', SOURCE)
        self.assertNotIn("board_id:", YAML)
        self.assertNotIn("servo_index:", YAML)
        self.assertNotIn("direct_servo_config_call_interval:", YAML)
        self.assertNotIn("direct_servo_config_startup_delay:", YAML)


if __name__ == "__main__":
    unittest.main()
