#!/usr/bin/env python3

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SPINAL = ROOT / "aerial_robot_nerve/spinal"
MODEL = ROOT / "aerial_robot_model"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


class ServoConfigAddressingTest(unittest.TestCase):
    def setUp(self):
        self.service = read(SPINAL / "srv/SetDirectServoConfig.srv")
        self.spine = read(
            SPINAL / "mcu_project/lib/Hydrus_Lib/Spine/spine.cpp"
        )
        self.bridge = read(MODEL / "src/servo_bridge/servo_bridge.cpp")
        self.rqt = read(SPINAL / "src/spinal/board_configurator.py")
        self.monitor = read(SPINAL / "src/spinal/servo_monitor.py")

    def test_ros_service_uses_board_and_board_local_index(self):
        request = self.service.split("---", 1)[0]
        self.assertIn("uint8 board_id", request)
        self.assertIn("uint8 servo_index", request)
        self.assertNotIn("uint8 slave_id", request)
        self.assertIn("payload excludes the Dynamixel ID", request)

    def test_spine_resolves_index_and_keeps_physical_id_internal(self):
        callback = self.spine.split("void servoConfigCallback", 1)[1]
        callback = callback.split("\n  bool init(", 1)[0]
        self.assertIn("req.board_id", callback)
        self.assertIn("req.servo_index", callback)
        self.assertIn("requestConfigByIndex", callback)
        self.assertNotIn("req.data[0]", callback)

    def test_host_callers_do_not_send_physical_id(self):
        for source in (self.bridge, self.rqt, self.monitor):
            self.assertNotIn("physical_servo_id", source)
            self.assertNotIn("srv.request.slave_id", source)
            self.assertNotIn("req.slave_id", source)

    def test_startup_yaml_global_id_is_resolved_from_board_info_order(self):
        self.assertIn("resolveGlobalServoAddress", self.bridge)
        self.assertIn("remaining_index -= board.servos.size()", self.bridge)
        self.assertNotIn("std::sort(boards.begin(), boards.end()", self.bridge)
        self.assertNotIn(
            'getInheritedValue(controller, group, "board_id"', self.bridge
        )
        self.assertNotIn(
            'getInheritedValue(controller, group, "servo_index"', self.bridge
        )

    def test_board_info_order_is_stable_when_send_flag_is_false(self):
        self.assertIn("for (unsigned int j = 0;", self.spine)
        self.assertIn("servo_.push_back", self.spine)
        self.assertIn("if (neuron_.at(i).can_servo_.servo_.at(j).getSendDataFlag())",
                      self.spine)


if __name__ == "__main__":
    unittest.main()
