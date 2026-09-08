#!/usr/bin/env python3

import contextlib
import importlib.util
import io
import math
from pathlib import Path
import unittest


SCRIPT = (
    Path(__file__).resolve().parents[1] / "scripts/servo_scale_converter.py"
)
SPEC = importlib.util.spec_from_file_location("servo_scale_converter", SCRIPT)
CONVERTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CONVERTER)


class ServoScaleConverterTest(unittest.TestCase):
    def test_converts_raw_units_to_si_scales(self):
        angle, velocity, torque = CONVERTER.calculate_scales(
            position_revolutions=1.0,
            position_resolution_bits=12,
            velocity_raw_rpm=60.0,
            current_raw_ma=1000.0,
            torque_current_ratio_nm_per_a=2.0,
        )

        self.assertAlmostEqual(angle, 2.0 * math.pi / 4096.0)
        self.assertAlmostEqual(velocity, 2.0 * math.pi)
        self.assertAlmostEqual(torque, 2.0)

    def test_rejects_non_positive_inputs(self):
        with self.assertRaises(ValueError):
            CONVERTER.calculate_scales(1.0, 0, 0.229, 2.69, 1.5)

    def test_cli_output_can_be_pasted_into_yaml(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            result = CONVERTER.main()

        self.assertEqual(result, 0)
        self.assertIn("angle_scale: 0.00153398078789", output.getvalue())
        self.assertIn("velocity_scale: 0.0239808239224", output.getvalue())
        self.assertIn("torque_scale: 0.004035", output.getvalue())


if __name__ == "__main__":
    unittest.main()
