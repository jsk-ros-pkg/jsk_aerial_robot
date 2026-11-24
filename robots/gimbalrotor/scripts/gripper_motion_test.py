#!/usr/bin/env python3
"""Run the Grape arm motion check on the gripper drive only."""

from pathlib import Path
import runpy


if __name__ == "__main__":
    arm_test = runpy.run_path(str(Path(__file__).with_name("arm_motion_test.py")))
    arm_test["run"](gripper_only=True)
