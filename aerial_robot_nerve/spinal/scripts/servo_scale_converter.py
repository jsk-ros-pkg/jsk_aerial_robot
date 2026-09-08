#!/usr/bin/env python3

"""Convert servo data-sheet raw units to Servo.yaml SI scale values."""

import math
from typing import Tuple


# Edit these five values using the raw units from the servo data sheet.
POSITION_REVOLUTIONS = 1.0              # rev
POSITION_RESOLUTION_BITS = 12           # 12 bit = 4096 pulse/rev
VELOCITY_RAW_RPM = 0.229                # rev/min per raw velocity unit
CURRENT_RAW_MA = 2.69                   # mA per raw current unit
TORQUE_CURRENT_RATIO_NM_PER_A = 0.71428571     # Nm/A



def calculate_scales(
    position_revolutions: float,
    position_resolution_bits: int,
    velocity_raw_rpm: float,
    current_raw_ma: float,
    torque_current_ratio_nm_per_a: float,
) -> Tuple[float, float, float]:
    """Return angle, velocity, and torque increments per raw servo unit."""
    if not isinstance(position_resolution_bits, int) or \
            position_resolution_bits <= 0:
        raise ValueError("position resolution must be a positive integer")

    values = (
        position_revolutions,
        velocity_raw_rpm,
        current_raw_ma,
        torque_current_ratio_nm_per_a,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in values):
        raise ValueError("all conversion inputs must be finite and greater than zero")

    position_raw_pulses = 2 ** position_resolution_bits
    angle_scale = position_revolutions * 2.0 * math.pi / position_raw_pulses
    velocity_scale = velocity_raw_rpm * 2.0 * math.pi / 60.0
    torque_scale = current_raw_ma / 1000.0 * torque_current_ratio_nm_per_a
    return angle_scale, velocity_scale, torque_scale


def main() -> int:
    angle_scale, velocity_scale, torque_scale = calculate_scales(
        POSITION_REVOLUTIONS,
        POSITION_RESOLUTION_BITS,
        VELOCITY_RAW_RPM,
        CURRENT_RAW_MA,
        TORQUE_CURRENT_RATIO_NM_PER_A,
    )

    print(f"angle_scale: {angle_scale:.12g}  # rad/pulse")
    print(f"velocity_scale: {velocity_scale:.12g}  # (rad/s)/raw")
    print(f"torque_scale: {torque_scale:.12g}  # Nm/raw")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
