# Amoeba Robot Hardware Coupling Implementation

## Overview

This document describes the coordinated changes made to properly reflect the real-world hardware mechanism where servo id:4 mechanically controls all four extendable joints simultaneously.

## Real Hardware Mechanism

- **Single Servo**: Only servo id:4 physically exists
- **Mechanical Coupling**: One servo drives all four prismatic joints through a mechanical linkage
- **Motion Pattern**: 
  - 1 full rotation (2π radians) of servo id:4 causes:
    - `extendable_joint1` and `extendable_joint3`: +0.1m extension (synchronized pair)
    - `extendable_joint2` and `extendable_joint4`: -0.1m retraction (synchronized pair)

## Files Modified

### 1. Servo.yaml Configuration

**File**: `/robots/amoeba/config/Servo.yaml`

**Changes**:
- Updated all four extendable joint controllers to use the same servo `id: 4`
- Added `mechanical_ratio` parameters to reflect coupling:
  - Controllers 1,3: `mechanical_ratio: 1.0` (forward motion)
  - Controllers 2,4: `mechanical_ratio: -1.0` (reverse motion)
- Added documentation comments explaining the hardware mechanism

**Safety**: Ensures software configuration matches physical hardware, preventing conflicts.

### 2. URDF Model Definition

**File**: `/robots/amoeba/urdf/amoeba.urdf.xacro`

**Changes**:
- Updated transmission `mechanicalReduction` values to reflect the coupling ratio
- Added conditional logic for joints 1,3 vs 2,4 to set appropriate reduction ratios
- Added comprehensive documentation comments explaining the hardware mechanism
- Calculation: 2π rotation → 0.1m displacement = 31.416 rad/m reduction ratio

**Safety**: Provides accurate simulation behavior and proper force/torque calculations.

### 3. Control Script

**File**: `/scripts/trans_mpc.py`

**Changes**:
- Completely refactored joint control logic to handle all four joints simultaneously
- Replaced single-joint control with synchronized multi-joint control
- Added `get_servo_positions()` method that returns coordinated positions for all joints
- Implemented proper coupling ratios: `[1.0, -1.0, 1.0, -1.0]`
- Enhanced logging to show motion of all joints
- Updated documentation to reflect real hardware mechanism

**Safety**: Ensures control commands match hardware capabilities and limitations.

## Usage

### Running the Control Script

```bash
# Default robot name (amoeba)
rosrun aerial_robot_planning trans_mpc.py

# Specific robot name
rosrun aerial_robot_planning trans_mpc.py my_amoeba_robot
```

### Expected Behavior

During trajectory execution, you should observe:
- **Joints 1,3**: Move from 0m → +0.1m → 0m (extension, synchronized)
- **Joints 2,4**: Move from 0m → -0.1m → 0m (retraction, synchronized)
- **Synchronized**: All four joints move in coordination, reflecting servo id:4 motion

### Logging Output

```
Joint positions: ['0.0500', '-0.0500', '0.0500', '-0.0500'] (m)
Motion pattern: j1,j3→0.050m, j2,j4→-0.050m
```

## Safety Considerations

1. **Hardware Protection**: Configuration ensures only the existing servo id:4 receives commands
2. **Motion Limits**: Joint limits (-0.1m to +0.1m) are enforced in URDF
3. **Synchronized Control**: All joints move together, preventing mechanical stress
4. **Gradual Motion**: Smooth trajectory ensures safe acceleration/deceleration

## Simulation vs Real Hardware

- **Simulation**: Uses individual controllers but with proper mechanical reductions
- **Real Hardware**: Single servo id:4 drives all joints through mechanical coupling
- **Compatibility**: Both modes work with the same control commands

## Troubleshooting

### Joint Not Moving
- Check that servo id:4 is active and responsive
- Verify topic `/robot_name/extendable_joints_ctrl` exists
- Ensure joint limits are not exceeded

### Unexpected Motion Pattern
- Verify coupling ratios in Servo.yaml match hardware
- Check mechanical reduction values in URDF
- Confirm trans_mpc.py uses correct joint ordering

### Simulation Issues
- Reload robot configuration after changing Servo.yaml or URDF
- Restart simulation to apply new transmission parameters
- Check for URDF parsing errors in console

## Technical Details

### Joint Coupling Mathematics

```
servo_rotation = trajectory_progress * 2π
base_displacement = (servo_rotation / 2π) * 0.1m
joint1_position = base_displacement * 1.0   # forward (sync with joint3)
joint2_position = base_displacement * (-1.0) # reverse (sync with joint4)
joint3_position = base_displacement * 1.0   # forward (sync with joint1)
joint4_position = base_displacement * (-1.0) # reverse (sync with joint2)
```

### Mechanical Reduction Calculation

```
2π radians → 0.1m displacement
1 radian → 0.1/(2π) ≈ 0.01592m
Reduction ratio = 2π/0.1 ≈ 62.832 rad/m
Practical value: ±31.416 (with sign indicating direction)
```

This implementation ensures robust, safe operation that accurately reflects the real hardware mechanism while maintaining compatibility with both simulation and physical deployment.
