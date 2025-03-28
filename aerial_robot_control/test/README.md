# Test Procedure

1. uncomment CMakeLists.txt, and catkin build aerial_robot_control

2. run the test code
```bash
python -m unittest tilt_qd/test_nmpc.py
```

3. roslaunch the beetle-omni, do
 - tracking normal traj.
 - tracking csv traj.

4. roslaunch the gimbalrotor with birotor
 - tracking normal traj.

5. roslaunch the gimbalrotor with trirotor
 - tracking normal traj.