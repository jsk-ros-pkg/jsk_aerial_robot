^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aerial_robot_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2020-01-06)
------------------
* Implementation for tilted Hydrus model (`#363 <https://github.com/tongtybj/aerial_robot/issues/363>`_)
* PWM saturation measures  (`#362 <https://github.com/tongtybj/aerial_robot/issues/362>`_)
* Correct rosbag record script (`#354 <https://github.com/tongtybj/aerial_robot/issues/354>`_)
* Enable Kalman Filter based IMU-Mocap estimation for Dragon (`#350 <https://github.com/tongtybj/aerial_robot/issues/350>`_)
* Add hydrus_xi (full_acutated model) model (`#340 <https://github.com/tongtybj/aerial_robot/issues/340>`_)
* Add rostest (`#337 <https://github.com/tongtybj/aerial_robot/issues/337>`_)
* Fix the bug about the sensor topic name from rosparam (`#334 <https://github.com/tongtybj/aerial_robot/issues/334>`_)
* Mutex for multi-thread spin (`#332 <https://github.com/tongtybj/aerial_robot/issues/332>`_)
* Add mutex to avoid the double access to the variables related to the sensor health check.
* Add several parameters for outdoor flight (`#331 <https://github.com/tongtybj/aerial_robot/issues/331>`_)
* Refactor the ego-motion estimation based on VIO (`#330 <https://github.com/tongtybj/aerial_robot/issues/330>`_)
* Handle multi snesor nodes for same type of sensor (e.g. imu1, imu2, vo1, vo2, vo3)
* Refactor the visual odometry sensor plugin
* Remove unnecessary ros packages (`#327 <https://github.com/tongtybj/aerial_robot/issues/327>`_)
* Fix bug to find the synchronized rotation and angular velocity of baselink (`#318 <https://github.com/tongtybj/aerial_robot/issues/318>`_)
* Multi-thread spin (`#319 <https://github.com/tongtybj/aerial_robot/issues/319>`_)

1.0.4 (2019-05-23)
------------------
* Fix bug: forbid the variables called by reference against the function  which contains mutex  (`#298 <https://github.com/tongtybj/aerial_robot/issues/298>`_)
* Improve psi command over range check (`#297 <https://github.com/tongtybj/aerial_robot/issues/297>`_)
* Refactor the script to calculate the control error RMS (`#291 <https://github.com/tongtybj/aerial_robot/issues/291>`_)
* Update the usage of low pass filter in mocap and barometer (`#280 <https://github.com/tongtybj/aerial_robot/issues/280>`_)
* Multi sensor temporal synchronization fusion and refactor the state estimator class  (`#276 <https://github.com/tongtybj/aerial_robot/issues/276>`_)
* Fix bug of acc control mode (`#274 <https://github.com/tongtybj/aerial_robot/issues/274>`_)
* Add mutex for the kinamtics update in the ego-motion estimation process (`#269 <https://github.com/tongtybj/aerial_robot/issues/269>`_)
* Fix the bug in keyboard teleop (`#270 <https://github.com/tongtybj/aerial_robot/issues/270>`_)
* Update teleop system: 1. update the keyboard command script; 2. support PlayStation DualShock4. (`#266 <https://github.com/tongtybj/aerial_robot/issues/266>`_)

1.0.3 (2019-01-08)
------------------
* Update the usage of new API of `Kalman Filter <https://github.com/tongtybj/kalman_filter/tree/f7efb4d72131c02bf1632c6e4b400e2aeda60358>`_  for ego-motion estimation (`#262 <https://github.com/tongtybj/aerial_robot/issues/262>`_)
* Update the ego-motion estimation system based on IMU + GPS  (`#261 <https://github.com/tongtybj/aerial_robot/issues/261>`_)
* Update solidedge_cog_inertia_tensor_converter.py  (`#234 <https://github.com/tongtybj/aerial_robot/issues/234>`_)

1.0.2 (2018-11-24)
------------------
* Update the vo sensor fusion plugin (`#233 <https://github.com/tongtybj/aerial_robot/issues/233>`_)
* Change the sensor time stamp update behavior in HealthStamp check function (`#232 <https://github.com/tongtybj/aerial_robot/issues/232>`_)
* Use kinamtics model to get the sensor transfrom w.r.t baselink frame, based on the full forward kinematics method, which is much faster than TF2 (`#226 <https://github.com/tongtybj/aerial_robot/issues/226>`_, `#231 <https://github.com/tongtybj/aerial_robot/issues/231>`_)
* Add force_landing to keyboard command interface (`#224 <https://github.com/tongtybj/aerial_robot/issues/224>`_)

1.0.1 (2018-11-05)
------------------
* modify solidedge_cog_inertia_tensor_converter and move to aerial_robot_base (#207)

1.0.0 (2018-09-26)
------------------
* first formal release
* Contributors: Moju Zhao, Tomoki Anzai, Fan Shi
