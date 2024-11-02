^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aerial_robot_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.4 (2024-11-02)
------------------
* [base][mocap]: add an option "enable_optitrack: true" (`#621 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/621>`_)
* [rosbag] add joy topic in control record Merge pull request (`#605 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/605>`_)
* [Mocap] remove the "required" flag to allow the onboard-sensor-only flight  (`#604 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/604>`_)
* [Livox LiDAR + FAST LIO] enable to use Livox ros driver and FAST LIO as default (`#600 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/600>`_)
* [CMake] use C++17 compiler for all packages that contain C++ files  (`#581 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/581>`_)
* [Typo] Fix typos (`#563 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/563>`_)
* [Navigation] improve keyboard teleopration that allows to move the robot with uniform linear motion (`#555 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/555>`_)
* [Navigation] improve the landing motion for all robots  (`#554 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/554>`_)

1.3.3 (2023-10-26)
------------------
* [Mini Quadrotor] support the common quadrotor platform (`#546 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/546>`_)
* [Estimate][IMU] add standard configuration for spianl IMU
* [Robot Control] refactor the control framework (`#526 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/526>`_ )
* [Robot Model] refactor the robot modelling framework (`#525 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/525>`_)

1.3.2 (2023-02-01)
------------------

1.3.1 (2022-07-02)
------------------
* [aerial_robot_base]  Explicitly stop timer and async spinner for safe destruction (`#515 <https://github.com/jsk-ros-pkg/aerial_robot/issues/515>`_)
* Improve the ros spin process for supporting ROS Noetic with Ubuntu Focal (`#507 <https://github.com/jsk-ros-pkg/aerial_robot/issues/507>`_)

1.3.0 (2021-08-06)
------------------

1.2.2 (2021-08-06)
------------------
* Add launch and yaml files for RTK GPS (`#472 <https://github.com/JSKAerialRobot/aerial_robot/issues/472>`_)
* Refactor launch of mocap (`#451 <https://github.com/JSKAerialRobot/aerial_robot/issues/451>`_, `#464 <https://github.com/JSKAerialRobot/aerial_robot/issues/464>`_)

1.2.1 (2021-02-17)
------------------
* Replace the old repositoty name in CHANGELOG.rst, correct the API of unittest, and update the version of dependency repositories (`#434 <https://github.com/JSKAerialRobot/aerial_robot/issues/434>`_)
* Refine the script to calculate the tracking error RMS from rostopic (`#420 <https://github.com/JSKAerialRobot/aerial_robot/issues/420>`_)


1.2.0 (2020-05-31)
------------------
* Refactor aeiral_robot_base (`#406 <https://github.com/JSKAerialRobot/aerial_robot/issues/406>`_)
* Merge pull request `#401 <https://github.com/JSKAerialRobot/aerial_robot/issues/401>`_ from JSKAerialRobot/dragon_special_control
* Implement several extra control methods for dragon (`#401 <https://github.com/JSKAerialRobot/aerial_robot/issues/401>`_)
* Add robot namesapce for whole system (`#399 <https://github.com/JSKAerialRobot/aerial_robot/issues/399>`_)
* Refactor Gazebo system (`#391 <https://github.com/JSKAerialRobot/aerial_robot/issues/391>`_)

1.1.1 (2020-04-19)
------------------
* Make magnetic declination reconfigurable and also extend GPS function `#395 <https://github.com/JSKAerialRobot/aerial_robot/issues/395>`_ from JSKAerialRobot/mag_calibration
* Make the transport hint (tcp/udp) of joy ros message reconfigurable from rosparam (`#389 <https://github.com/JSKAerialRobot/aerial_robot/issues/389>`_)
* Augument the state publish from GPS, including UTC time, fix status, accuracy of horizontal position and velocity, but reserve it right now and use the compact message type.
* Refine GPS waypoint navigation in final convergence phase (`#379 <https://github.com/JSKAerialRobot/aerial_robot/issues/379>`_)
* Reduce the publish rate of joy stick (`#382 <https://github.com/JSKAerialRobot/aerial_robot/issues/382>`_)
* Fix bug: block gps waypoint when doing force attitude control (`#381 <https://github.com/JSKAerialRobot/aerial_robot/issues/381>`_)
* Refine the rosbag record script (`#380 <https://github.com/JSKAerialRobot/aerial_robot/issues/380>`_)
* Make the altitude control in landing phase safer (`#378 <https://github.com/JSKAerialRobot/aerial_robot/issues/378>`_)
* Refactor altitude estimation related to vo and range sensor (`#376 <https://github.com/JSKAerialRobot/aerial_robot/issues/376>`_)
* Change GPS location variable type (`#374 <https://github.com/JSKAerialRobot/aerial_robot/issues/374>`_)
* GPS latitude/longitude waypoint (`#370 <https://github.com/JSKAerialRobot/aerial_robot/issues/370>`_)
* Refactor several functions about teleop (`#367 <https://github.com/JSKAerialRobot/aerial_robot/issues/367>`_)
* Fix bug: remove the wrong assignment of target_throttle to ros message (`#366 <https://github.com/JSKAerialRobot/aerial_robot/issues/366>`_)
* Publish flight_state to provide the current state (`#365 <https://github.com/JSKAerialRobot/aerial_robot/issues/365>`_)

1.1.0 (2020-01-06)
------------------
* Implementation for tilted Hydrus model (`#363 <https://github.com/JSKAerialRobot/aerial_robot/issues/363>`_)
* PWM saturation measures  (`#362 <https://github.com/JSKAerialRobot/aerial_robot/issues/362>`_)
* Correct rosbag record script (`#354 <https://github.com/JSKAerialRobot/aerial_robot/issues/354>`_)
* Enable Kalman Filter based IMU-Mocap estimation for Dragon (`#350 <https://github.com/JSKAerialRobot/aerial_robot/issues/350>`_)
* Add hydrus_xi (full_acutated model) model (`#340 <https://github.com/JSKAerialRobot/aerial_robot/issues/340>`_)
* Add rostest (`#337 <https://github.com/JSKAerialRobot/aerial_robot/issues/337>`_)
* Fix the bug about the sensor topic name from rosparam (`#334 <https://github.com/JSKAerialRobot/aerial_robot/issues/334>`_)
* Mutex for multi-thread spin (`#332 <https://github.com/JSKAerialRobot/aerial_robot/issues/332>`_)
* Add mutex to avoid the double access to the variables related to the sensor health check.
* Add several parameters for outdoor flight (`#331 <https://github.com/JSKAerialRobot/aerial_robot/issues/331>`_)
* Refactor the ego-motion estimation based on VIO (`#330 <https://github.com/JSKAerialRobot/aerial_robot/issues/330>`_)
* Handle multi snesor nodes for same type of sensor (e.g. imu1, imu2, vo1, vo2, vo3)
* Refactor the visual odometry sensor plugin
* Remove unnecessary ros packages (`#327 <https://github.com/JSKAerialRobot/aerial_robot/issues/327>`_)
* Fix bug to find the synchronized rotation and angular velocity of baselink (`#318 <https://github.com/JSKAerialRobot/aerial_robot/issues/318>`_)
* Multi-thread spin (`#319 <https://github.com/JSKAerialRobot/aerial_robot/issues/319>`_)

1.0.4 (2019-05-23)
------------------
* Fix bug: forbid the variables called by reference against the function  which contains mutex  (`#298 <https://github.com/JSKAerialRobot/aerial_robot/issues/298>`_)
* Improve psi command over range check (`#297 <https://github.com/JSKAerialRobot/aerial_robot/issues/297>`_)
* Refactor the script to calculate the control error RMS (`#291 <https://github.com/JSKAerialRobot/aerial_robot/issues/291>`_)
* Update the usage of low pass filter in mocap and barometer (`#280 <https://github.com/JSKAerialRobot/aerial_robot/issues/280>`_)
* Multi sensor temporal synchronization fusion and refactor the state estimator class  (`#276 <https://github.com/JSKAerialRobot/aerial_robot/issues/276>`_)
* Fix bug of acc control mode (`#274 <https://github.com/JSKAerialRobot/aerial_robot/issues/274>`_)
* Add mutex for the kinamtics update in the ego-motion estimation process (`#269 <https://github.com/JSKAerialRobot/aerial_robot/issues/269>`_)
* Fix the bug in keyboard teleop (`#270 <https://github.com/JSKAerialRobot/aerial_robot/issues/270>`_)
* Update teleop system: 1. update the keyboard command script; 2. support PlayStation DualShock4. (`#266 <https://github.com/JSKAerialRobot/aerial_robot/issues/266>`_)

1.0.3 (2019-01-08)
------------------
* Update the usage of new API of `Kalman Filter <https://github.com/JSKAerialRobot/kalman_filter/tree/f7efb4d72131c02bf1632c6e4b400e2aeda60358>`_  for ego-motion estimation (`#262 <https://github.com/JSKAerialRobot/aerial_robot/issues/262>`_)
* Update the ego-motion estimation system based on IMU + GPS  (`#261 <https://github.com/JSKAerialRobot/aerial_robot/issues/261>`_)
* Update solidedge_cog_inertia_tensor_converter.py  (`#234 <https://github.com/JSKAerialRobot/aerial_robot/issues/234>`_)

1.0.2 (2018-11-24)
------------------
* Update the vo sensor fusion plugin (`#233 <https://github.com/JSKAerialRobot/aerial_robot/issues/233>`_)
* Change the sensor time stamp update behavior in HealthStamp check function (`#232 <https://github.com/JSKAerialRobot/aerial_robot/issues/232>`_)
* Use kinamtics model to get the sensor transfrom w.r.t baselink frame, based on the full forward kinematics method, which is much faster than TF2 (`#226 <https://github.com/JSKAerialRobot/aerial_robot/issues/226>`_, `#231 <https://github.com/JSKAerialRobot/aerial_robot/issues/231>`_)
* Add force_landing to keyboard command interface (`#224 <https://github.com/JSKAerialRobot/aerial_robot/issues/224>`_)

1.0.1 (2018-11-05)
------------------
* modify solidedge_cog_inertia_tensor_converter and move to aerial_robot_base (#207)

1.0.0 (2018-09-26)
------------------
* first formal release
* Contributors: Moju Zhao, Tomoki Anzai, Fan Shi
