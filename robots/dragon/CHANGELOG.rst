^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dragon
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.4 (2024-11-02)
------------------
* Resolve dependency to other package (`#610 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/610>`_)
* [Navigation] trajectory generation based on optimization methods (e.g. minimum snap trajectory) (`#556 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/556>`_)
* [CMake] use C++17 compiler for all packages that contain C++ files  (`#581 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/581>`_)
* [MuJoCo] enable to use mujoco simulator (`#557 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/557>`_)
* [Navigation] improve the landing motion for all robots (`#554 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/554>`_)


1.3.3 (2023-10-26)
------------------
* [Aerial Robot Model][Underactuated][Plugin] fix the wrong args assignment for model constructor (`#561 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/561>`_)
* [Robot Control] refactor the control framework (`#526 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/526>`_)
* [Robot Model] refactor the robot modelling framework (`#525 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/525>`_)

1.3.2 (2023-02-01)
------------------
* [Dragon] improve operation procedure (`#528 <https://github.com/jsk-ros-pkg/aerial_robot/issues/528>`_)
* [Spinal][Neuron][Servo] solve the joint pulley round offset problem Merge pull request (`#534 <https://github.com/jsk-ros-pkg/aerial_robot/issues/534>`_)


1.3.1 (2022-07-02)
------------------
* [Spinal] Improve the process of servo bridge (`#499 <https://github.com/jsk-ros-pkg/aerial_robot/issues/499>`_)
* [DRAGON] add IIR LPF for IMU gyro data (`#514 <https://github.com/jsk-ros-pkg/aerial_robot/issues/514>`_)
* [Spinal] Support STM32H7 chip (`#504 <https://github.com/jsk-ros-pkg/aerial_robot/issues/504>`_)
* [Dragon] update the rotor and link configuration (`#497 <https://github.com/jsk-ros-pkg/aerial_robot/issues/497>`_)
* Support ROS Noetic with Ubuntu Focal (`#507 <https://github.com/jsk-ros-pkg/aerial_robot/issues/507>`_)


1.3.0 (2021-08-06)
------------------

1.2.2 (2021-08-06)
------------------
* Fix name in dragon.urdf.xacro (`#487 <https://github.com/JSKAerialRobot/aerial_robot/issues/487>`_)
* Full Thrust Vectoring Modeling and Control for Multilinked Aerial Robot With Two DoF Force Vectoring Apparatus (`#474 <https://github.com/JSKAerialRobot/aerial_robot/issues/474>`_)

1.2.1 (2021-02-17)
------------------
* Increase the retry times for Dragon and Hydrus gazebo test in Travis (`#443 <https://github.com/JSKAerialRobot/aerial_robot/issues/443>`_)
* Replace the old repositoty name in CHANGELOG.rst, update the version of dependency repositories, and relax the position convergence threshold in rostest (`#434 <https://github.com/JSKAerialRobot/aerial_robot/issues/434>`_)
* Automatically stop motors by checking the  impact  from the  ground in force landing phase. (`#419 <https://github.com/JSKAerialRobot/aerial_robot/issues/419>`_)
* Fix insufficient link library of dragon_aerial_robot_controllib (`#412 <https://github.com/JSKAerialRobot/aerial_robot/issues/412>`_)


1.2.0 (2020-05-31)
------------------
* Refactor aeiral_robot_base (`#406 <https://github.com/JSKAerialRobot/aerial_robot/issues/406>`_)
* Fix bugs  (`#404 <https://github.com/JSKAerialRobot/aerial_robot/issues/404>`_)
* Implement several extra control methods for dragon (`#401 <https://github.com/JSKAerialRobot/aerial_robot/issues/401>`_)
* Add robot namespace for whole system (`#399 <https://github.com/JSKAerialRobot/aerial_robot/issues/399>`_)
* Refactor aerial_robot_model/transformable_aerial_robot_model and implment Jacobians (`#398 <https://github.com/JSKAerialRobot/aerial_robot/issues/398>`_)
* Refactor Gazebo system (`#391 <https://github.com/JSKAerialRobot/aerial_robot/issues/391>`_)

1.1.1 (2020-04-19)
------------------
* Modify README.md of dragon (`#394 <https://github.com/JSKAerialRobot/aerial_robot/issues/394>`_)
* Make the altitude control in landing phase safer (`#378 <https://github.com/JSKAerialRobot/aerial_robot/issues/378>`_)
* GPS latitude/longitude waypoint (`#370 <https://github.com/JSKAerialRobot/aerial_robot/issues/370>`_)


1.1.0 (2020-01-06)
------------------
* Implementation for tilted Hydrus model (`#363 <https://github.com/JSKAerialRobot/aerial_robot/issues/363>`_)
* Implement robot model and transform control (lqi gain generator) for tilted hydrus model.
* Improve pwm saturation measures (`#362 <https://github.com/JSKAerialRobot/aerial_robot/issues/362>`_)
* Correct rosbag record script (`#354 <https://github.com/JSKAerialRobot/aerial_robot/issues/354>`_)
* Enable Kalman Filter based IMU-Mocap estimation for Dragon (`#350 <https://github.com/JSKAerialRobot/aerial_robot/issues/350>`_)
* Add API to access private members in transformable_aerial_robot_model (`#285 <https://github.com/JSKAerialRobot/aerial_robot/issues/285>`_)
* Add rostest (`#337 <https://github.com/JSKAerialRobot/aerial_robot/issues/337>`_)

1.0.4 (2019-05-23)
------------------
* Add actuator limit in servo_bridge (`#287 <https://github.com/JSKAerialRobot/aerial_robot/issues/287>`_)
* Refactor the function "DragonRobotModel::updateRobotModelImpl" (`#284 <https://github.com/JSKAerialRobot/aerial_robot/issues/284>`_)
* Add low pass filter function in servo bridge (`#289 <https://github.com/JSKAerialRobot/aerial_robot/issues/289>`_)
* Update the configuration for dragon quad type (`#277 <https://github.com/JSKAerialRobot/aerial_robot/issues/277>`_)

1.0.3 (2019-01-08)
------------------
* Add launch_gazebo flag to bringup files (`#265 <https://github.com/JSKAerialRobot/aerial_robot/issues/265>`_)
* Refactor the sensor plugins which uses `new Kalman Filter version <https://github.com/JSKAerialRobot/kalman_filter/tree/f7efb4d72131c02bf1632c6e4b400e2aeda60358>`_  (`#262 <https://github.com/JSKAerialRobot/aerial_robot/issues/262>`_
* Add torque (effort) publishing process from Spinal (`#254 <https://github.com/JSKAerialRobot/aerial_robot/issues/254>`_)

1.0.2 (2018-11-24)
------------------
* Remove old simulation servo bridge config file, and update the integrated servo config file (`#230 <https://github.com/JSKAerialRobot/aerial_robot/issues/230>`_)
* Add the gimbal vectoring test based on the desired tilt (`#229 <https://github.com/JSKAerialRobot/aerial_robot/issues/229>`_)
* Remove the unecessary vel feed-back state approximation for CoG control in gazebo simulation situation (`#218 <https://github.com/JSKAerialRobot/aerial_robot/issues/218>`_)

1.0.1 (2018-11-05)
------------------

1.0.0 (2018-09-26)
------------------
* first formal release
* Contributors: Moju Zhao, Tomoki Anzai
