^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hydrus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.4 (2024-11-02)
------------------
* [CMake] using C++17 for related packages (`#581 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/581>`_)
* [Typo] Fix typos (`#563 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/563>`_)
* [MuJoCo] enable to use mujoco simulator (`#557 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/557>`_)
* [Navigation] trajectory generation based on optimization methods (e.g. minimum snap trajectory)  (`#556 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/556>`_)

1.3.3 (2023-10-26)
------------------
* [Hydrus][URDF] remove unecessary URDF files (`#560 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/560>`_)
* Enable visualizing the rotor speed in gazebo (`#500 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/500>`_)
* [Hydrus] refactor CAD model realted to propeller to enable the visible rotation in gazebo and update the missing CAD model (remove the fixed propeller)
* [Robot Control] refactor the control framework (`#526 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/526>`_)
* [Robot Model] refactor the robot modelling framework (`#525 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/525>`_)


1.3.2 (2023-02-01)
------------------
* [Hydrus][Controller]Fix bug about thrust allocation in tilted robot (`#539 <https://github.com/jsk-ros-pkg/aerial_robot/issues/539>`_)

1.3.1 (2022-07-02)
------------------
* [Spinal] Improve the process of servo bridge (`#499 <https://github.com/jsk-ros-pkg/aerial_robot/issues/499>`_)
* [Spinal] Support STM32H7 chip (`#504 <https://github.com/jsk-ros-pkg/aerial_robot/issues/504>`_)
* Support ROS Noetic with Ubuntu Focal (`#507 <https://github.com/jsk-ros-pkg/aerial_robot/issues/507>`_)

1.3.0 (2021-08-06)
------------------

1.2.2 (2021-08-06)
------------------
* Add Hydrusx 15inch propeller model (`#477 <https://github.com/JSKAerialRobot/aerial_robot/issues/477>`_)
* Full Thrust Vectoring Modeling and Control for Multilinked Aerial Robot With Two DoF Force Vectoring Apparatus (`#474 <https://github.com/JSKAerialRobot/aerial_robot/issues/474>`_)
* Support RTK GPS for sensor fusion (`#472 <https://github.com/JSKAerialRobot/aerial_robot/issues/472>`_)
* Fix log message of gain reconfiguration (`#468 <https://github.com/JSKAerialRobot/aerial_robot/issues/468>`_)

1.2.1 (2021-02-17)
------------------
* Refactor the process to avoid satuation in z and yaw axes (`#444 <https://github.com/JSKAerialRobot/aerial_robot/issues/444>`_)
* Increase  retry times for Dragon and Hydrus gazebo test in Travis (`#443 <https://github.com/JSKAerialRobot/aerial_robot/issues/443>`_)
* Replace the old repositoty name in CHANGELOG.rst, update the API of unittest, and update the version of dependency repositories (`#434 <https://github.com/JSKAerialRobot/aerial_robot/issues/434>`_)
* Implement gimbal vectoring planner for hydrus_xi under actuated model (`#405 <https://github.com/JSKAerialRobot/aerial_robot/issues/405>`_)

1.2.0 (2020-05-31)
------------------
* Refactor aeiral_robot_base (`#406 <https://github.com/JSKAerialRobot/aerial_robot/issues/406>`_)
* Fix bugs  (`#404 <https://github.com/JSKAerialRobot/aerial_robot/issues/404>`_)
* Implement several extra control methods for dragon (`#401 <https://github.com/JSKAerialRobot/aerial_robot/issues/401>`_)
* Add robot namespace for whole system (`#399 <https://github.com/JSKAerialRobot/aerial_robot/issues/399>`_)
* Refactor aerial_robot_model/transformable_aerial_robot_model and implment Jacobians (`#398 <https://github.com/JSKAerialRobot/aerial_robot/issues/398>`_)
* Refactor the gain solver for optimal control (`#397 <https://github.com/JSKAerialRobot/aerial_robot/issues/397>`_)
* Refactor Gazebo system (`#391 <https://github.com/JSKAerialRobot/aerial_robot/issues/391>`_)

1.1.1 (2020-04-19)
------------------
* Refine the rosbag record script (`#380 <https://github.com/JSKAerialRobot/aerial_robot/issues/380>`_)
* Make the altitude control in landing phase safer (`#378 <https://github.com/JSKAerialRobot/aerial_robot/issues/378>`_)
* Refactor altitude estimation related to vo and range sensor (`#376 <https://github.com/JSKAerialRobot/aerial_robot/issues/376>`_)
* GPS latitude/longitude waypoint (`#370 <https://github.com/JSKAerialRobot/aerial_robot/issues/370>`_)
* Refactor several functions about teleop (`#367 <https://github.com/JSKAerialRobot/aerial_robot/issues/367>`_)

1.1.0 (2020-01-06)
------------------
* Implementation for tilted Hydrus model (`#363 <https://github.com/JSKAerialRobot/aerial_robot/issues/363>`_)
* Improve pwm saturation measures (`#362 <https://github.com/JSKAerialRobot/aerial_robot/issues/362>`_)
* Correct rosbag record script (`#354 <https://github.com/JSKAerialRobot/aerial_robot/issues/354>`_)
* Add hydrus_xi (full_acutated model) model (`#340 <https://github.com/JSKAerialRobot/aerial_robot/issues/340>`_)
* Add API to access private members in transformable_aerial_robot_model (`#285 <https://github.com/JSKAerialRobot/aerial_robot/issues/285>`_)
* Add rostest (`#337 <https://github.com/JSKAerialRobot/aerial_robot/issues/337>`_)
* Add several parameters for outdoor flight (`#331 <https://github.com/JSKAerialRobot/aerial_robot/issues/331>`_)
* Refactor the ego-motion estimation based on VIO (`#330 <https://github.com/JSKAerialRobot/aerial_robot/issues/330>`_)
* Change the model names regarding Hydrus (`#328 <https://github.com/JSKAerialRobot/aerial_robot/issues/328>`_)
* Enable the accelerometer bias estimation for z axis (`#323 <https://github.com/JSKAerialRobot/aerial_robot/issues/323>`_)
* Refactor the config file structure (`#314 <https://github.com/JSKAerialRobot/aerial_robot/issues/314>`_)

1.0.4 (2019-05-23)
------------------
* Fix bug: wrong RGB alignment of depth cloud (`#293 <https://github.com/JSKAerialRobot/aerial_robot/issues/293>`_)
* Add spwan pose arguments in bringup.launch of hydrus (`#292 <https://github.com/JSKAerialRobot/aerial_robot/issues/292>`_)
* Add actuator limit  in servo_bridge (`#287 <https://github.com/JSKAerialRobot/aerial_robot/issues/287>`_)
* Multi sensor temporal synchronization fusion (`#276 <https://github.com/JSKAerialRobot/aerial_robot/issues/276>`_)

1.0.3 (2019-01-08)
------------------
* Add launch_gazebo flag to bringup files (`#265 <https://github.com/JSKAerialRobot/aerial_robot/issues/265>`_)
* Update the usage of new API of `Kalman Filter <https://github.com/JSKAerialRobot/kalman_filter/tree/f7efb4d72131c02bf1632c6e4b400e2aeda60358>`_  for ego-motion estimation (`#262 <https://github.com/JSKAerialRobot/aerial_robot/issues/262>`_)
* Update the ego-motion estimation system based on IMU + GPS  (`#261 <https://github.com/JSKAerialRobot/aerial_robot/issues/261>`_)
* Add/Refactor rqt_plot scripts and sensor plugins to visualize the state estimation result from sensor fusion more clearly (`#261 <https://github.com/JSKAerialRobot/aerial_robot/issues/261>`_)
* Quantify the joint torque scale of hydrus real machine using Dynamixel MX28AR (`#255 <https://github.com/JSKAerialRobot/aerial_robot/issues/255>`_)
* Add torque (effort) publishing process from Spinal (`#254 <https://github.com/JSKAerialRobot/aerial_robot/issues/254>`_)
* Update the robot description (e.g., model urdf, sensor config) for egomotion-estimation (`#235 <https://github.com/JSKAerialRobot/aerial_robot/issues/235>`_, `#238 <https://github.com/JSKAerialRobot/aerial_robot/issues/238>`_, `#247 <https://github.com/JSKAerialRobot/aerial_robot/issues/247>`_, `#256 <https://github.com/JSKAerialRobot/aerial_robot/issues/256>`_)


1.0.2 (2018-11-24)
------------------

* Add urdf model and config file for hyrdus quad bringup attached with the ZED mini stereo camera, and set this model as the defualt in bringup launch file (`#233 <https://github.com/JSKAerialRobot/aerial_robot/issues/233>`_)
* Remove old simulation servo bridge config file, and update the integrated servo config file (`#230 <https://github.com/JSKAerialRobot/aerial_robot/issues/230>`_)

1.0.1 (2018-11-05)
------------------

1.0.0 (2018-09-26)
------------------
* first formal release
* Contributors: Moju Zhao, Tomoki Anzai, Fan Shi
