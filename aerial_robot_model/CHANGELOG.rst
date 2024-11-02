^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aerial_robot_model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.4 (2024-11-02)
------------------
* [CMake] using C++17 for related packages  (`#581 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/581>`_)
* [MuJoCo] enable to use mujoco simulatorMerge pull request (`#557 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/557>`_)

1.3.3 (2023-10-26)
------------------
* [Aerial Robot Model][Underactuated][Plugin] fix the wrong args assignment for model constructor (`#561 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/561>`_)
* [Robot Control] refactor the control framework (`#526 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/526>`_)
* [Robot Model] refactor the robot modelling framework (`#525 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/525>`_)

1.3.2 (2023-02-01)
------------------

1.3.1 (2022-07-02)
------------------
* [Spinal] Improve the process of servo bridge (`#499 <https://github.com/jsk-ros-pkg/aerial_robot/issues/499>`_)
* Fix the bug about the conversion of inertia tensor from SolidEdge (`#511 <https://github.com/jsk-ros-pkg/aerial_robot/issues/511>`_)
* Support ROS Noetic with Ubuntu Focal (`#507 <https://github.com/jsk-ros-pkg/aerial_robot/issues/507>`_)


1.3.0 (2021-08-06)
------------------

1.2.2 (2021-08-06)
------------------
* Full Thrust Vectoring Modeling and Control for Multilinked Aerial Robot With Two DoF Force Vectoring Apparatus (`#474 <https://github.com/JSKAerialRobot/aerial_robot/issues/474>`_)

1.2.1 (2021-02-17)
------------------
* Replace the old repositoty name in CHANGELOG.rst (`#434 <https://github.com/JSKAerialRobot/aerial_robot/issues/434>`_)
* Fix  naive assignment of the rotor direciton from the KDL model (`#418 <https://github.com/JSKAerialRobot/aerial_robot/issues/418>`_)
* Fix bug: correct the dimension of the gravity force vector (`#414 <https://github.com/JSKAerialRobot/aerial_robot/issues/414>`_)
* Implement gimbal vectoring planner for hydrus_xi under actuated model (`#405 <https://github.com/JSKAerialRobot/aerial_robot/issues/405>`_)


1.2.0 (2020-05-31)
------------------
* Refactor aeiral_robot_base (`#406 <https://github.com/JSKAerialRobot/aerial_robot/issues/406>`_)
* Implement several extra control methods for dragon (`#401 <https://github.com/JSKAerialRobot/aerial_robot/issues/401>`_)
* Add robot namespace for whole system (`#399 <https://github.com/JSKAerialRobot/aerial_robot/issues/399>`_)
* Refactor aerial_robot_model/transformable_aerial_robot_model and implment Jacobians (`#398 <https://github.com/JSKAerialRobot/aerial_robot/issues/398>`_)
* Refactor Gazebo system (`#391 <https://github.com/JSKAerialRobot/aerial_robot/issues/391>`_)

1.1.1 (2020-04-19)
------------------

1.1.0 (2020-01-06)
------------------
* Fix Bug: allow the negative angle value from spinal (`#342 <https://github.com/JSKAerialRobot/aerial_robot/issues/342>`_)
* Add API to access private members in transformable_aerial_robot_model (`#285 <https://github.com/JSKAerialRobot/aerial_robot/issues/285>`_)
* Add rostest (`#337 <https://github.com/JSKAerialRobot/aerial_robot/issues/337>`_)
* Fix bug for travis test: add run dependency, robot_state_publisher
* Add one-shot mode for motor pwm test (`#313 <https://github.com/JSKAerialRobot/aerial_robot/issues/313>`_)
* Fix bug: wrong clamping of servo target bit value (`#308 <https://github.com/JSKAerialRobot/aerial_robot/issues/308>`_)

1.0.4 (2019-05-23)
------------------
* Fix the bug about the rotation validation check (`#296 <https://github.com/JSKAerialRobot/aerial_robot/issues/296>`_)
* Add actuator limit in servo_bridge (`#287 <https://github.com/JSKAerialRobot/aerial_robot/issues/287>`_)
* Fix bug: The storage order from array to Eigen::Matrix (`#288 <https://github.com/JSKAerialRobot/aerial_robot/issues/288>`_)
* Add low pass filter function in servo bridge (`#289 <https://github.com/JSKAerialRobot/aerial_robot/issues/289>`_)

1.0.3 (2019-01-08)
------------------
* Add torque (effort) publishing process for real machine from Spinal (`#254 <https://github.com/JSKAerialRobot/aerial_robot/issues/254>`_)
* Add new APIs for transformable_aerial_robot_model  (`#243 <https://github.com/JSKAerialRobot/aerial_robot/issues/243>`_, `#244 <https://github.com/JSKAerialRobot/aerial_robot/issues/244>`_, `#250 <https://github.com/JSKAerialRobot/aerial_robot/issues/250>`_, `#251 <https://github.com/JSKAerialRobot/aerial_robot/issues/251>`_)

1.0.2 (2018-11-24)
------------------
* Create the improved and integrated servo bridge for both real machine and gazebo simulation (`#230 <https://github.com/JSKAerialRobot/aerial_robot/issues/230>`_)
* Implement full forward kinematics for tree sturcture (`#223 <https://github.com/JSKAerialRobot/aerial_robot/issues/223>`_)

1.0.1 (2018-11-05)
------------------
* modified the interactive marker processing for the tf broadcasting (#210)
* add API: getRotorsNormalFromCog (#206)

1.0.0 (2018-09-26)
------------------
* first formal release
* Contributors: Moju Zhao, Tomoki Anzai
