^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aerial_robot_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.4 (2024-11-02)
------------------
* [Navigation][Land] improve the landing process to be more safe and precise (`#603 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/603>`_)
* [navigation] failsafe for high input voltage (`#588 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/588>`_)
* [Control][Landing][Bug]  revert the missing initialization for variable about force landing Merge pull request (`#585 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/585>`_)
* [Navigation][Land] refactor the landing convergence condition  (`#583 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/583>`_)
* [CMake] use C++17 compiler for all packages that contain C++ files  (`#581 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/581>`_)
* [Typo] Fix typos (`#563 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/563>`_)
* [Navigation] trajectory generation based on optimization methods (e.g. minimum snap trajectory)  (`#556 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/556>`_)
* [Navigation] improve keyboard teleopration that allows to move the robot with uniform linear motion (`#555 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/555>`_)
* [Navigation] improve the landing motion for all robots  (`#554 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/554>`_)

1.3.3 (2023-10-26)
------------------
* [Navigation] refactor landing and halt command during vel acc-based mode (`#553 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/553>`_)
* [Robot Control] refactor the control framework (`#526 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/526>`_)
* [Robot Model] refactor the robot modelling framework (`#525 <https://github.com/jsk-ros-pkg/jsk_aerial_robot/issues/525>`_)

1.3.2 (2023-02-01)
------------------
* Refactor the flight navigation procedure: add message to inform user that ready for takeoff (`#528 <https://github.com/jsk-ros-pkg/aerial_robot/issues/528>`_)

1.3.1 (2022-07-02)
------------------

* Fix bug of non-return function for boolean function to support ROS Noetic with Ubuntu Focal (`#507 <https://github.com/jsk-ros-pkg/aerial_robot/issues/507>`_)
* Fix intialization bug for fully actuated control (`#502 <https://github.com/jsk-ros-pkg/aerial_robot/issues/502>`_)

1.3.0 (2021-08-06)
------------------

1.2.2 (2021-08-06)
------------------
* Fix bug about switching the flight navigation mode in x,y,z,yaw (`#492 <https://github.com/JSKAerialRobot/aerial_robot/issues/492>`_)
* Fix bug about switching flight control mode by xyControlModeCallback (`#476 <https://github.com/JSKAerialRobot/aerial_robot/issues/476>`_)
* Enable acc nav mode via teleop_command/ctrl_mode (`#470 <https://github.com/JSKAerialRobot/aerial_robot/issues/470>`_)
* Fixed bug: enable acc nav mode (`#449 <https://github.com/JSKAerialRobot/aerial_robot/issues/449>`_)


1.2.1 (2021-02-17)
------------------
* Refactor the process to avoid saturation in z and yaw axes (`#444 <https://github.com/JSKAerialRobot/aerial_robot/issues/444>`_)
* Add demo to follow a circle trajectory (`#441 <https://github.com/JSKAerialRobot/aerial_robot/issues/441>`_)
* Replace the old repositoty name in CHANGELOG.rst (`#434 <https://github.com/JSKAerialRobot/aerial_robot/issues/434>`_)
* Automatically stop motors by checking the  impact  from the  ground in force landing phase. (`#419 <https://github.com/JSKAerialRobot/aerial_robot/issues/419>`_)


1.2.0 (2020-05-31)
------------------
* Refactor aerial_robot_base (`#406 <https://github.com/JSKAerialRobot/aerial_robot/issues/406>`_)
* Refactor the gain solver for optimal control (`#397 <https://github.com/JSKAerialRobot/aerial_robot/issues/397>`_)
* first formal release
* Contributors: Moju Zhao
