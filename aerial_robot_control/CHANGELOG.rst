^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aerial_robot_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
