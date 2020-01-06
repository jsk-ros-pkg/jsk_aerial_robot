^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aerial_robot_model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix Bug: allow the negative angle value from spinal (`#342 <https://github.com/tongtybj/aerial_robot/issues/342>`_)
* Add API to access private members in transformable_aerial_robot_model (`#285 <https://github.com/tongtybj/aerial_robot/issues/285>`_)
* Add rostest (`#337 <https://github.com/tongtybj/aerial_robot/issues/337>`_)
* Fix bug for travis test: add run dependency, robot_state_publisher
* Add one-shot mode for motor pwm test (`#313 <https://github.com/tongtybj/aerial_robot/issues/313>`_)
* Fix bug: wrong clamping of servo target bit value (`#308 <https://github.com/tongtybj/aerial_robot/issues/308>`_)

1.0.4 (2019-05-23)
------------------
* Fix the bug about the rotation validation check (`#296 <https://github.com/tongtybj/aerial_robot/issues/296>`_)
* Add actuator limit in servo_bridge (`#287 <https://github.com/tongtybj/aerial_robot/issues/287>`_)
* Fix bug: The storage order from array to Eigen::Matrix (`#288 <https://github.com/tongtybj/aerial_robot/issues/288>`_)
* Add low pass filter function in servo bridge (`#289 <https://github.com/tongtybj/aerial_robot/issues/289>`_)

1.0.3 (2019-01-08)
------------------
* Add torque (effort) publishing process for real machine from Spinal (`#254 <https://github.com/tongtybj/aerial_robot/issues/254>`_)
* Add new APIs for transformable_aerial_robot_model  (`#243 <https://github.com/tongtybj/aerial_robot/issues/243>`_, `#244 <https://github.com/tongtybj/aerial_robot/issues/244>`_, `#250 <https://github.com/tongtybj/aerial_robot/issues/250>`_, `#251 <https://github.com/tongtybj/aerial_robot/issues/251>`_)

1.0.2 (2018-11-24)
------------------
* Create the improved and integrated servo bridge for both real machine and gazebo simulation (`#230 <https://github.com/tongtybj/aerial_robot/issues/230>`_)
* Implement full forward kinematics for tree sturcture (`#223 <https://github.com/tongtybj/aerial_robot/issues/223>`_)

1.0.1 (2018-11-05)
------------------
* modified the interactive marker processing for the tf broadcasting (#210)
* add API: getRotorsNormalFromCog (#206)

1.0.0 (2018-09-26)
------------------
* first formal release
* Contributors: Moju Zhao, Tomoki Anzai
