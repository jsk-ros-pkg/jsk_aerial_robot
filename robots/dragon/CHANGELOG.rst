^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dragon
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.4 (2019-05-23)
------------------
* Add actuator limit in servo_bridge (`#287 <https://github.com/tongtybj/aerial_robot/issues/287>`_)
* Refactor the function "DragonRobotModel::updateRobotModelImpl" (`#284 <https://github.com/tongtybj/aerial_robot/issues/284>`_)
* Add low pass filter function in servo bridge (`#289 <https://github.com/tongtybj/aerial_robot/issues/289>`_)
* Update the configuration for dragon quad type (`#277 <https://github.com/tongtybj/aerial_robot/issues/277>`_)

1.0.3 (2019-01-08)
------------------
* Add launch_gazebo flag to bringup files (`#265 <https://github.com/tongtybj/aerial_robot/issues/265>`_)
* Refactor the sensor plugins which uses `new Kalman Filter version <https://github.com/tongtybj/kalman_filter/tree/f7efb4d72131c02bf1632c6e4b400e2aeda60358>`_  (`#262 <https://github.com/tongtybj/aerial_robot/issues/262>`_
* Add torque (effort) publishing process from Spinal (`#254 <https://github.com/tongtybj/aerial_robot/issues/254>`_)

1.0.2 (2018-11-24)
------------------
* Remove old simulation servo bridge config file, and update the integrated servo config file (`#230 <https://github.com/tongtybj/aerial_robot/issues/230>`_)
* Add the gimbal vectoring test based on the desired tilt (`#229 <https://github.com/tongtybj/aerial_robot/issues/229>`_)
* Remove the unecessary vel feed-back state approximation for CoG control in gazebo simulation situation (`#218 <https://github.com/tongtybj/aerial_robot/issues/218>`_)

1.0.1 (2018-11-05)
------------------

1.0.0 (2018-09-26)
------------------
* first formal release
* Contributors: Moju Zhao, Tomoki Anzai
