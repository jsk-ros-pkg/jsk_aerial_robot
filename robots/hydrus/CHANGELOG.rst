^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hydrus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add launch_gazebo flag to bringup files (`#265 <https://github.com/tongtybj/aerial_robot/issues/265>`_)
* Update the usage of new API of `Kalman Filter <https://github.com/tongtybj/kalman_filter/tree/f7efb4d72131c02bf1632c6e4b400e2aeda60358>`_  for ego-motion estimation (`#262 <https://github.com/tongtybj/aerial_robot/issues/262>`_)
* Update the ego-motion estimation system based on IMU + GPS  (`#261 <https://github.com/tongtybj/aerial_robot/issues/261>`_)
* Add/Refactor rqt_plot scripts and sensor plugins to visualize the state estimation result from sensor fusion more clearly (`#261 <https://github.com/tongtybj/aerial_robot/issues/261>`_)
* Quantify the joint torque scale of hydrus real machine using Dynamixel MX28AR (`#255 <https://github.com/tongtybj/aerial_robot/issues/255>`_)
* Add torque (effort) publishing process from Spinal (`#254 <https://github.com/tongtybj/aerial_robot/issues/254>`_)
* Update the robot description (e.g., model urdf, sensor config) for egomotion-estimation (`#235 <https://github.com/tongtybj/aerial_robot/issues/235>`_, `#238 <https://github.com/tongtybj/aerial_robot/issues/238>`_, `#247 <https://github.com/tongtybj/aerial_robot/issues/247>`_, `#256 <https://github.com/tongtybj/aerial_robot/issues/256>`_)


1.0.2 (2018-11-24)
------------------

* Add urdf model and config file for hyrdus quad bringup attached with the ZED mini stereo camera, and set this model as the defualt in bringup launch file (`#233 <https://github.com/tongtybj/aerial_robot/issues/233>`_)
* Remove old simulation servo bridge config file, and update the integrated servo config file (`#230 <https://github.com/tongtybj/aerial_robot/issues/230>`_)

1.0.1 (2018-11-05)
------------------

1.0.0 (2018-09-26)
------------------
* first formal release
* Contributors: Moju Zhao, Tomoki Anzai, Fan Shi
