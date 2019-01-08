^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aerial_robot_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
