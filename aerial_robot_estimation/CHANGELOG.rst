^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aerial_robot_estimation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2018-09-26)
------------------

1.0.0 (2018-09-26)
------------------
* Update CMakeLists.txt:
  1. Eigen dependency description
  2. Cmake Policy 0046
* Merge pull request `#122 <https://github.com/tongtybj/aerial_robot/issues/122>`_ from chibi314/bug_fix_estimation_opencv_func
  fix goodFeaturesToTrack in optical_flow.cpp
* fix goodFeaturesToTrack in optical_flow.cpp
* Merge branch 'devel' into aerial_transportation
* Merge pull request `#78 <https://github.com/tongtybj/aerial_robot/issues/78>`_ from tongtybj/special_robots
  Special robots
* Move the hydrus robot model meterials from  aerial_robot_model to hydrus
* Merge pull request `#73 <https://github.com/tongtybj/aerial_robot/issues/73>`_ from tongtybj/cog_based_control
  Cog based estimaton, control and navigation
* Update the odom topic name
* Optical flow: Update the topic name of the baselink odometry
* change optical flow files to enable compilation without GPU
* Merge pull request `#47 <https://github.com/tongtybj/aerial_robot/issues/47>`_ from chibi314/optical_flow
  Optical flow
* update the launch file and add yaml file for rosparam
* change to nodelet
  fix bug about image crop
* add launch file
* add image crop scale
* implement camera velocity publisher
* add aerial_robot_estimation pkg
* Contributors: Moju Zhao, Tomoki Anzai
