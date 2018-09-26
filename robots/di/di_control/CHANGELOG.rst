^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package di_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2018-09-26)
------------------

1.0.0 (2018-09-26)
------------------
* Update CMakeLists.txt:
  1. Eigen dependency description
  2. Cmake Policy 0046
* Merge remote-tracking branch 'origin/devel' into fix_gps
* Merge remote-tracking branch 'origin/devel' into fix_bug_rospy_err
* Merge pull request `#137 <https://github.com/tongtybj/aerial_robot/issues/137>`_ from tongtybj/move_spinal_msg
  Move spinal msg
* move all ros messages from aerial_robot_base to aerial_robot_msgs
* move parts of ros messages from aerial_robot_msgs and aerial_robot_base to spinal
* Merge branch 'devel' into aerial_transportation
* Merge pull request `#81 <https://github.com/tongtybj/aerial_robot/issues/81>`_ from tongtybj/new_control_system
  Flight control plugin
* Make the flight control to be the plguin:
  1. change the existing flight control called differential flatness pid control to control x/y/z/yaw
  2. make the state machine of flight naviagtion more clear
  3. sperate the flight_navigation and flight_control module
* Merge pull request `#78 <https://github.com/tongtybj/aerial_robot/issues/78>`_ from tongtybj/special_robots
  Special robots
* Create a package to integate all special robots
* Contributors: Moju Zhao
