^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spinal_ros_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2018-09-26)
------------------
* set all of package.xml version to 0.0.0
* remove aerial_robot_nerve/spinal_ros_bridge/CHANGELOG.rst
* Merge pull request `#192 <https://github.com/tongtybj/aerial_robot/issues/192>`_ from tongtybj/rosserial_sub_buffer_size
  Fix the bad buffer size for subscriber in spinal_ros_bridge
* Fix the bad buffer size for subscriber in spinal_ros_bridge (original version is rosserial_sub_buffer_size)
* Merge remote-tracking branch 'origin/devel' into fix_gps
* Merge pull request `#131 <https://github.com/tongtybj/aerial_robot/issues/131>`_ from chibi314/spinal_ros_service
  Spinal ros service
* fixed typo (rosserive->rosservice)
* Merge pull request `#109 <https://github.com/tongtybj/aerial_robot/issues/109>`_ from tongtybj/aerial_robot_nerve
  Integrating real machine layer: mcu development
* Fix the bug about dependency for compile using ros message
* Achieve rosserial server function
* Update the dependency written in .rosinstall:
  Rosserail is no longer installed from source, but from debian packages.
  Because, we update the serial communication between pc and spinal, which does not need https://github.com/tongtybj/rosserial any more.
* Add new directory "aerial_robot_nerve", to manage the communication system among pc, spinal and neuron
  1. spinal_ros_bridge:
  a) serial communication between pc and spinal based on rosserial_server.
  b) ros_lib generator for spinal ros interface
  2. spinal: code project (TrueStudio) for spinal mcu
  3. neuron: code project (TrueStudio) for neuron mcu
  4. motor_test: the motor property (pwm, thrust, cuurent) identification
* Contributors: Moju Zhao, Tomoki Anzai
