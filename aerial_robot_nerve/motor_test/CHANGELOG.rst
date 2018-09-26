^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package motor_test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2018-09-26)
------------------
* Merge remote-tracking branch 'origin/devel' into ik
* Merge pull request `#148 <https://github.com/tongtybj/aerial_robot/issues/148>`_ from chibi314/motor_test_hydrus_xi
  Motor test hydrus xi
* fix motor_test package
* Add the motor test config yaml files
* Merge pull request `#109 <https://github.com/tongtybj/aerial_robot/issues/109>`_ from tongtybj/aerial_robot_nerve
  Integrating real machine layer: mcu development
* Add new directory "aerial_robot_nerve", to manage the communication system among pc, spinal and neuron
  1. spinal_ros_bridge:
  a) serial communication between pc and spinal based on rosserial_server.
  b) ros_lib generator for spinal ros interface
  2. spinal: code project (TrueStudio) for spinal mcu
  3. neuron: code project (TrueStudio) for neuron mcu
  4. motor_test: the motor property (pwm, thrust, cuurent) identification
* Contributors: Moju Zhao, Tomoki Anzai
