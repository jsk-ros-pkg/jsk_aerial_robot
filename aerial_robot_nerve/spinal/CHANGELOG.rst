^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spinal
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2018-09-26)
------------------
* Merge pull request `#169 <https://github.com/tongtybj/aerial_robot/issues/169>`_ from tongtybj/non_attitude_control
  Spinal内の姿勢制御のON/OFF切り替え
* Add flag to switch on/off regarding to the attitude control inside Spinal, via rosservice (rosserial)
* Merge pull request `#142 <https://github.com/tongtybj/aerial_robot/issues/142>`_ from tongtybj/fix_gps
  Improvement the GPS connection with Spinal via UART
* Improve the implemnetation of GPS uart rx part to be more robust (test with CAN net)
* Improve the interrupt callback method by modifying the IRQ_Handler()
* Change the connection method from DMA_RX_IT to normal RX_IT regarding to the GPS moduel via UART
* minal update owing to STM32CubeMx
* Merge pull request `#140 <https://github.com/tongtybj/aerial_robot/issues/140>`_ from tongtybj/modified_interrupt_func
  [spianal] Improvement of the usage of interupt callback funciton
* Merge remote-tracking branch 'origin/devel' into ik
* add transfer complete falg to trigerr the rosserial RX callback
* Merge pull request `#153 <https://github.com/tongtybj/aerial_robot/issues/153>`_ from tongtybj/uav_model_case_process
  Correct uav model case procesing, while using the spine system.
* Fix the wrong case processing to determine the uav model, especially for normal uav without transformability.
* Merge pull request `#150 <https://github.com/tongtybj/aerial_robot/issues/150>`_ from tongtybj/smooth_gyro
  This is a debug PR to check the behavior of embedded gyro sensor.
* Add debug line to compare the raw and smoothed gyro value.
* Improve the uart (dma) receive interrupt function declaration.
* Merge pull request `#134 <https://github.com/tongtybj/aerial_robot/issues/134>`_ from tongtybj/fix_gps
  Fix GPS dissconnetion problem while using CAN net
* Update the GPS related code and Cleanup the code of Ublox:
  Improve the declaration of DMA reveive interrupt and the related initialization
* Update the GPS related code and Cleanup the code of Ublox:
  1. removing unecessary routine and function
  2. rename the variables
* Update the initialize method for GPS.
  1. first initilize gps module (instance) before imu and barometer.
  2. improve the gps update rate in the internal moudle (ublox m8n)
* Merge remote-tracking branch 'origin/devel' into fix_gps
* Merge remote-tracking branch 'origin/devel' into fix_bug_rospy_err
* Merge pull request `#137 <https://github.com/tongtybj/aerial_robot/issues/137>`_ from tongtybj/move_spinal_msg
  Move spinal msg
* Add comment for rosservice of SetBoardConfig
* move parts of ros messages from aerial_robot_msgs and aerial_robot_base to spinal
* move hydrus/msg to spinal/msg
* Merge pull request `#131 <https://github.com/tongtybj/aerial_robot/issues/131>`_ from chibi314/spinal_ros_service
  Spinal ros service
* Fix the gps disconnection problem (while using CAN net) by changing the initialization way for GPS module
* change board_config_cmd topic to service
* change TX_BUFFER_WIDTH to 512 for GetBoardInfo
* fix typo: can_initalizer.h -> can_initializer.h
* change board_info topic to service
* Add missing library of spinal math in CMakeLists file.
* Merge pull request `#125 <https://github.com/tongtybj/aerial_robot/issues/125>`_ from tongtybj/new_dragon_model
  New dragon model
* Add the gyro smoothing flag for dragon in spinal
* Add .gitignore files for mcu project in TrueStudio
* Merge pull request `#109 <https://github.com/tongtybj/aerial_robot/issues/109>`_ from tongtybj/aerial_robot_nerve
  Integrating real machine layer: mcu development
* Add example to use ros logging in MUC
* Achieve rosserial server function
* Change the project path to the global one
* Add shell sripte to run muc SDK (True Studio)
* Add gitignore files for mcu project (TrueStudio)
* [spinal] Fix bug: wrong ros package for ros message
* Update simulation system:
  1. remove the submodeule: d_board. Instead, the wrapping process for flight controller in spinal is done by the catkinized package "spinal".
  2. one sample of moving the spinal associated ros message: PMatrixPseudoInverseWithInertia.msg from hydrus to spinal
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
* Contributors: KahnShi, Tomoki Anzai, tongtybj, 趙　漠居, 趙　漠居(Zhao, Moju)
