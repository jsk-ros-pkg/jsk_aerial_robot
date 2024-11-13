^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spinal
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.4 (2024-11-02)
------------------
* [Spinal] Fix bug that sends zero pwm value to motors on reset (`#625 <https://github.com/jsk-ros-pkg/aerial_robot/issues/625>`_)
* [Spinal][PwmTest] workaround to send pwm value to each servo independently (`#613 <https://github.com/jsk-ros-pkg/aerial_robot/issues/613>`_)
* Resolve dependency to other package Merge pull request (`#610 <https://github.com/jsk-ros-pkg/aerial_robot/issues/610>`_)
* [Spinal][IMU] developing a interface for ICM20948 Merge pull request (`#599 <https://github.com/jsk-ros-pkg/aerial_robot/issues/599>`_)
* [Spinal][STM32H7] correct the initialization order of spinal-IMU Merge pull request (`#586 <https://github.com/jsk-ros-pkg/aerial_robot/issues/586>`_)

1.3.3 (2023-10-26)
------------------

1.3.2 (2023-02-01)
------------------
* [Spinal][CAN] fix bug about the motor pwm in spinal-neuron system (`#538 <https://github.com/jsk-ros-pkg/aerial_robot/issues/538>`_)
* [rqt][spinal] fix the compatibility in Python3 (`#535 <https://github.com/jsk-ros-pkg/aerial_robot/issues/535>`_)
* [Spinal][Neuron][Servo] solve the joint pulley round offset problem (`#534 <https://github.com/jsk-ros-pkg/aerial_robot/issues/534>`_)
* [Spinal][motor][PWM] set the desired PWM output to timer handllers regardless of the CAN connection (`#531 <https://github.com/jsk-ros-pkg/aerial_robot/issues/531>`_)
* [Spinal] refactor the process when no nueron connected (`#533 <https://github.com/jsk-ros-pkg/aerial_robot/issues/533>`_)
* [Spinal][rosserial] use UART for STM32H7 as default (`#530 <https://github.com/jsk-ros-pkg/aerial_robot/issues/530>`_)
* [Spinal] remove unnecessary files (`#532 <https://github.com/jsk-ros-pkg/aerial_robot/issues/532>`_)
* [Spinal][rosserial] increase the size of OUTPUT_SIZE for publisher (`#522 <https://github.com/jsk-ros-pkg/aerial_robot/issues/522>`_)
* [Spinal] refactor the callback process according to the value of motor_number (`#519 <https://github.com/jsk-ros-pkg/aerial_robot/issues/519>`_)
* [rqt_gui][rosparam] improve the parse process about the servo information based on rosparam (`#518 <https://github.com/jsk-ros-pkg/aerial_robot/issues/518>`_)


1.3.1 (2022-07-02)
------------------
* [Spinal] Improve the process of servo bridge (`#499 <https://github.com/jsk-ros-pkg/aerial_robot/issues/499>`_)
* [DRAGON] add IIR LPF for IMU gyro data (`#514 <https://github.com/jsk-ros-pkg/aerial_robot/issues/514>`_)
* [Spinal] simplify the model assignment process in spinal (`#498 <https://github.com/jsk-ros-pkg/aerial_robot/issues/498>`_)
* [Spinal] Support STM32H7 chip (`#504 <https://github.com/jsk-ros-pkg/aerial_robot/issues/504>`_)
* Correct the python scripts for rqt gui to be compatible in ros noetic (`#510 <https://github.com/jsk-ros-pkg/aerial_robot/issues/510>`_)
* Support ROS Noetic with Ubuntu Focal (`#507 <https://github.com/jsk-ros-pkg/aerial_robot/issues/507>`_)


1.3.0 (2021-08-06)
------------------
* Implement RTOS for spinal and neuron  (`#483 <https://github.com/JSKAerialRobot/aerial_robot/issues/483>`_)
* Migrate the HAL_Driver to the latest version for spinal and neuron (`#417 <https://github.com/JSKAerialRobot/aerial_robot/issues/417>`_)
* Implement external magnetic encoder for joint servo in neuron (`#416 <https://github.com/JSKAerialRobot/aerial_robot/issues/416>`_)


1.2.2 (2021-08-06)
------------------
* Full Thrust Vectoring Modeling and Control for Multilinked Aerial Robot With Two DoF Force Vectoring Apparatus (`#474 <https://github.com/JSKAerialRobot/aerial_robot/issues/474>`_)
* Refactor interface in spinal to handle more external magnetometer built in GPS module (e.g., LIS3MDL)  (`#458 <https://github.com/JSKAerialRobot/aerial_robot/issues/458>`_)

1.2.1 (2021-02-17)
------------------
* Fix bugs in spinal regarding pwm test (`#440 <https://github.com/JSKAerialRobot/aerial_robot/issues/440>`_)
* Replace the old repositoty name in CHANGELOG.rst (`#434 <https://github.com/JSKAerialRobot/aerial_robot/issues/434>`_)
* Increase the initial IMU calibration duration since of the increase of the spinal initilization time (`#411 <https://github.com/JSKAerialRobot/aerial_robot/issues/411>`_)
* Fixed bug of rosserial data buffer overflow add delay for Dynamixel XL430  (`#409 <https://github.com/JSKAerialRobot/aerial_robot/issues/409>`_)


1.2.0 (2020-05-31)
------------------
* Refactor aeiral_robot_base (`#406 <https://github.com/JSKAerialRobot/aerial_robot/issues/406>`_)
* Implement several extra control methods for dragon (`#401 <https://github.com/JSKAerialRobot/aerial_robot/issues/401>`_)
* Add robot namespace for whole system (`#399 <https://github.com/JSKAerialRobot/aerial_robot/issues/399>`_)
* Refactor Gazebo system (`#391 <https://github.com/JSKAerialRobot/aerial_robot/issues/391>`_)

1.1.1 (2020-04-19)
------------------
* Make the magnetic declination configurable from ros and rqt_qui, and augument the state publish from GPS. (`#395 <https://github.com/JSKAerialRobot/aerial_robot/issues/395>`_)
* Abolish the tranformation calculation from ellipsoid to sphere in magnetometer calibration (`#372 <https://github.com/JSKAerialRobot/aerial_robot/issues/372>`_)
* Change GPS location variable type (`#374 <https://github.com/JSKAerialRobot/aerial_robot/issues/374>`_)
* Increase the waiting time for the external magnetometer (Ublox) to start (`#368 <https://github.com/JSKAerialRobot/aerial_robot/issues/368>`_)
* Publish flight_state to provide the current state (`#365 <https://github.com/JSKAerialRobot/aerial_robot/issues/365>`_)

1.1.0 (2020-01-06)
------------------
* Improve pwm saturation measures (`#362 <https://github.com/JSKAerialRobot/aerial_robot/issues/362>`_)
* Make the ADC scale in battery status configurable (`#358 <https://github.com/JSKAerialRobot/aerial_robot/issues/358>`_)
* Refactoring of IMU calibration system (`#357 <https://github.com/JSKAerialRobot/aerial_robot/issues/357>`_)
* Fix bug: the wrong magnetometer value publishing in dragon model (`#355 <https://github.com/JSKAerialRobot/aerial_robot/issues/355>`_)
* Add hydrus_xi (full_acutated model) model (`#340 <https://github.com/JSKAerialRobot/aerial_robot/issues/340>`_)
* Neuron Improvement in servo connection (`#329 <https://github.com/JSKAerialRobot/aerial_robot/issues/329>`_)
* Correct the compile configuration for spinal and neuron f1 (`#311 <https://github.com/JSKAerialRobot/aerial_robot/issues/311>`_)
* Implement dynamixel TTL and RS485 mixed mode (`#320 <https://github.com/JSKAerialRobot/aerial_robot/issues/320>`_)
* Increase the  max registration size for ros publisher/subscirber in spinal (`#321 <https://github.com/JSKAerialRobot/aerial_robot/issues/321>`_)
* Fix bug about servo and add GUI in Spinal (`#302 <https://github.com/JSKAerialRobot/aerial_robot/issues/302>`_, `#308 <https://github.com/JSKAerialRobot/aerial_robot/issues/308>`_)

1.0.4 (2019-05-23)
------------------
* Add the torque enable/disable flag for extra servo directly connected with spinal (`#252 <https://github.com/JSKAerialRobot/aerial_robot/issues/252>`_)
* Add attitude_flag when checking the timeout of the flight command (`#299 <https://github.com/JSKAerialRobot/aerial_robot/issues/299>`_)
* Add actuators (e.g. joint, gimbal) disable process in force landing phase (`#290 <https://github.com/JSKAerialRobot/aerial_robot/issues/290>`_)

1.0.3 (2019-01-08)
------------------
* Fix the wrong error id  from rosserial in spinal (`#264 <https://github.com/JSKAerialRobot/aerial_robot/issues/264>`_)
* Add TIM5 for reserve servo control timer in spinal (`#237 <https://github.com/JSKAerialRobot/aerial_robot/issues/237>`_)
* Update the voltage process in spinal by removing the voltage divider (`#245 <https://github.com/JSKAerialRobot/aerial_robot/issues/245>`_)
* Add the logging via rosseial for the flight control debug (`#239 <https://github.com/JSKAerialRobot/aerial_robot/issues/239>`_)
* Fix the inactive problem of GPS module in Spinal (`#241 <https://github.com/JSKAerialRobot/aerial_robot/issues/241>`_)

1.0.2 (2018-11-24)
------------------

1.0.1 (2018-11-05)
------------------
* Add stm32f4 version neuron project (#213)

1.0.0 (2018-09-26)
------------------
* first formal release
* Contributors: Moju Zhao, Tomoki Anzai, Fan Shi
