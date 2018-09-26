^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aerial_robot_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2018-09-26)
------------------
* set all of package.xml version to 0.0.0
* Merge remote-tracking branch 'origin/devel' into fix_gps
* Merge remote-tracking branch 'origin/devel' into fix_bug_rospy_err
* Merge pull request `#137 <https://github.com/tongtybj/aerial_robot/issues/137>`_ from tongtybj/move_spinal_msg
  Move spinal msg
* move all ros messages from aerial_robot_base to aerial_robot_msgs
* move parts of ros messages from aerial_robot_msgs and aerial_robot_base to spinal
* Merge pull request `#105 <https://github.com/tongtybj/aerial_robot/issues/105>`_ from tongtybj/voltage_based_pwm
  Conversion from thrust to pwm based on the voltage and nonlinear relationship.
* Upadate the motor info ros message and create the pwm message
* Merge branch 'devel' into aerial_transportation
* Merge pull request `#81 <https://github.com/tongtybj/aerial_robot/issues/81>`_ from tongtybj/new_control_system
  Flight control plugin
* Make the flight control to be the plguin:
  1. change the existing flight control called differential flatness pid control to control x/y/z/yaw
  2. make the state machine of flight naviagtion more clear
  3. sperate the flight_navigation and flight_control module
* Merge pull request `#52 <https://github.com/tongtybj/aerial_robot/issues/52>`_ from tongtybj/rosserial
  Flexible topic sized for rosserial.
* Use new integrated message type: RollPitchYawTerm to stand for gain and control term
* Change the fixed sized topics to be flexible related to the rosserial.
* Merge pull request `#3 <https://github.com/tongtybj/aerial_robot/issues/3>`_ from tongtybj/flight_command
  1. Update the hydrus x robot model
  2. Change the LQI control framework.
  3. Change the launch file structure first mainly for hydrusx.
  4. Add Uav communication with ground station using Xbee
  5. Add Failsafe system for UAV.
* Modified the ros message type:
  aerial_robot_msgs::RollPitchYawGain -> hydrus_transform_control::RollPitchYawGain
  aerial_robot_msgs::YawThrottleGain  -> aerial_robot_msgs::FourAxisGain
* Remove depreacated ros msg: RcData.msg
* Change the flight command structure from PC to MCU.
  - 3 axis angle command: roll, pitch and yaw
  - base throttle: throttle for z axis and PI term for yaw
* Resize the ControlTerm from float32 to int16
* Resize the RollPtichYawGain from float32 to int16 (half).
  For severe communication between PC and MCU
* Add the terrain check for the range sensor using original flowchats for better height control
* Add new message and motor info yaml(pwm force torque)
* Fix for the d_board hydrus control
* Remove unecessary message
* change the topic name to kduino (remove prefix "kduino")
* remove unnecessary message
* Move YawThrottleGain.msg from aerial_robot_base to aerial_robot_msgs
* this temperate
* Merge branch 'jade-devel' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into jade-devel
* add
* add some new message here
* sime
* ok
* Merge branch 'indigo-dev' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev
* Merge branch 'indigo-dev' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev
* some modification in aerial tracking and some comiit in aerial robot model
* 1. kalman filter for px4flow
  2. control input for old system
* fix the msg type of yaw pi term(10000x)
* add control term msg
* add control term check msg
* change the rpy_gain msg form float type to int type
* change the msg of kduinosimple
* fix the rotate angle problem
* some changes to commit before update to 14.04
* some changes to commit before update to 14.04
* some modification
* fix the casting problem(int => float)
* some modification for indigo, especially for odroid u3
* modified kduino simple imu msg
* some of the changes to be update
* modified the from cfg to aerial_robot_base to flight_navigation
* some modification for catkin system
* 1)add hydra movit config
  2)modified the aerial robot model for catkin system
* 1) add hydra directorry for transform control and moveit config
  2) modify the aerial robot model for catkin build system
* add renamed pkgs
* Contributors: Moju Zhao, Tomoki Anzai
