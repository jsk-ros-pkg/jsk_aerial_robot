^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aerial_robot_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2018-09-26)
------------------
* set all of package.xml version to 0.0.0
* Update the rosmsg type
* Merge pull request `#193 <https://github.com/tongtybj/aerial_robot/issues/193>`_ from tongtybj/default_takeoff_height
  Remove unecessary offeset in the defualt takeoff height setting
* Remove unecessary offeset in the defualt takeoff height setting
* Merge pull request `#182 <https://github.com/tongtybj/aerial_robot/issues/182>`_ from tongtybj/new_optitrack_inferface
  update the multicast IP address for new optitrack motive in windows
* update the multicast IP address for new optitrack motive in windows
* Merge pull request `#180 <https://github.com/tongtybj/aerial_robot/issues/180>`_ from tongtybj/sensor_tf
  Improvement of sensor plugins for sensor fusion
* Improve the sensor plugins:
  1. imu: remove unecessary callback from imu board "kduino", and add rotation information for imu ros message
  2. optical flow: improve the case process for raw velocity (global or local)
  3. altitude: improve the distance from baselink to ground
* 1. Improve the frame transformation between sensor and baeslink for each sensor module (plugin)
  2. Add NED frame in GPS sensor module (plugin)
* Merge pull request `#177 <https://github.com/tongtybj/aerial_robot/issues/177>`_ from tongtybj/new_kalman_filter_interface
  Improvement of sensor fusion with improved kalman filter method
* Update the usage of the kalman filter in each sensor handler, with the aim of the consideration of time latency
* Update the the derived kalman filter class, based on the improved base kalman filter API
* Merge pull request `#176 <https://github.com/tongtybj/aerial_robot/issues/176>`_ from tongtybj/vio
  Impove the accurate of sensor fusion (VIO + IMU), by considering the latency of sensor timestamp
* Add time delay of vio odometry timestamp, which improves the sensor fusion a lot
* Merge pull request `#168 <https://github.com/tongtybj/aerial_robot/issues/168>`_ from tongtybj/vio
  visual odometry / visual inertial odometry based ego-motion estimation
* Add the EKF (kf_xy_roll_pitch_bias) for vo/vio based egmotion-estimation
* Add debug verbose for vo/vio estimation filter
* Fix the bug of vo/vio based pos+vel estimation
* temp
* Merge pull request `#161 <https://github.com/tongtybj/aerial_robot/issues/161>`_ from tongtybj/non_height_offset
  remove unecessary height offset from flight system
* Update the flight control and navigation regarding to landing process without height offset
* Remove the height offset from motion capture
* Merge pull request `#159 <https://github.com/tongtybj/aerial_robot/issues/159>`_ from tongtybj/sensor_launch_file
  Improvement the sensor launch and yaml files for hydrus
* Move the location of rosbag record command files to correct plase
* Add rosbag record commands
* Improve the rule of launch files realted to sensors
* Improve the rule to set onboard sensor and ego-motion estimation (sensor fusion) based on YAML
* Clean up the verbose for ros param
* Merge pull request `#147 <https://github.com/tongtybj/aerial_robot/issues/147>`_ from tongtybj/ik
  Release motion planning method based on differential kinematics
* Crate the differentia kinematics based path planning framework,
  including the QP planner core, and plugin structure for cost and constraint
* Update CMakeLists.txt:
  1. Eigen dependency description
  2. Cmake Policy 0046
* Merge remote-tracking branch 'origin/devel' into fix_gps
* Merge remote-tracking branch 'origin/devel' into fix_bug_rospy_err
* Merge pull request `#137 <https://github.com/tongtybj/aerial_robot/issues/137>`_ from tongtybj/move_spinal_msg
  Move spinal msg
* move all ros messages from aerial_robot_base to aerial_robot_msgs
* move parts of ros messages from aerial_robot_msgs and aerial_robot_base to spinal
* Fix the bug for the rotation of CoG frame
* 1. Fix bug for nav callback, that omitting the flight command under takeoff and landing state
  2. Fix the bug for low battery landing mode, which the delta of target-estimated is calculated after the new target height is set
* Merge pull request `#126 <https://github.com/tongtybj/aerial_robot/issues/126>`_ from tongtybj/pos_vel_navigation
  Feed forward velocity control for translational and yaw motion.
* Add pos and vel simulaneous control mode for translational and yaw control, which performing bettwer maneuvering for trajectrory following.
* Merge pull request `#125 <https://github.com/tongtybj/aerial_robot/issues/125>`_ from tongtybj/new_dragon_model
  New dragon model
* Add the dynamic bettery internal resistance calculation for the voltage drop
* Merge pull request `#109 <https://github.com/tongtybj/aerial_robot/issues/109>`_ from tongtybj/aerial_robot_nerve
  Integrating real machine layer: mcu development
* Update the launch file to run serial process between pc and spinal
* Add new directory "aerial_robot_nerve", to manage the communication system among pc, spinal and neuron
  1. spinal_ros_bridge:
  a) serial communication between pc and spinal based on rosserial_server.
  b) ros_lib generator for spinal ros interface
  2. spinal: code project (TrueStudio) for spinal mcu
  3. neuron: code project (TrueStudio) for neuron mcu
  4. motor_test: the motor property (pwm, thrust, cuurent) identification
* 1.Add the threshold for yaw error to avoid large diffenrence between target yaw and state yaw.
  2.Add M_PI compensation for yaw error to have normalized yaw value
* Merge pull request `#107 <https://github.com/tongtybj/aerial_robot/issues/107>`_ from tongtybj/new_fight_config_cmd
  New flight config cmd message between ros and d_board
* Update the flight config cmd, which contains the cmd ID in the ros message to unify the protocol between ros and d_board
* Merge pull request `#105 <https://github.com/tongtybj/aerial_robot/issues/105>`_ from tongtybj/voltage_based_pwm
  Conversion from thrust to pwm based on the voltage and nonlinear relationship.
* Add the battery capacity check function in the flight navigation, which is implemented in spinaly in the past.
* Update the flight system to be suitable for the thrust->pwm conversion in the spinal board(d_board).
* Upadate the motor info ros message and create the pwm message
* Merge pull request `#104 <https://github.com/tongtybj/aerial_robot/issues/104>`_ from chibi314/bug_fix_typo
  fix typo in flatness_pid_controller.cpp
* fix typo in flatness_pid_controller.cpp
* Merge branch 'devel' into aerial_transportation
* Merge pull request `#72 <https://github.com/tongtybj/aerial_robot/issues/72>`_ from tongtybj/control
  Gyro moment compensation
* Fix the bug about the yaw velocity calculation and yaw angle lpf(no lpf for angle)
* Merge pull request `#102 <https://github.com/tongtybj/aerial_robot/issues/102>`_ from tongtybj/new_communication_protocol
  New communication protocol
* Refine the control system.
  1. send motor info and uav info before the motor arming phase from the base class.
  2. change the motor number management rule.
* Add the tf(transformation) broadcasting between world and cog(temporary parent)
* Update the RMS tracking errors, extending to the attitude pitch/roll axsi
* Merge pull request `#91 <https://github.com/tongtybj/aerial_robot/issues/91>`_ from tongtybj/dragon
  Commit from Dragon control system
* Add the LPF for the angular velocity while receiving mocap ground truth in the case of gazebo
* Fix the bug about the alt error calculation
* Merge branch 'devel' into aerial_transportation
* Add teleop_flag on/off switch process for the leveling landing
* Modified the flight configuration (e.g. control gains) to be able to do aerial transformation in gazebo simulation.
* Merge pull request `#87 <https://github.com/tongtybj/aerial_robot/issues/87>`_ from tongtybj/multilin-control
  Multilink based flight control
* 1.Update the flatness pid controller as a super class mode, which is inherited by the dragon gimbal control.
  2.Add yaw d control, since it is necessary for the gimbal control mode
* Merge pull request `#86 <https://github.com/tongtybj/aerial_robot/issues/86>`_ from tongtybj/estimation
  [State Estimation] Add the orientation of CoG frame
* Update the config for the state estimation for debug mode
* special estimation process:
  1. imu: the only module which can calcualte the orientation of CoG frame (except mocap which is necessary for gazebo env.)
  - the cog rpy and omega is calculated from these of baselink and the kinematics
  - also assign the cog rp(without yaw) and omega for ground_truth mode
  2. mocap: the only way to calculate the orientation of CoG frame in the case of gazebo simulation
  - also ssign the baselink yaw value for experiment_mode and ground_truth_mode
* Add the publishment for the CoG based orientation information,
  and correct the control/nav frame to be CoG, the estimation frame to be Baselink
* Add CoG frame orientation, indicating new 3 axis
* Merge pull request `#82 <https://github.com/tongtybj/aerial_robot/issues/82>`_ from tongtybj/cog_odometry
  Better CoG odometry
* Publish correct odom, especially the orientation of COG
* Correct the transform calculation between baselink and cog, since the orientation between baselink and cog may be different
* Add the header file for the differential flatness pid control
* Add groundtruth model for mocap in the real machine system
* Add estimate mode flag in the simulation.launch
* Merge pull request `#81 <https://github.com/tongtybj/aerial_robot/issues/81>`_ from tongtybj/new_control_system
  Flight control plugin
* Revert the imu calibration duration to 2 second
* Make the flight control to be the plguin:
  1. change the existing flight control called differential flatness pid control to control x/y/z/yaw
  2. make the state machine of flight naviagtion more clear
  3. sperate the flight_navigation and flight_control module
* Remove the so-called feedforward control in LQI, integrating into the error based feedback control like general PID
* Merge pull request `#73 <https://github.com/tongtybj/aerial_robot/issues/73>`_ from tongtybj/cog_based_control
  Cog based estimaton, control and navigation
* update the default config for optical flow
* fix the bug that force add the delay to the optical time stamp for sync
* Fix the wrong flag name for estimate mode
* Remove the lpf for yaw, since it causes a bad reuslt around -pi and pi
* Fix the bug for teleop vel mode interpolation
* Fix the bug which set wrong state (CoG -> Baselink)
* update the topic for the sensor fusion simulation
* For better debug
* add omage(angular vel) from ground truth
* fix the wrong state num
* add the change of flag status
* Add COG / Baselink flight navigation
  1. for pos nav(waypoint), we provide both COG target and Baselink target, the later one is converted to COG target
  2. for pos nav(waypoint), we also provide a vel-based waypoint, if the target point is far from the previous target point
* Change the state mask and related config yaml file
* Implement the COG based flight control
* Implmente the COG/BASELINK state estimation: 1. sensor fusion is porcess in the baselinkframe. 2 cog state is calculated with the baselink state, simple rigid body kinematics. 3: baselink and cog have same orientation
* Merge remote-tracking branch 'origin/kdl' into devel
* Change the way to assign the motor number from the rosparam to the subsribe result from fouraxisgain ros topic
* change the name for the link with FCU and IMU from root_link to baselink
* Merge pull request `#69 <https://github.com/tongtybj/aerial_robot/issues/69>`_ from tongtybj/control
  The non-principal inertial frame control system
* Remove the unnecessary the so-called feeforward gain from transform_control code, which is same with that of the feed-back control gain
* sensor fusion with optical flow: change back to the non-sync mode with pos-vel-acc-bais kf plugin
* add udp option for optical flow message, since we do the optical process on the other pc
* Update the remap for sensor fusion simulation launch file
* Merge pull request `#60 <https://github.com/tongtybj/aerial_robot/issues/60>`_ from tongtybj/outdoor
  Outdoor
* Fix the wrong state assignment in IMU plugin
* 1. Add new XY pos and vel estimation model: XY and Roll/Pitch Bias EKF.
  This estimates 6 states:  pos_x, vel_x, pos_y, vel_y, roll_bias, pitch_bias
  with 5 input: acc_xb, acc_yb, acc_zb, d_roll_bias, d_pitch_bias.
  2. Modify the imu, optical_flow, mocap plugin to use the new EKF estimation model.
  and confirm the validity of this new estimate model with imu-opti dataset.
* Modified the sensor plugins for the time synchronized framework of the KF, which is effective for the delay sensor like GPS
* change the sensor plugin update/correct function for sensor fusion to be suitable for EKF framework
* change the acc value transformation rule (body->world)
* Merge pull request `#52 <https://github.com/tongtybj/aerial_robot/issues/52>`_ from tongtybj/rosserial
  Flexible topic sized for rosserial.
* Merge pull request `#57 <https://github.com/tongtybj/aerial_robot/issues/57>`_ from tongtybj/control_frame
  Att control mode
* 1. add acc control mode from previous att control mode
  2. change the old xy control mode into control_mode + control_frame
* change the att_mode to acc_mode, and add control_frame
* move the const var of gravity rate from imu_sensor_plugin to basic_state_estimation
* Merge remote-tracking branch 'shi/att_fix' into control_frame
* Merge pull request `#54 <https://github.com/tongtybj/aerial_robot/issues/54>`_ from tongtybj/optical_flow
  [Optical flow] update sensor fusion with optical flow
* move the subscriber of transfom between cog and root link from sensor_base_plugin to bsaisc_state_estimation.
* 1. add atti control mode in flight_nav input system
  2. fix the bug about the switch among att/vel/pos control mode
* change the defualt uav odometry based on the root_link
* change the outlier check way, like a dynamic sigma change
* Add uav type ros message
* Change the fixed sized topics to be flexible related to the rosserial.
* Merge pull request `#47 <https://github.com/tongtybj/aerial_robot/issues/47>`_ from chibi314/optical_flow
  Optical flow
* Merge pull request `#46 <https://github.com/tongtybj/aerial_robot/issues/46>`_ from tongtybj/mocap
  fix the mocap kalman filter init bug
* fix the mocap kalman filter init bug
* Merge pull request `#42 <https://github.com/tongtybj/aerial_robot/issues/42>`_ from tongtybj/outdoor
  Outdoor Flight System
* Fix the wrong spell in the mocap plugin
* remove the wrong depend package in CMakeLists
* Update the debug mode for state estimation
* Add visual odometry plugin
* Update the optical flow pulgin
* Change the check method of state status
* Add CoG transform different from baselink transform(fix, board orientation based on imu board)
* Change the state status mode to estimation mode oriented
* Improve the flight_control mode related fuction:
  1. improve the att/vel/pos control mode switch processing
  2. add the att_mode in vel/pos mode if the x/y estimation is not already established
* Setup the launch file for the state estimation debug
* Change the pos noise sigma of mocap
* Change the vel noise sigma of GPS
* Add flag to enable or disable the joy_stick hear beat check
* Merge pull request `#23 <https://github.com/tongtybj/aerial_robot/issues/23>`_ from tongtybj/simulation
  Simulation
* add aerial_robot_estimation pkg
* Revert "remove simulation flag in sensor_base_plugin.h which is not necessary"
  This reverts commit 9c1e4a56901c948bb2e1eca6bd11bc5a13188bd5.
* change the subscribe topic name to a rosparam
* remove simulation flag in sensor_base_plugin.h which is not necessary
* Add GetMotorNum.srv
* Add motor num request service
* Merge pull request `#32 <https://github.com/tongtybj/aerial_robot/issues/32>`_ from tongtybj/outdoor
  Add the state estimation for the non-mocap enviroment.
* fix the wrong namespace of leddarone in yaml file
* add leddar one in the onbaord_sensors.launch, and the arg for serial port
* fix the wrong name of link
* add no height offset flag for the case that the sensor is very closed to the ground in the inital position
* Fix the wrong place of the yaw control frame in flight control system
* Merge remote-tracking branch 'origin/control' into outdoor
* Abolish the seting of vel_world_based_control -> pos_world_based_control process in the init phase
* 1. Add the clip function for the pos_error value of throttle(altitude).
  2. Set the p_term as zero in the landing phase for LQI control method.
* update the simulation launch file
* update the sensor fusion yaml for simulation
* update the parameter for gps in yaml file
* update the paramter for altitude in yaml file
* update the paramter for baro in yaml file
* add the correct initialization func of transform
* correct the order of the kalman filter initialization func
* add experiment estimate mode for altitude
* correct the order of the gps location value
* fix the wrong frame of yaw in acc transform
* use transform to calculate the yaw in cog frame
* Add the difference processing between body frame value and cog frame value
* Change the initialize process
* Add the yaw control frame selection: body(imu) frame or CoG frame
* Add two additional state: roll/pitch if the body frame(imu)
* Add the subscribe to get the transform from CoG to sensor body frame, which will be necessary in the future
* Update the dependency for necessary packages
* Merge pull request `#14 <https://github.com/tongtybj/aerial_robot/issues/14>`_ from tongtybj/new_structure
  Travis Config
* Fix the error of travis config
* Merge pull request `#12 <https://github.com/tongtybj/aerial_robot/issues/12>`_ from tongtybj/new_structure
  [New structure] Auto compile procedure
* Remove unnecessary dependent package
* Merge pull request `#4 <https://github.com/tongtybj/aerial_robot/issues/4>`_ from tongtybj/sensor_fusion
  Sensor fusion
* clean the bad coding
* Comment out the tf publish from navigation which is unnecessary
* Add simulation flag log out for sensor plugin
* Change the low battery checker status for uav
* Remove unnecessary log out in the cod
* - change the framework of the sensor plugins:
  imu, mocap, altitude, gps, optical_flow.
  - imu and mocap is tested with rosbag.
  -use rosbag to test alt, gps, opt
* 1. Change the state structure containing x/y/z/r/p/yaw in world frame + x/y/yaw in board frame, along with the structure of aerial_robot_base::State
  2. Change the get/set of state description in state_estimation, sensor_base_plugin, flight_control, flight_navigation.
* 1. Change the search method for plugin of sensor plugins and estimation fusion,
  along with the param setting for plugins in launch file and yaml file.
  2. Remove unnecessary file
* Fix the wrong acc-coord trnasformation bug
* Fix the uinsigned vs signed value comparison
* Add simulation flag for sensor and robot model launch file
* Remove unnecessary files
* Fix the imu-acc coordinate problem (COG vs Board )
  Also change the structure of the sensor plugins
* Merge pull request `#3 <https://github.com/tongtybj/aerial_robot/issues/3>`_ from tongtybj/flight_command
  1. Update the hydrus x robot model
  2. Change the LQI control framework.
  3. Change the launch file structure first mainly for hydrusx.
  4. Add Uav communication with ground station using Xbee
  5. Add Failsafe system for UAV.
* Change the rule of flight position control based on the experimental estiamtion
* Add the first activation phase for the sensor data health check, since there is some delay between the node initilization and callback function
* Change the timestamp update rule for mocap, using the ros::Time::now() for remote wireless transmission
* Change the timeout duration for mocap
* Change the throttle bias for general att pid control mode
* 1. Add joy stick heart beat function (normal landing mode for failsafe)
  2. Filter the flight command value not be zero for general att pid control mode
* Add sensor data health check func to confirm whether we get the fresh data from the sensor moudule.
  If not, we will provide the respound solution such as force landing. especially in the case of mocap.
  TODO: more plausible method for state eistimation besides the force landing.
* Add Acc.msg which is ImuData.msg previous, publishing the acceleration data
  based on different frame
* change the default serial port for imu module: /dev/ttyUSB0
* Change the ros node handle to private handle
* 1. change the LQI mode
  - shift yaw feed-forward control into the feedback part: error = target_yaw - state_yaw
  - add feed-forward control for throttle: error = target_throttle - state_throttle
  2. change the General Pid control code
  - merge the error calculation method into the LQI mode: reverse the signal of P and D gain for general yaw/throttle control
  - give different flight command sending method between LQI and general control mode
* Remove unnecessary code
* Modified the ros message type:
  aerial_robot_msgs::RollPitchYawGain -> hydrus_transform_control::RollPitchYawGain
  aerial_robot_msgs::YawThrottleGain  -> aerial_robot_msgs::FourAxisGain
* Change the flight command structure from PC to MCU.
  - 3 axis angle command: roll, pitch and yaw
  - base throttle: throttle for z axis and PI term for yaw
* Change the contents in xxxxx_sensors.launch. Summerized to "onboard_sensors.launch".
  Integrate the SensorsLoarder.yaml and SensorFusion.yaml
* Remove unnecessary rosmsg from CMakelists
* Resize the ControlTerm from float32 to int16
* Some change in the estimator_debugger.py
* Add q-euler conversion python code
* Change:
  1. integerate the sonar sensor and laser sensor to commom range sensor plugin
  2. seperate the optical_flow and sonar from the old optical flow sensor plugin.
* Test the baro-based altitude estiamte and control.
  Result: Soso
  Improvement: We have to test outside to check the behavior.
* Add Experiment State based Flight Control
* Add the joystick function to switch from att_control_mode to non_att_control_mode
* 1. Refine the joy callback
  2. Change the order of force landing and halt(force_landing->halt)
  3. Yaw control which can be switch automatically between vel_control and pos_control
* Add the terrain check for the range sensor using original flowchats for better height control
* Add barometer sensor plugin which is associated with mocap/range sensor plugins.
* 1. Remove the unnecessary callback func(roll, pitch, yaw, throttle)
  2. Add flying flag mode in estimator (for barometer)
* Add netusbcam usb2.0 device camera launch file
* change the force landing pwm for hawk
* Add telemetry communication launch file
* Add hearbeat check for gain_tuning_mode, and force land
* Fix the rtk-gps sensor fusion plugin
* Add hearbeat check for joy stick control in outdoor
* Add telemetry joy stick control
* hoge3
* hoge 2
* hoge temp
* Add range(leddar one) sensor plugin
* Add yaml file for new sensor module: leddar_one & rtk_gps
* Change the config files for tarot680 for sensor fusion, (imu + leddar + gps)
* Fix the takeoff phase without precise sensors(e.g. imu + sonar) for the height estimation, using undescending mode
* Merge remote-tracking branch 'origin/jade-devel' into jade-devel
* Change the pos pid control gain for the hydrus3.
  Also note that we add special amplified rate for the gyro integration in terms of the attitude estimation
* Add the z-axis estimation in takeoff phase(must not be below 0)
* Add force landing pwm
* Fix the throttle range (add yaw elements)
* Fix the wrong order of rpy to quaternion
* Add pos yaw usual/strong control switch rate
  1. callback to rereive the swich message
  2. string rate is 10 times
* Add xy velocity contorl usual/weak control switch part,
  1. callback for call weak control gain
  2. weak gain rate (0.2)
* Add python script for rosbag processing
* Add several sensors launch files such as leddar_one, rtk_gps
* change the way to control xy movement from joy stick (previous: push joy stick, now: push lef down trigger)
* Fix the bug abouth psi(yaw) flight nav receive part
* Fix the x/y joy control bug
* Remove the x/y vel 2 level mode, the vel will be controlled by joystick only if pushing the joystick
* Modified the pos gain for tarot810, not so good
* Fix the bad yaw filtering problem in IIR filter(use raw value in mocap)
* Comment out the joy launch for rook, since tx1 has problem with bluetooth
* change the flightnav contents for xy, psi,z independant control
* 1) shift state_mode\_ from flight_navigation to state_estimation
  2) add nav_msgs::Odometry publish in state_estimation
  3) change the states and nav_cmd topic name
* Add flight control changing mode in nav msgs
* Remove offset for pos x&y, meaning uav will start with the mocap coordinate
* Add all joints torque enable/disable flag
* Add force_landing_reset code in start phase
* Add Force Landing Mode
* Add config and launch files for tarot810
* Add ESC PWM calibrate rule
* Add motor info, particularily the pwm min/max for rook
* Fix the message type to receive cog_rotate
* Change the of motor info pub timing
* Fix the imu id problem
* Add pwm min/max in motor_info message
* Add new message and motor info yaml(pwm force torque)
* Fix for the d_board hydrus control
* Fix the d_board_imu id
* Fix for kduino in new aerial_robot_base platform
* change the topic name to kduino (remove prefix "kduino")
* preparation for aerial transformation using kduino
* modified mocap file: insert rosparam directly into launch file
* Move YawThrottleGain.msg from aerial_robot_base to aerial_robot_msgs
* Change the message type about optical_flow message from aeiral_robot_base::OpticalFlow to px_comm::OpticalFlow, and also removed the OpticalFlow.msg from aerial_robot_base
* Add TODO which is related to the relay field of topic
* Add necessary dependency
* Fix the launch file of  tarot680(mbzirc-task1 uav) to match the rult of new MCU system(rosserial)
* Modified the rook platform based on the new MCU system(rosserial), getting good result of  mocap-ground-truth-pos-control and optical_flow-egomation-estimation-vel-control.
* Fix the pub/sub for the new ros interface of MCU(STM32F7 rosserial)
* Merge remote-tracking branch 'origin/jade-devel' into jade-devel
  Conflicts:
  aerial_robot_base/src/flight_navigation.cpp
* this temperate
* Merge branch 'jade-devel' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into jade-devel
* add rook launch file, along with related sensor configuration file
* Merge branch 'indigo-dev2' into jade-devel
  Conflicts:
  aerial_robot_base/src/flight_navigation.cpp
* Merge branch 'indigo-dev2' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev2
  Conflicts:
  aerial_robot_base/src/flight_navigation.cpp
* Add teleop_flag which can stop/restart teleop
* add some new message here
* some launch param change for tarot680
* add mbzirc task1 tarot launch file
* final fix for the optical-flow based control for rook
* optical flow z control refined
* some fix about navigation
* fix the optical flow based control bug
* Merge branch 'jade-devel' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into jade-devel
* some change from rook
* add some new mode for sensor fusion, especially for optical flow sensor
* some change about optiacal flow control
* add px4flw sensor
* fix the bug of new pluginization, mocap ground truth flight success
* Merge branch 'indigo-dev2' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev2
* some modification about gaining tuning mode in navigation.cpp
* finish the kf for mocap and imu
* Merge branch 'jade-devel' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into jade-devel
* delete some unnecessary file
* complete compiling the sensor plugin
* finish writing pluginization abouth sensor and related change in flight_control/flight_navigation
* pluginization(cont'd)
* fix the kalman filter problem
* add the change from kf for rook
* some change in odroid xu4 1
* Merge branch 'indigo-dev2' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev2
  Conflicts:
  di/di_control/include/di_control/di_gimbal_control.h
  di/di_control/src/di_gimbal_control.cpp
  add add alt_tilt function and cfg for gimbal interval and duration
* temporary change from t430 about alt, cfg
* modification from di
* modification for di gimbal control
* add motor test package
* Merge branch 'indigo-dev2' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev2
* fix the yaw gain
* modified the tilt of pitch servo for each module
* hoge
* add kduino controlled di
* from di
* Merge branch 'indigo-dev2' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev2
* add limit for yaw intergral error term
* add di launch/
* add luanch files
* some fix for di directory
* Di: first commit, dynamixel init config, complete
* fix the d-board problem,
  also left the modfication for imu_module.h, should revert the change
* so
* ok
* ho
* so
* ok
* ok
* ok
* fix the type of roll/pitch limit
* ok
* ok
* hoge
* some
* some
* some change for rook to suitable for mocap
* some
* some
* yaw control
* change the gain for the dragon project
* some change for dragon project
* first commit for dragon project
* Merge branch 'indigo-dev' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev
  Conflicts:
  hydra/hydra_transform_control/config/Hydra3.yaml
* Merge branch 'indigo-dev' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev
* fix bug about kalman filter
* some change for aerial robot base in kalman filter
* some change for kalman filter
* Merge branch 'indigo-dev' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev
* fix the CMakelist.txt
* some fix from hydra
* hoge
* remove some unnecessary files
* Merge branch 'indigo-dev' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev
* this is from hydra-odroid
* modified the land mode for gneneral, rook type
* fix lot of porblem for the aeria tracking
* fix the bug in aerial tracking(abs -> fabs)
* hoge
* add the slow rate for land throttle mode in general mutltirotor control
* add some code for general multirotor
* fix the bug in optical flow file
* modified the kalman filter file
* modified the red ball tracking problem
* change something for sonar altitude control
* fix the flight_control for the general multirotor, especially for the rook(throttle, yaw:vel_local_base_mode)
* fix for the optical flow sensro altitude control
* add some change for rook
* add mutex for the kalman filter
* Fix the tracking problem for red ball tracking(bouding box)
* some fix about launch file for rook
* add some yaml file
* some change
* 1. kalman filter for px4flow
  2. control input for old system
* modfied rms
* fix the joint ctrl command problem
  tuning the xy pid pos control gain
  tuning the vel-mode yaw gain
* fix the dynamic reconfigure problem
* Merge branch 'indigo-dev' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev
* test
* fix the landing problem
* fix the feed-forward control problem
* Merge branch 'indigo-dev' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev
* add feedforward control
* some change from odroid
* Merge branch 'indigo-dev' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev
* fix the reset mode
* fix some plroblem
* some change for integaration flag(roll/pitch attitude control)
* fix flight control yaw term
* Merge branch 'indigo-dev' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev
* change the pwm_f conversion place
* fix
* Merge branch 'indigo-dev' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev
* fix the keycommand
* fix the publisher problem
* some change for transform control
* some changes
* some changes to commit before update to 14.04
* some changes to commit before update to 14.04
* hogehoge
* some modification
* fix the casting problem(int => float)
* add gain tunning mode form joy stick
* some modification for indigo, especially for odroid u3
* some modification for hydra transformation control
* some of the change for indigo version, especially for arm platform
* add rotate.cpp for mocap
* some of the changes to be update
* add interative marker control for dragon2
* correct most of the files to complete the hovering, aerial transform, vel control from joy
* modified and complete redd ball tracking in tracking package
* complete the build of aerial_robot_base_node
* correct all files in aerial robot base
* modified the config files in aerial robot base
* modified the hydra launch files
* change the name in cofig files rook2
* modified the naming about rosparam
* modified for connection to the aerial robot base
* add new files to package tracking
* add new package tracking
* modified mirror module files
* modified the optical flow module file
* modified the mocap files
* modified half of the sensors files
* modified kalman filter files
* modified digital filter(lpf)
* modified the state estimator files
* modified the control files
* modified the ctrol input array files(.h/.cpp)
* modified the from cfg to aerial_robot_base to flight_navigation
* some modification for catkin system
* add renamed pkgs
* Contributors: Moju Zhao, Tomoki Anzai, Fan Shi
