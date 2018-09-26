^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hydrus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2018-09-26)
------------------
* set all of package.xml version to 0.0.0
* Merge pull request `#197 <https://github.com/tongtybj/aerial_robot/issues/197>`_ from chibi314/refactor_transform_control
  Refactor Transform Control etc.
* modify dragon transform control to transformable_aerial_robot_ros derived class
* remove unnecessary param actuattor_state_sub_name
* add hydrus_robot_model
  modify transform_control to transform_control_ros-derived class
* remove unnecessary param baselink_name
* add transformable aerial robot model base class
  move AddExtraModule to aerial_robot_model from hydrus
* Add joint_state of hydrus robot  for rosbag
* remove unecessary dir "fcl"
* Merge pull request `#191 <https://github.com/tongtybj/aerial_robot/issues/191>`_ from tongtybj/baselink_name
  Add API to get the name of baselink_name
* Add API to get the name of baselink_name
* Merge pull request `#190 <https://github.com/tongtybj/aerial_robot/issues/190>`_ from tongtybj/remove_differential_motion
  Remove differential motion
* Remove the files associated with differential kinemtaics.
* update the github submodule FCL version
* Merge pull request `#187 <https://github.com/tongtybj/aerial_robot/issues/187>`_ from tongtybj/documentation
  Add documentation
* Update README.md for importance packages, as wel as the wiki.
* Merge pull request `#183 <https://github.com/tongtybj/aerial_robot/issues/183>`_ from KahnShi/devel
  Update Hydrus's readme for launch files.
* Update Hydrus's readme for launch files.
* Merge pull request `#180 <https://github.com/tongtybj/aerial_robot/issues/180>`_ from tongtybj/sensor_tf
  Improvement of sensor plugins for sensor fusion
* 1. Improve the frame transformation between sensor and baeslink for each sensor module (plugin)
  2. Add NED frame in GPS sensor module (plugin)
* Merge pull request `#178 <https://github.com/tongtybj/aerial_robot/issues/178>`_ from tongtybj/onboard_mocap
  [Hydrus] Add the mocap launch file in the onbard sensor list
* Add the mocap launch file in the onbard sensor list
* Merge pull request `#177 <https://github.com/tongtybj/aerial_robot/issues/177>`_ from tongtybj/new_kalman_filter_interface
  Improvement of sensor fusion with improved kalman filter method
* Update parameter for hydrus quad with intel euclid: the time latency of vio of realsense_sp regarding to IMU
* Merge pull request `#176 <https://github.com/tongtybj/aerial_robot/issues/176>`_ from tongtybj/vio
  Impove the accurate of sensor fusion (VIO + IMU), by considering the latency of sensor timestamp
* Modification of vio-imu sensor fusion with following points:
  1. change the noise of acc (horizontal) from 0.02 back to 0.05.
  2. use pos_vel_acc_bias for horiztonal translation motion (x, y), and use pos_ve_acc for altitude motion
* Add time delay of vio odometry timestamp, which improves the sensor fusion a lot
* Merge pull request `#170 <https://github.com/tongtybj/aerial_robot/issues/170>`_ from tongtybj/euclid_sensor_plugin
  Add camera sensor plugins for hydrus in gazebo
* Add sensor plugins (color camera and depth image) for Euclid in gazebo world
* Merge pull request `#168 <https://github.com/tongtybj/aerial_robot/issues/168>`_ from tongtybj/vio
  visual odometry / visual inertial odometry based ego-motion estimation
* Modified the angle estimation error (bias) noise from 0.0001 to 0.0002 (rad)
* [Important] modified the paramters for the vio (euclid) + imu based egomotion-estimation by real-machine
  1. change the filter from "kalman_filter/kf_pos_vel_acc" back to "kalman_filter/kf_pos_vel_acc_bias", for the horizontal motion
  2. change the noise sigma of vo (eculid, realsense sp) from 0.01 to 0.001, this may be risky for bad enviroment (e.g. outdoor)
  3. change the p gain of horizontal motion from 0.12 to 0.15
* update the pid(p) gain for xy position control for  the vio based egomotion-estimation. which is also suitable for mocap system
* Modified the rqt plot script
* Modified the paramters for the vio (euclid) + imu based egomotion-estimation by using the rosbag.
  1. change the filter from "kalman_filter/kf_pos_vel_acc_bias" to "kalman_filter/kf_pos_vel_acc", for the horizontal motion
  2. change the noise sigma of level acc from 0.5 to 0.2
* Add debug verbose for vo/vio estimation filter
* Add topic to rosbag_raw_sensors.sh and also add rqt_plot command script
* temp
* Merge pull request `#166 <https://github.com/tongtybj/aerial_robot/issues/166>`_ from tongtybj/kinematics_bug
  Fix the bug of the recursive procesing about the inertial for the kinematics.
* Fix the bug of the recursive procesing about the inertial for the tree structure of aerial multilinked robot.
* Merge pull request `#163 <https://github.com/tongtybj/aerial_robot/issues/163>`_ from tongtybj/auto_git_submodule
  Add code in CMakeLists.txt to automatically download FCL
* Add code in CMakeLists.txt to automatically download FCL
* Fix the wrong "arg" assignment: simulation -> real_machine for sensors.launch.includes
* Merge pull request `#159 <https://github.com/tongtybj/aerial_robot/issues/159>`_ from tongtybj/sensor_launch_file
  Improvement the sensor launch and yaml files for hydrus
* Move the location of rosbag record command files to correct plase
* Update the rule of yaml files for hydrus hex type
* Improve the format of YAML files
* Clean up xacro files
* Improve the rule of brinup.launch
* Improve the rule of launch files realted to sensors
* Improve the rule to set onboard sensor and ego-motion estimation (sensor fusion) based on YAML
* Remove the unecessary FCL test code
* Clean up the verbose for ros param
* Merge pull request `#158 <https://github.com/tongtybj/aerial_robot/issues/158>`_ from tongtybj/new_hydrusx
  Improvement of  hydrusx robot model description
* 1. Change hydrusx files structure (i.e. robots/, urdf/)
  2. Apply hierarchical xacro based robot model, for example "default.urdf.xacro" only contains the bone structure without any sensors and processors, then xxxx.urdf.xacro will inherit this default model with specific sensors or processors
* 1. Change hydrusx files structure (i.e. launch/, config/)
  2. Define paramter "model" for hydrusx to distinguish the specific robot model for hydrus (e.g. intel_euclid)
* Remove files related to hydrus3, which is depracated model
* Merge pull request `#147 <https://github.com/tongtybj/aerial_robot/issues/147>`_ from tongtybj/ik
  Release motion planning method based on differential kinematics
* Merge remote-tracking branch 'origin/devel' into ik
* Add instruction for hydrusx in Readme.md
* Add FCL distance / closest points calcualtion debug code, which is not so necessary
* Update the submoudle version (FCL)
* Add rotor overlap avoidnace (constraint) for DRAGON model
* Add link attitude (pitch, roll) limitation constraint for DRAGON model
* Gap Passing Path Planning based on Differential Kinematics for both Hydrus and Dragon
* Change the square (pow2) description from ^ to *
* 1. Add libccd in package.xml for FCL
  2. Add dependency for plugin in CMakeLists.txt
* Change FCL as sumodule, and update CMakeLists.txt
* Implement collision avoidance constraint based on FCL
* Crate the differentia kinematics based path planning framework,
  including the QP planner core, and plugin structure for cost and constraint
* Implement flight stability constraint (regarding to margin and singularity) for IK QP solver
* Implement flight stability constraint for IK QP solver
* Implement joint angle limitation avoidance for IK QP solver
* Implement end-effector IK program based on Jacobian Inverse (sr-inverse)
* First implementation about IK
* [Hydrusx] update the distance threshold for quad type
* Merge pull request `#146 <https://github.com/tongtybj/aerial_robot/issues/146>`_ from tongtybj/link_length
  Fix the calculation for link basic length
* Fix the calculation for link basic length
* Update CMakeLists.txt:
  1. Eigen dependency description
  2. Cmake Policy 0046
* Merge pull request `#143 <https://github.com/tongtybj/aerial_robot/issues/143>`_ from tongtybj/installation
  Improve the installation sequence
* Update README.md for main packages
* [Overwrite]: update the hydrusx quad servo offset
* Merge remote-tracking branch 'origin/devel' into fix_gps
* Merge remote-tracking branch 'origin/devel' into fix_bug_rospy_err
* Merge pull request `#137 <https://github.com/tongtybj/aerial_robot/issues/137>`_ from tongtybj/move_spinal_msg
  Move spinal msg
* move parts of ros messages from aerial_robot_msgs and aerial_robot_base to spinal
* move hydrus/msg to spinal/msg
* Merge pull request `#131 <https://github.com/tongtybj/aerial_robot/issues/131>`_ from chibi314/spinal_ros_service
  Spinal ros service
* change board_config_cmd topic to service
* change board_info topic to service
* Add suffix for the rviz and robot state publisher node, to enable multiple rviz display
* Add two new API
  1. the transform from root link to the assigned link with the desired joint states, serving as the simple FK
  2. enable to change the baselink dynamically.
* Add explict servo name for hydrusx model
* Merge pull request `#125 <https://github.com/tongtybj/aerial_robot/issues/125>`_ from tongtybj/new_dragon_model
  New dragon model
* Modified the dynamixel bridge for hydrus (non-gimbal) model.
  1. change the joint parameter setting way:  joint1/xxxxx -> joint/1/xxxxx)
  2. change the method to set the servo state from MCU, suing the servo global index in MCU.
* Merge pull request `#109 <https://github.com/tongtybj/aerial_robot/issues/109>`_ from tongtybj/aerial_robot_nerve
  Integrating real machine layer: mcu development
* Merge branch 'devel' of github.com:tongtybj/aerial_robot into devel
* Update the firction coefficient to 0.1 for hydrusx model
* Update simulation system:
  1. remove the submodeule: d_board. Instead, the wrapping process for flight controller in spinal is done by the catkinized package "spinal".
  2. one sample of moving the spinal associated ros message: PMatrixPseudoInverseWithInertia.msg from hydrus to spinal
* Update the model of hydrusx (for quad type), which does not have rollors
* Modified hydrus parameter
  1. the nav vel limitation: 0.3 m/s
  2. the yaw control term limitation
  3. the friction rate for gazebo simulation
* Update parameters about control for hydrusx quad
* Update the API to get "std::vector<xxx>" type of variables.
  e.g. getRotorsOriginFromCoG()
* Merge pull request `#90 <https://github.com/tongtybj/aerial_robot/issues/90>`_ from chibi314/aerial_transportation
  Aerial transportation
* Add get Function for thrust upper/lower bound
* Merge pull request `#106 <https://github.com/tongtybj/aerial_robot/issues/106>`_ from tongtybj/extra_module
  Extra module for modeling
* Fix the wrong param in the launch file
* Refined method to add/remove the extra module for the kinematics
* Merge pull request `#105 <https://github.com/tongtybj/aerial_robot/issues/105>`_ from tongtybj/voltage_based_pwm
  Conversion from thrust to pwm based on the voltage and nonlinear relationship.
* Update the model of hydrusx quad type for IJRR2017.
* Add the battery capacity check function in the flight navigation, which is implemented in spinaly in the past.
* Update the flight system to be suitable for the thrust->pwm conversion in the spinal board(d_board).
* Merge branch 'devel' into aerial_transportation
* Revert "move kinematics() func to public"
  This reverts commit 5c921497660be66f3a74d16cb12df3c037a0c6db.
* Merge pull request `#72 <https://github.com/tongtybj/aerial_robot/issues/72>`_ from tongtybj/control
  Gyro moment compensation
* Update the param of hydrusx(var_thre) and dragon (var thre, and edf max tilt)
* Add the calculation of the compensation of the cross term of the rotational dynamics
* Add the ros message to compensate the cross term in the rotational dynamics (Psuedo Inverse P Matrix & Inertia Tensor)
* Merge pull request `#102 <https://github.com/tongtybj/aerial_robot/issues/102>`_ from tongtybj/new_communication_protocol
  New communication protocol
* Change the paramter for hydrusx quad:
  1. the pos xy contrl p gian: 0.1 -> 0.2
  2. the var thre: 0.2 -> 0.16
* Add the assigment of the uav model (e.g. hydrus/ dragon)
* 1. Update the dynamixel bridge to be suitable for new communication system.
  2. Change the gimbal joint state to targetVal for dragon model.
* modify hydrus msgs for new communication protocol
* Modified the variance threshold of horizontal arrangement
* Change the name of thrust point from "propeller"
* 1. fix the variance of the horinzontal arrangement of thrust to be normalized
  2. change function "kinematics" to be virtual
  3. add virtual function "overlapCheck" to check the vertical overlap, no any meaning for hydrus type
  4. change the point of thrust force from "propeller" to "thrust"
* Update the var_thre for hydrus and dragon
* Update the new rult for the modelling(statics) check, to be suitable for the planning for dragon
* Update the collsion model for hydrusx
* modified the threshold name for the dist using var, in the case of hydrusx quad
* Add the new rotor dist check algorithm based on principal component analysis.
* Merge pull request `#91 <https://github.com/tongtybj/aerial_robot/issues/91>`_ from tongtybj/dragon
  Commit from Dragon control system
* add alt/err_thresh in hydrusx/hex/DifferentialFlatnessPidControlConfig.yaml
* enable to change gazebo world
* add nav_vel_limit in hydrus/config/hydrusx/TeleopNavigationConfig.yaml
* Fix the bug of cfg to tune LQI gains
* Merge branch 'devel' into aerial_transportation
* Add the initial control enable flag to d_board, since we have to activate the servo system in the case of dragon based CAN system
* Add the dragon flight control node which is inherited from hydrus/transform_control,
  and implement the joint servo enable/disable command according to the flight process(motor on, landing, force landing)
* Update the hydrus model configuration to be suitable for new cog-baselink kinematics
* Update the code about the kinemaitcs between CoG and baselink
* Merge pull request `#89 <https://github.com/tongtybj/aerial_robot/issues/89>`_ from tongtybj/new_dynamixel_bridge
  Update the dynamixel ros brdige
* Add the logout if the joint states have wrong information
* Update the dnyamixel ros brdige
* update the hydrusx model based on intel euclid
* Merge pull request `#87 <https://github.com/tongtybj/aerial_robot/issues/87>`_ from tongtybj/multilin-control
  Multilink based flight control
* Add the member class "stable state" input , which is necessary for the later gimbal control,
  and add the verbose flag
* Fix the wrong matrix to calculate the stable state for three axis mode
* Remove unnecessary file
* Merge pull request `#82 <https://github.com/tongtybj/aerial_robot/issues/82>`_ from tongtybj/cog_odometry
  Better CoG odometry
* Publish correct odom, especially the orientation of COG
* 1. add the desire coordinate callbackk, indicating that the different orientations between cog and baselink frame are allowed
  2. publish correct transform from cog to baselink
* Merge pull request `#81 <https://github.com/tongtybj/aerial_robot/issues/81>`_ from tongtybj/new_control_system
  Flight control plugin
* Remove unnecessary config file
* Make the flight control to be the plguin:
  1. change the existing flight control called differential flatness pid control to control x/y/z/yaw
  2. make the state machine of flight naviagtion more clear
  3. sperate the flight_navigation and flight_control module
* Merge pull request `#78 <https://github.com/tongtybj/aerial_robot/issues/78>`_ from tongtybj/special_robots
  Special robots
* Move the hydrus robot model meterials from  aerial_robot_model to hydrus
* Create a package to integate all special robots
* Contributors: Moju Zhao, Tomoki Anzai, Fan Shi
