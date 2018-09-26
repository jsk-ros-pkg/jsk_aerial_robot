^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dragon
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2018-09-26)
------------------

1.0.0 (2018-09-26)
------------------
* set all of package.xml version to 0.0.0
* Merge pull request `#197 <https://github.com/tongtybj/aerial_robot/issues/197>`_ from chibi314/refactor_transform_control
  Refactor Transform Control etc.
* modify dragon transform control to transformable_aerial_robot_ros derived class
* remove unnecessary param actuattor_state_sub_name
* remove unnecessary param baselink_name
* Merge pull request `#189 <https://github.com/tongtybj/aerial_robot/issues/189>`_ from tongtybj/gimbal_pitch_roll_control
  Improve the giambal controller for pitch and roll
* Merge pull request `#188 <https://github.com/tongtybj/aerial_robot/issues/188>`_ from tongtybj/fix_dragon_model_inertia_matrix
  Fix dragon model inertia matrix
* Merge pull request `#194 <https://github.com/tongtybj/aerial_robot/issues/194>`_ from tongtybj/dragon_collision_model
  Update the collision model of dragon link
* Update the collision model of dragon link, i.e., the padding model for dragon gimbal module
* [Dragon] Fix the bug of publish location
* Merge pull request `#190 <https://github.com/tongtybj/aerial_robot/issues/190>`_ from tongtybj/remove_differential_motion
  Remove differential motion
* Remove the files associated with differential kinemtaics.
* Update the inertia tensor for each component, according to the solidedge_cog_inertia_tensor_converter.py
* Add Converter to obtain the true inertia tensor regarding to the link CoG.
  Because we can only get the inertia tensor around the reference frame in SolidEdge.
* - Change the thresh ptich_roll P matrix determinant as a parameter
  - Modify two parameters: pitch_roll_control_rate_thresh\_ and pitch_roll_control_p_det_thresh\_ for latest version dragon
  - Confirm the good performance in gazebo about the latest version dragon to do trasnformation demo (3 modes) and gap passing
* Update the parameteres for the differential kinemtaics to squeeze horizontal openning.
* Merge pull request `#187 <https://github.com/tongtybj/aerial_robot/issues/187>`_ from tongtybj/documentation
  Add documentation
* Update README.md for importance packages, as wel as the wiki.
* Merge pull request `#186 <https://github.com/tongtybj/aerial_robot/issues/186>`_ from tongtybj/dragon_demo
  Update dragon bringup
* Merge pull request `#156 <https://github.com/tongtybj/aerial_robot/issues/156>`_ from tongtybj/fix_dragon_model_inertia_matrix
  Fix the inertial matrix of Dragon model
* Merge remote-tracking branch 'origin/fix_dragon_model_inertia_matrix' into dragon_demo
* Fix the wrong offset of head link
* Update (Fix) the inertial matrix of each module (e.g. link, gimbal) to be suitable for the rull of URDF, that is based on the inertial frame (cog frame). Previous inertial matrix is with repecti to the link origin frame, which is the rule of Solid Edge.
* Update bringup file for DRAGON to be suitable for branch:devel
* Add demo mode:
  mode0: sea horse
  mode1: twist like
  mode2: M like
  reset: normal shape
  reverse_reset: reverse normal shape
* Update the takeoff height
* Merge pull request `#173 <https://github.com/tongtybj/aerial_robot/issues/173>`_ from chibi314/improve_dragon_model
  improve dragon mesh file
* fix dragon mesh files
  fix bug of head link
  fix color
  fix file size
* delete dragon stl mesh files
* improve dragon mesh file
* Merge pull request `#168 <https://github.com/tongtybj/aerial_robot/issues/168>`_ from tongtybj/vio
  visual odometry / visual inertial odometry based ego-motion estimation
* temp
* Merge pull request `#159 <https://github.com/tongtybj/aerial_robot/issues/159>`_ from tongtybj/sensor_launch_file
  Improvement the sensor launch and yaml files for hydrus
* Clean up the verbose for ros param
* Merge pull request `#147 <https://github.com/tongtybj/aerial_robot/issues/147>`_ from tongtybj/ik
  Release motion planning method based on differential kinematics
* Merge remote-tracking branch 'origin/devel' into ik
* Add the instruction to  Dragon usage in README.md
* Add rotor overlap avoidnace (constraint) for DRAGON model
* Add link attitude (pitch, roll) limitation constraint for DRAGON model
* Gap Passing Path Planning based on Differential Kinematics for both Hydrus and Dragon
* Update the dragon collision geometry to be more friendly for FCL
* Implement collision avoidance constraint based on FCL
* Crate the differentia kinematics based path planning framework,
  including the QP planner core, and plugin structure for cost and constraint
* Implement flight stability constraint (regarding to margin and singularity) for IK QP solver
* Implement joint angle limitation avoidance for IK QP solver
* Implement end-effector IK program based on Jacobian Inverse (sr-inverse)
* fix dragon's readme
* Update CMakeLists.txt:
  1. Eigen dependency description
  2. Cmake Policy 0046
* Merge pull request `#143 <https://github.com/tongtybj/aerial_robot/issues/143>`_ from tongtybj/installation
  Improve the installation sequence
* Update README.md for main packages
* Merge remote-tracking branch 'origin/devel' into fix_gps
* Merge remote-tracking branch 'origin/devel' into fix_bug_rospy_err
* Merge pull request `#137 <https://github.com/tongtybj/aerial_robot/issues/137>`_ from tongtybj/move_spinal_msg
  Move spinal msg
* move all ros messages from aerial_robot_base to aerial_robot_msgs
* move parts of ros messages from aerial_robot_msgs and aerial_robot_base to spinal
* move hydrus/msg to spinal/msg
* Merge pull request `#136 <https://github.com/tongtybj/aerial_robot/issues/136>`_ from tongtybj/dragon_transformation_demo
  Add aerial transformation demonstration script
* Add aerial transformation demostration script
* Fix the wrong place of color assignment for dragon model
* Update the drogon model (the position change of intel euclid)
* Update the thresh of edf max tilt to zero for the base config file, which directly influences the real machine
* Fix the bug for dragon while do landing process, the body should be level
* Add suffix for the rviz and robot state publisher node, to enable multiple rviz display
* Merge pull request `#126 <https://github.com/tongtybj/aerial_robot/issues/126>`_ from tongtybj/pos_vel_navigation
  Feed forward velocity control for translational and yaw motion.
* Add pos and vel simulaneous control mode for translational and yaw control, which performing bettwer maneuvering for trajectrory following.
* Merge pull request `#125 <https://github.com/tongtybj/aerial_robot/issues/125>`_ from tongtybj/new_dragon_model
  New dragon model
* Merge pull request `#123 <https://github.com/tongtybj/aerial_robot/issues/123>`_ from tongtybj/modified_dragon_flight_control
  Modified dragon flight control
* Merge remote-tracking branch 'origin/modified_dragon_flight_control' into new_dragon_model
* Update the dragon model battery information
* fix wrong file path for battery config
* Change the defualt estiamte mode to 2 for dragon, whcih only use ground truth value (motion captrue in the real situation)
* Fix the wrong order for gimbal control
* Add battery info battery for dragon
* Update the dynamixel bridge for dragon (gimbal) model.
* Update the baselink module mass parameter
* Add the reconfigure server for yaw axis gimbal control
* Update the dragon model
* Change the allocation from gimbal force to gimbal angles based on the hovering thrust force as z axis nominal component.
* Add the gimbal force compensation resulted from the vertical (z axis) force which is approximated as gravity.
* Fix the mapping matrix from pid values to target gimbal force, which contains mass information
* Update the API to get "std::vector<xxx>" type of variables.
  e.g. getRotorsOriginFromCoG()
* Merge pull request `#105 <https://github.com/tongtybj/aerial_robot/issues/105>`_ from tongtybj/voltage_based_pwm
  Conversion from thrust to pwm based on the voltage and nonlinear relationship.
* Fix the wrong m_f_rate(thrust-moment rate) for dragon model, should be 0.
* Add the battery capacity check function in the flight navigation, which is implemented in spinaly in the past.
* Update the flight system to be suitable for the thrust->pwm conversion in the spinal board(d_board).
* Merge branch 'devel' into aerial_transportation
* Merge pull request `#72 <https://github.com/tongtybj/aerial_robot/issues/72>`_ from tongtybj/control
  Gyro moment compensation
* Update the param of hydrusx(var_thre) and dragon (var thre, and edf max tilt)
* Merge pull request `#102 <https://github.com/tongtybj/aerial_robot/issues/102>`_ from tongtybj/new_communication_protocol
  New communication protocol
* Add the assigment of the uav model (e.g. hydrus/ dragon)
* Refine the control system.
  1. send motor info and uav info before the motor arming phase from the base class.
  2. change the motor number management rule.
* 1. Update the dynamixel bridge to be suitable for new communication system.
  2. Change the gimbal joint state to targetVal for dragon model.
* Add the threshold for the variance of the thrust horizon cofiguration and the vertial overlap check
* 1. change the position of func "gimbalProcess" from "jointStateCallback()" to "kinematics()"
  2. add the calculation of the true thrust positions in the dual-rotor gimbal module for the overlap check function
  3. add overlap check function based on the true thrust positions in the dual-rotor gimbal
  4. add the API to get gimbal nominal angles for the visulization in the other process such as Moveit!
* Add the true thrust points of the dual-rotor gimbal module
* Update the var_thre for hydrus and dragon
* Correct the collsion model for dragon
* Correst the wrong library name
* Add test/monitoring script for dragon
* Merge pull request `#91 <https://github.com/tongtybj/aerial_robot/issues/91>`_ from tongtybj/dragon
  Commit from Dragon control system
* 2017.9.1 best paramteres tuning for all motion
* 2017.8.29 best paramteres tuning for s pose roll/pitch 0.4 tilt
* Update the gimbal based control gains
  1. The p gain for position control
  2. THe p,i,d gains for pitch/roll control
* 1. Fix the wrong d calculation for gimbal based pitch/roll d control
  2. Fix the wrong du calucaltion for gimbal based pitch_roll i control
* Add the initial stable configuration for gimbal based pitch/roll
* Add gimbal based pitch/roll control
* Modified the pid gain for dragon position control
* Modified the LQI gains for roll/pitch, from gazebo
* Modified the joint speed
* Add the baselink based state_vel for gazebo, when control dragon position, which is not strictly correct
* Increase the upper limit of the motor pwm
* Modified the i gain for alt: 10 -> 20
* Add the vel cutoff frequency from mocap without imu, in the case of dragon
* Modified the joystick control rate for altitude
* Modified the gimbal control for xy position, but is not fixed
* Modified the gain for pitch/roll LQI control
* 1. Fix the wrong order to call function "landing_process"
  2. Add the inactivate process of gimabal control for the early stage of takeoff
  3. Dsiable the servo off control in the landing/force landing  process
* Update dragon link model based on intel euclid version
* Add teleop_flag on/off switch process for the leveling landing
* Add the leveling process before landing and force_landing
* Increase the gain of alt control
* Change the damping rate of joint servo to be higher
* Add the desire tilting smoothing process, by providing another subscribe to do linear interpolation
* Add the fc(attached with imu and fcu) frame to provide better kinematics from cog to baselink,
  also, update the configuration to set the baselink to fc
* Modified the configuration:
  1. the servo configuration for real machine, quad type
  2. gimbal control gains, not perfect
  3. takeoff height: 0.4m
* Add the target force publisher
* Create the servo bridge node for dragon system, which is inherited from the hydrus one
* Modified the flight configuration (e.g. control gains) to be able to do aerial transformation in gazebo simulation.
* Add the dummy joint state publisher for the early development stage
* Add the dragon based control
  1. transform_control: the kinematics process for gimbal contained structure, also calculate the gimbal angle for level purpose
  2. gimbal_control: the flatness pid controller based, horizon and yaw control to generate the gimbal angles
* change the dist_thre for dragon quad type
* temoporary configuration for dragon to do roll/pitch/alt control
* Update the mass parameter of dragon
* Add the dragon flight control node which is inherited from hydrus/transform_control,
  and implement the joint servo enable/disable command according to the flight process(motor on, landing, force landing)
* Update the package, especially for the robot model
* Contributors: Moju Zhao, Tomoki Anzai
