^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aerial_robot_simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2018-09-26)
------------------
* Merge pull request `#197 <https://github.com/tongtybj/aerial_robot/issues/197>`_ from chibi314/refactor_transform_control
  Refactor Transform Control etc.
* remove unnecessary param baselink_name
* Merge pull request `#139 <https://github.com/tongtybj/aerial_robot/issues/139>`_ from chibi314/fix_bug_rospy_err
  fix typo in gazebo_control_bridge.py
* Merge remote-tracking branch 'origin/devel' into fix_gps
* Merge remote-tracking branch 'origin/devel' into fix_bug_rospy_err
* Merge pull request `#137 <https://github.com/tongtybj/aerial_robot/issues/137>`_ from tongtybj/move_spinal_msg
  Move spinal msg
* fix typo in gazebo_control_bridge.py
* move parts of ros messages from aerial_robot_msgs and aerial_robot_base to spinal
* Merge pull request `#109 <https://github.com/tongtybj/aerial_robot/issues/109>`_ from tongtybj/aerial_robot_nerve
  Integrating real machine layer: mcu development
* Update simulation system:
  1. remove the submodeule: d_board. Instead, the wrapping process for flight controller in spinal is done by the catkinized package "spinal".
  2. one sample of moving the spinal associated ros message: PMatrixPseudoInverseWithInertia.msg from hydrus to spinal
* Merge pull request `#90 <https://github.com/tongtybj/aerial_robot/issues/90>`_ from chibi314/aerial_transportation
  Aerial transportation
* Update submodule "d_board"
* Merge pull request `#107 <https://github.com/tongtybj/aerial_robot/issues/107>`_ from tongtybj/new_fight_config_cmd
  New flight config cmd message between ros and d_board
* Update the flight config cmd, which contains the cmd ID in the ros message to unify the protocol between ros and d_board
* Merge pull request `#105 <https://github.com/tongtybj/aerial_robot/issues/105>`_ from tongtybj/voltage_based_pwm
  Conversion from thrust to pwm based on the voltage and nonlinear relationship.
* Update the version of submodule(d_board)
* Add the option to enable c+11 explictly
* Update the simulation system, which not convert the force to pwm but use the force as the didrect input.
* Update the flight system to be suitable for the thrust->pwm conversion in the spinal board(d_board).
* Merge branch 'devel' into aerial_transportation
* Merge pull request `#72 <https://github.com/tongtybj/aerial_robot/issues/72>`_ from tongtybj/control
  Gyro moment compensation
* update the d_board code
* Add the calculation of the compensation of the cross term of the rotational dynamics
* Merge pull request `#102 <https://github.com/tongtybj/aerial_robot/issues/102>`_ from tongtybj/new_communication_protocol
  New communication protocol
* Update the version of d_board
* Update the control wrapper part in simulation system
* Update the submodule: d_board
* Merge pull request `#91 <https://github.com/tongtybj/aerial_robot/issues/91>`_ from tongtybj/dragon
  Commit from Dragon control system
* remove gazebo model
* enable to change gazebo world
* add randomPose param in gazebo_treasure_plugin
* remove aerial_transportation.world
* Merge branch 'devel' into aerial_transportation
* add disk object
* fix gazebo_treasure_plugin
* add dropping zone box
* add new world file for aerial transportation simulation
* Update the package, especially for the robot model
* Add the xacro model for dragon in gazebo
* Fix the wrong calculation between joint_num and motor_num in case of dragon model
* add gimabal servo part in the gazebo control bridge
* Modified the simulation system to be suitable for the new cog-baselink kinematics
* Add the dependency for the gazebo simulation
* Merge pull request `#81 <https://github.com/tongtybj/aerial_robot/issues/81>`_ from tongtybj/new_control_system
  Flight control plugin
* Make the flight control to be the plguin:
  1. change the existing flight control called differential flatness pid control to control x/y/z/yaw
  2. make the state machine of flight naviagtion more clear
  3. sperate the flight_navigation and flight_control module
* Remove the so-called feedforward control in LQI, integrating into the error based feedback control like general PID
* Merge pull request `#78 <https://github.com/tongtybj/aerial_robot/issues/78>`_ from tongtybj/special_robots
  Special robots
* Fix the wrong package name in package.xml
* Move the hydrus robot model meterials from  aerial_robot_model to hydrus
* Change the name from "hydrus_transform_control" to "hydrus"
* update the version of d_board
* Merge pull request `#73 <https://github.com/tongtybj/aerial_robot/issues/73>`_ from tongtybj/cog_based_control
  Cog based estimaton, control and navigation
* Implmente the COG/BASELINK state estimation: 1. sensor fusion is porcess in the baselinkframe. 2 cog state is calculated with the baselink state, simple rigid body kinematics. 3: baselink and cog have same orientation
* Merge remote-tracking branch 'origin/kdl' into devel
* Shift the rotor direction configuration into the URDF description
* change the name for the link with FCU and IMU from root_link to baselink
* Merge pull request `#52 <https://github.com/tongtybj/aerial_robot/issues/52>`_ from tongtybj/rosserial
  Flexible topic sized for rosserial.
* Update the version of d_board
* add process to set the correct uav type
* Merge pull request `#23 <https://github.com/tongtybj/aerial_robot/issues/23>`_ from tongtybj/simulation
  Simulation
* Setup for the hydrusx series(quad, hex) in the simulation system
* add new arg in simulation.launch: type
* update the version of mcu code(d_board)
* Add hydrus3 gazebo xacro
* Change the api to add force on the link
* add basic launch file for gazebo
* add robot model(hydurs3) and wrold for gazebo
* Two main elements:
  1. aerial_robot_hw_sim as the main interface for gazebo.
  2. simulation_attitude_controller as the wrapper of the attitude control in d_board
* Change the branch of d_board to simulation
* remove the rosserial in aerial_robot.rosinstall, since we can obtain it as submodules in aerial_robot_simulation
* Change the branch of d_board to simulation
* Add d_board micro-board codes as submodules
* Contributors: Moju Zhao, Tomoki Anzai
