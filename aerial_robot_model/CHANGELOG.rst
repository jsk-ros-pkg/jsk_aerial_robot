^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aerial_robot_model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2018-09-26)
------------------
* set all of package.xml version to 0.0.0
* Merge pull request `#197 <https://github.com/tongtybj/aerial_robot/issues/197>`_ from chibi314/refactor_transform_control
  Refactor Transform Control etc.
* add transformable aerial robot model base class
  move AddExtraModule to aerial_robot_model from hydrus
* Merge pull request `#147 <https://github.com/tongtybj/aerial_robot/issues/147>`_ from tongtybj/ik
  Release motion planning method based on differential kinematics
* Implement collision avoidance constraint based on FCL
* Add suffix for the rviz and robot state publisher node, to enable multiple rviz display
* update rviz config
* Merge pull request `#105 <https://github.com/tongtybj/aerial_robot/issues/105>`_ from tongtybj/voltage_based_pwm
  Conversion from thrust to pwm based on the voltage and nonlinear relationship.
* Update the simulation system, which not convert the force to pwm but use the force as the didrect input.
* Merge branch 'devel' into aerial_transportation
* Update the package, especially for the robot model
* Add color for stl model in rviz
* Update model:
  1. Add stl model for dragon, which is lighter than dae file
  2. Fix the wrong origin position of ducted_fan_gimbal_pitch
  3. Update the collision model for each module
  4. Update the max force for dragon model
  5. Update the friction paramter for the each type of joint
* add new dragon ros model, which contains correct inertial paramters calculated from CAD
* Merge pull request `#87 <https://github.com/tongtybj/aerial_robot/issues/87>`_ from tongtybj/multilin-control
  Multilink based flight control
* Add the member class "stable state" input , which is necessary for the later gimbal control,
  and add the verbose flag
* Merge pull request `#78 <https://github.com/tongtybj/aerial_robot/issues/78>`_ from tongtybj/special_robots
  Special robots
* Move the hydrus robot model meterials from  aerial_robot_model to hydrus
* Merge pull request `#73 <https://github.com/tongtybj/aerial_robot/issues/73>`_ from tongtybj/cog_based_control
  Cog based estimaton, control and navigation
* For better debug
* Merge remote-tracking branch 'origin/kdl' into devel
* Modified the robot xacro file and yaml to be suitable for the new rotor direction configuration
* Shift the rotor direction configuration into the URDF description
* fix the wrong condition for the robot_state_publisher
* Modified the config(yaml) of hydrusx and hydrus3 to be suitable for KDL based kinematics
* modified the urdf model and related tf publisher code to be suitable for KDL based kinematics
* change the name for the link with FCU and IMU from root_link to baselink
* Merge pull request `#52 <https://github.com/tongtybj/aerial_robot/issues/52>`_ from tongtybj/rosserial
  Flexible topic sized for rosserial.
* add temporary implementation for the rotor interface to get root link number
* Merge pull request `#42 <https://github.com/tongtybj/aerial_robot/issues/42>`_ from tongtybj/outdoor
  Outdoor Flight System
* Update the hydrusx model dscription
* Update the hydrus3 model dscription
* Merge pull request `#23 <https://github.com/tongtybj/aerial_robot/issues/23>`_ from tongtybj/simulation
  Simulation
* Setup for the hydrusx series(quad, hex) in the simulation system
* Fix the wrong origion setting for the collision model
* Update the model for hydrusx model
* Update the description of hydrus3 common/link urdf, new elements: base_inertia
* Create hydrus3 model which have quad type, penta type, and associated init angle states yaml
* Add new arg: type, which indicates the number of links or joints
* improve the model description of hydrus3
* add the init joint angle when only show the kinematics of the aerial robot
* Improve the discription of transformable aerial robot's urdf:
  1. Change the kinematics rule: the link origin of link n is at the joint n-1, so we also create a virtual link_center link;  the x axis of link n is towrads joint n, which is opposite to the previous one.
  2. Specify the dynamics of the link: inertia parameters.
* add rotor_interface as the hardware interface which is used in gazebo system
* remove the simulation part from aerial_robot_model.launch
* remove the gazebo related part from aerial_robot_model
* Merge pull request `#19 <https://github.com/tongtybj/aerial_robot/issues/19>`_ from tongtybj/hydrusx_octo
  [Hydrusx octo] Add model configuration for octo type
* Add model and transformation config file for hydrusx octo type
* Merge pull request `#17 <https://github.com/tongtybj/aerial_robot/issues/17>`_ from tongtybj/gazebo_model
  [Gazebo model] Fix the wrong robot namespace
* Fix the wrong robot namespace for hydrus3
* Move the robot state publisher place
* Remove the depreacated target
* Merge pull request `#11 <https://github.com/tongtybj/aerial_robot/issues/11>`_ from tongtybj/gazebo_model
  Remove the depreacated target
* Remove the depreacated target
* Merge pull request `#9 <https://github.com/tongtybj/aerial_robot/issues/9>`_ from tongtybj/gazebo_model
  Remove unnecessary model files
* Remove unnecessary model files
* Merge pull request `#8 <https://github.com/tongtybj/aerial_robot/issues/8>`_ from tongtybj/gazebo_model
  Update README.md
* Update README.md
* Merge pull request `#7 <https://github.com/tongtybj/aerial_robot/issues/7>`_ from tongtybj/gazebo_model
  Gazebo model
* 1. Update the hydrus3 model which has original color: SolidEdge Collada Export
  2. Refine the urdf xacro description for hydrus3
  3. Add Gazebo elements to urdf xacro file
  4. merge gazebo display launch file to aerial_robot_model.lunach
* Remove depreacated launch files related to aerial robot model. i.e. crest_env, traj_motion
* Merge pull request `#4 <https://github.com/tongtybj/aerial_robot/issues/4>`_ from tongtybj/sensor_fusion
  Sensor fusion
* Add hydrux quad model
* Add simulation flag for sensor and robot model launch file
* Merge pull request `#3 <https://github.com/tongtybj/aerial_robot/issues/3>`_ from tongtybj/flight_command
  1. Update the hydrus x robot model
  2. Change the LQI control framework.
  3. Change the launch file structure first mainly for hydrusx.
  4. Add Uav communication with ground station using Xbee
  5. Add Failsafe system for UAV.
* Modified the color diffuse parameter which enable coloring
* Update the model of hydrus series.
  Hydrusx_penta, Hydrus_hex
* Add summerized launch file for aerial robot model. Also add readmed.md
* Cleanup aerial_robot_model package. Remove unnecessary launch file.
  We also need to cleanup again to optimize the sturcture of this files.
  TODO: interactive marker operation
* Change the display_xacro_xxx_file for latest version
* Merge remote-tracking branch 'origin/jade-devel' into jade-devel
* Change the model of hydrus3 from stl style to dea style, which is more compatible for gazebo and eus.
* change the stl model of hydrus3, but still have bug
* Fix the wrong robot name
* Fix some point for hydurs
* change name from hydra to hydrus
* some commit about gazebo multirotr
* Merge branch 'indigo-dev' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev
  Conflicts:
  hydra/hydra_transform_control/config/Hydra3.yaml
* ok
* Merge branch 'indigo-dev' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev
* some
* Merge branch 'indigo-dev' of ssh://aries.jsk.t.u-tokyo.ac.jp/home/jsk/chou/git/chou-ros-pkg/aerial_robot into indigo-dev
* some modification in aerial tracking and some comiit in aerial robot model
* 1. kalman filter for px4flow
  2. control input for old system
* add hydra x config
* add hydrax config
* hoge
* fix the dynamic reconfigure problem
* add lagrange moethod to get the best thrust value , along with the hydrax model
* hydrax model
* fix the pricipal intertial computation problem
* some changes
* some changes to commit before update to 14.04
* some changes to commit before update to 14.04
* add lqi and hamilton method
* modified the cog/pricipal_calc for hydra3, as well as the modelling
* add gain tunning mode form joy stick
* add files related to the aerial_robot_model
* some of the changes to be update
* add other object interactive model
* add interative marker control for dragon2
* add some new files for aerial transform
* correct most of the files to complete the hovering, aerial transform, vel control from joy
* modified aerial robot model files
* correct all files in aerial robot base
* delete unecessary files
* 1)add hydra movit config
  2)modified the aerial robot model for catkin system
* 1) add hydra directorry for transform control and moveit config
  2) modify the aerial robot model for catkin build system
* add pkgs related to aerial_robot
* Contributors: Moju Zhao, Tomoki Anzai
