cmake_minimum_required(VERSION 2.8.3)
project(spinal)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  message_generation
  rospy
  rosserial_msgs
  rqt_gui
  rqt_gui_py
)

add_message_files(
  FILES
  PMatrixPseudoInverseUnit.msg
  PMatrixPseudoInverseWithInertia.msg
  ServoInfo.msg
  BoardInfo.msg
  ServoState.msg
  ServoStates.msg
  ServoControlCmd.msg
  ServoTorqueCmd.msg
  ServoTorqueStates.msg
  Gyro.msg
  SimpleImu.msg
  FourAxisCommand.msg
  Imu.msg
  Barometer.msg
  Gps.msg
  GpsFull.msg
  RollPitchYawTerm.msg
  RollPitchYawTerms.msg
  MotorInfo.msg
  PwmInfo.msg
  Pwms.msg
  UavInfo.msg
  DesireCoord.msg
  FlightConfigCmd.msg
  Vector3Int16.msg
  TorqueAllocationMatrixInv.msg
  )

add_service_files(
  FILES
  GetBoardInfo.srv
  SetBoardConfig.srv
  SetAttitudeGains.srv
  ImuCalib.srv
  MagDeclination.srv
)

generate_messages(
  DEPENDENCIES
  rosserial_msgs
  std_msgs
)

set(SPINAL_DIRS mcu_project/lib/Jsk_Lib)

catkin_package(
  INCLUDE_DIRS  ${SPINAL_DIRS}
  LIBRARIES spinal_flight_controller spinal_math
  CATKIN_DEPENDS roscpp rospy rqt_gui rqt_gui_py
#  DEPENDS system_lib
)

include_directories(
  ${SPINAL_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-DSIMULATION)

add_library(spinal_math
  ${SPINAL_DIRS}/math/AP_Math.cpp
  ${SPINAL_DIRS}/math/matrix3.cpp
  ${SPINAL_DIRS}/math/vector2.cpp
  ${SPINAL_DIRS}/math/vector3.cpp
  ${SPINAL_DIRS}/math/quaternion.cpp
  ${SPINAL_DIRS}/math/location.cpp
  )

add_library(spinal_flight_controller
  ${SPINAL_DIRS}/flight_control/attitude/attitude_control.cpp)
target_link_libraries(spinal_flight_controller ${catkin_LIBRARIES} spinal_math)
add_dependencies(spinal_flight_controller ${PROJECT_NAME}_generate_messages_cpp)

# generate ros_lib for STM32F7
add_custom_target(${PROJECT_NAME}_generate_stm32f7_ros_lib ALL COMMAND ${CATKIN_DEVEL_PREFIX}/env.sh ${PYTHON_EXECUTABLE}  ${PROJECT_SOURCE_DIR}/scripts/make_libraries.py --save_path ${PROJECT_SOURCE_DIR}/mcu_project/boards/stm32F7 --support_rtos)
add_dependencies(${PROJECT_NAME}_generate_stm32f7_ros_lib ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# generate ros_lib for STM32H7
add_custom_target(${PROJECT_NAME}_generate_stm32h7_ros_lib ALL COMMAND ${CATKIN_DEVEL_PREFIX}/env.sh ${PYTHON_EXECUTABLE}  ${PROJECT_SOURCE_DIR}/scripts/make_libraries.py --save_path ${PROJECT_SOURCE_DIR}/mcu_project/boards/stm32H7 --support_rtos --support_ethernet)
add_dependencies(${PROJECT_NAME}_generate_stm32h7_ros_lib ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)


