cmake_minimum_required(VERSION 3.0.2)
project(spinal)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  std_msgs
  message_generation
  rospy
  rqt_gui
  rqt_gui_py
)

catkin_python_setup()

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
  PwmTest.msg
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
  std_msgs
)

set(SPINAL_DIRS mcu_project/lib/Jsk_Lib)

catkin_package(
  INCLUDE_DIRS  ${SPINAL_DIRS}
  LIBRARIES spinal_flight_controller spinal_math
  CATKIN_DEPENDS roscpp std_srvs std_msgs rospy rqt_gui rqt_gui_py message_runtime
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


install(DIRECTORY ${SPINAL_DIRS}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY bin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )


install(DIRECTORY scripts resource launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(FILES rqt_gui_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(TARGETS spinal_math spinal_flight_controller
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
