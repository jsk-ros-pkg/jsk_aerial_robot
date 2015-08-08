cmake_minimum_required(VERSION 2.8.3)
project(hydra_transform_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  aerial_robot_msgs
  dynamixel_msgs
  dynamixel_controllers
  tf_conversions
  message_generation
  cmake_modules
)


find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

add_message_files(FILES
  HydraParam.msg
  TransformRes.msg
  )
generate_messages(DEPENDENCIES std_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp   aerial_robot_msgs dynamixel_msgs dynamixel_controllers tf_conversions message_runtime
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
)

add_executable(dynamicxel_bridge_node src/dynamicxel_bridge.cpp) 
target_link_libraries (dynamicxel_bridge_node  ${catkin_LIBRARIES})

add_executable(demo_transform_node src/demo_transform.cpp )
target_link_libraries (demo_transform_node  ${catkin_LIBRARIES})
add_dependencies(demo_transform_node aerial_robot_msgs_generate_message_cpp)

add_library (transform_control src/transform_control.cpp)
target_link_libraries (transform_control ${catkin_LIBRARIES})
add_dependencies(transform_control aerial_robot_msgs_generate_message_cpp)
add_executable (hydra_transform_control_node src/hydra_transform_control_node.cpp)
target_link_libraries (hydra_transform_control_node transform_control ${catkin_LIBRARIES})
add_dependencies(hydra_transform_control_node aerial_robot_msgs_generate_message_cpp)

install(PROGRAMS
  scripts/dynamixel_param.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS dynamicxel_bridge_node demo_transform_node transform_control hydra_transform_control_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


