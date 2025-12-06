cmake_minimum_required(VERSION 3.0.2)
project(hugmy)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_model
  pluginlib
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  jsk_recognition_msgs
  aerial_robot_msgs
  cv_bridge
  tf
  image_transport
  )

# find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS aerial_robot_control pluginlib aerial_robot_model
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${GAZEBO_INCLUDE_DIRS}
  )

# add_definitions(${GAZEBO_CXX_FLAGS})
# link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(hugmy_navigation src/navigation/hugmy_navigation.cpp)
target_link_libraries(hugmy_navigation ${catkin_LIBRARIES})

add_library(hugmy_deformation_planning src/control/deformation_planning.cpp)
target_link_libraries(hugmy_deformation_planning ${catkin_LIBRARIES})
add_dependencies(hugmy_deformation_planning ${catkin_EXPORTED_TARGETS})

add_library(hugmy_morphing_arm_controller src/control/morphing_arm_controller.cpp)
target_link_libraries(hugmy_morphing_arm_controller ${catkin_LIBRARIES})

add_library(lpv_lqi_qp_controller src/control/lpv_lqi_qp_controller.cpp)
target_link_libraries(lpv_lqi_qp_controller ${catkin_LIBRARIES})
add_dependencies(lpv_lqi_qp_controller ${catkin_EXPORTED_TARGETS})

add_library(lqi_null_controller src/control/lqi_null_controller.cpp)
target_link_libraries(lqi_null_controller ${catkin_LIBRARIES})
add_dependencies(lqi_null_controller ${catkin_EXPORTED_TARGETS})

add_executable(random_haptics_threshold_node src/control/test/haptics_thre_test.cpp)
target_link_libraries(random_haptics_threshold_node ${catkin_LIBRARIES})

add_executable(random_haptics_feeling_node src/control/test/haptics_feeling_test.cpp)
target_link_libraries(random_haptics_feeling_node ${catkin_LIBRARIES})

# add_executable(guidance_manual_controller src/control/test/haptics_guidance_manual_test.cpp
#   src/control/air_pressure_controller.cpp
#   src/control/haptics_controller.cpp)
# target_link_libraries(guidance_manual_controller ${catkin_LIBRARIES})

# add_executable(haptics_integrated_controller src/control/integrated.cpp
#   src/control/air_pressure_controller.cpp
#   src/control/haptics_controller.cpp
#   src/control/rviz_visualizer.cpp
#   )
# target_link_libraries(haptics_integrated_controller ${catkin_LIBRARIES})
# add_dependencies(haptics_integrated_controller ${catkin_EXPORTED_TARGETS})


add_executable(thrust_and_pressure_cal src/control/thrust_and_pressure_cal.cpp)
target_link_libraries(thrust_and_pressure_cal ${catkin_LIBRARIES})
target_include_directories(thrust_and_pressure_cal PRIVATE ${EIGEN3_INCLUDE_DIRS})


add_library(hugmy_air_pressure src/control/air_pressure_controller.cpp)
target_link_libraries(hugmy_air_pressure ${catkin_LIBRARIES})
add_dependencies(hugmy_air_pressure ${catkin_EXPORTED_TARGETS})

add_executable(air_pressure_control_node src/nodes/organize_air_pressure.cpp)
target_link_libraries(air_pressure_control_node  hugmy_air_pressure ${catkin_LIBRARIES})
add_dependencies(air_pressure_control_node ${catkin_EXPORTED_TARGETS})

add_executable(deformation_planning_node src/nodes/deformation_planning_node.cpp)
target_link_libraries(deformation_planning_node  hugmy_deformation_planning ${catkin_LIBRARIES})
add_dependencies(deformation_planning_node ${catkin_EXPORTED_TARGETS})


add_executable(joint_model src/control/joint_model.cpp)
target_link_libraries(joint_model ${catkin_LIBRARIES})
target_include_directories(joint_model PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(perching_node src/control/perching.cpp)
target_link_libraries(perching_node ${catkin_LIBRARIES})
add_dependencies(perching_node ${catkin_EXPORTED_TARGETS})

install(DIRECTORY config launch plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

